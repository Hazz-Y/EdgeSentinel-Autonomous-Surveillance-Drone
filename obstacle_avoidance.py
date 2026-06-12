"""
Autonomous Drone Obstacle Avoidance Flight
WITH Integrated Live Dashboard (Flask web server)

Hardware: Pixhawk 2.4.8 + Jetson Nano + Intel RealSense D455
Protocol: MAVLink via DroneKit + pyrealsense2

════════════════════════════════════════════════════════
DEPTH COLORMAP (Jet - corrected):
  RED   = CLOSE / NEAR objects  → OBSTACLE
  BLUE  = FAR  / OPEN space     → SAFE
  BLACK = No depth data / void  → SAFE
  invert_depth_values=1 ensures near=red, far=blue.
  No vertical flip needed.
════════════════════════════════════════════════════════

Dashboard access (open on any device on the same WiFi):
  http://<JETSON_IP>:5000
  (IP printed in terminal on startup)

Requirements:
  pip install dronekit pymavlink pyrealsense2 opencv-python numpy flask
"""

import time, sys, math, threading, socket, io
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
from flask import Flask, Response, render_template_string, jsonify
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ══════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════

CONNECTION_STRING   = "/dev/ttyACM0"
BAUD_RATE           = 57600

FLIGHT_ALTITUDE     = 1.0
FORWARD_DISTANCE    = 6.0
SIDESTEP_DISTANCE   = 1.0
CRUISE_SPEED        = 0.6
SIDESTEP_SPEED      = 0.4

ALTITUDE_TOLERANCE  = 0.15
POSITION_TOLERANCE  = 0.4
WAYPOINT_TIMEOUT    = 30.0

FRAME_W, FRAME_H    = 640, 480
CENTER_X_START      = int(FRAME_W * 0.25)
CENTER_X_END        = int(FRAME_W * 0.75)
CENTER_Y_START      = int(FRAME_H * 0.30)
CENTER_Y_END        = int(FRAME_H * 0.70)
LEFT_X_END          = int(FRAME_W * 0.25)
RIGHT_X_START       = int(FRAME_W * 0.75)

RED_RATIO_THRESHOLD   = 0.15
CLEAR_RATIO_THRESHOLD = 0.80

OBSTACLE_HOLD_SEC   = 1.0
OBSTACLE_MAX_HOLD   = 10
MODE_RETRY_DELAY    = 0.5
MODE_MAX_RETRIES    = 20
ARM_RETRY_DELAY     = 1.0
ARM_MAX_RETRIES     = 20
ARM_DELAY           = 2

DASHBOARD_PORT      = 5000

# ══════════════════════════════════════════════════════
#  SHARED LIVE STATE  (thread-safe)
# ══════════════════════════════════════════════════════

class DroneState:
    def __init__(self):
        self._lock = threading.Lock()
        self.data = {
            "altitude": 0.0, "heading": 0.0, "armed": False,
            "mode": "-", "gps_fix": 0, "ekf_ok": False,
            "lat": 0.0, "lon": 0.0,
            "decision": "WAITING", "decision_reason": "Initializing …",
            "center_red": 0.0, "left_clear": 0.0, "right_clear": 0.0,
            "obstacle": False, "left_ok": False, "right_ok": False,
            "leg": "PRE-FLIGHT", "dist_to_wp": 0.0,
            "log_lines": [], "connected": False,
            "timestamp": "-",
        }

    def update(self, **kwargs):
        with self._lock:
            self.data.update(kwargs)
            self.data["timestamp"] = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def get(self):
        with self._lock:
            return dict(self.data)

    def add_log(self, msg: str):
        with self._lock:
            ts = datetime.now().strftime("%H:%M:%S")
            self.data["log_lines"].append(f"[{ts}] {msg}")
            if len(self.data["log_lines"]) > 100:
                self.data["log_lines"] = self.data["log_lines"][-100:]


STATE = DroneState()


def log(msg: str):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)
    STATE.add_log(msg)


def abort(msg: str):
    log(f"ABORT ─ {msg}")
    sys.exit(1)


# ══════════════════════════════════════════════════════
#  REALSENSE D455  -  depth + color analysis
# ══════════════════════════════════════════════════════

class DepthCamera:
    def __init__(self):
        self.pipeline   = rs.pipeline()
        self.config     = rs.config()
        self.colorizer  = rs.colorizer()
        self._frame_lock = threading.Lock()
        self._latest_bgr = None   # annotated frame
        self._running    = False

    def start(self):
        self.config.enable_stream(rs.stream.depth, FRAME_W, FRAME_H, rs.format.z16, 30)
        self.pipeline.start(self.config)
        self._running = True
        log("RealSense warming up …")
        for _ in range(30):
            self.pipeline.wait_for_frames()
        log("✓ RealSense D455 ready")

        # ★ Jet colormap with depth inversion:
        #   color_scheme 0 = Jet  →  low-value = blue, high-value = red (default)
        #   invert_depth_values 1  →  near objects (small mm) become HIGH in the
        #   colormap  →  RED; far objects become BLUE; zero/no-data stays BLACK.
        self.colorizer.set_option(rs.option.color_scheme, 0)         # Jet
        self.colorizer.set_option(rs.option.invert_depth_values, 1)  # near=RED, far=BLUE

        threading.Thread(target=self._capture_loop, daemon=True).start()

    def _capture_loop(self):
        while self._running:
            try:
                frames      = self.pipeline.wait_for_frames(timeout_ms=2000)
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue
                colored = self.colorizer.colorize(depth_frame)
                raw     = np.asanyarray(colored.get_data())
                # RealSense gives RGB → convert to BGR for OpenCV
                bgr     = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
                # No vertical flip needed - invert_depth_values handles near=red
                annotated = self._annotate(bgr)
                with self._frame_lock:
                    self._latest_bgr = annotated
            except Exception:
                time.sleep(0.1)

    # ── color zone helpers ───────────────────────────────

    @staticmethod
    def _red_ratio(crop: np.ndarray) -> float:
        hsv   = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        m1    = cv2.inRange(hsv, (0,   80, 80), (10,  255, 255))
        m2    = cv2.inRange(hsv, (165, 80, 80), (180, 255, 255))
        total = crop.shape[0] * crop.shape[1]
        return np.count_nonzero(cv2.bitwise_or(m1, m2)) / max(total, 1)

    @staticmethod
    def _clear_ratio(crop: np.ndarray) -> float:
        hsv        = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        blue_mask  = cv2.inRange(hsv, (95,  50, 30), (135, 255, 255))
        black_mask = cv2.inRange(hsv, (0,   0,  0),  (180, 255,  40))
        total      = crop.shape[0] * crop.shape[1]
        return np.count_nonzero(cv2.bitwise_or(blue_mask, black_mask)) / max(total, 1)

    def _annotate(self, frame: np.ndarray) -> np.ndarray:
        """Draw zone boxes + decision banner and update shared state."""
        vis = frame.copy()

        ctr_crop  = frame[CENTER_Y_START:CENTER_Y_END, CENTER_X_START:CENTER_X_END]
        lft_crop  = frame[CENTER_Y_START:CENTER_Y_END, 0:LEFT_X_END]
        rgt_crop  = frame[CENTER_Y_START:CENTER_Y_END, RIGHT_X_START:FRAME_W]

        ctr_red   = self._red_ratio(ctr_crop)
        lft_clr   = self._clear_ratio(lft_crop)
        rgt_clr   = self._clear_ratio(rgt_crop)

        obstacle  = ctr_red >= RED_RATIO_THRESHOLD
        left_ok   = lft_clr >= CLEAR_RATIO_THRESHOLD
        right_ok  = rgt_clr >= CLEAR_RATIO_THRESHOLD

        decision, reason = self._decide(obstacle, left_ok, right_ok,
                                         ctr_red, lft_clr, rgt_clr)

        STATE.update(
            center_red=round(ctr_red, 3), left_clear=round(lft_clr, 3),
            right_clear=round(rgt_clr, 3), obstacle=obstacle,
            left_ok=left_ok, right_ok=right_ok,
            decision=decision, decision_reason=reason,
        )

        # Zone rectangles
        def zone_rect(x1, y1, x2, y2, ok_color, bad_color, ok):
            cv2.rectangle(vis, (x1, y1), (x2, y2),
                          ok_color if ok else bad_color, 2)

        zone_rect(CENTER_X_START, CENTER_Y_START, CENTER_X_END, CENTER_Y_END,
                  (0, 220, 60), (0, 0, 220), not obstacle)
        zone_rect(0, CENTER_Y_START, LEFT_X_END, CENTER_Y_END,
                  (220, 220, 0), (60, 60, 60), left_ok)
        zone_rect(RIGHT_X_START, CENTER_Y_START, FRAME_W, CENTER_Y_END,
                  (220, 220, 0), (60, 60, 60), right_ok)

        # Zone labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(vis, f"LEFT {lft_clr:.0%}",
                    (4, CENTER_Y_START - 6), font, 0.45,
                    (220, 220, 0) if left_ok else (80, 80, 80), 1)
        cv2.putText(vis, f"FWD {ctr_red:.0%} RED",
                    (CENTER_X_START + 4, CENTER_Y_START - 6), font, 0.45,
                    (0, 0, 220) if obstacle else (0, 220, 60), 1)
        cv2.putText(vis, f"RIGHT {rgt_clr:.0%}",
                    (RIGHT_X_START + 4, CENTER_Y_START - 6), font, 0.45,
                    (220, 220, 0) if right_ok else (80, 80, 80), 1)

        # Crosshair
        cx, cy = FRAME_W // 2, FRAME_H // 2
        cv2.line(vis, (cx - 18, cy), (cx + 18, cy), (255, 255, 255), 1)
        cv2.line(vis, (cx, cy - 18), (cx, cy + 18), (255, 255, 255), 1)

        # Colormap legend watermark
        cv2.putText(vis, "RED=NEAR  BLUE=FAR  BLACK=NONE", (4, FRAME_H - 8),
                    font, 0.38, (180, 180, 180), 1)

        # Decision banner
        banner_colors = {
            "MOVE FORWARD": (0, 160, 50),
            "DODGE LEFT":   (0, 160, 200),
            "DODGE RIGHT":  (0, 130, 200),
            "HOLD":         (0, 50, 200),
            "WAITING":      (60, 60, 60),
        }
        bc = banner_colors.get(decision, (60, 60, 60))
        cv2.rectangle(vis, (0, FRAME_H - 34), (FRAME_W, FRAME_H), bc, -1)
        cv2.putText(vis, f"  {decision}  |  {reason}",
                    (6, FRAME_H - 11), font, 0.52, (255, 255, 255), 1)

        return vis

    @staticmethod
    def _decide(obstacle, left_ok, right_ok, ctr_red, lft_clr, rgt_clr):
        if not obstacle:
            return "MOVE FORWARD", f"Path clear  red={ctr_red:.0%}"
        if left_ok:
            return "DODGE LEFT",   f"Left clear {lft_clr:.0%} - dodging"
        if right_ok:
            return "DODGE RIGHT",  f"Right clear {rgt_clr:.0%} - dodging"
        return "HOLD",             f"All blocked  L:{lft_clr:.0%} R:{rgt_clr:.0%}"

    def get_frame(self):
        with self._frame_lock:
            return self._latest_bgr.copy() if self._latest_bgr is not None else None

    def get_jpeg(self) -> bytes:
        frame = self.get_frame()
        if frame is None:
            ph = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
            cv2.putText(ph, "Waiting for RealSense …",
                        (80, FRAME_H // 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (100, 100, 100), 2)
            frame = ph
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        return jpeg.tobytes()

    def analyze_zones(self):
        frame = self.get_frame()
        if frame is None:
            return None
        ctr  = frame[CENTER_Y_START:CENTER_Y_END, CENTER_X_START:CENTER_X_END]
        lft  = frame[CENTER_Y_START:CENTER_Y_END, 0:LEFT_X_END]
        rgt  = frame[CENTER_Y_START:CENTER_Y_END, RIGHT_X_START:FRAME_W]
        cr   = self._red_ratio(ctr)
        lc   = self._clear_ratio(lft)
        rc   = self._clear_ratio(rgt)
        return {
            "center_red": cr, "left_clear": lc, "right_clear": rc,
            "obstacle": cr >= RED_RATIO_THRESHOLD,
            "left_ok":  lc >= CLEAR_RATIO_THRESHOLD,
            "right_ok": rc >= CLEAR_RATIO_THRESHOLD,
        }

    def stop(self):
        self._running = False
        time.sleep(0.3)
        try: self.pipeline.stop()
        except: pass
        log("RealSense stopped")


# ══════════════════════════════════════════════════════
#  FLASK DASHBOARD
# ══════════════════════════════════════════════════════

app    = Flask(__name__)
camera = DepthCamera()

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Drone Dashboard</title>
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Barlow+Condensed:wght@400;600;800&display=swap" rel="stylesheet"/>
<style>
:root{
  --bg:#070c10;--panel:#0c1520;--border:#162035;
  --accent:#00d4ff;--green:#00ff88;--red:#ff3333;
  --yellow:#ffc230;--dim:#3a5068;--text:#b8d0e8;
  --mono:'Share Tech Mono',monospace;
  --display:'Barlow Condensed',sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:var(--mono);min-height:100vh;display:flex;flex-direction:column}
header{display:flex;align-items:center;justify-content:space-between;padding:9px 20px;border-bottom:1px solid var(--border);background:linear-gradient(90deg,#080f1a,#0b1828)}
header h1{font-family:var(--display);font-weight:800;font-size:1.25rem;letter-spacing:.18em;color:var(--accent);text-transform:uppercase}
header h1 span{color:var(--text);font-weight:400}
#cb{font-size:.65rem;padding:3px 10px;border-radius:2px;font-family:var(--display);font-weight:600;letter-spacing:.1em;text-transform:uppercase}
#cb.ok{background:var(--green);color:#000}#cb.err{background:var(--red);color:#fff}

.layout{display:grid;grid-template-columns:1fr 300px;gap:10px;padding:10px;flex:1}
.left{display:flex;flex-direction:column;gap:10px}

/* feed */
.feed-wrap{position:relative;background:#000;border:1px solid var(--border);border-radius:4px;overflow:hidden;aspect-ratio:4/3}
.feed-wrap img{width:100%;height:100%;object-fit:cover;display:block}
.fo{position:absolute;top:7px;left:7px;display:flex;gap:6px;pointer-events:none}
.badge{font-family:var(--display);font-weight:600;font-size:.65rem;letter-spacing:.1em;padding:2px 8px;border-radius:2px;text-transform:uppercase}
.badge.live{background:var(--red);color:#fff;animation:blink 1s step-end infinite}
.badge.cam{background:rgba(0,0,0,.6);color:var(--accent);border:1px solid var(--accent)}
.badge.cmap{background:rgba(0,0,0,.6);color:var(--yellow);border:1px solid var(--yellow)}
@keyframes blink{50%{opacity:0}}

/* colormap legend */
.cmap-legend{display:flex;align-items:center;gap:8px;padding:5px 10px;background:var(--panel);border:1px solid var(--border);border-radius:3px;font-size:.6rem;letter-spacing:.08em;text-transform:uppercase}
.cmap-bar{height:10px;flex:1;border-radius:2px;background:linear-gradient(to right,#000 0%,#00008b 15%,#0000ff 30%,#00ffff 50%,#ffff00 70%,#ff4400 85%,#ff0000 100%)}
.cmap-near{color:var(--red);font-weight:bold}
.cmap-far{color:#4488ff;font-weight:bold}
.cmap-none{color:var(--dim);font-weight:bold}

/* decision bar */
.dec-bar{border:1px solid var(--border);border-radius:4px;padding:10px 14px;display:flex;align-items:center;gap:14px;background:var(--panel);transition:border-color .25s}
.dec-bar.clear{border-color:var(--green)}.dec-bar.dodge{border-color:var(--yellow)}.dec-bar.hold{border-color:var(--red)}
.dec-label{font-family:var(--display);font-weight:800;font-size:1.5rem;letter-spacing:.08em;min-width:170px;text-transform:uppercase}
.dec-label.clear{color:var(--green)}.dec-label.dodge{color:var(--yellow)}.dec-label.hold{color:var(--red)}.dec-label.wait{color:var(--dim)}
.dec-reason{font-size:.75rem;color:var(--text);opacity:.8}

/* zones */
.zones{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.zc{border:1px solid var(--border);border-radius:3px;padding:8px 5px;text-align:center;transition:border-color .25s,background .25s}
.zc .zn{font-family:var(--display);font-size:.6rem;font-weight:600;letter-spacing:.1em;text-transform:uppercase;color:var(--dim);margin-bottom:3px}
.zc .zp{font-size:1.25rem;font-weight:bold;line-height:1}
.zc .zs{font-size:.55rem;margin-top:3px;text-transform:uppercase;letter-spacing:.08em}
.zc.obstacle{border-color:var(--red);background:rgba(255,51,51,.07)}
.zc.clear{border-color:var(--green);background:rgba(0,255,136,.05)}
.zc.blocked{border-color:var(--dim);background:rgba(0,0,0,.2)}

/* right column */
.right{display:flex;flex-direction:column;gap:10px}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:4px;padding:11px 13px}
.pt{font-family:var(--display);font-weight:600;font-size:.6rem;letter-spacing:.2em;text-transform:uppercase;color:var(--dim);margin-bottom:9px;border-bottom:1px solid var(--border);padding-bottom:5px}

/* telemetry */
.tg{display:grid;grid-template-columns:1fr 1fr;gap:7px}
.ti .tl{font-size:.58rem;color:var(--dim);text-transform:uppercase;letter-spacing:.12em;margin-bottom:2px}
.ti .tv{font-size:1rem;font-weight:bold;color:var(--accent)}
.ti .tv.armed{color:var(--green)}.ti .tv.disarmed{color:var(--red)}

/* alt gauge */
.gr{display:flex;align-items:center;gap:8px;margin-top:8px}
.gl{font-size:.6rem;color:var(--dim);text-transform:uppercase;letter-spacing:.1em;min-width:28px}
.gbw{flex:1;height:4px;background:var(--border);border-radius:2px;overflow:hidden}
.gb{height:100%;border-radius:2px;transition:width .3s,background .3s}
.gv{font-size:.95rem;font-weight:bold;color:var(--accent);min-width:42px;text-align:right}

/* compass */
.cw{display:flex;justify-content:center;margin:5px 0}
.gps{font-size:.7rem;color:var(--accent);text-align:center;margin-top:3px;letter-spacing:.04em}

/* log */
.lb{height:160px;overflow-y:auto;font-size:.64rem;line-height:1.65;color:#5a7f98;padding:3px 0}
.lb::-webkit-scrollbar{width:3px}.lb::-webkit-scrollbar-thumb{background:var(--border)}
.le{padding:1px 0;border-bottom:1px solid rgba(22,32,53,.5)}
.le .lt{color:var(--dim)}
.le .lm.ok{color:var(--green)}.le .lm.warn{color:var(--yellow)}.le .lm.err{color:var(--red)}

#tsb{text-align:right;font-size:.55rem;color:var(--dim);padding:3px 14px 7px}
</style>
</head>
<body>
<header>
  <h1>DRONE <span>AVOIDANCE</span> DASHBOARD</h1>
  <div style="display:flex;gap:10px;align-items:center">
    <span style="font-size:.65rem;color:var(--dim)">D455 + Pixhawk 2.4.8 + Jetson Nano</span>
    <div id="cb" class="err">NO PIXHAWK</div>
  </div>
</header>

<div class="layout">
  <div class="left">

    <div class="feed-wrap">
      <img id="feed" src="/video_feed" alt="Depth"/>
      <div class="fo">
        <span class="badge live">● LIVE</span>
        <span class="badge cam">DEPTH · JET</span>
        <span class="badge cmap">RED=NEAR · BLUE=FAR</span>
      </div>
    </div>

    <!-- Colormap legend bar -->
    <div class="cmap-legend">
      <span class="cmap-none">■ BLACK = NO DATA</span>
      <div class="cmap-bar"></div>
      <span class="cmap-far">■ BLUE = FAR</span>
      <span style="color:var(--dim)">→</span>
      <span class="cmap-near">■ RED = NEAR</span>
    </div>

    <div class="dec-bar" id="dbar">
      <div class="dec-label" id="dlabel">WAITING</div>
      <div>
        <div style="font-size:.58rem;color:var(--dim);text-transform:uppercase;letter-spacing:.1em;margin-bottom:3px">Current Action</div>
        <div class="dec-reason" id="dreason">Initializing …</div>
      </div>
    </div>

    <div class="panel">
      <div class="pt">Depth Zone Analysis</div>
      <div class="zones">
        <div class="zc" id="zleft"><div class="zn">◀ Left</div><div class="zp" id="zl-p">-</div><div class="zs" id="zl-s">-</div></div>
        <div class="zc" id="zctr"><div class="zn">▲ Forward</div><div class="zp" id="zc-p">-</div><div class="zs" id="zc-s">-</div></div>
        <div class="zc" id="zright"><div class="zn">▶ Right</div><div class="zp" id="zr-p">-</div><div class="zs" id="zr-s">-</div></div>
      </div>
    </div>
  </div>

  <div class="right">

    <div class="panel">
      <div class="pt">Navigation</div>
      <div class="cw"><canvas id="compass" width="116" height="116"></canvas></div>
      <div class="gps" id="gps">GPS: - , -</div>
    </div>

    <div class="panel">
      <div class="pt">Telemetry</div>
      <div class="tg">
        <div class="ti"><div class="tl">Altitude</div><div class="tv" id="t-alt">-</div></div>
        <div class="ti"><div class="tl">Heading</div><div class="tv" id="t-hdg">-</div></div>
        <div class="ti"><div class="tl">Mode</div><div class="tv" id="t-mode" style="color:var(--accent)">-</div></div>
        <div class="ti"><div class="tl">Armed</div><div class="tv" id="t-arm">-</div></div>
        <div class="ti"><div class="tl">GPS Fix</div><div class="tv" id="t-gps">-</div></div>
        <div class="ti"><div class="tl">EKF</div><div class="tv" id="t-ekf">-</div></div>
      </div>
      <div class="gr">
        <div class="gl">ALT</div>
        <div class="gbw"><div class="gb" id="altbar" style="width:0%;background:var(--accent)"></div></div>
        <div class="gv" id="altv">0m</div>
      </div>
    </div>

    <div class="panel">
      <div class="pt">Mission</div>
      <div class="tg">
        <div class="ti"><div class="tl">Leg</div><div class="tv" id="t-leg" style="color:var(--yellow)">-</div></div>
        <div class="ti"><div class="tl">Dist to WP</div><div class="tv" id="t-dist">-</div></div>
      </div>
    </div>

    <div class="panel" style="flex:1">
      <div class="pt">Event Log</div>
      <div class="lb" id="logbox"></div>
    </div>
  </div>
</div>
<div id="tsb">Last update: <span id="ts">-</span></div>

<script>
const c = document.getElementById('compass'), ctx = c.getContext('2d');
function drawCompass(deg){
  const w=c.width,h=c.height,cx=w/2,cy=h/2,r=46;
  ctx.clearRect(0,0,w,h);
  ctx.beginPath();ctx.arc(cx,cy,r,0,Math.PI*2);
  ctx.fillStyle='#0a1220';ctx.fill();
  ctx.strokeStyle='#162035';ctx.lineWidth=1.5;ctx.stroke();
  for(let i=0;i<36;i++){
    const a=(i*10-90)*Math.PI/180,inner=i%9===0?r-12:r-7;
    ctx.beginPath();
    ctx.moveTo(cx+Math.cos(a)*inner,cy+Math.sin(a)*inner);
    ctx.lineTo(cx+Math.cos(a)*(r-2),cy+Math.sin(a)*(r-2));
    ctx.strokeStyle=i%9===0?'#2a4060':'#162035';
    ctx.lineWidth=i%9===0?1.5:1;ctx.stroke();
  }
  [['N','#00d4ff',0],['E','#5a7f98',90],['S','#5a7f98',180],['W','#5a7f98',270]].forEach(([l,col,d])=>{
    const a=(d-90)*Math.PI/180;
    ctx.fillStyle=col;ctx.font='bold 9px "Barlow Condensed",sans-serif';
    ctx.textAlign='center';ctx.textBaseline='middle';
    ctx.fillText(l,cx+Math.cos(a)*(r-17),cy+Math.sin(a)*(r-17));
  });
  const na=(deg-90)*Math.PI/180;
  ctx.beginPath();ctx.moveTo(cx,cy);
  ctx.lineTo(cx+Math.cos(na)*(r-8),cy+Math.sin(na)*(r-8));
  ctx.strokeStyle='#00d4ff';ctx.lineWidth=2;ctx.lineCap='round';ctx.stroke();
  ctx.beginPath();ctx.arc(cx,cy,3,0,Math.PI*2);
  ctx.fillStyle='#00d4ff';ctx.fill();
  ctx.fillStyle='#b8d0e8';ctx.font='bold 12px "Share Tech Mono",monospace';
  ctx.textAlign='center';ctx.textBaseline='middle';
  ctx.fillText(Math.round(deg)+'°',cx,cy+r+11);
}
drawCompass(0);

let lastLog=0;
async function poll(){
  try{
    const r=await fetch('/state'), s=await r.json();
    // conn
    const cb=document.getElementById('cb');
    cb.textContent=s.connected?'CONNECTED':'NO PIXHAWK';
    cb.className=s.connected?'ok':'err';
    // decision
    const dec=s.decision||'WAITING';
    const dl=document.getElementById('dlabel'), db=document.getElementById('dbar');
    dl.textContent=dec;
    const cls=dec.includes('FORWARD')?'clear':dec.includes('DODGE')?'dodge':dec==='HOLD'?'hold':'wait';
    dl.className='dec-label '+cls; db.className='dec-bar '+cls;
    document.getElementById('dreason').textContent=s.decision_reason||'-';
    // zones
    const cr=Math.round(s.center_red*100),lc=Math.round(s.left_clear*100),rc=Math.round(s.right_clear*100);
    document.getElementById('zc-p').textContent=cr+'%';
    document.getElementById('zc-s').textContent=s.obstacle?'🔴 OBSTACLE':'✅ CLEAR';
    document.getElementById('zctr').className='zc '+(s.obstacle?'obstacle':'clear');
    document.getElementById('zl-p').textContent=lc+'%';
    document.getElementById('zl-s').textContent=s.left_ok?'✅ CLEAR':'⬛ BLOCKED';
    document.getElementById('zleft').className='zc '+(s.left_ok?'clear':'blocked');
    document.getElementById('zr-p').textContent=rc+'%';
    document.getElementById('zr-s').textContent=s.right_ok?'✅ CLEAR':'⬛ BLOCKED';
    document.getElementById('zright').className='zc '+(s.right_ok?'clear':'blocked');
    // telem
    document.getElementById('t-alt').textContent=s.altitude+' m';
    document.getElementById('t-hdg').textContent=s.heading+'°';
    document.getElementById('t-mode').textContent=s.mode;
    const ta=document.getElementById('t-arm');
    ta.textContent=s.armed?'ARMED':'DISARMED';
    ta.className='tv '+(s.armed?'armed':'disarmed');
    document.getElementById('t-gps').textContent=['None','None','2D','3D','3D+'][Math.min(s.gps_fix,4)];
    document.getElementById('t-ekf').textContent=s.ekf_ok?'✅ OK':'⚠ WARN';
    const ap=Math.min((s.altitude/10)*100,100);
    document.getElementById('altbar').style.width=ap+'%';
    document.getElementById('altv').textContent=s.altitude+'m';
    document.getElementById('t-leg').textContent=s.leg;
    document.getElementById('t-dist').textContent=s.dist_to_wp+'m';
    document.getElementById('gps').textContent='GPS: '+s.lat+', '+s.lon;
    drawCompass(s.heading||0);
    document.getElementById('ts').textContent=s.timestamp;
    // log
    const lb=document.getElementById('logbox');
    if(s.log_lines&&s.log_lines.length!==lastLog){
      lastLog=s.log_lines.length;
      lb.innerHTML=s.log_lines.slice(-70).map(l=>{
        const m=l.match(/^\[(.+?)\] (.+)$/);
        if(!m)return`<div class="le">${l}</div>`;
        const cls=m[2].startsWith('✓')?'ok':m[2].startsWith('⚠')?'warn':m[2].startsWith('ABORT')?'err':'';
        return`<div class="le"><span class="lt">[${m[1]}]</span> <span class="lm ${cls}">${m[2]}</span></div>`;
      }).join('');
      lb.scrollTop=lb.scrollHeight;
    }
  }catch(e){}
  setTimeout(poll,220);
}
poll();
</script>
</body>
</html>"""


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/video_feed")
def video_feed():
    def gen():
        while True:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + camera.get_jpeg() + b"\r\n"
            time.sleep(1 / 25)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/state")
def get_state():
    return jsonify(STATE.get())


def start_dashboard():
    local_ip = _local_ip()
    print("=" * 55)
    print(f"  Dashboard  →  http://{local_ip}:{DASHBOARD_PORT}")
    print(f"  Local      →  http://localhost:{DASHBOARD_PORT}")
    print("=" * 55)
    app.run(host="0.0.0.0", port=DASHBOARD_PORT, threaded=True,
            debug=False, use_reloader=False)


def _local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]; s.close(); return ip
    except:
        return "127.0.0.1"


# ══════════════════════════════════════════════════════
#  MAVLINK / DRONEKIT HELPERS
# ══════════════════════════════════════════════════════

def get_horizontal_distance(loc1, loc2) -> float:
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)


def offset_location(origin, d_north, d_east):
    R = 6378137.0
    return LocationGlobalRelative(
        origin.lat + math.degrees(d_north / R),
        origin.lon + math.degrees(d_east / (R * math.cos(math.radians(origin.lat)))),
        origin.alt,
    )


def get_yaw_degrees(vehicle) -> float:
    return (math.degrees(vehicle.attitude.yaw) + 360) % 360


def heading_to_ned(distance, heading_deg):
    r = math.radians(heading_deg)
    return distance * math.cos(r), distance * math.sin(r)


def send_yaw_command(vehicle, heading_deg, relative=False):
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
        heading_deg, 30, 1, 1 if relative else 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def wait_for_yaw(vehicle, target, tol=5.0, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        diff = abs((get_yaw_degrees(vehicle) - target + 180) % 360 - 180)
        if diff <= tol:
            log(f"  ✓ Yaw {get_yaw_degrees(vehicle):.1f}° → {target:.1f}°")
            return
        time.sleep(0.2)
    log(f"  ⚠ Yaw timeout")


def check_altitude(vehicle, target_alt, label=""):
    alt  = vehicle.location.global_relative_frame.alt
    diff = abs(alt - target_alt)
    tag  = f"[{label}] " if label else ""
    flag = "✓" if diff <= ALTITUDE_TOLERANCE else "⚠ ALT DRIFT"
    log(f"  {tag}Alt: {alt:.2f}m  err: {diff:.2f}m  {flag}")
    return alt


def goto_wp(vehicle, wp, target_alt, label, heading_lock=None,
            timeout=WAYPOINT_TIMEOUT):
    vehicle.simple_goto(wp, airspeed=CRUISE_SPEED)
    deadline = time.time() + timeout
    while time.time() < deadline:
        current = vehicle.location.global_relative_frame
        dist    = get_horizontal_distance(current, wp)
        STATE.update(dist_to_wp=round(dist, 2))
        check_altitude(vehicle, target_alt, label=label)
        log(f"    → [{label}] dist: {dist:.2f}m")
        if heading_lock is not None:
            send_yaw_command(vehicle, heading_lock)
        if dist <= POSITION_TOLERANCE:
            log(f"✓ Reached [{label}]  dist={dist:.2f}m")
            return True
        if not vehicle.armed:
            abort("Drone disarmed mid-flight!")
        time.sleep(0.4)
    log(f"⚠ Timeout reaching [{label}]")
    return False


# ══════════════════════════════════════════════════════
#  CONNECT / MODE / ARM / TAKEOFF
# ══════════════════════════════════════════════════════

def connect_vehicle():
    log(f"Connecting to Pixhawk on {CONNECTION_STRING} @ {BAUD_RATE} …")
    try:
        vehicle = connect(CONNECTION_STRING, baud=BAUD_RATE,
                          wait_ready=True, timeout=60, heartbeat_timeout=30)
    except Exception as e:
        abort(f"Connection failed: {e}")
    STATE.update(connected=True, mode=vehicle.mode.name,
                 gps_fix=vehicle.gps_0.fix_type, ekf_ok=vehicle.ekf_ok)
    log(f"✓ Connected | FW:{vehicle.version} | GPS:{vehicle.gps_0.fix_type} | EKF:{vehicle.ekf_ok}")
    return vehicle


def telemetry_loop(vehicle):
    """Background thread - keeps STATE in sync with vehicle."""
    while True:
        try:
            loc = vehicle.location.global_relative_frame
            STATE.update(
                altitude = round(loc.alt or 0.0, 2),
                heading  = round(get_yaw_degrees(vehicle), 1),
                armed    = vehicle.armed,
                mode     = vehicle.mode.name,
                gps_fix  = vehicle.gps_0.fix_type,
                ekf_ok   = vehicle.ekf_ok,
                lat      = round(loc.lat or 0.0, 7),
                lon      = round(loc.lon or 0.0, 7),
            )
        except:
            pass
        time.sleep(0.2)


def set_guided_mode(vehicle):
    log("Setting GUIDED mode …")
    for _ in range(MODE_MAX_RETRIES):
        if vehicle.mode.name == "GUIDED":
            log("✓ GUIDED confirmed"); return
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(MODE_RETRY_DELAY)
    abort("Could not enter GUIDED mode")


def arm_vehicle(vehicle):
    log("Arming …")
    for _ in range(30):
        if vehicle.gps_0.fix_type >= 3: break
        time.sleep(1)
    else:
        log("WARNING: No 3D GPS fix")
    for _ in range(20):
        if vehicle.ekf_ok: break
        time.sleep(1)
    else:
        log("WARNING: EKF not healthy")
    for attempt in range(1, ARM_MAX_RETRIES + 1):
        if vehicle.armed:
            log(f"✓ ARMED (attempt {attempt})"); return
        vehicle.armed = True
        time.sleep(ARM_RETRY_DELAY)
    abort(f"Failed to ARM after {ARM_MAX_RETRIES} attempts")


def takeoff(vehicle, alt):
    time.sleep(ARM_DELAY)
    log(f"Takeoff to {alt}m …")
    vehicle.simple_takeoff(alt)
    while True:
        a = vehicle.location.global_relative_frame.alt
        log(f"  Climbing: {a:.2f}m")
        if a >= alt - ALTITUDE_TOLERANCE:
            log(f"✓ Reached {a:.2f}m"); break
        if not vehicle.armed:
            abort("Disarmed during takeoff!")
        time.sleep(0.5)


# ══════════════════════════════════════════════════════
#  OBSTACLE AVOIDANCE FLIGHT ENGINE
# ══════════════════════════════════════════════════════

class ObstacleAvoidanceFlight:
    def __init__(self, vehicle, cam: DepthCamera):
        self.vehicle      = vehicle
        self.camera       = cam
        self.heading_fwd  = 0.0
        self.heading_back = 0.0
        self.origin       = None

    def _forward_step_wp(self, pos, step_m):
        n, e = heading_to_ned(step_m, self.heading_fwd)
        return offset_location(pos, n, e)

    def _sidestep_wp(self, pos, direction, dist_m):
        h = (self.heading_fwd + (-90 if direction == "left" else 90)) % 360
        n, e = heading_to_ned(dist_m, h)
        return offset_location(pos, n, e)

    def _face(self, h):
        log(f"  Facing {h:.1f}° …")
        send_yaw_command(self.vehicle, h)
        wait_for_yaw(self.vehicle, h)

    def _check(self):
        readings = [z for _ in range(3)
                    if (z := self.camera.analyze_zones()) and time.sleep(0.05) is None]
        if not readings:
            return {"obstacle": False, "left_ok": True, "right_ok": True,
                    "center_red": 0.0, "left_clear": 1.0, "right_clear": 1.0}
        return {
            "obstacle":    any(r["obstacle"]   for r in readings),
            "left_ok":     all(r["left_ok"]    for r in readings),
            "right_ok":    all(r["right_ok"]   for r in readings),
            "center_red":  max(r["center_red"] for r in readings),
            "left_clear":  min(r["left_clear"] for r in readings),
            "right_clear": min(r["right_clear"] for r in readings),
        }

    def _verify_clear(self):
        log("  Verifying path clear after sidestep …")
        for _ in range(5):
            if self._check()["obstacle"]:
                log("  ⚠ Still blocked"); return False
            time.sleep(0.2)
        log("  ✓ Path clear"); return True

    def _wait_sidestep(self, wp, heading_lock, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            dist = get_horizontal_distance(
                self.vehicle.location.global_relative_frame, wp)
            STATE.update(dist_to_wp=round(dist, 2))
            check_altitude(self.vehicle, FLIGHT_ALTITUDE, label="SIDESTEP")
            send_yaw_command(self.vehicle, heading_lock)
            if dist <= POSITION_TOLERANCE:
                log("  ✓ Sidestep complete"); return
            time.sleep(0.3)
        log("  ⚠ Sidestep timeout")

    def fly_with_avoidance(self, target_wp, facing_heading, leg_label):
        STEP_M = 0.5
        log(f"\n{'─'*55}\n  LEG: {leg_label}  |  Facing: {facing_heading:.1f}°\n{'─'*55}")
        STATE.update(leg=leg_label)
        self._face(facing_heading)

        while True:
            current = self.vehicle.location.global_relative_frame
            dist    = get_horizontal_distance(current, target_wp)
            STATE.update(dist_to_wp=round(dist, 2))
            log(f"\n  Dist to [{leg_label}]: {dist:.2f}m")

            if dist <= POSITION_TOLERANCE:
                log(f"✓ Arrived at [{leg_label}]"); return True

            z = self._check()
            log(f"  Depth: fwd_red={z['center_red']:.0%}  "
                f"L_clr={z['left_clear']:.0%}  R_clr={z['right_clear']:.0%}")

            if not z["obstacle"]:
                log("  ✓ Path CLEAR → forward")
                step = min(STEP_M, dist)
                wp   = self._forward_step_wp(
                    LocationGlobalRelative(current.lat, current.lon, FLIGHT_ALTITUDE), step)
                goto_wp(self.vehicle, wp, FLIGHT_ALTITUDE,
                        label="FWD step", heading_lock=facing_heading)
            else:
                log(f"  🔴 OBSTACLE (red={z['center_red']:.0%})")
                dodged = False
                for attempt in range(OBSTACLE_MAX_HOLD):
                    z = self._check()
                    if not z["obstacle"]:
                        log("  ✓ Obstacle cleared"); dodged = True; break
                    if z["left_ok"]:
                        log(f"  ← DODGE LEFT {SIDESTEP_DISTANCE}m")
                        sw = self._sidestep_wp(
                            LocationGlobalRelative(current.lat, current.lon, FLIGHT_ALTITUDE),
                            "left", SIDESTEP_DISTANCE)
                        send_yaw_command(self.vehicle, facing_heading)
                        self.vehicle.simple_goto(sw, airspeed=SIDESTEP_SPEED)
                        self._wait_sidestep(sw, facing_heading)
                        current = self.vehicle.location.global_relative_frame
                        if self._verify_clear():
                            dodged = True; break
                    elif z["right_ok"]:
                        log(f"  → DODGE RIGHT {SIDESTEP_DISTANCE}m")
                        sw = self._sidestep_wp(
                            LocationGlobalRelative(current.lat, current.lon, FLIGHT_ALTITUDE),
                            "right", SIDESTEP_DISTANCE)
                        send_yaw_command(self.vehicle, facing_heading)
                        self.vehicle.simple_goto(sw, airspeed=SIDESTEP_SPEED)
                        self._wait_sidestep(sw, facing_heading)
                        current = self.vehicle.location.global_relative_frame
                        if self._verify_clear():
                            dodged = True; break
                    else:
                        log(f"  ⏸ Both sides blocked - hold ({attempt+1}/{OBSTACLE_MAX_HOLD})")
                        time.sleep(OBSTACLE_HOLD_SEC)
                if not dodged:
                    log("  ⚠ No clear path - best effort")

            send_yaw_command(self.vehicle, facing_heading)
            time.sleep(0.1)


# ══════════════════════════════════════════════════════
#  RTL MONITOR
# ══════════════════════════════════════════════════════

def wait_for_rtl_land(vehicle):
    log("RTL active - monitoring …")
    STATE.update(leg="RTL LANDING")
    while True:
        alt = vehicle.location.global_relative_frame.alt
        log(f"  Alt: {alt:.2f}m  Mode: {vehicle.mode.name}  Armed: {vehicle.armed}")
        if not vehicle.armed:
            log("✓ RTL complete - landed + disarmed"); return
        if alt <= 0.3:
            log("✓ Near ground …")
            time.sleep(5)
            if not vehicle.armed:
                log("✓ Auto-disarmed"); return
            vehicle.armed = False
            time.sleep(2)
            log("✓ Disarmed"); return
        time.sleep(0.5)


# ══════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════

def main():
    log("=" * 55)
    log("  Obstacle Avoidance + Live Dashboard")
    log("  RealSense D455 + Pixhawk 2.4.8 + Jetson Nano")
    log(f"  Target: {FORWARD_DISTANCE}m forward @ {FLIGHT_ALTITUDE}m")
    log("  Colormap: RED=NEAR  BLUE=FAR  BLACK=NO DATA")
    log("=" * 55)

    # ── Start RealSense ──────────────────────────────────
    camera.start()

    # ── Start dashboard in background thread ─────────────
    threading.Thread(target=start_dashboard, daemon=True).start()
    time.sleep(1.0)   # give Flask a moment to bind

    vehicle = None
    try:
        vehicle = connect_vehicle()

        # Start background telemetry push to STATE
        threading.Thread(target=telemetry_loop, args=(vehicle,), daemon=True).start()

        set_guided_mode(vehicle)
        arm_vehicle(vehicle)
        takeoff(vehicle, FLIGHT_ALTITUDE)

        heading_fwd  = get_yaw_degrees(vehicle)
        heading_back = (heading_fwd + 180) % 360
        log(f"  Forward heading: {heading_fwd:.1f}°  Return: {heading_back:.1f}°")

        raw    = vehicle.location.global_relative_frame
        origin = LocationGlobalRelative(raw.lat, raw.lon, FLIGHT_ALTITUDE)
        n, e   = heading_to_ned(FORWARD_DISTANCE, heading_fwd)
        target = offset_location(origin, n, e)
        log(f"  Origin: {origin.lat:.7f},{origin.lon:.7f}  Target: {target.lat:.7f},{target.lon:.7f}")

        av = ObstacleAvoidanceFlight(vehicle, camera)
        av.heading_fwd  = heading_fwd
        av.heading_back = heading_back
        av.origin       = origin

        # ── LEG 1 ────────────────────────────────────────
        log("\n" + "═" * 55)
        log("  LEG 1: Flying FORWARD to target")
        log("═" * 55)
        av.fly_with_avoidance(target, heading_fwd, "TARGET")

        log("Hovering at target 3s …")
        STATE.update(leg="HOVER @ TARGET")
        for _ in range(6):
            check_altitude(vehicle, FLIGHT_ALTITUDE, label="TARGET HOVER")
            time.sleep(0.5)

        # ── LEG 2 ────────────────────────────────────────
        log("\n" + "═" * 55)
        log("  LEG 2: Returning to LAUNCH")
        log("═" * 55)
        send_yaw_command(vehicle, heading_back)
        wait_for_yaw(vehicle, heading_back)
        av.fly_with_avoidance(origin, heading_back, "LAUNCH POINT")

        log("Hovering at launch 2s …")
        STATE.update(leg="HOVER @ LAUNCH")
        for _ in range(4):
            check_altitude(vehicle, FLIGHT_ALTITUDE, label="RTL HOVER")
            time.sleep(0.5)

        # ── RTL land ─────────────────────────────────────
        log("Switching to RTL …")
        vehicle.mode = VehicleMode("RTL")
        for _ in range(10):
            if vehicle.mode.name == "RTL":
                log("✓ RTL confirmed"); break
            time.sleep(0.5)
        wait_for_rtl_land(vehicle)

        log("\n" + "=" * 55)
        log("  Mission complete ✓")
        log("=" * 55)
        STATE.update(leg="COMPLETE", decision="WAITING",
                     decision_reason="Mission complete")

    except KeyboardInterrupt:
        log("Keyboard interrupt!")
        if vehicle and vehicle.armed:
            log("Emergency LAND …")
            vehicle.mode = VehicleMode("LAND")

    finally:
        camera.stop()
        if vehicle:
            log("Closing vehicle connection …")
            vehicle.close()

    # Keep Flask alive so dashboard stays accessible after mission
    log("Dashboard still running - press Ctrl+C to quit")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
