"""
Drone Mission Dashboard - Web Server
Streams the RealSense D455 annotated depth feed and live drone
telemetry / obstacle-avoidance decisions over a local network.

Access from any device on the same WiFi:
  http://<JETSON_IP>:5000

Get Jetson IP with:  hostname -I | awk '{print $1}'

Requirements:
  pip install flask dronekit pymavlink pyrealsense2 opencv-python numpy

Run SEPARATELY from the main flight script, OR integrate by importing
the shared DepthCamera and vehicle objects.
This script is self-contained for standalone dashboard use.
"""

import time
import math
import threading
import io
import socket
import queue
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
from flask import Flask, Response, render_template_string, jsonify

# ─────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────

DASHBOARD_PORT      = 5000
FRAME_W, FRAME_H    = 640, 480

# Zone boundaries (match obstacle_avoidance.py)
CENTER_X_START      = int(FRAME_W * 0.25)
CENTER_X_END        = int(FRAME_W * 0.75)
CENTER_Y_START      = int(FRAME_H * 0.30)
CENTER_Y_END        = int(FRAME_H * 0.70)
LEFT_X_END          = int(FRAME_W * 0.25)
RIGHT_X_START       = int(FRAME_W * 0.75)

RED_RATIO_THRESHOLD = 0.15
CLEAR_RATIO_THRESHOLD = 0.80

# ─────────────────────────────────────────────
# SHARED STATE (thread-safe)
# ─────────────────────────────────────────────

class DroneState:
    def __init__(self):
        self._lock = threading.Lock()
        self.data  = {
            "altitude"       : 0.0,
            "heading"        : 0.0,
            "armed"          : False,
            "mode"           : "-",
            "gps_fix"        : 0,
            "ekf_ok"         : False,
            "lat"            : 0.0,
            "lon"            : 0.0,
            "decision"       : "WAITING",
            "decision_reason": "Initializing …",
            "center_red"     : 0.0,
            "left_clear"     : 0.0,
            "right_clear"    : 0.0,
            "obstacle"       : False,
            "left_ok"        : False,
            "right_ok"       : False,
            "leg"            : "-",
            "dist_to_wp"     : 0.0,
            "log_lines"      : [],
            "connected"      : False,
            "timestamp"      : "-",
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
            if len(self.data["log_lines"]) > 80:
                self.data["log_lines"] = self.data["log_lines"][-80:]


state = DroneState()

# ─────────────────────────────────────────────
# REALSENSE DEPTH CAMERA
# ─────────────────────────────────────────────

class DashboardCamera:
    def __init__(self):
        self.pipeline   = rs.pipeline()
        self.config     = rs.config()
        self.colorizer  = rs.colorizer()
        self.colorizer.set_option(rs.option.color_scheme, 0)  # Jet: red=near, blue=far
        self._frame_lock = threading.Lock()
        self._latest_bgr = None
        self._running    = False

    def start(self):
        self.config.enable_stream(rs.stream.depth, FRAME_W, FRAME_H, rs.format.z16, 30)
        self.pipeline.start(self.config)
        self._running = True
        state.add_log("RealSense D455 starting up …")
        for _ in range(30):          # warm-up frames
            self.pipeline.wait_for_frames()
        state.add_log("✓ RealSense D455 ready")
        t = threading.Thread(target=self._capture_loop, daemon=True)
        t.start()

    def _capture_loop(self):
        while self._running:
            try:
                frames      = self.pipeline.wait_for_frames(timeout_ms=2000)
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue
                colored = self.colorizer.colorize(depth_frame)
                bgr     = np.asanyarray(colored.get_data())
                bgr_cv  = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
                annotated = self._annotate(bgr_cv)
                with self._frame_lock:
                    self._latest_bgr = annotated
            except Exception as e:
                time.sleep(0.1)

    @staticmethod
    def _red_ratio(crop):
        hsv   = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        m1    = cv2.inRange(hsv, (0,   80, 80), (10,  255, 255))
        m2    = cv2.inRange(hsv, (165, 80, 80), (180, 255, 255))
        total = crop.shape[0] * crop.shape[1]
        return np.count_nonzero(cv2.bitwise_or(m1, m2)) / max(total, 1)

    @staticmethod
    def _clear_ratio(crop):
        hsv        = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        blue_mask  = cv2.inRange(hsv, (95,  50, 30), (135, 255, 255))
        black_mask = cv2.inRange(hsv, (0,   0,  0),  (180, 255,  40))
        total      = crop.shape[0] * crop.shape[1]
        return np.count_nonzero(cv2.bitwise_or(blue_mask, black_mask)) / max(total, 1)

    def _annotate(self, frame: np.ndarray) -> np.ndarray:
        vis = frame.copy()

        # Crop zones
        center_crop = frame[CENTER_Y_START:CENTER_Y_END, CENTER_X_START:CENTER_X_END]
        left_crop   = frame[CENTER_Y_START:CENTER_Y_END, 0:LEFT_X_END]
        right_crop  = frame[CENTER_Y_START:CENTER_Y_END, RIGHT_X_START:FRAME_W]

        ctr_red  = self._red_ratio(center_crop)
        lft_clr  = self._clear_ratio(left_crop)
        rgt_clr  = self._clear_ratio(right_crop)

        obstacle = ctr_red  >= RED_RATIO_THRESHOLD
        left_ok  = lft_clr  >= CLEAR_RATIO_THRESHOLD
        right_ok = rgt_clr  >= CLEAR_RATIO_THRESHOLD

        # Update shared state
        decision, reason = self._decide(obstacle, left_ok, right_ok, ctr_red, lft_clr, rgt_clr)
        state.update(
            center_red=round(ctr_red, 3),
            left_clear=round(lft_clr, 3),
            right_clear=round(rgt_clr, 3),
            obstacle=obstacle,
            left_ok=left_ok,
            right_ok=right_ok,
            decision=decision,
            decision_reason=reason,
        )

        # ── Draw zone boxes ──────────────────────────────────
        # Center zone
        cv2.rectangle(vis,
                      (CENTER_X_START, CENTER_Y_START),
                      (CENTER_X_END,   CENTER_Y_END),
                      (0, 0, 220) if obstacle else (0, 220, 80), 2)
        # Left zone
        cv2.rectangle(vis,
                      (0, CENTER_Y_START),
                      (LEFT_X_END, CENTER_Y_END),
                      (0, 220, 220) if left_ok else (60, 60, 60), 2)
        # Right zone
        cv2.rectangle(vis,
                      (RIGHT_X_START, CENTER_Y_START),
                      (FRAME_W, CENTER_Y_END),
                      (0, 220, 220) if right_ok else (60, 60, 60), 2)

        # ── Zone labels ──────────────────────────────────────
        font   = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(vis, f"LEFT {lft_clr:.0%}",
                    (4, CENTER_Y_START - 6), font, 0.45,
                    (0, 220, 220) if left_ok else (80, 80, 80), 1)
        cv2.putText(vis, f"CENTER {ctr_red:.0%} RED",
                    (CENTER_X_START + 4, CENTER_Y_START - 6), font, 0.45,
                    (0, 0, 220) if obstacle else (0, 220, 80), 1)
        cv2.putText(vis, f"RIGHT {rgt_clr:.0%}",
                    (RIGHT_X_START + 4, CENTER_Y_START - 6), font, 0.45,
                    (0, 220, 220) if right_ok else (80, 80, 80), 1)

        # ── Decision banner at bottom ────────────────────────
        banner_y = FRAME_H - 36
        banner_colors = {
            "MOVE FORWARD" : (0, 180, 60),
            "DODGE LEFT"   : (200, 180, 0),
            "DODGE RIGHT"  : (200, 140, 0),
            "HOLD"         : (0, 60, 200),
            "WAITING"      : (80, 80, 80),
        }
        bcolor = banner_colors.get(decision, (80, 80, 80))
        cv2.rectangle(vis, (0, banner_y), (FRAME_W, FRAME_H), bcolor, -1)
        cv2.putText(vis, f"  {decision}  |  {reason}",
                    (6, FRAME_H - 12), font, 0.55, (255, 255, 255), 1)

        # ── Crosshair ────────────────────────────────────────
        cx, cy = FRAME_W // 2, FRAME_H // 2
        cv2.line(vis, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 1)
        cv2.line(vis, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 1)

        return vis

    @staticmethod
    def _decide(obstacle, left_ok, right_ok, ctr_red, lft_clr, rgt_clr):
        if not obstacle:
            return "MOVE FORWARD", f"Path clear (red={ctr_red:.0%})"
        if left_ok:
            return "DODGE LEFT",   f"Left clear ({lft_clr:.0%}) - sidestepping"
        if right_ok:
            return "DODGE RIGHT",  f"Right clear ({rgt_clr:.0%}) - sidestepping"
        return "HOLD", f"All blocked - L:{lft_clr:.0%} R:{rgt_clr:.0%} wait…"

    def get_jpeg(self) -> bytes:
        with self._frame_lock:
            frame = self._latest_bgr
        if frame is None:
            # Return a placeholder frame
            placeholder = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Waiting for RealSense …",
                        (80, FRAME_H // 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (120, 120, 120), 2)
            frame = placeholder
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpeg.tobytes()

    def stop(self):
        self._running = False
        time.sleep(0.3)
        try:
            self.pipeline.stop()
        except Exception:
            pass


# ─────────────────────────────────────────────
# DRONEKIT TELEMETRY THREAD (optional)
# ─────────────────────────────────────────────

def start_telemetry_thread(connection_string: str = "/dev/ttyTHS1",
                           baud: int = 57600):
    """
    Connects to Pixhawk in background and feeds telemetry into state.
    Runs as a daemon thread - dashboard works without it too.
    """
    def _run():
        try:
            from dronekit import connect as dk_connect
            state.add_log(f"Connecting to Pixhawk on {connection_string} …")
            vehicle = dk_connect(connection_string, baud=baud,
                                 wait_ready=True, timeout=30,
                                 heartbeat_timeout=15)
            state.update(connected=True)
            state.add_log(f"✓ Connected | FW: {vehicle.version}")

            while True:
                loc = vehicle.location.global_relative_frame
                state.update(
                    altitude  = round(loc.alt or 0.0, 2),
                    heading   = round((math.degrees(vehicle.attitude.yaw) + 360) % 360, 1),
                    armed     = vehicle.armed,
                    mode      = vehicle.mode.name,
                    gps_fix   = vehicle.gps_0.fix_type,
                    ekf_ok    = vehicle.ekf_ok,
                    lat       = round(loc.lat or 0.0, 7),
                    lon       = round(loc.lon or 0.0, 7),
                )
                time.sleep(0.2)
        except Exception as e:
            state.add_log(f"Telemetry error: {e}")
            state.update(connected=False)

    t = threading.Thread(target=_run, daemon=True)
    t.start()


# ─────────────────────────────────────────────
# FLASK APP
# ─────────────────────────────────────────────

app = Flask(__name__)
camera = DashboardCamera()

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>Drone Avoidance Dashboard</title>
<link rel="preconnect" href="https://fonts.googleapis.com"/>
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Barlow+Condensed:wght@400;600;800&display=swap" rel="stylesheet"/>
<style>
  :root {
    --bg:       #090d12;
    --panel:    #0d1420;
    --border:   #1a2840;
    --accent:   #00d4ff;
    --green:    #00ff88;
    --red:      #ff3a3a;
    --yellow:   #ffc230;
    --dim:      #3a5068;
    --text:     #c8dff0;
    --mono:     'Share Tech Mono', monospace;
    --display:  'Barlow Condensed', sans-serif;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--mono);
    min-height: 100vh;
    display: flex;
    flex-direction: column;
  }

  /* ── Header ── */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 24px;
    border-bottom: 1px solid var(--border);
    background: linear-gradient(90deg, #0a1628 0%, #0d1c30 100%);
  }
  header h1 {
    font-family: var(--display);
    font-weight: 800;
    font-size: 1.4rem;
    letter-spacing: 0.15em;
    color: var(--accent);
    text-transform: uppercase;
  }
  header h1 span { color: var(--text); font-weight: 400; }
  #conn-badge {
    font-size: 0.7rem;
    padding: 3px 10px;
    border-radius: 2px;
    font-family: var(--display);
    font-weight: 600;
    letter-spacing: 0.1em;
    text-transform: uppercase;
  }
  #conn-badge.ok  { background: var(--green);  color: #000; }
  #conn-badge.err { background: var(--red);    color: #fff; }

  /* ── Layout ── */
  .layout {
    display: grid;
    grid-template-columns: 1fr 320px;
    grid-template-rows: auto 1fr;
    gap: 12px;
    padding: 12px;
    flex: 1;
  }

  /* ── Feed panel ── */
  .feed-panel {
    grid-row: 1 / 3;
    display: flex;
    flex-direction: column;
    gap: 10px;
  }
  .feed-wrap {
    position: relative;
    background: #000;
    border: 1px solid var(--border);
    border-radius: 4px;
    overflow: hidden;
    aspect-ratio: 4/3;
  }
  .feed-wrap img {
    width: 100%;
    height: 100%;
    object-fit: cover;
    display: block;
  }
  .feed-overlay {
    position: absolute;
    top: 8px; left: 8px;
    display: flex;
    gap: 6px;
    pointer-events: none;
  }
  .badge {
    font-family: var(--display);
    font-weight: 600;
    font-size: 0.7rem;
    letter-spacing: 0.1em;
    padding: 2px 8px;
    border-radius: 2px;
    text-transform: uppercase;
  }
  .badge.live { background: var(--red); color: #fff; animation: blink 1s step-end infinite; }
  .badge.cam  { background: rgba(0,0,0,0.6); color: var(--accent); border: 1px solid var(--accent); }
  @keyframes blink { 50% { opacity: 0; } }

  /* ── Decision bar ── */
  .decision-bar {
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 12px 16px;
    display: flex;
    align-items: center;
    gap: 16px;
    background: var(--panel);
    transition: border-color 0.3s;
  }
  .decision-bar.clear  { border-color: var(--green); }
  .decision-bar.dodge  { border-color: var(--yellow); }
  .decision-bar.hold   { border-color: var(--red); }

  .decision-label {
    font-family: var(--display);
    font-weight: 800;
    font-size: 1.6rem;
    letter-spacing: 0.08em;
    min-width: 180px;
    text-transform: uppercase;
  }
  .decision-label.clear  { color: var(--green); }
  .decision-label.dodge  { color: var(--yellow); }
  .decision-label.hold   { color: var(--red); }
  .decision-label.wait   { color: var(--dim); }

  .decision-reason {
    font-size: 0.8rem;
    color: var(--text);
    opacity: 0.8;
  }

  /* ── Right column ── */
  .right-col {
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  /* ── Section panel ── */
  .panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 12px 14px;
  }
  .panel-title {
    font-family: var(--display);
    font-weight: 600;
    font-size: 0.65rem;
    letter-spacing: 0.2em;
    text-transform: uppercase;
    color: var(--dim);
    margin-bottom: 10px;
    border-bottom: 1px solid var(--border);
    padding-bottom: 6px;
  }

  /* ── Gauge rows ── */
  .gauge-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 8px;
  }
  .gauge-label {
    font-size: 0.65rem;
    color: var(--dim);
    text-transform: uppercase;
    letter-spacing: 0.1em;
    min-width: 70px;
  }
  .gauge-val {
    font-size: 1.1rem;
    font-weight: bold;
    text-align: right;
  }
  .gauge-bar-wrap {
    flex: 1;
    margin: 0 10px;
    height: 4px;
    background: var(--border);
    border-radius: 2px;
    overflow: hidden;
  }
  .gauge-bar {
    height: 100%;
    border-radius: 2px;
    transition: width 0.3s, background 0.3s;
  }

  /* ── Zone analysis ── */
  .zones {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 6px;
    margin-top: 4px;
  }
  .zone-card {
    border: 1px solid var(--border);
    border-radius: 3px;
    padding: 8px 6px;
    text-align: center;
    transition: border-color 0.3s, background 0.3s;
  }
  .zone-card .zone-name {
    font-family: var(--display);
    font-size: 0.65rem;
    font-weight: 600;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: var(--dim);
    margin-bottom: 4px;
  }
  .zone-card .zone-pct {
    font-size: 1.3rem;
    font-weight: bold;
    line-height: 1;
  }
  .zone-card .zone-status {
    font-size: 0.6rem;
    margin-top: 3px;
    text-transform: uppercase;
    letter-spacing: 0.08em;
  }

  .zone-card.obstacle { border-color: var(--red);    background: rgba(255,58,58,0.08); }
  .zone-card.clear    { border-color: var(--green);  background: rgba(0,255,136,0.06); }
  .zone-card.blocked  { border-color: var(--dim);    background: rgba(0,0,0,0.2); }

  /* ── Telemetry grid ── */
  .telem-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }
  .telem-item { }
  .telem-item .t-label {
    font-size: 0.6rem;
    color: var(--dim);
    text-transform: uppercase;
    letter-spacing: 0.12em;
    margin-bottom: 2px;
  }
  .telem-item .t-val {
    font-size: 1.05rem;
    font-weight: bold;
    color: var(--accent);
  }
  .telem-item .t-val.armed   { color: var(--green); }
  .telem-item .t-val.disarmed{ color: var(--red); }
  .telem-item .t-val.guided  { color: var(--accent); }

  /* ── Log panel ── */
  .log-panel {
    flex: 1;
    min-height: 0;
  }
  .log-box {
    height: 180px;
    overflow-y: auto;
    font-size: 0.68rem;
    line-height: 1.7;
    color: #6a8fa8;
    padding: 4px 0;
  }
  .log-box::-webkit-scrollbar { width: 4px; }
  .log-box::-webkit-scrollbar-thumb { background: var(--border); }
  .log-entry { padding: 1px 0; border-bottom: 1px solid rgba(26,40,64,0.4); }
  .log-entry .ts { color: var(--dim); }
  .log-entry .msg.ok  { color: var(--green); }
  .log-entry .msg.warn{ color: var(--yellow); }
  .log-entry .msg.err { color: var(--red); }

  /* ── Compass ── */
  .compass-wrap {
    display: flex;
    justify-content: center;
    margin: 6px 0;
  }
  canvas#compass { display: block; }

  /* ── GPS ── */
  .gps-coords {
    font-size: 0.75rem;
    color: var(--accent);
    text-align: center;
    margin-top: 4px;
    letter-spacing: 0.05em;
  }

  /* ── Timestamp ── */
  #ts-bar {
    text-align: right;
    font-size: 0.6rem;
    color: var(--dim);
    padding: 4px 14px 8px;
  }
</style>
</head>
<body>

<header>
  <h1>DRONE <span>AVOIDANCE</span> DASHBOARD</h1>
  <div style="display:flex;gap:10px;align-items:center;">
    <span style="font-size:0.7rem;color:var(--dim);">RealSense D455 + Pixhawk 2.4.8</span>
    <div id="conn-badge" class="err">DISCONNECTED</div>
  </div>
</header>

<div class="layout">

  <!-- Left: Feed + Decision -->
  <div class="feed-panel">
    <div class="feed-wrap">
      <img id="depth-feed" src="/video_feed" alt="Depth Feed"/>
      <div class="feed-overlay">
        <span class="badge live">● LIVE</span>
        <span class="badge cam">DEPTH · JET COLORMAP</span>
      </div>
    </div>

    <div class="decision-bar" id="dec-bar">
      <div class="decision-label" id="dec-label">WAITING</div>
      <div>
        <div style="font-size:0.6rem;color:var(--dim);text-transform:uppercase;letter-spacing:0.1em;margin-bottom:3px;">Current Action</div>
        <div class="decision-reason" id="dec-reason">Initializing system …</div>
      </div>
    </div>

    <!-- Zone analysis cards -->
    <div class="panel">
      <div class="panel-title">Depth Zone Analysis</div>
      <div class="zones">
        <div class="zone-card" id="zone-left">
          <div class="zone-name">◀ Left</div>
          <div class="zone-pct" id="z-left-pct">-</div>
          <div class="zone-status" id="z-left-st">checking</div>
        </div>
        <div class="zone-card" id="zone-center">
          <div class="zone-name">▲ Forward</div>
          <div class="zone-pct" id="z-ctr-pct">-</div>
          <div class="zone-status" id="z-ctr-st">checking</div>
        </div>
        <div class="zone-card" id="zone-right">
          <div class="zone-name">▶ Right</div>
          <div class="zone-pct" id="z-right-pct">-</div>
          <div class="zone-status" id="z-right-st">checking</div>
        </div>
      </div>
    </div>
  </div>

  <!-- Right: Telemetry + Log -->
  <div class="right-col">

    <!-- Compass + Altitude -->
    <div class="panel">
      <div class="panel-title">Navigation</div>
      <div class="compass-wrap">
        <canvas id="compass" width="120" height="120"></canvas>
      </div>
      <div class="gps-coords" id="gps-txt">GPS: - , -</div>
    </div>

    <!-- Telemetry -->
    <div class="panel">
      <div class="panel-title">Telemetry</div>
      <div class="telem-grid">
        <div class="telem-item">
          <div class="t-label">Altitude</div>
          <div class="t-val" id="t-alt">-</div>
        </div>
        <div class="telem-item">
          <div class="t-label">Heading</div>
          <div class="t-val" id="t-hdg">-</div>
        </div>
        <div class="telem-item">
          <div class="t-label">Mode</div>
          <div class="t-val guided" id="t-mode">-</div>
        </div>
        <div class="telem-item">
          <div class="t-label">Armed</div>
          <div class="t-val" id="t-armed">-</div>
        </div>
        <div class="telem-item">
          <div class="t-label">GPS Fix</div>
          <div class="t-val" id="t-gps">-</div>
        </div>
        <div class="telem-item">
          <div class="t-label">EKF</div>
          <div class="t-val" id="t-ekf">-</div>
        </div>
      </div>

      <!-- Altitude gauge -->
      <div style="margin-top:10px;">
        <div class="gauge-row">
          <div class="gauge-label">ALT</div>
          <div class="gauge-bar-wrap"><div class="gauge-bar" id="alt-bar" style="width:0%;background:var(--accent)"></div></div>
          <div class="gauge-val" id="alt-val" style="color:var(--accent)">0.0m</div>
        </div>
      </div>
    </div>

    <!-- Mission leg -->
    <div class="panel">
      <div class="panel-title">Mission Status</div>
      <div class="telem-grid">
        <div class="telem-item">
          <div class="t-label">Current Leg</div>
          <div class="t-val" id="t-leg" style="color:var(--yellow)">-</div>
        </div>
        <div class="telem-item">
          <div class="t-label">Dist to WP</div>
          <div class="t-val" id="t-dist">-</div>
        </div>
      </div>
    </div>

    <!-- Log -->
    <div class="panel log-panel">
      <div class="panel-title">Event Log</div>
      <div class="log-box" id="log-box"></div>
    </div>
  </div>

</div>
<div id="ts-bar">Last update: <span id="ts">-</span></div>

<script>
// ── Compass canvas ────────────────────────────────────────
const compassCanvas = document.getElementById('compass');
const ctx = compassCanvas.getContext('2d');

function drawCompass(headingDeg) {
  const w = compassCanvas.width, h = compassCanvas.height;
  const cx = w / 2, cy = h / 2, r = 48;
  ctx.clearRect(0, 0, w, h);

  // Background circle
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.fillStyle = '#0a1220'; ctx.fill();
  ctx.strokeStyle = '#1a2840'; ctx.lineWidth = 1.5; ctx.stroke();

  // Tick marks
  for (let i = 0; i < 36; i++) {
    const a = (i * 10 - 90) * Math.PI / 180;
    const inner = i % 9 === 0 ? r - 12 : r - 7;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(a) * inner, cy + Math.sin(a) * inner);
    ctx.lineTo(cx + Math.cos(a) * (r - 2), cy + Math.sin(a) * (r - 2));
    ctx.strokeStyle = i % 9 === 0 ? '#3a5068' : '#1e2e42';
    ctx.lineWidth = i % 9 === 0 ? 1.5 : 1;
    ctx.stroke();
  }

  // Cardinal labels
  const cardinals = [['N','#00d4ff',0],['E','#6a8fa8',90],['S','#6a8fa8',180],['W','#6a8fa8',270]];
  cardinals.forEach(([label, color, deg]) => {
    const a = (deg - 90) * Math.PI / 180;
    const lx = cx + Math.cos(a) * (r - 18);
    const ly = cy + Math.sin(a) * (r - 18);
    ctx.fillStyle = color;
    ctx.font = 'bold 9px "Barlow Condensed", sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(label, lx, ly);
  });

  // Heading needle
  const needleAngle = (headingDeg - 90) * Math.PI / 180;
  const nx = cx + Math.cos(needleAngle) * (r - 8);
  const ny = cy + Math.sin(needleAngle) * (r - 8);
  ctx.beginPath();
  ctx.moveTo(cx, cy); ctx.lineTo(nx, ny);
  ctx.strokeStyle = '#00d4ff'; ctx.lineWidth = 2;
  ctx.lineCap = 'round'; ctx.stroke();

  // Center dot
  ctx.beginPath(); ctx.arc(cx, cy, 3, 0, Math.PI * 2);
  ctx.fillStyle = '#00d4ff'; ctx.fill();

  // Heading value
  ctx.fillStyle = '#c8dff0';
  ctx.font = 'bold 13px "Share Tech Mono", monospace';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(Math.round(headingDeg) + '°', cx, cy + r + 12);
}

drawCompass(0);

// ── Polling ───────────────────────────────────────────────
let lastLogLen = 0;

async function poll() {
  try {
    const r = await fetch('/state');
    const s = await r.json();

    // Connection badge
    const badge = document.getElementById('conn-badge');
    badge.textContent = s.connected ? 'CONNECTED' : 'NO PIXHAWK';
    badge.className   = s.connected ? 'ok' : 'err';

    // Decision bar
    const dec   = s.decision || 'WAITING';
    const label = document.getElementById('dec-label');
    const bar   = document.getElementById('dec-bar');
    label.textContent = dec;
    const cls = dec.includes('FORWARD') ? 'clear' : dec.includes('DODGE') ? 'dodge' :
                dec === 'HOLD' ? 'hold' : 'wait';
    label.className = 'decision-label ' + cls;
    bar.className   = 'decision-bar '   + cls;
    document.getElementById('dec-reason').textContent = s.decision_reason || '-';

    // Zones
    const ctrRed  = Math.round(s.center_red  * 100);
    const lftClr  = Math.round(s.left_clear  * 100);
    const rgtClr  = Math.round(s.right_clear * 100);

    document.getElementById('z-ctr-pct').textContent  = ctrRed + '%';
    document.getElementById('z-ctr-st').textContent   = s.obstacle ? '🔴 OBSTACLE' : '✅ CLEAR';
    document.getElementById('zone-center').className  = 'zone-card ' + (s.obstacle ? 'obstacle' : 'clear');

    document.getElementById('z-left-pct').textContent = lftClr + '%';
    document.getElementById('z-left-st').textContent  = s.left_ok  ? '✅ CLEAR' : '⬛ BLOCKED';
    document.getElementById('zone-left').className    = 'zone-card ' + (s.left_ok  ? 'clear' : 'blocked');

    document.getElementById('z-right-pct').textContent = rgtClr + '%';
    document.getElementById('z-right-st').textContent  = s.right_ok ? '✅ CLEAR' : '⬛ BLOCKED';
    document.getElementById('zone-right').className    = 'zone-card ' + (s.right_ok ? 'clear' : 'blocked');

    // Telemetry
    document.getElementById('t-alt').textContent  = s.altitude  + ' m';
    document.getElementById('t-hdg').textContent  = s.heading   + '°';
    document.getElementById('t-mode').textContent = s.mode;
    const armedEl = document.getElementById('t-armed');
    armedEl.textContent  = s.armed ? 'ARMED' : 'DISARMED';
    armedEl.className    = 't-val ' + (s.armed ? 'armed' : 'disarmed');
    document.getElementById('t-gps').textContent  = ['None','None','2D','3D','3D+DGPS'][Math.min(s.gps_fix,4)] || s.gps_fix;
    document.getElementById('t-ekf').textContent  = s.ekf_ok ? '✅ OK' : '⚠ WARN';

    // Altitude bar (0-10m scale)
    const altPct = Math.min((s.altitude / 10) * 100, 100);
    document.getElementById('alt-bar').style.width = altPct + '%';
    document.getElementById('alt-val').textContent = s.altitude + 'm';

    // Mission
    document.getElementById('t-leg').textContent  = s.leg;
    document.getElementById('t-dist').textContent = s.dist_to_wp + 'm';

    // GPS
    document.getElementById('gps-txt').textContent = `GPS: ${s.lat}, ${s.lon}`;

    // Compass
    drawCompass(s.heading || 0);

    // Timestamp
    document.getElementById('ts').textContent = s.timestamp;

    // Log
    const logBox = document.getElementById('log-box');
    if (s.log_lines && s.log_lines.length !== lastLogLen) {
      lastLogLen = s.log_lines.length;
      logBox.innerHTML = s.log_lines.slice(-60).map(line => {
        const m = line.match(/^\[(.+?)\] (.+)$/);
        if (!m) return `<div class="log-entry">${line}</div>`;
        const ts  = m[1], msg = m[2];
        const cls = msg.startsWith('✓') ? 'ok' : msg.startsWith('⚠') ? 'warn' :
                    msg.startsWith('ABORT') ? 'err' : '';
        return `<div class="log-entry"><span class="ts">[${ts}]</span> <span class="msg ${cls}">${msg}</span></div>`;
      }).join('');
      logBox.scrollTop = logBox.scrollHeight;
    }

  } catch(e) { /* server busy */ }
  setTimeout(poll, 250);
}

poll();
</script>
</body>
</html>"""


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/video_feed")
def video_feed():
    def generate():
        while True:
            jpeg = camera.get_jpeg()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
            time.sleep(1 / 25)   # 25 fps
    return Response(generate(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/state")
def get_state():
    return jsonify(state.get())


@app.route("/log/<path:msg>", methods=["POST"])
def post_log(msg):
    """Allow the flight script to push log messages via HTTP POST."""
    state.add_log(msg)
    return "ok"


@app.route("/update", methods=["POST"])
def post_update():
    """Allow the flight script to push state updates via HTTP POST."""
    from flask import request
    data = request.get_json(force=True, silent=True) or {}
    state.update(**data)
    return "ok"


# ─────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────

def get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


if __name__ == "__main__":
    local_ip = get_local_ip()

    print("=" * 55)
    print("  Drone Avoidance Dashboard")
    print("=" * 55)
    print(f"  Starting RealSense D455 …")

    try:
        camera.start()
    except Exception as e:
        print(f"  ⚠ RealSense not found: {e}")
        print("  Dashboard will run with placeholder feed")

    # Optionally start telemetry (comment out if flight script handles vehicle)
    # start_telemetry_thread("/dev/ttyTHS1", 57600)

    print()
    print(f"  ✓ Dashboard running")
    print()
    print(f"  ┌─────────────────────────────────────┐")
    print(f"  │  Local:    http://localhost:{DASHBOARD_PORT}       │")
    print(f"  │  Network:  http://{local_ip}:{DASHBOARD_PORT}  │")
    print(f"  └─────────────────────────────────────┘")
    print()
    print(f"  Open the NETWORK address on any device on the same WiFi")
    print(f"  Press Ctrl+C to stop")
    print("=" * 55)

    app.run(host="0.0.0.0", port=DASHBOARD_PORT, threaded=True, debug=False)
