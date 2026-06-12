"""
GPS Status & Armable Check
Hardware : Pixhawk 2.4.8 + Jetson Nano
Connection: UART (TELEM2) or USB

Checks:
  - GPS fix type & satellite count
  - EKF status
  - Battery voltage
  - All pre-arm conditions
  - Final verdict: ARMABLE or NOT ARMABLE
"""

import time
import sys
from dronekit import connect, VehicleMode

# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────
CONNECTION_STRING = "/dev/ttyACM0"   # UART: Jetson GPIO (recommended)
# CONNECTION_STRING = "/dev/ttyACM0" # USB (if powered externally)
BAUD_RATE         = 57600
TIMEOUT           = 60               # seconds to wait for connection

# GPS fix type meanings
GPS_FIX_TYPES = {
    0: "No GPS",
    1: "No Fix",
    2: "2D Fix  ⚠️  (not good enough)",
    3: "3D Fix  ✅",
    4: "DGPS    ✅",
    5: "RTK Float ✅",
    6: "RTK Fixed ✅",
}

# ─────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────
def log(tag, msg):
    symbols = {"OK": "✅", "WARN": "⚠️ ", "ERR": "❌", "INFO": "ℹ️ "}
    sym = symbols.get(tag, "   ")
    print(f"  {sym}  {msg}")

def separator(title=""):
    if title:
        pad = (54 - len(title)) // 2
        print("\n" + "─" * pad + f"  {title}  " + "─" * pad)
    else:
        print("─" * 60)

# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────
def main():
    print("=" * 60)
    print("   GPS Status & Armable Check - Pixhawk 2.4.8")
    print("=" * 60)

    # ── Connect ──────────────────────────────────────────────────
    print(f"\n  Connecting to {CONNECTION_STRING} @ {BAUD_RATE} baud …")
    try:
        vehicle = connect(
            CONNECTION_STRING,
            baud=BAUD_RATE,
            wait_ready=True,
            timeout=TIMEOUT,
            heartbeat_timeout=30,
        )
    except Exception as e:
        log("ERR", f"Connection failed: {e}")
        sys.exit(1)

    print(f"  Connected! Firmware: {vehicle.version}")
    log("INFO", f"Mode   : {vehicle.mode.name}")
    log("INFO", f"Armed  : {vehicle.armed}")
    log("INFO", f"System : {vehicle.system_status.state}")

    # ── Polling Loop - repeat until armable ──────────────────────
    try:
        CHECK_INTERVAL = 3          # seconds between each re-check
        attempt        = 1

        while True:
            separator(f"ARMABLE CHECK  (attempt {attempt})")
            attempt += 1

            # Re-read live values every loop
            gps      = vehicle.gps_0
            fix_type = gps.fix_type
            fix_label= GPS_FIX_TYPES.get(fix_type, f"Unknown ({fix_type})")
            sats     = gps.satellites_visible if gps.satellites_visible is not None else 0
            ekf_ok   = vehicle.ekf_ok
            batt     = vehicle.battery
            home     = vehicle.home_location
            att      = vehicle.attitude
            level_ok = abs(att.roll) < 0.1 and abs(att.pitch) < 0.1

            # Print current snapshot
            log("INFO", f"GPS Fix    : {fix_type} - {fix_label}")
            log("INFO", f"Satellites : {sats}")
            log("OK"   if ekf_ok else "ERR",
                f"EKF        : {'Healthy' if ekf_ok else 'NOT healthy'}")
            if batt.voltage:
                log("INFO", f"Battery    : {batt.voltage:.2f} V  |  {batt.level}%")
            log("INFO", f"Mode       : {vehicle.mode.name}")
            log("OK"   if level_ok else "WARN",
                f"Level      : {'OK' if level_ok else 'Tilted - place on flat surface'}")
            log("OK"   if home else "WARN",
                f"Home       : {'Set' if home else 'Not set yet'}")

            is_armable = vehicle.is_armable

            if is_armable:
                # ── SUCCESS - exit the loop ───────────────────────
                print("""
  ╔══════════════════════════════════════════╗
  ║   ✅  VEHICLE IS ARMABLE                 ║
  ║   All pre-arm checks passed!             ║
  ║   Safe to proceed with flight.           ║
  ╚══════════════════════════════════════════╝
                """)
                break

            else:
                # ── NOT YET - print reasons and wait ─────────────
                print("""
  ╔══════════════════════════════════════════╗
  ║   ❌  NOT ARMABLE YET - retrying …       ║
  ╚══════════════════════════════════════════╝
                """)
                print("  Failing checks:")
                if fix_type < 3:
                    log("ERR",  "→ No GPS 3D fix. Move outdoors, wait for satellites.")
                if sats < 6:
                    log("WARN", f"→ Only {sats} satellites visible (need ≥ 6).")
                if not ekf_ok:
                    log("ERR",  "→ EKF not healthy. Keep drone still for 30s.")
                if batt.voltage and batt.voltage < 10.5:
                    log("ERR",  "→ Battery voltage too low. Charge battery.")
                if home is None:
                    log("WARN", "→ Home location not set yet.")
                if not level_ok:
                    log("WARN", "→ Drone is tilted. Place on flat surface.")

                print(f"\n  Rechecking in {CHECK_INTERVAL}s …  (Ctrl+C to quit)\n")
                time.sleep(CHECK_INTERVAL)

    except KeyboardInterrupt:
        print("\n\n  ⚠️   Interrupted by user. Closing connection …\n")

    finally:
        separator()
        vehicle.close()
        print("\n  Connection closed.\n")


if __name__ == "__main__":
    main()
