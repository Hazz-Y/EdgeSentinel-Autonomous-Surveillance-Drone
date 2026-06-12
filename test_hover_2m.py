"""
Autonomous Drone Flight Script
Hardware: Pixhawk 2.4.8 + Jetson Nano
Protocol: MAVLink via DroneKit

Flight Plan:
  1. Connect to Pixhawk via serial
  2. Switch to GUIDED mode (with retry)
  3. ARM motors (with retry)
  4. Takeoff to 2 meters → hover 3 sec → land
  5. Verify 2m altitude (±0.1m tolerance)
  6. Disarm after landing

Requirements:
  pip install dronekit pymavlink
"""

import time
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ─────────────────────────────────────────────
#  CONFIGURATION  (edit these as needed)
# ─────────────────────────────────────────────
CONNECTION_STRING  = "/dev/ttyACM0"   # Jetson UART → Pixhawk TELEM2
                                       # Alternatives: /dev/ttyUSB0 (USB), /dev/ttyACM0
BAUD_RATE          = 57600            # Match Pixhawk SERIAL2_BAUD (57 = 57600)
TARGET_ALTITUDE    = 2.0              # metres
ALTITUDE_TOLERANCE = 0.1             # ±0.1 m
HOVER_DURATION     = 3               # seconds to hover at altitude
MODE_RETRY_LIMIT   = 20              # max attempts to set GUIDED
ARM_RETRY_LIMIT    = 20              # max attempts to arm
MODE_RETRY_DELAY   = 0.5             # seconds between mode retries
ARM_RETRY_DELAY    = 0.5             # seconds between arm retries
TAKEOFF_TIMEOUT    = 30              # seconds to reach target altitude
LAND_TIMEOUT       = 30              # seconds to wait for landing

# ─────────────────────────────────────────────
#  LOGGING HELPER
# ─────────────────────────────────────────────
def log(tag: str, msg: str):
    ts = time.strftime("%H:%M:%S")
    symbols = {"OK": "✅", "WARN": "⚠️ ", "ERR": "❌", "INFO": "ℹ️ ", "WAIT": "⏳"}
    sym = symbols.get(tag, "  ")
    print(f"[{ts}] {sym}  {msg}")


# ─────────────────────────────────────────────
#  STEP 1 - Connect to Pixhawk
# ─────────────────────────────────────────────
def connect_to_vehicle() -> "Vehicle":
    log("INFO", f"Connecting to Pixhawk on {CONNECTION_STRING} @ {BAUD_RATE} baud …")
    try:
        vehicle = connect(
            CONNECTION_STRING,
            baud=BAUD_RATE,
            wait_ready=True,        # blocks until heartbeat + params received
            timeout=60,
            heartbeat_timeout=30,
        )
        log("OK",   f"Connected!  Firmware: {vehicle.version}")
        log("INFO", f"GPS fix type : {vehicle.gps_0.fix_type}  "
                    f"(need ≥3 for arming outdoors)")
        log("INFO", f"Battery      : {vehicle.battery.voltage:.2f} V  "
                    f"level={vehicle.battery.level}%")
        log("INFO", f"Current mode : {vehicle.mode.name}")
        log("INFO", f"System status: {vehicle.system_status.state}")
        return vehicle
    except Exception as exc:
        log("ERR", f"Connection failed: {exc}")
        log("ERR", "Check: cable, baud rate, serial port, Pixhawk power.")
        sys.exit(1)


# ─────────────────────────────────────────────
#  STEP 2 - Set GUIDED mode (with retry/spam)
# ─────────────────────────────────────────────
def set_guided_mode(vehicle) -> None:
    log("INFO", "Attempting to switch to GUIDED mode …")

    for attempt in range(1, MODE_RETRY_LIMIT + 1):
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(MODE_RETRY_DELAY)

        if vehicle.mode.name == "GUIDED":
            log("OK", f"Mode confirmed GUIDED (attempt {attempt})")
            return

        log("WAIT", f"Mode is '{vehicle.mode.name}', retrying … ({attempt}/{MODE_RETRY_LIMIT})")

    log("ERR", f"Failed to enter GUIDED mode after {MODE_RETRY_LIMIT} attempts.")
    log("ERR", "Possible causes: pre-arm checks failing, RC not in correct position, "
               "or Pixhawk firmware issue.")
    vehicle.close()
    sys.exit(1)


# ─────────────────────────────────────────────
#  STEP 3 - ARM the drone (with retry/spam)
# ─────────────────────────────────────────────
def arm_vehicle(vehicle) -> None:
    log("INFO", "Waiting for pre-arm checks to clear …")

    # Wait for 'armable' flag (GPS fix, calibration, etc.)
    wait_start = time.time()
    while not vehicle.is_armable:
        elapsed = time.time() - wait_start
        log("WAIT", f"Not armable yet … ({elapsed:.0f}s)  "
                    f"GPS={vehicle.gps_0.fix_type}  "
                    f"EKF={vehicle.ekf_ok}  "
                    f"Status={vehicle.system_status.state}")
        if elapsed > 60:
            log("ERR", "Drone never became armable within 60 s. Aborting.")
            vehicle.close()
            sys.exit(1)
        time.sleep(1)

    log("OK", "Pre-arm checks passed. Arming …")

    for attempt in range(1, ARM_RETRY_LIMIT + 1):
        vehicle.armed = True
        time.sleep(ARM_RETRY_DELAY)

        if vehicle.armed:
            log("OK", f"Motors ARMED ✅ (attempt {attempt})")
            return

        log("WAIT", f"Not armed yet, retrying … ({attempt}/{ARM_RETRY_LIMIT})")

    log("ERR", f"Failed to arm after {ARM_RETRY_LIMIT} attempts.")
    log("ERR", "Common causes: safety switch not pressed, "
               "RC throttle not at minimum, or arming denied by param.")
    vehicle.close()
    sys.exit(1)


# ─────────────────────────────────────────────
#  STEP 4 - Takeoff, hover, verify altitude, land
# ─────────────────────────────────────────────
def takeoff_hover_land(vehicle) -> None:

    # ── 4a. Wait 2 seconds after arm (motors spin up) ──────────────
    log("INFO", "Waiting 2 s post-arm for motors to stabilise …")
    time.sleep(2)

    # ── 4b. Takeoff command ─────────────────────────────────────────
    log("INFO", f"Sending TAKEOFF command to {TARGET_ALTITUDE} m …")
    vehicle.simple_takeoff(TARGET_ALTITUDE)

    # ── 4c. Monitor climb & verify target altitude ──────────────────
    log("WAIT", f"Climbing to {TARGET_ALTITUDE} m "
                f"(tolerance ±{ALTITUDE_TOLERANCE} m) …")

    reached    = False
    t_start    = time.time()

    while time.time() - t_start < TAKEOFF_TIMEOUT:
        alt = vehicle.location.global_relative_frame.alt
        log("INFO", f"Current altitude: {alt:.2f} m")

        lo = TARGET_ALTITUDE - ALTITUDE_TOLERANCE
        hi = TARGET_ALTITUDE + ALTITUDE_TOLERANCE

        if lo <= alt <= hi:
            log("OK", f"Target altitude reached! Alt={alt:.2f} m  "
                       f"(within ±{ALTITUDE_TOLERANCE} m of {TARGET_ALTITUDE} m)")
            reached = True
            break

        if alt >= TARGET_ALTITUDE * 0.98:
            # Within 2 % of target → almost there, keep polling
            pass

        time.sleep(0.5)

    if not reached:
        actual_alt = vehicle.location.global_relative_frame.alt
        log("ERR", f"Did NOT reach {TARGET_ALTITUDE} m within {TAKEOFF_TIMEOUT} s. "
                   f"Current alt = {actual_alt:.2f} m. Landing for safety.")
        _emergency_land(vehicle)
        vehicle.close()
        sys.exit(1)

    # ── 4d. Hover for 3 seconds ─────────────────────────────────────
    log("INFO", f"Hovering for {HOVER_DURATION} s …")
    for remaining in range(HOVER_DURATION, 0, -1):
        alt = vehicle.location.global_relative_frame.alt
        log("INFO", f"  Hovering … {remaining}s left  |  alt={alt:.2f} m")
        time.sleep(1)

    # ── 4e. Initiate LAND mode (NOT RTL) ───────────────────────────
    log("INFO", "Switching to LAND mode …")
    vehicle.mode = VehicleMode("LAND")

    # Verify LAND mode accepted
    deadline = time.time() + 5
    while time.time() < deadline:
        if vehicle.mode.name == "LAND":
            log("OK", "LAND mode confirmed.")
            break
        time.sleep(0.3)
    else:
        log("WARN", f"Mode is still '{vehicle.mode.name}'. "
                    "Proceeding anyway - drone should descend.")

    # ── 4f. Wait until landed (altitude < 0.15 m) ──────────────────
    log("WAIT", "Waiting for landing …")
    t_land = time.time()
    while time.time() - t_land < LAND_TIMEOUT:
        alt = vehicle.location.global_relative_frame.alt
        log("INFO", f"  Descending … alt={alt:.2f} m")
        if alt <= 0.15:
            log("OK", "Drone has landed.")
            break
        time.sleep(0.5)
    else:
        log("WARN", "Land timeout reached. Assuming landed. Check drone manually!")

    time.sleep(2)   # brief pause to let flight controller settle


# ─────────────────────────────────────────────
#  STEP 5 - Disarm
# ─────────────────────────────────────────────
def disarm_vehicle(vehicle) -> None:
    log("INFO", "Disarming …")
    vehicle.armed = False
    time.sleep(1)

    if not vehicle.armed:
        log("OK", "Motors DISARMED ✅")
    else:
        log("WARN", "Disarm command sent but armed flag still True. "
                    "Flight controller may auto-disarm after timeout.")


# ─────────────────────────────────────────────
#  EMERGENCY LAND (called on any mid-flight abort)
# ─────────────────────────────────────────────
def _emergency_land(vehicle) -> None:
    log("WARN", "EMERGENCY LAND triggered!")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(10)


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────
def main():
    print("=" * 60)
    print("  Autonomous Drone Flight - Pixhawk 2.4.8 + Jetson Nano")
    print("=" * 60)

    # Step 1 - Connect
    vehicle = connect_to_vehicle()

    try:
        # Step 2 - GUIDED mode
        set_guided_mode(vehicle)

        # Step 3 - ARM
        arm_vehicle(vehicle)

        # Step 4 - Takeoff → hover → land
        takeoff_hover_land(vehicle)

        # Step 5 - Disarm
        disarm_vehicle(vehicle)

        log("OK", "Mission complete! 🎉")

    except KeyboardInterrupt:
        log("WARN", "User interrupted! Initiating emergency land …")
        _emergency_land(vehicle)
        disarm_vehicle(vehicle)

    except Exception as exc:
        log("ERR", f"Unexpected error: {exc}")
        log("WARN", "Initiating emergency land …")
        _emergency_land(vehicle)
        disarm_vehicle(vehicle)
        raise

    finally:
        log("INFO", "Closing vehicle connection.")
        vehicle.close()


if __name__ == "__main__":
    main()
