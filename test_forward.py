"""
Autonomous Drone Straight Flight Script
Hardware: Pixhawk 2.4.8 + Jetson Nano
Protocol: MAVLink via DroneKit

Flight Plan:
  1. Connect to Pixhawk via serial
  2. Switch to GUIDED mode (with retry)
  3. ARM (with retry)
  4. Wait 2s → Takeoff to 5 meters
  5. Read current YAW heading (direction drone is facing)
  6. Fly 5 meters FORWARD in the drone's facing direction at 5m altitude
  7. Hover at 5m for 2 seconds
  8. Fly 5 meters BACK to origin at 5m altitude
  9. Hover at origin for 2 seconds
  10. LAND in place (NOT RTL)
  11. Disarm

Height is monitored and corrected throughout the full flight path.

Requirements:
  pip install dronekit pymavlink

Serial port on Jetson Nano:
  UART (TELEM2 of Pixhawk) → /dev/ttyTHS1  @ 57600 baud
  USB-Serial adapter        → /dev/ttyUSB0  @ 57600 baud
"""

import time
import sys
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ─────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────

CONNECTION_STRING    = "/dev/ttyACM0"    # change to /dev/ttyUSB0 if using USB adapter
BAUD_RATE            = 57600

FLIGHT_ALTITUDE      = 5.0    # metres - takeoff and cruise height
FLIGHT_DISTANCE      = 5.0    # metres - how far to fly forward (and back)
CRUISE_SPEED         = 1.0    # m/s    - travel speed between waypoints
HOVER_DURATION       = 2      # seconds to pause at forward point and at origin

ALTITUDE_TOLERANCE   = 0.15   # metres - ±window for altitude check
POSITION_TOLERANCE   = 0.4    # metres - horizontal distance to consider waypoint "reached"
WAYPOINT_TIMEOUT     = 40.0   # seconds - max wait per waypoint before moving on

MODE_RETRY_DELAY     = 0.5
MODE_MAX_RETRIES     = 20

ARM_RETRY_DELAY      = 1.0
ARM_MAX_RETRIES      = 20

ARM_DELAY            = 2      # seconds to wait post-arm before takeoff

# ─────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────

def log(msg: str):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def abort(msg: str):
    log(f"ABORT ─ {msg}")
    sys.exit(1)


def get_horizontal_distance(loc1, loc2) -> float:
    """
    Flat-earth distance between two LocationGlobalRelative points (horizontal only).
    Accurate for short distances < 100 m.
    """
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)


def offset_location_by_heading(origin, distance: float, heading_deg: float) -> LocationGlobalRelative:
    """
    Return a new GPS point that is `distance` metres away from `origin`
    in the direction of `heading_deg` (0=North, 90=East, 180=South, 270=West).
    Uses flat-earth approximation - accurate to < 1 mm at 5 m range.
    """
    heading_rad = math.radians(heading_deg)
    d_north = distance * math.cos(heading_rad)
    d_east  = distance * math.sin(heading_rad)

    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east  / (earth_radius * math.cos(math.radians(origin.lat)))

    return LocationGlobalRelative(
        origin.lat + math.degrees(d_lat),
        origin.lon + math.degrees(d_lon),
        origin.alt,                          # altitude stays constant
    )


def get_yaw_degrees(vehicle) -> float:
    """
    Read current vehicle heading in degrees (0-360, 0=North, clockwise).
    Uses attitude.yaw (radians, -π to +π) and converts to compass bearing.
    """
    yaw_rad = vehicle.attitude.yaw
    yaw_deg = math.degrees(yaw_rad)
    # Normalize to 0-360
    return (yaw_deg + 360) % 360


def check_altitude(vehicle, target_alt: float, label: str = ""):
    """
    Log current altitude and warn if outside tolerance.
    Does NOT send a correction - DroneKit's simple_goto maintains altitude
    automatically in GUIDED mode.
    """
    alt = vehicle.location.global_relative_frame.alt
    diff = abs(alt - target_alt)
    status = "✓" if diff <= ALTITUDE_TOLERANCE else "⚠ ALT DRIFT"
    tag = f"  [{label}]" if label else ""
    log(f"{tag} Alt: {alt:.2f}m  target: {target_alt:.2f}m  diff: {diff:.2f}m  {status}")
    return alt


# ─────────────────────────────────────────────
# STEP 1 - Connect
# ─────────────────────────────────────────────

def connect_vehicle():
    log(f"Connecting to Pixhawk on {CONNECTION_STRING} @ {BAUD_RATE} baud …")
    try:
        vehicle = connect(
            CONNECTION_STRING,
            baud=BAUD_RATE,
            wait_ready=True,
            timeout=60,
            heartbeat_timeout=30,
        )
    except Exception as e:
        abort(f"Connection failed: {e}")

    log("✓ Connected to vehicle")
    log(f"  Firmware   : {vehicle.version}")
    log(f"  GPS fix    : {vehicle.gps_0.fix_type}  (need ≥3 for 3D fix)")
    log(f"  Armed      : {vehicle.armed}")
    log(f"  Mode       : {vehicle.mode.name}")
    log(f"  EKF OK     : {vehicle.ekf_ok}")
    log(f"  Heading    : {get_yaw_degrees(vehicle):.1f}°")
    return vehicle


# ─────────────────────────────────────────────
# STEP 2 - GUIDED mode with retry
# ─────────────────────────────────────────────

def set_guided_mode(vehicle):
    log("Setting mode → GUIDED …")
    if vehicle.mode.name == "GUIDED":
        log("✓ Already in GUIDED mode")
        return

    for attempt in range(1, MODE_MAX_RETRIES + 1):
        log(f"  Attempt {attempt}/{MODE_MAX_RETRIES}")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(MODE_RETRY_DELAY)
        if vehicle.mode.name == "GUIDED":
            log("✓ Mode confirmed: GUIDED")
            return

    abort(f"Could not enter GUIDED mode after {MODE_MAX_RETRIES} attempts. "
          f"Current: {vehicle.mode.name}")


# ─────────────────────────────────────────────
# STEP 3 - ARM with retry
# ─────────────────────────────────────────────

def arm_vehicle(vehicle):
    log("Arming motors …")

    log("  Waiting for 3D GPS fix …")
    for _ in range(30):
        if vehicle.gps_0.fix_type >= 3:
            break
        time.sleep(1)
    else:
        log("WARNING: No 3D GPS fix - proceeding (SITL / indoor?)")

    log("  Waiting for EKF to be healthy …")
    for _ in range(20):
        if vehicle.ekf_ok:
            break
        time.sleep(1)
    else:
        log("WARNING: EKF not healthy - proceeding anyway")

    for attempt in range(1, ARM_MAX_RETRIES + 1):
        if vehicle.armed:
            log(f"✓ Drone ARMED (confirmed on attempt {attempt})")
            return
        log(f"  Arm attempt {attempt}/{ARM_MAX_RETRIES}")
        vehicle.armed = True
        time.sleep(ARM_RETRY_DELAY)

    abort(f"Failed to ARM after {ARM_MAX_RETRIES} attempts. "
          "Fix pre-arm errors on GCS (compass / accel / GPS / RC cal).")


# ─────────────────────────────────────────────
# STEP 4 - Takeoff
# ─────────────────────────────────────────────

def takeoff(vehicle, target_alt: float):
    log(f"Waiting {ARM_DELAY}s post-arm before takeoff …")
    time.sleep(ARM_DELAY)

    log(f"Taking off to {target_alt} m …")
    vehicle.simple_takeoff(target_alt)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        log(f"  Climbing … altitude: {alt:.2f} m  (target: {target_alt} m)")
        if alt >= target_alt - ALTITUDE_TOLERANCE:
            log(f"✓ Takeoff altitude reached: {alt:.2f} m")
            break
        if not vehicle.armed:
            abort("Drone disarmed unexpectedly during takeoff!")
        time.sleep(0.5)


# ─────────────────────────────────────────────
# GOTO with altitude monitoring
# ─────────────────────────────────────────────

def goto_and_monitor(vehicle, target_wp, target_alt: float, label: str,
                     timeout: float = WAYPOINT_TIMEOUT):
    """
    Send simple_goto to target_wp, then block until the drone arrives
    within POSITION_TOLERANCE metres horizontally.
    Altitude is logged every 0.3s throughout; DroneKit maintains it automatically.
    """
    vehicle.simple_goto(target_wp, airspeed=CRUISE_SPEED)
    log(f"  Flying to {label} …")

    deadline = time.time() + timeout
    while time.time() < deadline:
        current = vehicle.location.global_relative_frame
        dist    = get_horizontal_distance(current, target_wp)
        check_altitude(vehicle, target_alt, label=label)
        log(f"    Horizontal dist to {label}: {dist:.2f} m")

        if dist <= POSITION_TOLERANCE:
            log(f"✓ Reached {label}  (dist={dist:.2f}m)")
            return

        if not vehicle.armed:
            abort("Drone disarmed unexpectedly mid-flight!")

        time.sleep(0.3)

    log(f"⚠ Timeout reaching {label} - continuing anyway")


def hover_and_monitor(vehicle, target_alt: float, duration: int, label: str):
    """
    Hover in place for `duration` seconds, logging altitude every 0.5s.
    """
    log(f"Hovering at {label} for {duration}s …")
    for _ in range(duration * 2):    # 0.5s steps
        check_altitude(vehicle, target_alt, label=f"HOVER @ {label}")
        time.sleep(0.5)
    log(f"✓ Hover complete at {label}")


# ─────────────────────────────────────────────
# STEP 5-9 - Straight flight out and back
# ─────────────────────────────────────────────

def fly_straight_and_back(vehicle, distance: float, altitude: float):
    """
    1. Lock the drone's current heading (yaw) as the forward direction.
    2. Compute a GPS waypoint `distance` metres ahead in that direction.
    3. Fly there (altitude maintained by GUIDED mode).
    4. Hover for HOVER_DURATION seconds.
    5. Fly back to the origin point.
    6. Hover for HOVER_DURATION seconds.
    """
    # ── Read heading BEFORE flight ──────────────────────────
    heading = get_yaw_degrees(vehicle)
    back_heading = (heading + 180) % 360    # exact reverse direction

    log("-" * 55)
    log(f"  Forward heading : {heading:.1f}°")
    log(f"  Return heading  : {back_heading:.1f}°")
    log(f"  Distance        : {distance} m each way")
    log(f"  Cruise altitude : {altitude} m")
    log("-" * 55)

    # Grab current GPS as origin (at flight altitude)
    raw = vehicle.location.global_relative_frame
    origin_wp = LocationGlobalRelative(raw.lat, raw.lon, altitude)

    # Compute forward waypoint
    forward_wp = offset_location_by_heading(origin_wp, distance, heading)

    log(f"  Origin  : {origin_wp.lat:.7f}, {origin_wp.lon:.7f}  alt={altitude}m")
    log(f"  Forward : {forward_wp.lat:.7f}, {forward_wp.lon:.7f}  alt={altitude}m")

    # ── LEG 1: Fly forward ───────────────────────────────────
    log("─── LEG 1: Flying FORWARD ───")
    goto_and_monitor(vehicle, forward_wp, altitude, label="FORWARD POINT")

    # ── Hover at forward point ────────────────────────────────
    hover_and_monitor(vehicle, altitude, HOVER_DURATION, label="FORWARD POINT")

    # ── LEG 2: Fly back to origin ─────────────────────────────
    log("─── LEG 2: Flying BACK to origin ───")
    goto_and_monitor(vehicle, origin_wp, altitude, label="ORIGIN")

    # ── Hover at origin ──────────────────────────────────────
    hover_and_monitor(vehicle, altitude, HOVER_DURATION, label="ORIGIN")

    log("✓ Straight flight complete - back at origin")


# ─────────────────────────────────────────────
# STEP 10 - Land (NOT RTL)
# ─────────────────────────────────────────────

def land_vehicle(vehicle):
    log("Initiating LAND mode …")
    vehicle.mode = VehicleMode("LAND")

    for _ in range(10):
        if vehicle.mode.name == "LAND":
            log("✓ Mode confirmed: LAND")
            break
        time.sleep(0.5)
    else:
        log("WARNING: LAND mode not confirmed - continuing")

    log("Descending …")
    while True:
        alt = vehicle.location.global_relative_frame.alt
        log(f"  Altitude: {alt:.2f} m")
        if not vehicle.armed:
            log("✓ Drone landed and auto-disarmed")
            return
        if alt <= 0.1:
            log("✓ Ground detected (alt ≤ 0.1 m)")
            break
        time.sleep(0.5)

    time.sleep(3)   # let FC settle before disarm attempt


# ─────────────────────────────────────────────
# STEP 11 - Disarm
# ─────────────────────────────────────────────

def disarm_vehicle(vehicle):
    if not vehicle.armed:
        log("✓ Already disarmed")
        return

    log("Disarming …")
    vehicle.armed = False
    for _ in range(10):
        if not vehicle.armed:
            log("✓ Drone DISARMED")
            return
        time.sleep(0.5)

    # Force disarm MAVLink fallback
    log("  Trying force-disarm via MAVLink …")
    vehicle.message_factory.command_long_send(
        vehicle._master.target_system,
        vehicle._master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0,
    )
    time.sleep(2)
    log("✓ Force-disarm sent" if not vehicle.armed else "⚠ Could not confirm disarm - check GCS!")


# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────

def main():
    log("=" * 55)
    log("  Straight Flight  |  5m forward + back  |  5m alt")
    log("  Pixhawk 2.4.8 + Jetson Nano")
    log("=" * 55)

    vehicle = None
    try:
        vehicle = connect_vehicle()                          # Step 1
        set_guided_mode(vehicle)                             # Step 2
        arm_vehicle(vehicle)                                 # Step 3
        takeoff(vehicle, FLIGHT_ALTITUDE)                    # Step 4
        fly_straight_and_back(                               # Steps 5-9
            vehicle,
            distance=FLIGHT_DISTANCE,
            altitude=FLIGHT_ALTITUDE,
        )
        land_vehicle(vehicle)                                # Step 10
        disarm_vehicle(vehicle)                              # Step 11

        log("=" * 55)
        log("  Mission complete ✓")
        log("=" * 55)

    except KeyboardInterrupt:
        log("Keyboard interrupt!")
        if vehicle and vehicle.armed:
            log("Emergency LAND triggered …")
            vehicle.mode = VehicleMode("LAND")

    finally:
        if vehicle:
            log("Closing vehicle connection …")
            vehicle.close()


if __name__ == "__main__":
    main()
