"""
Autonomous Drone Circle Flight Script
Hardware: Pixhawk 2.4.8 + Jetson Nano
Protocol: MAVLink via DroneKit

Flight Plan:
  1. Connect to Pixhawk via serial (Jetson Nano UART)
  2. Switch to GUIDED mode (with retry loop)
  3. ARM the drone (with retry loop)
  4. Wait 2 seconds, take off to 3 meters (safe altitude for circle)
  5. Fly a circle of 2 meter radius using NED velocity commands
     - Circle is divided into NUM_POINTS waypoints
     - Each point sent as a position offset from the takeoff location
     - Waits for drone to reach each waypoint within POSITION_TOLERANCE
  6. Return to center (takeoff point) after completing circle
  7. LAND in place (NOT RTL)
  8. Disarm drone

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

CONNECTION_STRING   = "/dev/ttyACM0"   # change to /dev/ttyUSB0 if needed
BAUD_RATE           = 57600

TAKEOFF_ALTITUDE    = 2.0    # metres  - fly circle at this height
CIRCLE_RADIUS       = 1.0    # metres
NUM_POINTS          = 36     # waypoints around the circle (360/36 = 10° steps)
POINT_SPEED         = 0.8    # m/s  - travel speed between waypoints
POSITION_TOLERANCE  = 0.4    # metres - how close to each waypoint before moving on
ALTITUDE_TOLERANCE  = 0.15   # metres - altitude acceptance window

MODE_RETRY_DELAY    = 0.5
MODE_MAX_RETRIES    = 20

ARM_RETRY_DELAY     = 1.0
ARM_MAX_RETRIES     = 20

ARM_DELAY           = 2      # seconds to wait after arm before takeoff

# ─────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────

def log(msg: str):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def abort(msg: str):
    log(f"ABORT ─ {msg}")
    sys.exit(1)


def get_distance_metres(loc1, loc2) -> float:
    """
    Approximate flat-earth distance between two LocationGlobalRelative points.
    Accurate enough for small distances (< 100 m).
    """
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)


def offset_location(origin, d_north: float, d_east: float) -> LocationGlobalRelative:
    """
    Return a new LocationGlobalRelative that is d_north metres north and
    d_east metres east of `origin`, at the same altitude.
    Uses flat-earth approximation (fine for 2-5 m radius).
    """
    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east  / (earth_radius * math.cos(math.radians(origin.lat)))
    return LocationGlobalRelative(
        origin.lat + math.degrees(d_lat),
        origin.lon + math.degrees(d_lon),
        origin.alt,
    )


def send_ned_velocity(vehicle, vx: float, vy: float, vz: float):
    """
    Send a SET_POSITION_TARGET_LOCAL_NED message to command
    body-frame NED velocity. vz positive = DOWN.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                                           # time_boot_ms (ignored)
        0, 0,                                        # target system, component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,                          # type_mask: velocity only
        0, 0, 0,                                     # x, y, z  (ignored)
        vx, vy, vz,                                  # vx, vy, vz  m/s
        0, 0, 0,                                     # ax, ay, az  (ignored)
        0, 0,                                        # yaw, yaw_rate (ignored)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


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
            log(f"✓ Mode confirmed: GUIDED")
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

    if vehicle.armed:
        log("✓ Drone ARMED")
    else:
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
        log(f"  Altitude: {alt:.2f} m")
        if alt >= target_alt - ALTITUDE_TOLERANCE:
            log(f"✓ Takeoff altitude reached: {alt:.2f} m")
            break
        if not vehicle.armed:
            abort("Drone disarmed unexpectedly during takeoff!")
        time.sleep(0.5)


# ─────────────────────────────────────────────
# STEP 5 - Fly circle of given radius
# ─────────────────────────────────────────────

def fly_circle(vehicle, radius: float, num_points: int, altitude: float):
    """
    Fly a horizontal circle of `radius` metres centred on the takeoff point.
    Uses simple_goto() to each of `num_points` evenly-spaced waypoints.
    The first waypoint is due EAST of the takeoff point.
    """
    # Record the takeoff / centre position
    centre = vehicle.location.global_relative_frame
    centre_point = LocationGlobalRelative(centre.lat, centre.lon, altitude)

    log(f"Circle parameters:")
    log(f"  Centre     : {centre.lat:.7f}, {centre.lon:.7f}")
    log(f"  Radius     : {radius} m")
    log(f"  Waypoints  : {num_points}  ({360 / num_points:.1f}° steps)")
    log(f"  Altitude   : {altitude} m")

    # ── Move to start of circle (0° = East) ──────────────────
    log("Moving to circle start point (East) …")
    start = offset_location(centre_point, 0, radius)
    vehicle.simple_goto(start, airspeed=POINT_SPEED)
    wait_for_waypoint(vehicle, start, label="circle start")

    # ── Fly each waypoint ────────────────────────────────────
    log(f"Starting circle … ({num_points} waypoints)")
    for i in range(num_points + 1):          # +1 closes the loop back to 0°
        angle_deg = (360.0 / num_points) * i
        angle_rad = math.radians(angle_deg)

        # NED convention: North = +X, East = +Y
        d_north =  radius * math.sin(angle_rad)   # 0° = East, so sin for N
        d_east  =  radius * math.cos(angle_rad)   # cos for E

        # Recalculate: standard bearing 0°=North
        # Use: N = R*cos(θ), E = R*sin(θ)  where θ is measured from North CW
        d_north =  radius * math.cos(angle_rad)
        d_east  =  radius * math.sin(angle_rad)

        wp = offset_location(centre_point, d_north, d_east)
        log(f"  → Waypoint {i+1:02d}/{num_points}  angle={angle_deg:6.1f}°  "
            f"N={d_north:+.2f}m  E={d_east:+.2f}m")
        vehicle.simple_goto(wp, airspeed=POINT_SPEED)
        wait_for_waypoint(vehicle, wp, label=f"WP {i+1}")

    log("✓ Full circle completed")

    # ── Return to centre ────────────────────────────────────
    log("Returning to centre (takeoff point) …")
    vehicle.simple_goto(centre_point, airspeed=POINT_SPEED)
    wait_for_waypoint(vehicle, centre_point, label="centre")
    log("✓ Back at centre")


def wait_for_waypoint(vehicle, target, label: str = "waypoint", timeout: float = 30.0):
    """
    Block until the drone is within POSITION_TOLERANCE of `target`,
    or until `timeout` seconds elapse.
    Also monitors altitude drift.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        current = vehicle.location.global_relative_frame
        dist = get_distance_metres(current, target)
        alt  = current.alt
        log(f"    [{label}] dist={dist:.2f}m  alt={alt:.2f}m")

        if dist <= POSITION_TOLERANCE:
            log(f"  ✓ Reached {label} (dist={dist:.2f}m)")
            return

        if not vehicle.armed:
            abort("Drone disarmed unexpectedly mid-flight!")

        time.sleep(0.3)

    log(f"  ⚠ Timeout reaching {label} - continuing to next point")


# ─────────────────────────────────────────────
# STEP 6 - Land (NOT RTL)
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

    time.sleep(3)   # let FC settle


# ─────────────────────────────────────────────
# STEP 7 - Disarm
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

    # Force disarm fallback
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
    log("  Drone Circle Flight  |  r=2m  |  Pixhawk 2.4.8")
    log("=" * 55)

    vehicle = None
    try:
        vehicle = connect_vehicle()          # Step 1
        set_guided_mode(vehicle)             # Step 2
        arm_vehicle(vehicle)                 # Step 3
        takeoff(vehicle, TAKEOFF_ALTITUDE)   # Step 4
        fly_circle(                          # Step 5
            vehicle,
            radius=CIRCLE_RADIUS,
            num_points=NUM_POINTS,
            altitude=TAKEOFF_ALTITUDE,
        )
        land_vehicle(vehicle)                # Step 6
        disarm_vehicle(vehicle)              # Step 7

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
