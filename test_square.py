"""
Autonomous Drone Square Flight Script
Hardware: Pixhawk 2.4.8 + Jetson Nano
Protocol: MAVLink via DroneKit

Flight Plan:
  1.  Connect to Pixhawk via serial
  2.  Switch to GUIDED mode (with retry)
  3.  ARM (with retry)
  4.  Wait 2s → Takeoff to 5 meters
  5.  Lock current heading as FORWARD direction
  6.  Fly a 3×3 m square (4 corners) at 5 m altitude:
        Corner A (start/origin)
          → Corner B  (3m forward)
          → Corner C  (3m right)
          → Corner D  (3m back)
          → Corner A  (3m left  - back to launch point)
  7.  Hover at launch point for 2 seconds
  8.  LAND in place (NOT RTL)
  9.  Disarm

Altitude is monitored and logged throughout the entire flight.
All waypoints are computed from the drone's facing direction at takeoff.

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

CONNECTION_STRING   = "/dev/ttyACM0"   # change to /dev/ttyUSB0 if using USB adapter
BAUD_RATE           = 57600

FLIGHT_ALTITUDE     = 5.0    # metres - takeoff and cruise height
SQUARE_SIDE         = 3.0    # metres - side length of the square
CRUISE_SPEED        =1.5    # m/s    - speed between waypoints (slow = precise)
CORNER_HOVER        = 2    # seconds to pause at each corner
FINAL_HOVER         = 3      # seconds to hover at launch point before landing

ALTITUDE_TOLERANCE  = 0.15   # metres - ±window for altitude warnings
POSITION_TOLERANCE  = 0.35   # metres - horizontal dist to consider waypoint reached
WAYPOINT_TIMEOUT    = 45.0   # seconds - max wait per waypoint before moving on

MODE_RETRY_DELAY    = 0.5
MODE_MAX_RETRIES    = 20

ARM_RETRY_DELAY     = 1.0
ARM_MAX_RETRIES     = 20

ARM_DELAY           = 2      # seconds to wait post-arm before takeoff

# ─────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────

def log(msg: str):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def abort(msg: str):
    log(f"ABORT ─ {msg}")
    sys.exit(1)


def get_horizontal_distance(loc1, loc2) -> float:
    """Flat-earth horizontal distance in metres between two GPS points."""
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)


def offset_location(origin: LocationGlobalRelative,
                    d_north: float,
                    d_east: float) -> LocationGlobalRelative:
    """
    Return a GPS point offset by d_north metres north and d_east metres east
    from origin, at the same altitude.
    Flat-earth approximation - accurate to < 1 mm at distances < 50 m.
    """
    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east  / (earth_radius * math.cos(math.radians(origin.lat)))
    return LocationGlobalRelative(
        origin.lat + math.degrees(d_lat),
        origin.lon + math.degrees(d_lon),
        origin.alt,
    )


def heading_to_ned(distance: float, heading_deg: float):
    """
    Convert a distance + compass heading (0=N, 90=E) into (d_north, d_east).
    """
    r = math.radians(heading_deg)
    return distance * math.cos(r), distance * math.sin(r)


def get_yaw_degrees(vehicle) -> float:
    """
    Read vehicle heading in 0-360° (0=North, clockwise).
    """
    yaw_rad = vehicle.attitude.yaw
    return (math.degrees(yaw_rad) + 360) % 360


def check_altitude(vehicle, target_alt: float, label: str = "") -> float:
    alt  = vehicle.location.global_relative_frame.alt
    diff = abs(alt - target_alt)
    ok   = diff <= ALTITUDE_TOLERANCE
    tag  = f"[{label}] " if label else ""
    flag = "✓" if ok else "⚠ ALT DRIFT"
    log(f"  {tag}Alt: {alt:.2f}m  target: {target_alt:.2f}m  err: {diff:.2f}m  {flag}")
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
    log(f"  GPS fix    : {vehicle.gps_0.fix_type}  (need ≥3)")
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
        log(f"  Climbing … alt: {alt:.2f}m  (target: {target_alt}m)")
        if alt >= target_alt - ALTITUDE_TOLERANCE:
            log(f"✓ Takeoff altitude reached: {alt:.2f}m")
            break
        if not vehicle.armed:
            abort("Drone disarmed unexpectedly during takeoff!")
        time.sleep(0.5)


# ─────────────────────────────────────────────
# GOTO with continuous altitude monitoring
# ─────────────────────────────────────────────

def goto_and_monitor(vehicle, target_wp: LocationGlobalRelative,
                     target_alt: float, label: str,
                     timeout: float = WAYPOINT_TIMEOUT):
    """
    Command drone to target_wp and block until arrival within POSITION_TOLERANCE.
    Logs altitude every 0.3s. DroneKit GUIDED mode maintains altitude automatically.
    """
    vehicle.simple_goto(target_wp, airspeed=CRUISE_SPEED)

    deadline = time.time() + timeout
    while time.time() < deadline:
        current = vehicle.location.global_relative_frame
        dist    = get_horizontal_distance(current, target_wp)
        check_altitude(vehicle, target_alt, label=label)
        log(f"    → [{label}] horiz dist: {dist:.2f}m")

        if dist <= POSITION_TOLERANCE:
            log(f"✓ Reached [{label}]  dist={dist:.2f}m")
            return

        if not vehicle.armed:
            abort("Drone disarmed unexpectedly mid-flight!")

        time.sleep(0.3)

    log(f"⚠ Timeout reaching [{label}] - continuing anyway")


def hover_at(vehicle, target_alt: float, duration: float, label: str):
    """Hover in place for `duration` seconds, logging altitude."""
    log(f"Hovering at [{label}] for {duration}s …")
    steps = int(duration / 0.5)
    for _ in range(max(steps, 1)):
        check_altitude(vehicle, target_alt, label=f"HOVER @ {label}")
        time.sleep(0.5)
    log(f"✓ Hover complete at [{label}]")


# ─────────────────────────────────────────────
# STEP 5-7 - Square flight
# ─────────────────────────────────────────────

def fly_square(vehicle, side: float, altitude: float):
    """
    Fly a square of `side` metres at `altitude` metres.

    The square is oriented relative to the drone's facing direction
    at the moment this function is called:

        FORWARD heading = drone's yaw (locked once)
        RIGHT   heading = forward + 90°

    Square corners (labelled relative to origin/launch point):

               FORWARD
                  ↑
        A ────────── B
        |            |
        |   3×3 m    |   RIGHT →
        |   square   |
        D ────────── C
        ↑
      ORIGIN (launch point)

    Flight path:  A → B → C → D → A  (clockwise when viewed from above)
    Returns to A (origin) at the end.
    """

    # ── Lock heading ────────────────────────────────────────
    heading_fwd   = get_yaw_degrees(vehicle)
    heading_right = (heading_fwd + 90) % 360

    log("=" * 55)
    log(f"  Square flight plan")
    log(f"  Side length     : {side} m")
    log(f"  Altitude        : {altitude} m")
    log(f"  Forward heading : {heading_fwd:.1f}°")
    log(f"  Right heading   : {heading_right:.1f}°")
    log("=" * 55)

    # ── Compute unit vectors in NED ──────────────────────────
    # fwd_n, fwd_e  = 1 metre in forward direction
    # rgt_n, rgt_e  = 1 metre in right direction
    fwd_n, fwd_e = heading_to_ned(1.0, heading_fwd)
    rgt_n, rgt_e = heading_to_ned(1.0, heading_right)

    # ── Lock origin (corner A) ───────────────────────────────
    raw    = vehicle.location.global_relative_frame
    origin = LocationGlobalRelative(raw.lat, raw.lon, altitude)

    # ── Compute all 4 corners ─────────────────────────────────
    #
    #   A = origin                  (launch point)
    #   B = A + side * fwd          (forward)
    #   C = B + side * right        (forward + right)
    #   D = A + side * right        (right only)
    #
    corner_A = origin    # launch point - we return here at the end

    corner_B = offset_location(
        origin,
        d_north = side * fwd_n,
        d_east  = side * fwd_e,
    )

    corner_C = offset_location(
        origin,
        d_north = side * fwd_n + side * rgt_n,
        d_east  = side * fwd_e + side * rgt_e,
    )

    corner_D = offset_location(
        origin,
        d_north = side * rgt_n,
        d_east  = side * rgt_e,
    )

    corners = [
        ("A - ORIGIN / LAUNCH",  corner_A),
        ("B - FORWARD",          corner_B),
        ("C - FORWARD+RIGHT",    corner_C),
        ("D - RIGHT",            corner_D),
    ]

    log("  Computed waypoints:")
    for name, wp in corners:
        log(f"    [{name}]  {wp.lat:.7f}, {wp.lon:.7f}  alt={wp.alt}m")
    log("")

    # ── Fly the square: A → B → C → D → A ───────────────────

    # Already at A (just took off) - fly to B
    log("─── LEG 1: A → B  (forward) ───")
    goto_and_monitor(vehicle, corner_B, altitude, label="B - FORWARD")
    hover_at(vehicle, altitude, CORNER_HOVER, label="B")

    log("─── LEG 2: B → C  (right) ───")
    goto_and_monitor(vehicle, corner_C, altitude, label="C - FORWARD+RIGHT")
    hover_at(vehicle, altitude, CORNER_HOVER, label="C")

    log("─── LEG 3: C → D  (backward) ───")
    goto_and_monitor(vehicle, corner_D, altitude, label="D - RIGHT")
    hover_at(vehicle, altitude, CORNER_HOVER, label="D")

    log("─── LEG 4: D → A  (left - back to launch point) ───")
    goto_and_monitor(vehicle, corner_A, altitude, label="A - ORIGIN/LAUNCH")

    log("✓ Square complete - back at launch point")

    # ── Final hover at origin before landing ─────────────────
    hover_at(vehicle, altitude, FINAL_HOVER, label="ORIGIN - pre-land")


# ─────────────────────────────────────────────
# STEP 8 - Land (NOT RTL)
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
        log(f"  Altitude: {alt:.2f}m")
        if not vehicle.armed:
            log("✓ Drone landed and auto-disarmed")
            return
        if alt <= 0.1:
            log("✓ Ground detected (alt ≤ 0.1 m)")
            break
        time.sleep(0.5)

    time.sleep(3)   # let FC settle


# ─────────────────────────────────────────────
# STEP 9 - Disarm
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
    log("  Square Flight  |  3×3m  |  5m alt  |  Return to Launch")
    log("  Pixhawk 2.4.8 + Jetson Nano")
    log("=" * 55)

    vehicle = None
    try:
        vehicle = connect_vehicle()                      # Step 1
        set_guided_mode(vehicle)                         # Step 2
        arm_vehicle(vehicle)                             # Step 3
        takeoff(vehicle, FLIGHT_ALTITUDE)                # Step 4
        fly_square(vehicle, SQUARE_SIDE, FLIGHT_ALTITUDE)  # Steps 5-7
        land_vehicle(vehicle)                            # Step 8
        disarm_vehicle(vehicle)                          # Step 9

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
