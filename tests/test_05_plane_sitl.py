import math
import os
import signal
import subprocess
import time
import pytest
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS, MissionItem
from pyMAVLinCS.setup_logger import setup_logger, default_logger


@pytest.fixture(scope="module")
def plane_sitl():
    """Starts ArduPlane SITL on instance 1 (UDP port 14560) and tears it down after the module."""
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sim_vehicle = os.path.join(
        workspace, "tests", "ardupilot", "Tools", "autotest", "sim_vehicle.py"
    )

    cmd = [
        sim_vehicle,
        "-v", "ArduPlane",
        "-I", "1",                   # Instance 1 -> UDP port 14560 (avoids conflict with ArduCopter on 14550)
        "-N",                        # No recompilation
        "-w",                        # Wipe EEPROM parameters to default
        "--speedup", "3",            # Speed up physics
        "--mavproxy-args", "--daemon"
    ]

    process = subprocess.Popen(
        cmd,
        cwd=workspace,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    try:
        time.sleep(5)
        yield "udpin:127.0.0.1:14560"
    finally:
        try:
            pgid = os.getpgid(process.pid)
            os.killpg(pgid, signal.SIGTERM)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except ProcessLookupError:
            pass


@pytest.fixture(scope="module")
def plane_drone(plane_sitl, tmp_path_factory):
    """Initializes MAVLinCS connected to ArduPlane SITL, disables ARMING_CHECK, and waits for 3D GPS fix."""
    setup_logger(logger=default_logger, debug_mode=True, add_timestamp_to_console=True)
    tlog_file = str(tmp_path_factory.mktemp("logs") / "pytest_plane_sitl.tlog")

    print(f"\nConnecting to ArduPlane SITL at {plane_sitl}...")
    mav = MAVLinCS(
        address=plane_sitl,
        timeout_heartbeat=60,
        mavlogfile=tlog_file,
        relative_path_to_cwd=False,
        pos_rate=10,
        send_automatic_timesync=True,
        timesync_delay=2,
    )

    try:
        # Disable pre-arm checks for simulation
        mav.write_parameter("ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(1)

        # Wait for 3D GPS Fix
        assert mav.wait_gps_3d_fix(timeout=45) is True, "Timeout: No 3D GPS Fix on ArduPlane SITL"

        # Wait for EKF to allow switching to GUIDED mode
        start_wait = time.time()
        while time.time() - start_wait < 30:
            if mav.set_mode("GUIDED", timeout_ack=2, timeout_set_mode=2):
                break
            time.sleep(1)
        else:
            pytest.fail("ArduPlane SITL EKF did not allow GUIDED mode after 30s.")

        yield mav

    finally:
        mav.mission_ended = False
        mav.allow_actions = True
        mav.close()


def test_plane_telemetry_and_modes(plane_drone):
    """Verifies ArduPlane vehicle type, fixed-wing mode mapping, and mode transitions."""
    assert plane_drone.vehicle_type_int() == mavutil.mavlink.MAV_TYPE_FIXED_WING
    assert plane_drone.vehicle_type_str() == "MAV_TYPE_FIXED_WING"

    mapping = plane_drone.mode_mapping()
    assert isinstance(mapping, dict)
    for expected_mode in ("MANUAL", "FBWA", "FBWB", "CRUISE", "TAKEOFF", "LOITER", "AUTO", "RTL", "GUIDED"):
        assert expected_mode in mapping

    # Test switching between Fixed-Wing specific modes
    for target_mode in ("FBWA", "LOITER", "GUIDED"):
        assert plane_drone.set_mode(target_mode, timeout_ack=3, timeout_set_mode=4) is True
        assert plane_drone.mode() == target_mode


def test_plane_auto_takeoff_mission_and_guided_flight(plane_drone):
    """Uploads and executes a Fixed-Wing AUTO mission (runway takeoff -> waypoint) then tests do_reposition & RTL."""
    gps0 = plane_drone.position_gps()
    assert not math.isnan(gps0.lat) and not math.isnan(gps0.lon)

    # Project waypoints 250m North and 250m North + 200m East (appropriate for fixed-wing turn radius)
    wp1 = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=250.0, east_displacement=0.0)
    wp2 = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=250.0, east_displacement=200.0)

    # Seq 0: HOME, Seq 1: Fixed-Wing NAV_TAKEOFF (param1 = 15 deg pitch, z = 45m), Seq 2 & 3: Waypoints, Seq 4: RTL
    mission = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=gps0.lat, y=gps0.lon, z=0.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, param1=15.0, x=gps0.lat, y=gps0.lon, z=45.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=wp1.latitude, y=wp1.longitude, z=50.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=wp2.latitude, y=wp2.longitude, z=50.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH),
    ]

    assert plane_drone.upload_mission(waypoints=mission, clear_existing=True, timeout=15.0) is True

    # Start mission from the ground (automatically arms and enters AUTO mode)
    assert plane_drone.start_mission(timeout=10) is True
    assert plane_drone.motors_armed() is True
    assert plane_drone.wait_mode_changed("AUTO", timeout=5) is True

    # Wait for the aircraft to take off and climb above 25m relative altitude
    start_climb = time.time()
    climbed = False
    while time.time() - start_climb < 45:
        if plane_drone.position_gps().relative_alt >= 25.0 and plane_drone.airspeed() > 10.0:
            climbed = True
            break
        time.sleep(0.5)
    assert climbed is True, f"ArduPlane did not climb as expected (alt={plane_drone.position_gps().relative_alt}m)"

    # Verify aeronautical telemetry getters in flight (VFR_HUD & WIND)
    hud = plane_drone.vfr_hud()
    assert hud.airspeed > 8.0 and plane_drone.airspeed() > 8.0
    assert hud.groundspeed > 8.0 and plane_drone.groundspeed() > 8.0
    assert 0 <= plane_drone.heading() <= 360
    assert 0 <= plane_drone.throttle() <= 100
    assert not math.isnan(plane_drone.climb_rate())
    wnd = plane_drone.wind()
    assert not math.isnan(wnd.direction) and not math.isnan(wnd.speed)

    # Wait for ArduPlane to reach proximity of wp1 (fixed-wing acceptance radius ~90m)
    assert plane_drone.wait_proximity_with_gps_pos(
        lat=wp1.latitude,
        lon=wp1.longitude,
        radius_acceptance=90.0,
        timeout=60,
    ) is True

    # Reposition in flight to wp2 using do_reposition (with force_guided=True, loiter_radius=65m, ccw=True)
    assert plane_drone.do_reposition(
        latitude=wp2.latitude,
        longitude=wp2.longitude,
        altitude=60.0,
        loiter_radius=65.0,
        ccw=True,
        force_guided=True,
        timeout_ack=3,
    ) is True
    assert plane_drone.wait_mode_changed("GUIDED", timeout=5) is True

    # Test change_speed on ArduPlane in GUIDED mode (airspeed, restore default airspeed, and cruise throttle)
    assert plane_drone.change_speed(
        speed=18.0,
        speed_type=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
        timeout_ack=3,
    ) is True
    assert plane_drone.change_speed(
        speed=-2.0,
        speed_type=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
        timeout_ack=3,
    ) is True
    assert plane_drone.change_speed(
        speed=-1.0,
        throttle=65.0,
        timeout_ack=3,
    ) is True

    assert plane_drone.wait_proximity_with_gps_pos(
        lat=wp2.latitude,
        lon=wp2.longitude,
        radius_acceptance=95.0,
        timeout=60,
    ) is True

    # Trigger Return-To-Launch (RTL) and verify mode change
    assert plane_drone.return_to_launch(timeout_ack=3) is True
    assert plane_drone.wait_mode_changed("RTL", timeout=5) is True

    # Force disarm to conclude test cleanly
    assert plane_drone.disarm(force=True, timeout_ack=3, timeout_disarming=10) is True
    assert plane_drone.motors_disarmed() is True
