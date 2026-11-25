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
def quadplane_sitl():
    """Starts ArduPlane QuadPlane (VTOL) SITL on instance 2 (UDP port 14570) and tears it down after the module."""
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sim_vehicle = os.path.join(
        workspace, "tests", "ardupilot", "Tools", "autotest", "sim_vehicle.py"
    )

    cmd = [
        sim_vehicle,
        "-v", "ArduPlane",
        "-f", "quadplane",           # QuadPlane VTOL frame (uses already-built arduplane binary)
        "-I", "2",                   # Instance 2 -> UDP port 14570 (avoids conflict with Copter:14550 & Plane:14560)
        "-N",                        # No recompilation
        "-w",                        # Wipe EEPROM parameters to default
        "--speedup", "3",            # Speed up physics
        "--mavproxy-args", "--daemon",
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
        yield "udpin:127.0.0.1:14570"
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
def quadplane_drone(quadplane_sitl, tmp_path_factory):
    """Initializes MAVLinCS connected to QuadPlane SITL, disables arming checks, and waits for 3D GPS fix."""
    setup_logger(logger=default_logger, debug_mode=True, add_timestamp_to_console=True)
    tlog_file = str(tmp_path_factory.mktemp("logs") / "pytest_quadplane_sitl.tlog")

    print(f"\nConnecting to QuadPlane SITL at {quadplane_sitl}...")
    mav = MAVLinCS(
        address=quadplane_sitl,
        timeout_heartbeat=60,
        mavlogfile=tlog_file,
        relative_path_to_cwd=False,
        pos_rate=10,
        send_automatic_timesync=True,
        timesync_delay=2,
    )

    try:
        # Disable pre-arm checks for simulation (ARMING_SKIPCHK=-1 on ArduPilot 4.7+, ARMING_CHECK=0 on older versions)
        if not mav.write_parameter("ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32, timeout_ack=1.0):
            mav.write_parameter("ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, timeout_ack=1.0)
        time.sleep(1)

        # Wait for 3D GPS Fix
        assert mav.wait_gps_3d_fix(timeout=45) is True, "Timeout: No 3D GPS Fix on QuadPlane SITL"

        # Wait for EKF to allow switching to GUIDED mode
        start_wait = time.time()
        while time.time() - start_wait < 30:
            if mav.set_mode("GUIDED", timeout_ack=2, timeout_set_mode=2):
                break
            time.sleep(1)
        else:
            pytest.fail("QuadPlane SITL EKF did not allow GUIDED mode after 30s.")

        yield mav

    finally:
        mav.mission_ended = False
        mav.allow_actions = True
        mav.close()


def test_quadplane_telemetry_and_vtol_modes(quadplane_drone):
    """Verifies QuadPlane VTOL telemetry (vtol_state) and Q-mode switching."""
    # On ground in GUIDED (non-Q mode), QuadPlane reports a non-undefined VTOL state (FW until entering a Q-mode)
    assert quadplane_drone.vtol_state() != mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED

    mapping = quadplane_drone.mode_mapping()
    assert isinstance(mapping, dict)
    for expected_mode in ("QLOITER", "QHOVER", "QLAND", "QRTL", "GUIDED", "AUTO", "FBWA"):
        assert expected_mode in mapping

    # Entering QLOITER puts QuadPlane in MC state
    assert quadplane_drone.set_mode("QLOITER", timeout_ack=3, timeout_set_mode=4) is True
    assert quadplane_drone.mode() == "QLOITER"
    start_wait = time.time()
    while time.time() - start_wait < 5:
        if quadplane_drone.vtol_state() == mavutil.mavlink.MAV_VTOL_STATE_MC:
            break
        time.sleep(0.1)
    assert quadplane_drone.vtol_state() == mavutil.mavlink.MAV_VTOL_STATE_MC
    assert quadplane_drone.vtol_state_str() == "MAV_VTOL_STATE_MC"

    for target_mode in ("QHOVER", "GUIDED"):
        assert quadplane_drone.set_mode(target_mode, timeout_ack=3, timeout_set_mode=4) is True
        assert quadplane_drone.mode() == target_mode


def test_quadplane_vtol_takeoff_transition_and_qland(quadplane_drone):
    """Tests vertical VTOL takeoff(), in-air climb, AUTO vtol_transition() MC<->FW, and vertical land() (QLAND)."""
    gps0 = quadplane_drone.position_gps()
    assert not math.isnan(gps0.lat) and not math.isnan(gps0.lon)

    # 1. Vertical VTOL takeoff in GUIDED mode (tests Q_MAV_TYPE=0 VTOL detection via EXTENDED_SYS_STATE)
    assert quadplane_drone.arm(timeout_arming=10) is True
    assert quadplane_drone.motors_armed() is True

    assert quadplane_drone.takeoff(
        altitude=25.0,
        precision=2.0,
        timeout_ack=3,
        timeout_set_mode=5,
        timeout_takeoff=45,
    ) is True
    assert quadplane_drone.mode() == "GUIDED"
    assert quadplane_drone.vtol_state() in (
        mavutil.mavlink.MAV_VTOL_STATE_MC,
        mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_MC,
    )
    assert abs(quadplane_drone.position_gps().relative_alt - 25.0) <= 4.0

    # 2. In-air altitude change via takeoff() while already airborne (uses SET_POSITION_TARGET_GLOBAL_INT)
    assert quadplane_drone.takeoff(
        altitude=35.0,
        precision=2.0,
        timeout_ack=3,
        timeout_set_mode=5,
        timeout_takeoff=30,
    ) is True
    assert quadplane_drone.mode() == "GUIDED"
    assert quadplane_drone.position_gps().relative_alt >= 32.0

    # 3. Upload a long-leg AUTO mission to test vtol_transition() MC -> FW -> MC in AUTO mode
    wp_far = MAVLinCS.cartesian_to_geographic_point(
        gps0.lat, gps0.lon, north_displacement=800.0, east_displacement=0.0
    )
    mission = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=gps0.lat, y=gps0.lon, z=0.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=wp_far.latitude, y=wp_far.longitude, z=45.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, x=wp_far.latitude, y=wp_far.longitude, z=45.0),
    ]
    assert quadplane_drone.upload_mission(waypoints=mission, clear_existing=True, timeout=15.0) is True
    assert quadplane_drone.start_mission(timeout=10) is True
    assert quadplane_drone.wait_mode_changed("AUTO", timeout=5) is True

    # Explicitly command FW transition in AUTO and wait for physical transition completion
    assert quadplane_drone.vtol_transition(
        mavutil.mavlink.MAV_VTOL_STATE_FW,
        timeout_ack=3,
        timeout_transition=40,
    ) is True
    assert quadplane_drone.vtol_state() == mavutil.mavlink.MAV_VTOL_STATE_FW
    assert quadplane_drone.vtol_state_str() == "MAV_VTOL_STATE_FW"
    assert quadplane_drone.airspeed() > 10.0

    # Command MC transition in AUTO and wait for physical transition back to multicopter
    assert quadplane_drone.vtol_transition(
        mavutil.mavlink.MAV_VTOL_STATE_MC,
        timeout_ack=3,
        timeout_transition=30,
    ) is True
    assert quadplane_drone.vtol_state() == mavutil.mavlink.MAV_VTOL_STATE_MC
    assert quadplane_drone.vtol_state_str() == "MAV_VTOL_STATE_MC"

    # 4. Vertical VTOL landing via land() -> switches to QLAND and waits for touchdown + auto-disarm
    assert quadplane_drone.land(timeout_ack=3, timeout_set_mode=5, timeout_land=90) is True
    assert quadplane_drone.mode() == "QLAND"
    assert quadplane_drone.motors_disarmed() is True
