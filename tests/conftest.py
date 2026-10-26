import os
import signal
import subprocess
import time
import pytest
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS
from pyMAVLinCS.setup_logger import setup_logger, default_logger


@pytest.fixture(scope="session")
def sitl():
    """Starts ArduCopter SITL via sim_vehicle.py and ensures its shutdown at the end of the session."""
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sim_vehicle = os.path.join(
        workspace, "tests", "ardupilot", "Tools", "autotest", "sim_vehicle.py"
    )

    cmd = [
        sim_vehicle,
        "-v", "ArduCopter",
        "-f", "quad",
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
        # Allow time for sim_vehicle.py to initialize MAVProxy on port 14550
        time.sleep(5)
        yield "udpin:127.0.0.1:14550"
    finally:
        # Ultra-robust teardown to never block a GitHub Actions job
        try:
            pgid = os.getpgid(process.pid)
            os.killpg(pgid, signal.SIGTERM)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except ProcessLookupError:
            pass # Process already terminated


@pytest.fixture(scope="session")
def drone(sitl, tmp_path_factory):
    """Initializes MAVLinCS, configures ARMING_CHECK, and waits for EKF to be ready."""
    setup_logger(logger=default_logger, debug_mode=True, add_timestamp_to_console=True)
    tlog_file = str(tmp_path_factory.mktemp("logs") / "pytest_sitl.tlog")

    print(f"\nConnecting to SITL at {sitl}...")
    mav = MAVLinCS(
        address=sitl,
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

        # 1. Wait for 3D GPS Fix
        assert mav.wait_gps_3d_fix(timeout=45) is True, "Timeout: No 3D GPS Fix on SITL"

        # 2. Wait for ArduCopter EKF to accept GUIDED mode (prevents 1st test failure)
        start_wait = time.time()
        while time.time() - start_wait < 30:
            if mav.set_mode("GUIDED", timeout_ack=2, timeout_set_mode=2):
                break
            time.sleep(1)
        else:
            pytest.fail("SITL EKF did not allow GUIDED mode after 30s.")

        yield mav

    finally:
        # Clean cleanup at the end of all tests
        mav.mission_ended = False
        mav.allow_actions = True
        mav.close()


@pytest.fixture(autouse=True)
def reset_drone_flags(drone):
    """Automatically resets states and callbacks between each test."""
    drone.mission_ended = False
    drone.allow_actions = True
    drone.set_additional_recv_msg_callback(None)
    drone.set_additional_send_callback(None)
    drone.set_statustext_callback(None)
    drone.set_mcc_callback(None)
    yield
    drone.mission_ended = False
    drone.allow_actions = True
