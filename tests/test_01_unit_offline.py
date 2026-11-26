import logging
import math
import time
import pytest
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS, MCC, MissionItem, setup_logger
from pyMAVLinCS.mavecstra import MavLogger, StatustextReceiver
from pyMAVLinCS.mission_control_code import EMPTY


def test_quaternion_to_euler():
    # Identity quaternion (w=1, x=0, y=0, z=0) -> roll=0, pitch=0, yaw=0
    angles = MAVLinCS.quaternion_to_euler([1.0, 0.0, 0.0, 0.0])
    assert math.isclose(angles.roll, 0.0, abs_tol=1e-6)
    assert math.isclose(angles.pitch, 0.0, abs_tol=1e-6)
    assert math.isclose(angles.yaw, 0.0, abs_tol=1e-6)

    # 90 degrees yaw rotation around Z: w=cos(pi/4), z=sin(pi/4)
    s = math.sqrt(0.5)
    angles_yaw90 = MAVLinCS.quaternion_to_euler([s, 0.0, 0.0, s])
    assert math.isclose(angles_yaw90.roll, 0.0, abs_tol=1e-6)
    assert math.isclose(angles_yaw90.pitch, 0.0, abs_tol=1e-6)
    assert math.isclose(angles_yaw90.yaw, math.pi / 2, abs_tol=1e-6)

    # Gimbal lock clamping safety (sinp slightly > 1.0 due to floating-point inaccuracy)
    angles_pitch90 = MAVLinCS.quaternion_to_euler([s, 0.0, s + 1e-7, 0.0])
    assert math.isclose(angles_pitch90.pitch, math.pi / 2, abs_tol=1e-5)


def test_distance_between_two_local_points():
    # 3-4-5 triangle in 2D (z ignored)
    d2d = MAVLinCS.distance_between_two_local_points(0.0, 3.0, 0.0, 4.0)
    assert math.isclose(d2d, 5.0, rel_tol=1e-5)

    # When one Z is NaN or None -> falls back to 2D distance
    d2d_nan = MAVLinCS.distance_between_two_local_points(0.0, 3.0, 0.0, 4.0, z_i=0.0, z_f=math.nan)
    assert math.isclose(d2d_nan, 5.0, rel_tol=1e-5)

    # In 3D
    d3d = MAVLinCS.distance_between_two_local_points(0.0, 3.0, 0.0, 4.0, 0.0, 12.0)
    assert math.isclose(d3d, 13.0, rel_tol=1e-5)


def test_geographic_projections_roundtrip():
    lat0, lon0 = -35.363261, 149.165237
    # Cartesian projection: +100m North, +50m East
    pt = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=100.0, east_displacement=50.0)

    # Inverse calculation (positive North and East)
    dn, de = MAVLinCS.geographic_to_cartesian_point(lat0, lon0, pt.latitude, pt.longitude)
    assert math.isclose(dn, 100.0, rel_tol=1e-2)
    assert math.isclose(de, 50.0, rel_tol=1e-2)

    # Inverse calculation in South-West direction (negative displacements)
    dn_sw, de_sw = MAVLinCS.geographic_to_cartesian_point(pt.latitude, pt.longitude, lat0, lon0)
    assert math.isclose(dn_sw, -100.0, rel_tol=1e-2)
    assert math.isclose(de_sw, -50.0, rel_tol=1e-2)

    # Direct distance between the two GPS points (2D and 3D with alt and relative_alt)
    dist_2d = MAVLinCS.distance_between_two_gps_points(lat0, pt.latitude, lon0, pt.longitude)
    expected_2d = math.hypot(100.0, 50.0)
    assert math.isclose(dist_2d, expected_2d, rel_tol=1e-2)

    # Same latitude branch (abs(lat2 - lat1) < 1e-15)
    dist_same_lat = MAVLinCS.distance_between_two_gps_points(lat0, lat0, lon0, pt.longitude)
    assert math.isclose(dist_same_lat, 50.0, rel_tol=1e-2)

    # 3D with absolute altitude
    dist_3d_alt = MAVLinCS.distance_between_two_gps_points(
        lat0, pt.latitude, lon0, pt.longitude, alt_i=10.0, alt_f=60.0
    )
    assert math.isclose(dist_3d_alt, math.hypot(expected_2d, 50.0), rel_tol=1e-2)

    # 3D with relative altitude
    dist_3d_rel = MAVLinCS.distance_between_two_gps_points(
        lat0, pt.latitude, lon0, pt.longitude, relative_alt_i=0.0, relative_alt_f=50.0
    )
    assert math.isclose(dist_3d_rel, math.hypot(expected_2d, 50.0), rel_tol=1e-2)

    # Priority: alt takes precedence over relative_alt when both are provided
    dist_priority = MAVLinCS.distance_between_two_gps_points(
        lat0, pt.latitude, lon0, pt.longitude,
        relative_alt_i=0.0, relative_alt_f=500.0,
        alt_i=10.0, alt_f=60.0,
    )
    assert math.isclose(dist_priority, dist_3d_alt, rel_tol=1e-5)


def test_polar_projection():
    lat0, lon0 = 48.7098, 2.1355
    pt_north = MAVLinCS.polar_to_geographic_point(lat0, lon0, distance_m=50.0, bearing=0.0)
    assert pt_north.latitude > lat0
    assert math.isclose(pt_north.longitude, lon0, abs_tol=1e-5)


def test_mcc_registration_lookup_and_subclassing():
    code = MCC(value=9001, name="PYTEST_MCC_1", level="INFO", description="Unit test")
    assert MCC.get(9001) == code
    assert MCC.get_name("PYTEST_MCC_1") == code
    assert MCC.get(999999) == EMPTY
    assert MCC.get_name("NON_EXISTENT_MCC") == EMPTY
    assert int(code) == 9001
    assert code == 9001
    assert code == "PYTEST_MCC_1"
    assert (code == 3.14) is False
    assert repr(code) == "[INFO][PYTEST_MCC_1] Unit test"
    assert code in {code}

    # Check duplicate prevention
    with pytest.raises(ValueError):
        MCC(value=9001, name="OTHER_NAME", level="INFO", description="Duplicate value")
    with pytest.raises(ValueError):
        MCC(value=9002, name="PYTEST_MCC_1", level="INFO", description="Duplicate name")

    # Subclassing creates an isolated registry containing EMPTY
    class SubMCC(MCC):
        pass

    assert SubMCC.get(0) == EMPTY
    assert SubMCC.get_name("EMPTY") == EMPTY
    assert SubMCC.get(9001) == EMPTY  # Isolated from base MCC
    sub_code = SubMCC(value=9001, name="PYTEST_MCC_1", level="DEBUG", description="In subclass")
    assert SubMCC.get(9001) == sub_code


def test_statustext_receiver_chunk_reassembly():
    receiver = StatustextReceiver()
    key = (1, 1)
    assert receiver.last_statustext_from_sysidcompid(key) is None

    class DummyStatustext:
        def __init__(self, text, severity=6, mid=42, chunk_seq=0, timestamp=None):
            self.text = text
            self.severity = severity
            self.id = mid
            self.chunk_seq = chunk_seq
            self._timestamp = timestamp or time.time()

        def get_srcSystem(self):
            return 1

        def get_srcComponent(self):
            return 1

    # Chunk 0 (exact 50 chars -> expects more chunks)
    chunk0_text = "A" * 50
    msg0 = DummyStatustext(text=chunk0_text, mid=10, chunk_seq=0)
    assert receiver.parse(msg0) is None

    # Chunk 1 (< 50 chars -> completes the message)
    chunk1_text = "BCDEF"
    msg1 = DummyStatustext(text=chunk1_text, mid=10, chunk_seq=1)
    completed = receiver.parse(msg1)
    assert completed is not None
    assert completed.text == "A" * 50 + "BCDEF"
    assert receiver.last_statustext_from_sysidcompid(key) == completed

    # Duplicate message within 2 seconds is suppressed
    msg_dup = DummyStatustext(text="A" * 50 + "BCDEF", mid=0, chunk_seq=0)
    assert receiver.parse(msg_dup) is None


def test_mavlogger_and_setup_logger(tmp_path):
    # 1. Test MavLogger binary .tlog creation and writing (auto-adds .tlog on disk)
    tlog_base = str(tmp_path / "test_flight")
    mav_logger = MavLogger(file=tlog_base, relative_path_to_cwd=False)
    assert mav_logger.file is not None

    mav = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
    hb_msg = mav.heartbeat_encode(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )
    hb_msg.pack(mav)
    mav_logger.log(hb_msg)
    mav_logger.close()
    assert mav_logger.file is None

    tlog_path = tmp_path / "test_flight.tlog"
    assert tlog_path.exists()
    assert tlog_path.stat().st_size == 8 + len(hb_msg.get_msgbuf())

    # 2. Test setup_logger with file output and idempotency
    log_path = tmp_path / "unit_test.log"
    custom_logger = logging.getLogger("unit_test_logger_instance")
    custom_logger.handlers.clear()
    setup_logger(
        logger=custom_logger,
        log_file=str(log_path),
        relative_path_to_cwd=False,
        debug_mode=True,
        add_timestamp_to_console=True,
        add_timestamp_to_file=True,
    )
    # Calling setup_logger a second time should be a no-op (no duplicate handlers)
    handler_count = len(custom_logger.handlers)
    setup_logger(logger=custom_logger, log_file=str(log_path), relative_path_to_cwd=False)
    assert len(custom_logger.handlers) == handler_count

    custom_logger.info("Hello pyMAVLinCS logger")
    for h in custom_logger.handlers:
        h.flush()
    assert log_path.exists()
    assert "Hello pyMAVLinCS logger" in log_path.read_text(encoding="utf-8")


def test_offline_open_close_and_thread_lifecycle(tmp_path):
    # Test connection with timeout_heartbeat=0 (no SITL required), thread activity, close(), and re-open()
    mav = MAVLinCS(
        address="udpin:127.0.0.1:0",
        timeout_heartbeat=0,
        pos_rate=0,
        mavlogfile=None,
    )
    assert mav.mavthread.is_thread_active() is True
    assert mav.dialect == "ardupilotmega"
    # Before any message is received, getters return safe defaults (NaN / None / UNKNOWN)
    assert math.isnan(mav.position_gps().lat)
    assert math.isnan(mav.position_local().x)
    assert math.isnan(mav.angles().roll)
    assert math.isnan(mav.angular_rates().rollspeed)
    assert math.isnan(mav.speed().vx)
    assert math.isnan(mav.speed_module())
    assert math.isnan(mav.vfr_hud().airspeed)
    assert math.isnan(mav.airspeed())
    assert math.isnan(mav.groundspeed())
    assert math.isnan(mav.heading())
    assert math.isnan(mav.throttle())
    assert math.isnan(mav.climb_rate())
    assert math.isnan(mav.wind().speed)
    assert math.isnan(mav.nav_controller_output().xtrack_error)
    assert math.isnan(mav.ekf_status().flags)
    assert mav.airspeed_healthy() is None
    assert mav.motors_armed() is None
    assert mav.motors_disarmed() is None
    assert mav.mode() == "UNKNOWN"
    assert mav.autopilot_str() == "UNKNOWN"
    assert math.isnan(mav.autopilot_int())
    assert mav.vehicle_type_str() == "UNKNOWN"
    assert math.isnan(mav.vehicle_type_int())
    assert math.isnan(mav.vtol_state())
    assert mav.vtol_state_str() == "UNKNOWN"
    assert math.isnan(mav.landed_state())
    assert mav.landed_state_str() == "UNKNOWN"
    assert math.isnan(mav.custom_mode())
    assert math.isnan(mav.base_mode())
    assert math.isnan(mav.custom_main_mode())
    assert math.isnan(mav.custom_sub_mode())
    assert math.isnan(mav.battery_percentage())
    assert math.isnan(mav.battery_voltage())
    assert mav.gps_3d_fix() is None
    assert math.isnan(mav.servo_pwm(1))
    assert math.isnan(mav.gimbal_angles().pitch)
    assert math.isnan(mav.current_waypoint_seq())
    assert mav.is_waypoint_mission_completed() is None
    assert math.isnan(mav.home_position().latitude_home)
    assert math.isnan(mav.ekf_origin().latitude)
    assert math.isnan(mav.get_distance_with_gps_pos(0.0, 0.0))
    assert math.isnan(mav.get_distance_with_local_pos(0.0, 0.0))
    assert mav.mode_mapping() is None

    mav.close()
    assert mav.mavthread.is_thread_active() is False

    # Re-open connection explicitly via open()
    mav.open(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0)
    assert mav.mavthread.is_thread_active() is True
    mav.close()
    assert mav.mavthread.is_thread_active() is False

    # Context manager (__enter__ / __exit__) closes connection on normal exit and on exception
    with MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None) as ctx_mav:
        assert ctx_mav.mavthread.is_thread_active() is True
    assert ctx_mav.mavthread.is_thread_active() is False

    with pytest.raises(RuntimeError, match="boom"):
        with MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None) as err_mav:
            assert err_mav.mavthread.is_thread_active() is True
            raise RuntimeError("boom")
    assert err_mav.mavthread.is_thread_active() is False


def test_upload_mission_offline_handshake_and_retries():
    # Initialize an offline MAVLinCS instance without waiting for a heartbeat
    mav = MAVLinCS(
        address="udpin:127.0.0.1:0",
        timeout_heartbeat=0,
        pos_rate=0,
        mavlogfile=None,
    )
    try:
        # 1. Input validation edge cases
        assert mav.upload_mission([], clear_existing=False) is False
        assert mav.upload_mission([{"x": 1.0, "y": 2.0}], clear_existing=False) is False
        assert mav.upload_mission([(16, 1.0)], clear_existing=False) is False
        assert mav.upload_mission([12345], clear_existing=False) is False
        assert mav.upload_mission([(16, 0.0, 0.0, 10.0)], clear_existing=False, timeout=0) is False

        # 2. Simulate an autopilot responding to MISSION_COUNT & MISSION_ITEM_INT
        #    Including a simulated dropped first MISSION_COUNT to test automatic retry!
        sent_messages = []
        count_attempts = 0
        item1_attempts = 0
        ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)

        def push_incoming(msg):
            msg.pack(ap_mav)
            msg._timestamp = time.time()
            with mav.mavthread._lock:
                mav.mavthread._queue.append(msg)
                mav.mavthread._msg_received.set()

        def on_gcs_send(m):
            nonlocal count_attempts, item1_attempts
            sent_messages.append(m)
            mtype = m.get_type()
            if mtype == "MISSION_CLEAR_ALL":
                ack = ap_mav.mission_ack_encode(
                    mav.master.source_system,
                    mav.master.source_component,
                    mavutil.mavlink.MAV_MISSION_ACCEPTED,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
                push_incoming(ack)
            elif mtype == "MISSION_COUNT":
                count_attempts += 1
                # Drop the 1st MISSION_COUNT to trigger a retry, respond on the 2nd
                if count_attempts >= 2:
                    req0 = ap_mav.mission_request_encode(
                        mav.master.source_system,
                        mav.master.source_component,
                        0,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                    push_incoming(req0)
            elif mtype == "MISSION_ITEM_INT":
                if m.seq == 0:
                    req1 = ap_mav.mission_request_int_encode(
                        mav.master.source_system,
                        mav.master.source_component,
                        1,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                    push_incoming(req1)
                elif m.seq == 1:
                    item1_attempts += 1
                    # Drop the 1st transmission of seq=1 to trigger a retry, accept on the 2nd
                    if item1_attempts >= 2:
                        req2 = ap_mav.mission_request_int_encode(
                            mav.master.source_system,
                            mav.master.source_component,
                            2,
                            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                        )
                        push_incoming(req2)
                elif m.seq == 2:
                    ack = ap_mav.mission_ack_encode(
                        mav.master.source_system,
                        mav.master.source_component,
                        mavutil.mavlink.MAV_MISSION_ACCEPTED,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                    push_incoming(ack)

        mav.set_additional_send_callback(on_gcs_send)

        # Mix MissionItem, dict, and tuple formats in the same mission
        mixed_waypoints = [
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=48.0, y=2.0, z=0.0),
            {"command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, "lat": 48.0, "lon": 2.0, "alt": 10.0},
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0, 0.0),
        ]

        ok = mav.upload_mission(
            waypoints=mixed_waypoints,
            clear_existing=True,
            timeout=5.0,
            retry_timeout=0.05,
            max_retries=3,
        )
        assert ok is True
        assert count_attempts == 2
        assert item1_attempts == 2
    finally:
        mav.close()


def test_get_and_write_parameter_offline():
    mav = MAVLinCS(
        address="udpin:127.0.0.1:0",
        timeout_heartbeat=0,
        pos_rate=0,
        mavlogfile=None,
    )
    try:
        # 1. Input validation edge cases
        assert math.isnan(mav.get_parameter(""))
        assert math.isnan(mav.get_parameter("PARAM_NAME_TOO_LONG_17"))
        assert math.isnan(mav.get_parameter("SYSID_MYGCS", timeout=0))
        assert mav.write_parameter("", 1.0) is False
        assert mav.write_parameter("PARAM_NAME_TOO_LONG_17", 1.0) is False
        assert mav.write_parameter("SYSID_MYGCS", math.nan) is False
        assert mav.write_parameter("SYSID_MYGCS", math.inf) is False

        # 2. timeout_ack=0 sends PARAM_SET without waiting for confirmation
        assert mav.write_parameter("SYSID_MYGCS", 255, timeout_ack=0) is True

        # 3. Simulate autopilot parameter protocol with packet loss & read-only rejection
        ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        param_set_attempts = 0
        param_read_attempts = 0
        param_list_requested = False

        def push_param_value(name: str, val: float):
            pkt = ap_mav.param_value_encode(
                name.encode("ascii"),
                float(val),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                100,
                0,
            ).pack(ap_mav)
            msg = mav.master.mav.decode(bytearray(pkt))
            mav.master.post_message(msg)

        def on_gcs_send(m):
            nonlocal param_set_attempts, param_read_attempts, param_list_requested
            mtype = m.get_type()
            if mtype == "PARAM_REQUEST_LIST":
                param_list_requested = True
                push_param_value("RTL_ALT", 1500.0)
            elif mtype == "PARAM_SET":
                pid = (m.param_id.decode("ascii", errors="ignore") if isinstance(m.param_id, bytes) else str(m.param_id)).rstrip("\x00")
                if pid == "WPNAV_SPEED":
                    param_set_attempts += 1
                    # Drop the 1st PARAM_SET to trigger a retry, confirm on the 2nd
                    if param_set_attempts >= 2:
                        push_param_value("WPNAV_SPEED", m.param_value)
                elif pid == "READONLY_PARAM":
                    # Simulate ArduPilot rejecting a read-only parameter by returning old_value=10.0
                    push_param_value("READONLY_PARAM", 10.0)
            elif mtype == "PARAM_REQUEST_READ":
                pid = (m.param_id.decode("ascii", errors="ignore") if isinstance(m.param_id, bytes) else str(m.param_id)).rstrip("\x00")
                if pid == "BATT_LOW_VOLT":
                    param_read_attempts += 1
                    # Drop the 1st PARAM_REQUEST_READ to trigger a retry, reply on the 2nd
                    if param_read_attempts >= 2:
                        push_param_value("BATT_LOW_VOLT", 10.5)

        mav.set_additional_send_callback(on_gcs_send)

        # Write parameter with retry and confirmation
        assert mav.write_parameter(
            "WPNAV_SPEED",
            500.0,
            timeout_ack=2.0,
            retry_timeout=0.05,
            max_retries=3,
        ) is True
        assert param_set_attempts == 2

        # Write read-only parameter -> returns False because confirmed value (10.0) != requested (99.0)
        assert mav.write_parameter("READONLY_PARAM", 99.0, timeout_ack=1.0) is False

        # Read parameter with retry
        val = mav.get_parameter("BATT_LOW_VOLT", timeout=2.0, retry_timeout=0.05, max_retries=3)
        assert math.isclose(val, 10.5, rel_tol=1e-5)
        assert param_read_attempts == 2

        # Subsequent call with use_cache=True returns cached value from master.param_state without network request
        assert "BATT_LOW_VOLT" in mav.master.param_state[(1, 1)].params
        cached_val = mav.get_parameter("BATT_LOW_VOLT", use_cache=True)
        assert math.isclose(cached_val, 10.5, rel_tol=1e-5)
        assert param_read_attempts == 2

        # Request all parameters to populate cache
        assert mav.request_all_parameters() is True
        assert param_list_requested is True
        assert math.isclose(mav.get_parameter("RTL_ALT", use_cache=True), 1500.0, rel_tol=1e-5)
    finally:
        mav.close()


def test_landing_and_return_path_commands_offline():
    mav = MAVLinCS(
        address="udpin:127.0.0.1:0",
        timeout_heartbeat=0,
        pos_rate=0,
        mavlogfile=None,
    )
    try:
        # Invalid altitude validation on do_go_around
        assert mav.do_go_around(altitude=-5.0) is False
        assert mav.do_go_around(altitude=math.nan) is False
        assert mav.do_go_around(altitude=math.inf) is False

        # Simulate autopilot COMMAND_ACK responses
        ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        recorded_cmds = []

        def push_ack(command: int, result: int):
            ack = ap_mav.command_ack_encode(
                command,
                result,
                0,
                0,
                mav.master.source_system,
                mav.master.source_component,
            )
            ack.pack(ap_mav)
            ack._timestamp = time.time()
            with mav.mavthread._lock:
                mav.mavthread._queue.append(ack)
                mav.mavthread._msg_received.set()

        def on_gcs_send(m):
            if m.get_type() == "COMMAND_LONG":
                recorded_cmds.append((m.command, m.param1))
                if m.command == mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START:
                    push_ack(m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                elif m.command == mavutil.mavlink.MAV_CMD_DO_LAND_START:
                    push_ack(m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                elif m.command == mavutil.mavlink.MAV_CMD_DO_GO_AROUND:
                    # Accept non-zero altitude, refuse zero altitude to test both branches
                    res = (
                        mavutil.mavlink.MAV_RESULT_ACCEPTED
                        if m.param1 > 0
                        else mavutil.mavlink.MAV_RESULT_FAILED
                    )
                    push_ack(m.command, res)

        mav.set_additional_send_callback(on_gcs_send)

        assert mav.do_return_path_start(timeout_ack=1.0) is True
        assert mav.do_land_start(timeout_ack=1.0) is True
        assert mav.do_go_around(altitude=50.0, timeout_ack=1.0) is True
        assert mav.do_go_around(altitude=0.0, timeout_ack=1.0) is False

        assert recorded_cmds == [
            (mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START, 0.0),
            (mavutil.mavlink.MAV_CMD_DO_LAND_START, 0.0),
            (mavutil.mavlink.MAV_CMD_DO_GO_AROUND, 50.0),
            (mavutil.mavlink.MAV_CMD_DO_GO_AROUND, 0.0),
        ]
    finally:
        mav.close()


def test_land_vehicle_types_offline():
    """Tests land() offline across Multirotor (LAND), VTOL/QuadPlane (QLAND), and Fixed-Wing (DO_LAND_START -> AUTO / AUTOLAND fallback)."""
    scenarios = [
        # (mav_type, vtol_state, accept_do_land_start, accept_autoland, expected_result, expected_cmds, expected_final_mode)
        (
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_VTOL_STATE_MC,  # ArduCopter reports MAV_VTOL_STATE_MC in EXTENDED_SYS_STATE
            False,
            False,
            True,
            [(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 9.0)],  # Copter LAND = 9
            "LAND",
        ),
        (
            mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
            mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
            False,
            False,
            True,
            [(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 20.0)],  # Plane QLAND = 20
            "QLAND",
        ),
        (
            getattr(mavutil.mavlink, "MAV_TYPE_VTOL_TAILSITTER", mavutil.mavlink.MAV_TYPE_VTOL_RESERVED2),
            mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
            False,
            False,
            True,
            [(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 20.0)],  # Plane QLAND = 20 (mapped via Fixed-Wing fallback)
            "QLAND",
        ),
        (
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            mavutil.mavlink.MAV_VTOL_STATE_FW,  # QuadPlane with Q_MAV_TYPE=0 detected via EXTENDED_SYS_STATE
            False,
            False,
            True,
            [(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 20.0)],  # Plane QLAND = 20
            "QLAND",
        ),
        (
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
            True,  # Mission contains DO_LAND_START
            False,
            True,
            [(mavutil.mavlink.MAV_CMD_DO_LAND_START, 0.0)],  # DO_LAND_START -> AUTO (10)
            "AUTO",
        ),
        (
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
            False,  # No DO_LAND_START in mission -> fallback to AUTOLAND (26)
            True,
            True,
            [
                (mavutil.mavlink.MAV_CMD_DO_LAND_START, 0.0),
                (mavutil.mavlink.MAV_CMD_DO_SET_MODE, 26.0),
            ],
            "AUTOLAND",
        ),
        (
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
            False,  # Both DO_LAND_START and AUTOLAND refused -> returns False
            False,
            False,
            [
                (mavutil.mavlink.MAV_CMD_DO_LAND_START, 0.0),
                (mavutil.mavlink.MAV_CMD_DO_SET_MODE, 26.0),
            ],
            "GUIDED",
        ),
    ]

    for mav_type, vtol_state, accept_dls, accept_autoland, expected_ok, expected_cmds, expected_mode in scenarios:
        mav = MAVLinCS(
            address="udpin:127.0.0.1:0",
            timeout_heartbeat=0,
            pos_rate=0,
            mavlogfile=None,
        )
        try:
            mav.master.target_system = 1
            ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
            recorded_cmds = []

            def inject(msg):
                pkt = msg.pack(ap_mav)
                decoded = mav.master.mav.decode(bytearray(pkt))
                decoded._timestamp = time.time()
                mav.master.post_message(decoded)
                with mav.mavthread._lock:
                    mav.mavthread._queue.append(decoded)
                    mav.mavthread._msg_received.set()

            def send_hb(custom_mode: int, armed: bool):
                base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                if armed:
                    base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                inject(ap_mav.heartbeat_encode(
                    mav_type,
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    base_mode,
                    custom_mode,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                ))

            # Initial armed heartbeat in GUIDED mode (4 for Copter, 15 for Plane/VTOL)
            guided_mode_id = 4 if mav_type == mavutil.mavlink.MAV_TYPE_QUADROTOR else 15
            send_hb(guided_mode_id, armed=True)
            inject(ap_mav.extended_sys_state_encode(
                vtol_state,
                mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
            ))

            def on_gcs_send(m):
                if m.get_type() != "COMMAND_LONG":
                    return
                recorded_cmds.append((m.command, m.param2))
                if m.command == mavutil.mavlink.MAV_CMD_DO_LAND_START:
                    if accept_dls:
                        inject(ap_mav.command_ack_encode(
                            m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                            mav.master.source_system, mav.master.source_component,
                        ))
                        # Switch to AUTO (10) and simulate touchdown + disarm
                        send_hb(10, armed=False)
                    else:
                        inject(ap_mav.command_ack_encode(
                            m.command, mavutil.mavlink.MAV_RESULT_FAILED, 0, 0,
                            mav.master.source_system, mav.master.source_component,
                        ))
                elif m.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    target_mode = int(m.param2)
                    if target_mode == 26 and not accept_autoland:
                        inject(ap_mav.command_ack_encode(
                            m.command, mavutil.mavlink.MAV_RESULT_FAILED, 0, 0,
                            mav.master.source_system, mav.master.source_component,
                        ))
                    else:
                        inject(ap_mav.command_ack_encode(
                            m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                            mav.master.source_system, mav.master.source_component,
                        ))
                        send_hb(target_mode, armed=False)

            mav.set_additional_send_callback(on_gcs_send)

            result = mav.land(timeout_ack=1.0, timeout_set_mode=1.0, timeout_land=2.0)
            assert result is expected_ok
            assert recorded_cmds == expected_cmds
            assert mav.mode() == expected_mode
        finally:
            mav.close()


def test_takeoff_vehicle_types_offline():
    """Tests takeoff() offline across Multirotor (GUIDED+NAV_TAKEOFF), VTOL/QuadPlane (GUIDED+NAV_TAKEOFF/goto), and Fixed-Wing (TKOFF_ALT/TKOFF_LVL_PITCH + TAKEOFF mode)."""
    # 1. Invalid inputs
    mav_inv = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
    try:
        assert mav_inv.takeoff(0.0) is False
        assert mav_inv.takeoff(-5.0) is False
        assert mav_inv.takeoff(math.nan) is False
        assert mav_inv.takeoff(math.inf) is False
        assert mav_inv.takeoff(10.0, pitch=math.nan) is False
    finally:
        mav_inv.close()

    def run_takeoff_case(
        mav_type: int,
        vtol_state: int,
        landed_state: int,
        altitude: float,
        pitch: float,
        final_rel_alt: float,
        accept_tkoff_alt: bool = True,
        update_tkoff_params: bool = False,
    ):
        mav = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
        try:
            mav.master.target_system = 1
            ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
            recorded_cmds = []
            recorded_params = {}
            recorded_gotos = []

            def inject(msg):
                pkt = msg.pack(ap_mav)
                decoded = mav.master.mav.decode(bytearray(pkt))
                decoded._timestamp = time.time()
                mav.master.post_message(decoded)
                with mav.mavthread._lock:
                    mav.mavthread._queue.append(decoded)
                    mav.mavthread._msg_received.set()

            def send_hb(custom_mode: int):
                inject(ap_mav.heartbeat_encode(
                    mav_type,
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                    custom_mode,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                ))

            def send_gps(rel_alt_m: float):
                inject(ap_mav.global_position_int_encode(
                    1000, int(48.0 * 1e7), int(2.0 * 1e7), int(rel_alt_m * 1000), int(rel_alt_m * 1000), 0, 0, 0, 0
                ))

            # Initial heartbeat in STABILIZE (0 for Copter, 2 for Plane/VTOL) so mode switch is also verified
            init_mode = 0 if mav_type == mavutil.mavlink.MAV_TYPE_QUADROTOR else 2
            send_hb(init_mode)
            send_gps(0.0 if landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND else 10.0)
            inject(ap_mav.extended_sys_state_encode(vtol_state, landed_state))

            def on_gcs_send(m):
                mtype = m.get_type()
                if mtype == "PARAM_SET":
                    pid = (m.param_id.decode("ascii", errors="ignore") if isinstance(m.param_id, bytes) else str(m.param_id)).rstrip("\x00")
                    recorded_params[pid] = m.param_value
                    if pid == "TKOFF_ALT" and not accept_tkoff_alt:
                        inject(ap_mav.param_value_encode(pid.encode("ascii"), 50.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 100, 0))
                    else:
                        inject(ap_mav.param_value_encode(pid.encode("ascii"), m.param_value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 100, 0))
                elif mtype == "SET_POSITION_TARGET_GLOBAL_INT":
                    recorded_gotos.append(m.alt)
                    send_gps(final_rel_alt)
                elif mtype == "COMMAND_LONG":
                    recorded_cmds.append((m.command, m.param1, m.param2, m.param7))
                    if m.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                        target_mode = int(m.param2)
                        inject(ap_mav.command_ack_encode(
                            m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                            mav.master.source_system, mav.master.source_component,
                        ))
                        send_hb(target_mode)
                        if target_mode == 13:  # ArduPlane TAKEOFF mode
                            send_gps(final_rel_alt)
                    elif m.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                        inject(ap_mav.command_ack_encode(
                            m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                            mav.master.source_system, mav.master.source_component,
                        ))
                        send_gps(final_rel_alt)

            mav.set_additional_send_callback(on_gcs_send)
            ok = mav.takeoff(
                altitude=altitude,
                pitch=pitch,
                precision=1.0,
                timeout_ack=1.0,
                timeout_set_mode=1.0,
                timeout_takeoff=2.0,
                update_tkoff_params=update_tkoff_params,
            )
            return ok, mav.mode(), recorded_cmds, recorded_params, recorded_gotos
        finally:
            mav.close()

    # 2. Multirotor (ArduCopter reports MAV_VTOL_STATE_MC) on ground -> GUIDED + MAV_CMD_NAV_TAKEOFF
    ok, mode, cmds, params, gotos = run_takeoff_case(
        mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_VTOL_STATE_MC,
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, altitude=12.0, pitch=0.0, final_rel_alt=12.0
    )
    assert ok is True and mode == "GUIDED" and params == {} and gotos == []
    assert cmds == [
        (mavutil.mavlink.MAV_CMD_DO_SET_MODE, float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 4.0, 0.0),
        (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 12.0),
    ]

    # 3. VTOL QuadPlane (MAV_TYPE_VTOL_QUADROTOR) on ground -> GUIDED + MAV_CMD_NAV_TAKEOFF (pitch forced to 0)
    ok, mode, cmds, params, gotos = run_takeoff_case(
        mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR, mavutil.mavlink.MAV_VTOL_STATE_MC,
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, altitude=20.0, pitch=15.0, final_rel_alt=20.0
    )
    assert ok is True and mode == "GUIDED" and params == {} and gotos == []
    assert cmds == [
        (mavutil.mavlink.MAV_CMD_DO_SET_MODE, float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 15.0, 0.0),
        (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 20.0),
    ]

    # 4. QuadPlane reporting MAV_TYPE_FIXED_WING (Q_MAV_TYPE=0) + vtol_state=MC -> detected as VTOL -> GUIDED + NAV_TAKEOFF
    ok, mode, cmds, params, gotos = run_takeoff_case(
        mavutil.mavlink.MAV_TYPE_FIXED_WING, mavutil.mavlink.MAV_VTOL_STATE_MC,
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, altitude=25.0, pitch=0.0, final_rel_alt=25.0
    )
    assert ok is True and mode == "GUIDED" and params == {} and gotos == []
    assert cmds == [
        (mavutil.mavlink.MAV_CMD_DO_SET_MODE, float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 15.0, 0.0),
        (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 25.0),
    ]

    # 5. VTOL already airborne -> GUIDED + SET_POSITION_TARGET_GLOBAL_INT (no NAV_TAKEOFF)
    ok, mode, cmds, params, gotos = run_takeoff_case(
        mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR, mavutil.mavlink.MAV_VTOL_STATE_MC,
        mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, altitude=30.0, pitch=0.0, final_rel_alt=30.0
    )
    assert ok is True and mode == "GUIDED" and gotos == [30.0]
    assert cmds == [
        (mavutil.mavlink.MAV_CMD_DO_SET_MODE, float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 15.0, 0.0),
    ]

    # 6. Pure Fixed-Wing default (update_tkoff_params=False) -> does NOT modify TKOFF_ALT / TKOFF_LVL_PITCH, enters TAKEOFF mode
    ok, mode, cmds, params, gotos = run_takeoff_case(
        mavutil.mavlink.MAV_TYPE_FIXED_WING, mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, altitude=40.0, pitch=15.0, final_rel_alt=42.0
    )
    assert ok is True and mode == "TAKEOFF" and gotos == [] and params == {}
    assert cmds == [
        (mavutil.mavlink.MAV_CMD_DO_SET_MODE, float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 13.0, 0.0),
    ]

    # 7. Pure Fixed-Wing with update_tkoff_params=True & pitch=15 -> writes TKOFF_ALT & TKOFF_LVL_PITCH, enters TAKEOFF mode
    ok, mode, cmds, params, gotos = run_takeoff_case(
        mavutil.mavlink.MAV_TYPE_FIXED_WING, mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, altitude=40.0, pitch=15.0, final_rel_alt=42.0,
        update_tkoff_params=True
    )
    assert ok is True and mode == "TAKEOFF" and gotos == []
    assert params == {"TKOFF_ALT": 40.0, "TKOFF_LVL_PITCH": 15.0}

    # 8. Pure Fixed-Wing with update_tkoff_params=True & pitch=0 -> writes only TKOFF_ALT (preserves autopilot TKOFF_LVL_PITCH)
    ok, mode, cmds, params, gotos = run_takeoff_case(
        mavutil.mavlink.MAV_TYPE_FIXED_WING, mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, altitude=35.0, pitch=0.0, final_rel_alt=35.0,
        update_tkoff_params=True
    )
    assert ok is True and mode == "TAKEOFF" and params == {"TKOFF_ALT": 35.0}

    # 9. Pure Fixed-Wing with update_tkoff_params=True and negative pitch or refused TKOFF_ALT -> returns False
    ok, _, _, _, _ = run_takeoff_case(
        mavutil.mavlink.MAV_TYPE_FIXED_WING, mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, altitude=40.0, pitch=-5.0, final_rel_alt=40.0,
        update_tkoff_params=True
    )
    assert ok is False
    ok, _, _, _, _ = run_takeoff_case(
        mavutil.mavlink.MAV_TYPE_FIXED_WING, mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, altitude=40.0, pitch=15.0, final_rel_alt=40.0,
        accept_tkoff_alt=False, update_tkoff_params=True
    )
    assert ok is False


def test_vtol_state_and_transition_offline():
    """Tests vtol_state(), vtol_state_str(), and vtol_transition() offline across ArduPilot and PX4 behaviors."""
    mav = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
    try:
        mav.master.target_system = 1
        ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)

        def inject(msg):
            pkt = msg.pack(ap_mav)
            decoded = mav.master.mav.decode(bytearray(pkt))
            decoded._timestamp = time.time()
            mav.master.post_message(decoded)
            with mav.mavthread._lock:
                mav.mavthread._queue.append(decoded)
                mav.mavthread._msg_received.set()

        # 1. Verify vtol_state() and vtol_state_str() for all valid MAV_VTOL_STATE values
        for state_val, state_name in [
            (mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED, "MAV_VTOL_STATE_UNDEFINED"),
            (mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW, "MAV_VTOL_STATE_TRANSITION_TO_FW"),
            (mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_MC, "MAV_VTOL_STATE_TRANSITION_TO_MC"),
            (mavutil.mavlink.MAV_VTOL_STATE_MC, "MAV_VTOL_STATE_MC"),
            (mavutil.mavlink.MAV_VTOL_STATE_FW, "MAV_VTOL_STATE_FW"),
        ]:
            inject(ap_mav.extended_sys_state_encode(state_val, mavutil.mavlink.MAV_LANDED_STATE_IN_AIR))
            assert mav.vtol_state() == state_val
            assert mav.vtol_state_str() == state_name

        # 2. Invalid target states (including strings and bools) & invalid timeout_transition must be rejected before sending
        for bad_state in (
            mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
            mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW,
            mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_MC,
            99,
            "FW",
            "MC",
            "INVALID",
            True,
            False,
        ):
            assert mav.vtol_transition(bad_state) is False
        assert mav.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_FW, timeout_transition=-1.0) is False
        assert mav.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_FW, timeout_transition=math.nan) is False

        # 3. Simulate ArduPlane QuadPlane in AUTO mode
        inject(ap_mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            10,  # AUTO
            mavutil.mavlink.MAV_STATE_ACTIVE,
        ))
        inject(ap_mav.extended_sys_state_encode(
            mavutil.mavlink.MAV_VTOL_STATE_MC,
            mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
        ))

        recorded = []
        ack_result = [mavutil.mavlink.MAV_RESULT_ACCEPTED]
        auto_complete = [True]

        def on_gcs_send(m):
            if m.get_type() == "COMMAND_LONG" and m.command == mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION:
                recorded.append((m.param1, m.param2))
                inject(ap_mav.command_ack_encode(
                    m.command, ack_result[0], 0, 0,
                    mav.master.source_system, mav.master.source_component,
                ))
                if ack_result[0] == mavutil.mavlink.MAV_RESULT_ACCEPTED and auto_complete[0]:
                    target = int(m.param1)
                    in_prog = (
                        mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW
                        if target == mavutil.mavlink.MAV_VTOL_STATE_FW
                        else mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_MC
                    )
                    inject(ap_mav.extended_sys_state_encode(in_prog, mavutil.mavlink.MAV_LANDED_STATE_IN_AIR))
                    inject(ap_mav.extended_sys_state_encode(target, mavutil.mavlink.MAV_LANDED_STATE_IN_AIR))

        mav.set_additional_send_callback(on_gcs_send)

        # ArduPilot resets force_immediate (param2=0) + waits for FW completion
        assert mav.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_FW, force_immediate=True, timeout_ack=1.0, timeout_transition=2.0) is True
        assert mav.vtol_state() == mavutil.mavlink.MAV_VTOL_STATE_FW
        assert recorded[-1] == (float(mavutil.mavlink.MAV_VTOL_STATE_FW), 0.0)

        # Transition back to MC using enum int + timeout_transition=0 (immediate return after ACK)
        assert mav.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_MC, timeout_ack=1.0, timeout_transition=0) is True
        assert recorded[-1] == (float(mavutil.mavlink.MAV_VTOL_STATE_MC), 0.0)

        # Timeout while waiting for transition that never completes
        auto_complete[0] = False
        inject(ap_mav.extended_sys_state_encode(
            mavutil.mavlink.MAV_VTOL_STATE_MC,
            mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
        ))
        assert mav.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_FW, timeout_ack=1.0, timeout_transition=0.15) is False

        # Refused ACK returns False
        ack_result[0] = mavutil.mavlink.MAV_RESULT_FAILED
        assert mav.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_FW, timeout_ack=1.0) is False

        # 4. Simulate PX4 autopilot: force_immediate=True preserves param2=1.0
        ack_result[0] = mavutil.mavlink.MAV_RESULT_ACCEPTED
        auto_complete[0] = True
        inject(ap_mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_PX4,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        ))
        assert mav.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_MC, force_immediate=True, timeout_ack=1.0, timeout_transition=1.0) is True
        assert recorded[-1] == (float(mavutil.mavlink.MAV_VTOL_STATE_MC), 1.0)
    finally:
        mav.close()


def test_mode_and_mode_mapping_fallbacks_offline():
    """Tests mode() and mode_mapping() fallbacks for ArduPilot VTOL and Copter MAV_TYPEs not in pymavlink's default map."""
    cases = [
        (mavutil.mavlink.MAV_TYPE_VTOL_RESERVED2, 20, "QLAND", "AUTOLAND"),
    ]
    if hasattr(mavutil.mavlink, "MAV_TYPE_VTOL_GYRODYNE"):
        cases.append((mavutil.mavlink.MAV_TYPE_VTOL_GYRODYNE, 20, "QLAND", "AUTOLAND"))
    if hasattr(mavutil.mavlink, "MAV_TYPE_GENERIC_MULTIROTOR"):
        cases.append((mavutil.mavlink.MAV_TYPE_GENERIC_MULTIROTOR, 9, "LAND", "GUIDED"))

    for mav_type, custom_mode, expected_mode, extra_mode in cases:
        mav = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
        try:
            mav.master.target_system = 1
            ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)

            def inject(msg):
                pkt = msg.pack(ap_mav)
                decoded = mav.master.mav.decode(bytearray(pkt))
                decoded._timestamp = time.time()
                mav.master.post_message(decoded)

            # 1. HIGH_LATENCY2 fallback (before any HEARTBEAT is received)
            inject(ap_mav.high_latency2_encode(
                0, mav_type, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, custom_mode,
                *([0] * 23)
            ))
            assert mav.mode() == expected_mode

            # 2. HEARTBEAT mode() and mode_mapping()
            inject(ap_mav.heartbeat_encode(
                mav_type,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            ))
            assert mav.mode() == expected_mode
            mapping = mav.mode_mapping()
            assert mapping is not None
            assert expected_mode in mapping and extra_mode in mapping
            assert mapping[expected_mode] == custom_mode
        finally:
            mav.close()


def test_arm_non_armable_modes_offline():
    """Tests arm() mode checks for ArduPlane/QuadPlane (INITIALISING, QLAND, QRTL, AUTOTUNE) and ArduCopter (AUTO_RTL, AUTOTUNE, FLIP, SYSTEMID, AVOID_ADSB, AUTOROTATE)."""
    def run_arm_case(mav_type: int, init_mode_id: int):
        mav = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
        try:
            mav.master.target_system = 1
            ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
            recorded_cmds = []

            def inject(msg):
                pkt = msg.pack(ap_mav)
                decoded = mav.master.mav.decode(bytearray(pkt))
                decoded._timestamp = time.time()
                mav.master.post_message(decoded)
                with mav.mavthread._lock:
                    mav.mavthread._queue.append(decoded)
                    mav.mavthread._msg_received.set()

            def send_hb(custom_mode: int, armed: bool = False):
                base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                if armed:
                    base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                inject(ap_mav.heartbeat_encode(
                    mav_type,
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    base_mode,
                    custom_mode,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                ))

            current_mode = [init_mode_id]
            send_hb(init_mode_id, armed=False)

            def on_gcs_send(m):
                if m.get_type() != "COMMAND_LONG":
                    return
                recorded_cmds.append((m.command, m.param2))
                inject(ap_mav.command_ack_encode(
                    m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                    mav.master.source_system, mav.master.source_component,
                ))
                if m.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    current_mode[0] = int(m.param2)
                    send_hb(current_mode[0], armed=False)
                elif m.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    send_hb(current_mode[0], armed=True)

            mav.set_additional_send_callback(on_gcs_send)
            ok = mav.arm(timeout_ack=1.0, timeout_arming=1.0)
            return ok, mav.mode(), recorded_cmds
        finally:
            mav.close()

    # 1. ArduPlane INITIALISING (mode 16) -> blocked without sending any command
    ok, mode, cmds = run_arm_case(mavutil.mavlink.MAV_TYPE_FIXED_WING, 16)
    assert ok is False and mode == "INITIALISING" and cmds == []

    # 2. QuadPlane QLAND (20) and QRTL (21) -> switches to GUIDED (15) before arming
    for q_mode_id in (20, 21):
        ok, mode, cmds = run_arm_case(mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR, q_mode_id)
        assert ok is True and mode == "GUIDED"
        assert cmds == [
            (mavutil.mavlink.MAV_CMD_DO_SET_MODE, 15.0),
            (mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0),
        ]

    # 3. ArduCopter non-armable modes -> switches to GUIDED (4) before arming
    # AUTOTUNE(15), FLIP(14), SYSTEMID(25), AVOID_ADSB(19), AUTOROTATE(26), AUTO_RTL(27)
    for copter_mode_id in (14, 15, 19, 25, 26, 27):
        ok, mode, cmds = run_arm_case(mavutil.mavlink.MAV_TYPE_QUADROTOR, copter_mode_id)
        assert ok is True and mode == "GUIDED"
        assert cmds == [
            (mavutil.mavlink.MAV_CMD_DO_SET_MODE, 4.0),
            (mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0),
        ]

    # 4. ArduPlane AUTOTUNE (8) IS armable -> arms directly without switching to GUIDED
    ok, mode, cmds = run_arm_case(mavutil.mavlink.MAV_TYPE_FIXED_WING, 8)
    assert ok is True and mode == "AUTOTUNE"
    assert cmds == [(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)]


def test_aeronautical_and_flight_status_telemetry_offline():
    """Tests landed_state(), landed_state_str(), nav_controller_output(), airspeed_healthy(), and ekf_status() offline."""
    mav = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
    try:
        mav.master.target_system = 1
        ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)

        def inject(msg):
            pkt = msg.pack(ap_mav)
            decoded = mav.master.mav.decode(bytearray(pkt))
            decoded._timestamp = time.time()
            mav.master.post_message(decoded)

        # 1. landed_state() & landed_state_str()
        for val, name in [
            (mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED, "MAV_LANDED_STATE_UNDEFINED"),
            (mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, "MAV_LANDED_STATE_ON_GROUND"),
            (mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, "MAV_LANDED_STATE_IN_AIR"),
            (mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF, "MAV_LANDED_STATE_TAKEOFF"),
            (mavutil.mavlink.MAV_LANDED_STATE_LANDING, "MAV_LANDED_STATE_LANDING"),
        ]:
            inject(ap_mav.extended_sys_state_encode(mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED, val))
            assert mav.landed_state() == val
            assert mav.landed_state_str() == name

        # 2. nav_controller_output()
        inject(ap_mav.nav_controller_output_encode(12.5, -3.25, 90, 95, 250, 1.5, -0.75, 4.25))
        nav = mav.nav_controller_output()
        assert math.isclose(nav.nav_roll, 12.5, rel_tol=1e-5)
        assert math.isclose(nav.nav_pitch, -3.25, rel_tol=1e-5)
        assert nav.nav_bearing == 90
        assert nav.target_bearing == 95
        assert nav.wp_dist == 250
        assert math.isclose(nav.alt_error, 1.5, rel_tol=1e-5)
        assert math.isclose(nav.aspd_error, -0.75, rel_tol=1e-5)
        assert math.isclose(nav.xtrack_error, 4.25, rel_tol=1e-5)

        # 3. airspeed_healthy(): not present -> None, present & healthy -> True, present & unhealthy -> False
        bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
        inject(ap_mav.sys_status_encode(0, 0, 0, 0, 12000, -1, 100, 0, 0, 0, 0, 0, 0))
        assert mav.airspeed_healthy() is None
        inject(ap_mav.sys_status_encode(bit, bit, bit, 0, 12000, -1, 100, 0, 0, 0, 0, 0, 0))
        assert mav.airspeed_healthy() is True
        inject(ap_mav.sys_status_encode(bit, bit, 0, 0, 12000, -1, 100, 0, 0, 0, 0, 0, 0))
        assert mav.airspeed_healthy() is False

        # 4. ekf_status()
        ekf_flags = (
            mavutil.mavlink.EKF_ATTITUDE
            | mavutil.mavlink.EKF_VELOCITY_HORIZ
            | mavutil.mavlink.EKF_POS_HORIZ_ABS
        )
        inject(ap_mav.ekf_status_report_encode(ekf_flags, 0.12, 0.08, 0.05, 0.03, 0.0))
        ekf = mav.ekf_status()
        assert ekf.flags == ekf_flags
        assert math.isclose(ekf.velocity_variance, 0.12, rel_tol=1e-5)
        assert math.isclose(ekf.pos_horiz_variance, 0.08, rel_tol=1e-5)
        assert math.isclose(ekf.pos_vert_variance, 0.05, rel_tol=1e-5)
        assert math.isclose(ekf.compass_variance, 0.03, rel_tol=1e-5)
        assert math.isclose(ekf.terrain_alt_variance, 0.0, abs_tol=1e-6)
    finally:
        mav.close()


def test_return_to_launch_vehicle_types_offline():
    """Tests return_to_launch() offline across Copter (RTL), Fixed-Wing (RTL), ArduPlane QuadPlane (QRTL / use_qrtl=False), and PX4 VTOL (RTL)."""
    cases = [
        # (autopilot, mav_type, vtol_state, use_qrtl, expected_cmd, expected_param2, expected_mode)
        (
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_VTOL_STATE_MC,
            True,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0.0,
            "GUIDED",
        ),
        (
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
            True,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0.0,
            "GUIDED",
        ),
        (
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            True,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            21.0,  # Plane QRTL = 21
            "QRTL",
        ),
        (
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            mavutil.mavlink.MAV_VTOL_STATE_FW,  # QuadPlane with Q_MAV_TYPE=0
            True,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            21.0,
            "QRTL",
        ),
        (
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            False,  # Explicit standard RTL
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0.0,
            "GUIDED",
        ),
        (
            mavutil.mavlink.MAV_AUTOPILOT_PX4,
            mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            True,  # PX4 uses MAV_CMD_NAV_RETURN_TO_LAUNCH
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0.0,
            None,
        ),
    ]

    for autopilot, mav_type, vtol_state, use_qrtl, exp_cmd, exp_p2, exp_mode in cases:
        mav = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
        try:
            mav.master.target_system = 1
            ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
            recorded = []

            def inject(msg):
                pkt = msg.pack(ap_mav)
                decoded = mav.master.mav.decode(bytearray(pkt))
                decoded._timestamp = time.time()
                mav.master.post_message(decoded)
                with mav.mavthread._lock:
                    mav.mavthread._queue.append(decoded)
                    mav.mavthread._msg_received.set()

            init_mode = 4 if mav_type == mavutil.mavlink.MAV_TYPE_QUADROTOR else 15
            inject(ap_mav.heartbeat_encode(
                mav_type,
                autopilot,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                init_mode,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            ))
            inject(ap_mav.extended_sys_state_encode(vtol_state, mavutil.mavlink.MAV_LANDED_STATE_IN_AIR))

            def on_gcs_send(m):
                if m.get_type() != "COMMAND_LONG":
                    return
                recorded.append((m.command, m.param2))
                inject(ap_mav.command_ack_encode(
                    m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                    mav.master.source_system, mav.master.source_component,
                ))
                if m.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    inject(ap_mav.heartbeat_encode(
                        mav_type,
                        autopilot,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                        int(m.param2),
                        mavutil.mavlink.MAV_STATE_ACTIVE,
                    ))

            mav.set_additional_send_callback(on_gcs_send)
            assert mav.return_to_launch(use_qrtl=use_qrtl, timeout_ack=1.0, timeout_set_mode=1.0) is True
            assert recorded == [(exp_cmd, exp_p2)]
            if exp_mode is not None:
                assert mav.mode() == exp_mode
        finally:
            mav.close()


def test_download_mission_offline_handshake_and_retries():
    """Tests download_mission() offline: zero timeout, empty mission, packet-loss retries, global/local/legacy item unscaling, and error ACK."""
    mav = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
    try:
        mav.master.target_system = 1
        assert mav.download_mission(timeout=0) is None

        ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)

        def push_incoming(msg):
            msg.pack(ap_mav)
            msg._timestamp = time.time()
            with mav.mavthread._lock:
                mav.mavthread._queue.append(msg)
                mav.mavthread._msg_received.set()

        # 1. Empty mission (count=0) -> returns [] and sends MISSION_ACK(ACCEPTED)
        acks_sent = []

        def on_send_empty(m):
            mtype = m.get_type()
            if mtype == "MISSION_REQUEST_LIST":
                push_incoming(ap_mav.mission_count_encode(
                    mav.master.source_system, mav.master.source_component, 0
                ))
            elif mtype == "MISSION_ACK":
                acks_sent.append(m.type)

        mav.set_additional_send_callback(on_send_empty)
        assert mav.download_mission(timeout=2.0, retry_timeout=0.05) == []
        assert acks_sent == [mavutil.mavlink.MAV_MISSION_ACCEPTED]

        # 2. Full download with retries + global INT (1e7), local INT (1e4), and legacy MISSION_ITEM
        list_attempts = 0
        seq1_attempts = 0
        acks_sent.clear()

        def on_send_full(m):
            nonlocal list_attempts, seq1_attempts
            mtype = m.get_type()
            if mtype == "MISSION_REQUEST_LIST":
                list_attempts += 1
                if list_attempts >= 2:  # Drop 1st request to verify retry
                    push_incoming(ap_mav.mission_count_encode(
                        mav.master.source_system, mav.master.source_component, 3
                    ))
            elif mtype == "MISSION_REQUEST_INT":
                if m.seq == 0:
                    push_incoming(ap_mav.mission_item_int_encode(
                        mav.master.source_system, mav.master.source_component, 0,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                        1.0, 2.0, 3.0, 4.0,
                        int(48.1234567 * 1e7), int(2.7654321 * 1e7), 50.0,
                    ))
                elif m.seq == 1:
                    seq1_attempts += 1
                    if seq1_attempts >= 2:  # Drop 1st request for seq 1 to verify retry
                        push_incoming(ap_mav.mission_item_int_encode(
                            mav.master.source_system, mav.master.source_component, 1,
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                            0.0, 0.0, 0.0, 0.0,
                            int(12.5 * 1e4), int(-8.25 * 1e4), -20.0,
                        ))
                elif m.seq == 2:
                    # Legacy MISSION_ITEM (already float, no scaling)
                    push_incoming(ap_mav.mission_item_encode(
                        mav.master.source_system, mav.master.source_component, 2,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 1,
                        0.0, 0.0, 0.0, 0.0,
                        48.5, 2.5, 0.0,
                    ))
            elif mtype == "MISSION_ACK":
                acks_sent.append(m.type)

        mav.set_additional_send_callback(on_send_full)
        downloaded = mav.download_mission(timeout=5.0, retry_timeout=0.05, max_retries=3)
        assert downloaded is not None and len(downloaded) == 3
        assert list_attempts == 2 and seq1_attempts == 2
        assert acks_sent == [mavutil.mavlink.MAV_MISSION_ACCEPTED]

        assert downloaded[0].command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        assert math.isclose(downloaded[0].x, 48.1234567, abs_tol=1e-6)
        assert math.isclose(downloaded[0].y, 2.7654321, abs_tol=1e-6)
        assert math.isclose(downloaded[0].z, 50.0, rel_tol=1e-5)
        assert downloaded[0].param1 == 1.0 and downloaded[0].param4 == 4.0

        assert downloaded[1].frame == mavutil.mavlink.MAV_FRAME_LOCAL_NED
        assert math.isclose(downloaded[1].x, 12.5, abs_tol=1e-4)
        assert math.isclose(downloaded[1].y, -8.25, abs_tol=1e-4)
        assert math.isclose(downloaded[1].z, -20.0, rel_tol=1e-5)

        assert downloaded[2].command == mavutil.mavlink.MAV_CMD_DO_LAND_START
        assert math.isclose(downloaded[2].x, 48.5, abs_tol=1e-5)
        assert math.isclose(downloaded[2].y, 2.5, abs_tol=1e-5)

        # 3. Error MISSION_ACK -> returns None
        def on_send_error(m):
            if m.get_type() == "MISSION_REQUEST_LIST":
                push_incoming(ap_mav.mission_ack_encode(
                    mav.master.source_system, mav.master.source_component,
                    mavutil.mavlink.MAV_MISSION_ERROR,
                ))

        mav.set_additional_send_callback(on_send_error)
        assert mav.download_mission(timeout=2.0, retry_timeout=0.05) is None

        # 4. No response (max_retries exceeded) -> returns None
        mav.set_additional_send_callback(lambda m: None)
        assert mav.download_mission(timeout=2.0, retry_timeout=0.02, max_retries=1) is None
    finally:
        mav.close()


def test_safety_guards_and_invalid_inputs(drone):
    # PWM out of bounds [1000, 2000]
    assert drone.set_servo(servo_channel=9, pwm=950) is False
    assert drone.set_servo(servo_channel=9, pwm=2050) is False

    # NaN coordinates in set_gimbal_target and invalid inputs in change_speed
    assert drone.set_gimbal_target(math.nan, 2.0, 10.0) is False
    assert drone.change_speed(math.nan) is False
    assert drone.change_speed(5.0, speed_type=99) is False
    assert drone.change_speed(speed=-1.0, throttle=50.0) is False

    # Invalid flight mode names and tuples
    assert drone.set_mode("UNKNOWN_MODE_XYZ") is False
    assert drone.set_mode((1, 2)) is False
    assert drone.wait_mode_changed("UNKNOWN_MODE_XYZ", timeout=1) is False
    assert drone.wait_mode_changed((1, 2), timeout=1) is False

    # Negative or zero message frequency, or invalid message type
    assert drone.set_msg_rate("ATTITUDE", rate=0) is False
    assert drone.set_msg_rate("ATTITUDE", rate=-5) is False
    assert drone.set_msg_rate("UNKNOWN_MAV_MSG_XYZ", rate=2) is False

    # Zero-timeout early returns on all wait_* methods
    assert drone.wait_motors_armed(timeout=0) is None
    assert drone.wait_motors_disarmed(timeout=0) is None
    assert drone.wait_mode_changed("GUIDED", timeout=0) is None
    assert drone.wait_gps_3d_fix(timeout=0) is None
    assert drone.wait_proximity_with_gps_pos(0.0, 0.0, timeout=0) is None
    assert drone.wait_proximity_with_local_pos(0.0, 0.0, timeout=0) is None
    assert math.isnan(drone.wait_for_home(timeout=0).latitude_home)
    assert drone.wait_command_ack(command=400, timeout=0) is None
    assert drone.wait_mission_ack(timeout=0) is None

    # Test blocking via allow_actions = False
    drone.allow_actions = False
    assert drone.arm() is False
    assert drone.disarm() is False
    assert drone.takeoff(10.0) is False
    assert drone.land() is False
    assert drone.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_FW) is False
    assert drone.return_to_launch() is False
    assert drone.do_land_start() is False
    assert drone.do_go_around() is False
    assert drone.do_return_path_start() is False
    assert drone.set_mode("GUIDED") is False
    assert drone.set_gps_pos_target(0.0, 0.0, 10.0) is False
    assert drone.do_reposition(0.0, 0.0, 10.0) is False
    assert drone.set_local_pos_target(0.0, 0.0, -10.0) is False
    assert drone.set_speed(1.0, 0.0, 0.0) is False
    assert drone.change_speed(5.0) is False
    assert drone.set_servo(9, 1500) is False
    assert drone.set_gimbal_target(0.0, 0.0, 10.0) is False
    assert drone.set_gimbal_angles(-45.0, 0.0) is False
    assert drone.set_gimbal_neutral() is False
    assert drone.set_gimbal_retract() is False
    assert drone.write_parameter("SYSID_MYGCS", 255, mavutil.mavlink.MAV_PARAM_TYPE_INT16) is False
    assert drone.send_waypoint_count(1) is False
    assert drone.send_waypoint(0) is False
    assert drone.clear_waypoints() is False
    assert drone.upload_mission([(16, 0.0, 0.0, 10.0)]) is False
    assert drone.start_mission() is False
    assert drone.reboot() is False
    assert drone.do_follow(target_system=2) is None
    assert drone.do_follow_with_takeoff(target_system=2) is None
    assert drone.send_landing_target(0.0, 0.0, 1.0) is None

    # Test blocking via mission_ended = True
    drone.allow_actions = True
    drone.mission_ended = True
    assert drone.arm() is False
    assert drone.disarm() is False
    assert drone.takeoff(10.0) is False
    assert drone.land() is False
    assert drone.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_FW) is False
    assert drone.return_to_launch() is False
    assert drone.do_land_start() is False
    assert drone.do_go_around() is False
    assert drone.do_return_path_start() is False
    assert drone.do_follow(target_system=2) is None
    assert drone.do_follow_with_takeoff(target_system=2) is None
    assert drone.send_landing_target(0.0, 0.0, 1.0) is None
    assert drone.set_home(0.0, 0.0, 0.0) is False
    assert drone.set_home_current_pos() is False
    assert drone.set_ekf_origin(0.0, 0.0, 0.0) is False
    assert math.isnan(drone.request_home_position().latitude_home)
    assert math.isnan(drone.get_parameter("SYSID_MYGCS"))
    assert drone.download_mission() is None
    assert drone.send_timesync() is False
    assert drone.send_statustext("Should fail") is False
    assert drone.send_named_value_int(1, "TEST") is False
    assert drone.send_mcc(MCC.get(9001)) is False
    assert drone.wait_motors_armed(timeout=1) is None
    assert drone.wait_motors_disarmed(timeout=1) is None
    assert drone.wait_mode_changed("GUIDED", timeout=1) is None
    assert drone.wait_gps_3d_fix(timeout=1) is None
    assert drone.wait_proximity_with_gps_pos(0.0, 0.0, timeout=1) is None
    assert drone.wait_proximity_with_local_pos(0.0, 0.0, timeout=1) is None
    assert math.isnan(drone.wait_for_home(timeout=1).latitude_home)
    assert drone.wait_command_ack(command=400, timeout=1) is None
    assert drone.wait_mission_ack(timeout=1) is None
    assert drone.recv_match(type="HEARTBEAT", blocking=True, timeout=1) is None
