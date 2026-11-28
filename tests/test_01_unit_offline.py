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
                    req3 = ap_mav.mission_request_int_encode(
                        mav.master.source_system,
                        mav.master.source_component,
                        3,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                    push_incoming(req3)
                elif m.seq == 3:
                    ack = ap_mav.mission_ack_encode(
                        mav.master.source_system,
                        mav.master.source_component,
                        mavutil.mavlink.MAV_MISSION_ACCEPTED,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                    push_incoming(ack)

        mav.set_additional_send_callback(on_gcs_send)

        # Mix MissionItem, dict, and tuple formats (without any manual seq 0 HOME; ArduPilot seq 0 is prepended automatically)
        mixed_waypoints = [
            {"command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, "lat": 48.0, "lon": 2.0, "alt": 10.0},
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=48.001, y=2.001, z=15.0),
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
        # Verify MISSION_COUNT was 4 on ArduPilot (1 auto-prepended seq 0 HOME + 3 real items)
        sent_counts = [m.count for m in sent_messages if m.get_type() == "MISSION_COUNT"]
        assert sent_counts[-1] == 4
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
                        mav.master.source_system, mav.master.source_component, 4
                    ))
            elif mtype == "MISSION_REQUEST_INT":
                if m.seq == 0:
                    # ArduPilot wire seq 0 = HOME (automatically stripped by download_mission)
                    push_incoming(ap_mav.mission_item_int_encode(
                        mav.master.source_system, mav.master.source_component, 0,
                        mavutil.mavlink.MAV_FRAME_GLOBAL,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                        0.0, 0.0, 0.0, 0.0,
                        int(48.0 * 1e7), int(2.0 * 1e7), 120.0,
                    ))
                elif m.seq == 1:
                    seq1_attempts += 1
                    if seq1_attempts >= 2:  # Drop 1st request for seq 1 to verify retry
                        push_incoming(ap_mav.mission_item_int_encode(
                            mav.master.source_system, mav.master.source_component, 1,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                            1.0, 2.0, 3.0, 4.0,
                            int(48.1234567 * 1e7), int(2.7654321 * 1e7), 50.0,
                        ))
                elif m.seq == 2:
                    push_incoming(ap_mav.mission_item_int_encode(
                        mav.master.source_system, mav.master.source_component, 2,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                        0.0, 0.0, 0.0, 0.0,
                        int(12.5 * 1e4), int(-8.25 * 1e4), -20.0,
                    ))
                elif m.seq == 3:
                    # Legacy MISSION_ITEM (already float, no scaling)
                    push_incoming(ap_mav.mission_item_encode(
                        mav.master.source_system, mav.master.source_component, 3,
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


def test_set_current_waypoint_offline():
    """Tests set_current_waypoint() offline: input validation, ArduPilot vs PX4 seq=-1, reset flag, and denied ACK."""
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

        # Simulate ArduPlane autopilot
        inject(ap_mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            10,  # AUTO
            mavutil.mavlink.MAV_STATE_ACTIVE,
        ))

        # 1. Invalid inputs rejected before sending
        for bad in (-2, "3", True, False, 1.5):
            assert mav.set_current_waypoint(bad) is False
        assert mav.set_current_waypoint(-1, reset=False) is False
        assert mav.set_current_waypoint(-1, reset=True) is False  # Rejected on ArduPilot

        recorded = []

        def on_gcs_send(m):
            if m.get_type() == "COMMAND_LONG" and m.command == mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
                recorded.append((m.param1, m.param2))
                res = mavutil.mavlink.MAV_RESULT_DENIED if m.param1 >= 10 else mavutil.mavlink.MAV_RESULT_ACCEPTED
                inject(ap_mav.command_ack_encode(
                    m.command, res, 0, 0,
                    mav.master.source_system, mav.master.source_component,
                ))

        mav.set_additional_send_callback(on_gcs_send)

        # 2. Valid jumps & reset flag
        assert mav.set_current_waypoint(3, timeout_ack=1.0) is True
        assert mav.set_current_waypoint(1, reset=True, timeout_ack=1.0) is True
        # Out-of-range sequence denied by autopilot
        assert mav.set_current_waypoint(99, timeout_ack=1.0) is False

        # 3. PX4 allows seq=-1 with reset=True
        inject(ap_mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            mavutil.mavlink.MAV_AUTOPILOT_PX4,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        ))
        assert mav.set_current_waypoint(-1, reset=True, timeout_ack=1.0) is True

        assert recorded == [
            (3.0, 0.0),
            (1.0, 1.0),
            (99.0, 0.0),
            (-1.0, 1.0),
        ]
    finally:
        mav.close()


def test_rally_points_and_geofence_offline():
    """Tests upload_rally_points(), download_rally_points(), upload_geofence(), and download_geofence() offline."""
    mav = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
    try:
        mav.master.target_system = 1

        # 1. Validation edge cases
        assert mav.upload_rally_points([]) is False
        assert mav.upload_rally_points([(48.0, 2.0)]) is False  # Bad tuple length
        assert mav.upload_rally_points([MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=48.0, y=2.0, z=50.0)]) is False
        assert mav.upload_rally_points([(48.0, 2.0, 50.0, mavutil.mavlink.MAV_FRAME_LOCAL_NED)]) is False

        assert mav.upload_geofence([]) is False
        assert mav.upload_geofence([{"lat": 48.0, "lon": 2.0}]) is False  # Missing command
        assert mav.upload_geofence([MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=48.0, y=2.0, z=0.0)]) is False
        assert mav.upload_geofence([
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=2, x=48.0, y=2.0, z=0.0)
        ]) is False  # < 3 vertices
        assert mav.upload_geofence([
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, param1=0.0, x=48.0, y=2.0, z=0.0)
        ]) is False  # radius <= 0
        assert mav.upload_geofence([
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT, frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED, x=0.0, y=0.0, z=0.0)
        ]) is False  # Non-global frame

        # 2. Simulate full MAVLink 2 mission/fence/rally storage on autopilot
        ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        storage: dict[int, list] = {
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE: [],
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY: [],
        }
        expected_count: dict[int, int] = {}

        def push(msg):
            msg.pack(ap_mav)
            msg._timestamp = time.time()
            with mav.mavthread._lock:
                mav.mavthread._queue.append(msg)
                mav.mavthread._msg_received.set()

        def on_send(m):
            mtype = m.get_type()
            mt = getattr(m, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            if mtype == "MISSION_CLEAR_ALL":
                storage[mt] = []
                push(ap_mav.mission_ack_encode(
                    mav.master.source_system, mav.master.source_component,
                    mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type=mt,
                ))
            elif mtype == "MISSION_COUNT":
                expected_count[mt] = m.count
                storage[mt] = []
                if m.count > 0:
                    push(ap_mav.mission_request_int_encode(
                        mav.master.source_system, mav.master.source_component, 0, mission_type=mt,
                    ))
                else:
                    push(ap_mav.mission_ack_encode(
                        mav.master.source_system, mav.master.source_component,
                        mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type=mt,
                    ))
            elif mtype == "MISSION_ITEM_INT":
                storage[mt].append(m)
                if m.seq + 1 < expected_count[mt]:
                    push(ap_mav.mission_request_int_encode(
                        mav.master.source_system, mav.master.source_component, m.seq + 1, mission_type=mt,
                    ))
                else:
                    push(ap_mav.mission_ack_encode(
                        mav.master.source_system, mav.master.source_component,
                        mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type=mt,
                    ))
            elif mtype == "MISSION_REQUEST_LIST":
                push(ap_mav.mission_count_encode(
                    mav.master.source_system, mav.master.source_component, len(storage.get(mt, [])), mission_type=mt,
                ))
            elif mtype == "MISSION_REQUEST_INT":
                stored = storage[mt][m.seq]
                push(ap_mav.mission_item_int_encode(
                    mav.master.source_system, mav.master.source_component, m.seq,
                    stored.frame, stored.command, 0, 1,
                    stored.param1, stored.param2, stored.param3, stored.param4,
                    stored.x, stored.y, stored.z, mission_type=mt,
                ))

        mav.set_additional_send_callback(on_send)

        # 3. Upload & download Rally Points (tuple + dict + MissionItem)
        rally_in = [
            (48.1, 2.1, 60.0),
            {"lat": 48.2, "lon": 2.2, "alt": 75.0},
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT, x=48.3, y=2.3, z=80.0),
        ]
        assert mav.upload_rally_points(rally_in, clear_existing=True, timeout=5.0) is True
        rally_out = mav.download_rally_points(timeout=5.0)
        assert rally_out is not None and len(rally_out) == 3
        assert all(r.command == mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT for r in rally_out)
        assert math.isclose(rally_out[0].x, 48.1, abs_tol=1e-6) and math.isclose(rally_out[2].z, 80.0, rel_tol=1e-5)

        # 4. Upload & download Geofence (polygon inclusion + exclusion circle + return point)
        fence_in = [
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT, x=48.0, y=2.0, z=0.0),
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=3, x=48.01, y=2.01, z=0.0),
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=3, x=48.01, y=2.02, z=0.0),
            MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=3, x=48.02, y=2.015, z=0.0),
            {"command": mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, "param1": 45.0, "lat": 48.012, "lon": 2.015},
        ]
        assert mav.upload_geofence(fence_in, clear_existing=True, timeout=5.0) is True
        fence_out = mav.download_geofence(timeout=5.0)
        assert fence_out is not None and len(fence_out) == 5
        assert fence_out[0].command == mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT
        assert fence_out[4].command == mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
        assert math.isclose(fence_out[4].param1, 45.0, rel_tol=1e-5)
    finally:
        mav.close()


def test_loiter_altitude_and_heading_commands_offline():
    """Tests loiter_here(), change_altitude(), and change_heading() offline across ArduPlane and ArduCopter."""
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

        # ArduPlane in GUIDED mode with valid GPS position
        inject(ap_mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            15,  # GUIDED
            mavutil.mavlink.MAV_STATE_ACTIVE,
        ))
        inject(ap_mav.global_position_int_encode(
            1000, int(48.1 * 1e7), int(2.2 * 1e7), int(120.0 * 1000), int(55.0 * 1000), 0, 0, 0, 0
        ))

        # 1. Invalid inputs rejected before sending
        assert mav.loiter_here(radius=-10.0) is False
        assert mav.loiter_here(radius=math.nan) is False
        assert mav.loiter_here(altitude=math.inf) is False

        assert mav.change_altitude(math.nan) is False
        assert mav.change_altitude(50.0, frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED) is False
        assert mav.change_altitude(50.0, rate=-1.0) is False
        assert mav.change_altitude(0.0, rate=2.0) is False
        assert mav.change_altitude(-1.0, rate=2.0) is False

        assert mav.change_heading(-1.0) is False
        assert mav.change_heading(360.0) is False
        assert mav.change_heading(math.nan) is False
        assert mav.change_heading(90.0, heading_type=99) is False
        assert mav.change_heading(90.0, heading_type=True) is False
        assert mav.change_heading(90.0, max_accel=-1.0) is False

        recorded = []

        def on_send(m):
            mtype = m.get_type()
            if mtype == "COMMAND_LONG":
                recorded.append(("LONG", m.command, m.param1, m.param2, m.param3, m.param4))
                inject(ap_mav.command_ack_encode(
                    m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                    mav.master.source_system, mav.master.source_component,
                ))
            elif mtype == "COMMAND_INT":
                recorded.append(("INT", m.command, m.frame, m.param1, m.param2, m.param3, m.param4, m.x, m.y, m.z))
                inject(ap_mav.command_ack_encode(
                    m.command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                    mav.master.source_system, mav.master.source_component,
                ))

        mav.set_additional_send_callback(on_send)

        # 2. loiter_here(): default -> NAV_LOITER_UNLIM; custom radius/ccw/alt -> DO_REPOSITION at current GPS
        assert mav.loiter_here(timeout_ack=1.0) is True
        assert recorded[-1][0:2] == ("LONG", mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)

        assert mav.loiter_here(radius=75.0, ccw=True, altitude=65.0, timeout_ack=1.0) is True
        assert recorded[-1][0:2] == ("INT", mavutil.mavlink.MAV_CMD_DO_REPOSITION)
        assert recorded[-1][5] == 75.0  # param3 = loiter_radius
        assert recorded[-1][6] == 1.0   # param4 = ccw
        assert recorded[-1][7] == int(48.1 * 1e7) and recorded[-1][8] == int(2.2 * 1e7)
        assert recorded[-1][9] == 65.0  # z = altitude

        # 3. change_altitude() on ArduPlane: default -> DO_CHANGE_ALTITUDE; with rate -> GUIDED_CHANGE_ALTITUDE
        assert mav.change_altitude(80.0, timeout_ack=1.0) is True
        assert recorded[-1] == ("LONG", mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE, 80.0, float(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT), 0.0, 0.0)

        assert mav.change_altitude(90.0, rate=2.5, timeout_ack=1.0) is True
        assert recorded[-1][0:2] == ("INT", mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE)
        assert recorded[-1][5] == 2.5 and recorded[-1][9] == 90.0

        # 4. change_heading() on ArduPlane: COG + DEFAULT cancel
        assert mav.change_heading(135.0, heading_type=mavutil.mavlink.HEADING_TYPE_COURSE_OVER_GROUND, max_accel=4.0, timeout_ack=1.0) is True
        assert recorded[-1][0:2] == ("INT", mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING)
        assert recorded[-1][3:6] == (0.0, 135.0, 4.0)

        assert mav.change_heading(heading_type=mavutil.mavlink.HEADING_TYPE_DEFAULT, timeout_ack=1.0) is True
        assert recorded[-1][3] == float(mavutil.mavlink.HEADING_TYPE_DEFAULT)

        # 5. change_altitude() on ArduCopter -> delegates to DO_REPOSITION at current GPS
        inject(ap_mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            4,  # GUIDED
            mavutil.mavlink.MAV_STATE_ACTIVE,
        ))
        assert mav.change_altitude(20.0, rate=1.0, timeout_ack=1.0) is True
        assert recorded[-1][0:2] == ("INT", mavutil.mavlink.MAV_CMD_DO_REPOSITION)
        assert recorded[-1][9] == 20.0
    finally:
        mav.close()


def test_parachute_and_landing_gear_offline():
    """Tests release_parachute() and set_landing_gear() offline (input validation, MAVLink params, and ACKs)."""
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

        # 1. Invalid inputs rejected before sending
        for bad_action in (-1, 3, 99, True, False, "RELEASE"):
            assert mav.release_parachute(action=bad_action) is False
        for bad_deploy in (0, 1, -1, "down", None):
            assert mav.set_landing_gear(deploy=bad_deploy) is False
        for bad_id in (-2, True, False, 0.5, "all"):
            assert mav.set_landing_gear(deploy=True, landing_gear_id=bad_id) is False

        recorded = []
        ack_result = [mavutil.mavlink.MAV_RESULT_ACCEPTED]

        def on_send(m):
            if m.get_type() == "COMMAND_LONG":
                recorded.append((m.command, m.param1, m.param2))
                inject(ap_mav.command_ack_encode(
                    m.command, ack_result[0], 0, 0,
                    mav.master.source_system, mav.master.source_component,
                ))

        mav.set_additional_send_callback(on_send)

        # 2. release_parachute(): default PARACHUTE_RELEASE, ENABLE, DISABLE, and refused ACK
        assert mav.release_parachute(timeout_ack=1.0) is True
        assert recorded[-1] == (mavutil.mavlink.MAV_CMD_DO_PARACHUTE, float(mavutil.mavlink.PARACHUTE_RELEASE), 0.0)

        assert mav.release_parachute(action=mavutil.mavlink.PARACHUTE_ENABLE, timeout_ack=1.0) is True
        assert recorded[-1] == (mavutil.mavlink.MAV_CMD_DO_PARACHUTE, float(mavutil.mavlink.PARACHUTE_ENABLE), 0.0)

        assert mav.release_parachute(action=mavutil.mavlink.PARACHUTE_DISABLE, timeout_ack=1.0) is True
        assert recorded[-1] == (mavutil.mavlink.MAV_CMD_DO_PARACHUTE, float(mavutil.mavlink.PARACHUTE_DISABLE), 0.0)

        ack_result[0] = mavutil.mavlink.MAV_RESULT_FAILED
        assert mav.release_parachute(timeout_ack=1.0) is False

        # 3. set_landing_gear(): deploy=True (param2=0), deploy=False (param2=1), custom gear ID, and refused ACK
        ack_result[0] = mavutil.mavlink.MAV_RESULT_ACCEPTED
        assert mav.set_landing_gear(deploy=True, timeout_ack=1.0) is True
        assert recorded[-1] == (mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, -1.0, 0.0)

        assert mav.set_landing_gear(deploy=False, timeout_ack=1.0) is True
        assert recorded[-1] == (mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, -1.0, 1.0)

        assert mav.set_landing_gear(deploy=True, landing_gear_id=0, timeout_ack=1.0) is True
        assert recorded[-1] == (mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, 0.0, 0.0)

        ack_result[0] = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        assert mav.set_landing_gear(deploy=False, timeout_ack=1.0) is False
    finally:
        mav.close()


def test_mission_file_import_export_offline(tmp_path):
    """Tests plug-and-play load_mission_file(), save_mission_file(), upload_mission_file(), and download_mission_file() across MP, QGC, ArduPilot, and PX4."""
    import json
    from pathlib import Path
    from pymavlink import mavwp
    from pyMAVLinCS import MissionPlan
    from pyMAVLinCS.mavtypes import HomePosition

    repo_root = Path(__file__).resolve().parent.parent

    # 1. Real ArduPilot WPL 110 mission, zero-home WPL 110, fence WPL 110, rally WPL 110, and QGC .plan files
    sample_wpl = repo_root / "tests" / "ardupilot" / "Tools" / "autotest" / "ArduCopter_Tests" / "CopterMission" / "copter_mission.txt"
    if sample_wpl.exists():
        loaded_wpl = MAVLinCS.load_mission_file(sample_wpl)
        assert isinstance(loaded_wpl, MissionPlan)
        # 13 lines in file -> seq 0 HOME extracted to home_position + 12 actual flight items in waypoints
        assert len(loaded_wpl.waypoints) == 12
        assert loaded_wpl.waypoints[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        assert loaded_wpl.home_position is not None
        assert math.isclose(loaded_wpl.home_position.latitude_home, -35.362881, abs_tol=1e-6)

    zero_home_wpl = repo_root / "tests" / "ardupilot" / "Tools" / "autotest" / "ArduPlane_Tests" / "BootInAUTO" / "mission.txt"
    if zero_home_wpl.exists():
        loaded_zh = MAVLinCS.load_mission_file(zero_home_wpl)
        assert loaded_zh.home_position is None
        assert loaded_zh.waypoints[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

    fence_wpl = repo_root / "tests" / "ardupilot" / "modules" / "mavlink" / "pymavlink" / "tests" / "fence-110.txt"
    if fence_wpl.exists():
        loaded_fence = MAVLinCS.load_mission_file(fence_wpl)
        assert loaded_fence.waypoints == []
        assert len(loaded_fence.geofence) == 10
        assert loaded_fence.geofence[0].command == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION

    rally_wpl = repo_root / "tests" / "ardupilot" / "modules" / "mavlink" / "pymavlink" / "tests" / "rally-110.txt"
    if rally_wpl.exists():
        loaded_rally = MAVLinCS.load_mission_file(rally_wpl)
        assert loaded_rally.waypoints == []
        assert len(loaded_rally.rally_points) >= 2
        assert loaded_rally.rally_points[0].command == mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT

    sample_plan = repo_root / "tests" / "ardupilot" / "libraries" / "SITL" / "examples" / "Webots" / "mission_test_sitl.plan"
    if sample_plan.exists():
        loaded_qgc = MAVLinCS.load_mission_file(sample_plan)
        assert len(loaded_qgc.waypoints) == 6
        assert loaded_qgc.waypoints[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        assert loaded_qgc.home_position is not None
        assert math.isclose(loaded_qgc.home_position.altitude_home, 584.04, rel_tol=1e-4)

    # 2. Full roundtrip: WPL 110 (.waypoints) + pymavlink.mavwp interoperability + Mission Planner cmd=0 quirk
    waypoints = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, x=48.100000, y=2.200000, z=45.0, param1=15.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=48.102500, y=2.203500, z=50.0, param1=2.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH),
    ]
    wpl_file = tmp_path / "mission.waypoints"
    MAVLinCS.save_mission_file(wpl_file, waypoints, home_position=(48.100000, 2.200000, 120.0))

    # pymavlink.mavwp sees 4 WPL rows (seq 0 HOME + 3 mission items)
    loader = mavwp.MAVWPLoader()
    assert loader.load(str(wpl_file)) == 1 + len(waypoints)

    wpl_reloaded = MAVLinCS.load_mission_file(wpl_file)
    assert len(wpl_reloaded.waypoints) == len(waypoints)
    assert wpl_reloaded.home_position == HomePosition(48.100000, 2.200000, 120.0)
    assert wpl_reloaded.waypoints[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    assert math.isclose(wpl_reloaded.waypoints[0].param1, 15.0, rel_tol=1e-6)
    assert math.isclose(wpl_reloaded.waypoints[1].x, 48.102500, abs_tol=1e-6)

    # Cross-format conversion: MissionPlan without HOME -> .waypoints -> .plan -> MissionPlan (never eats WP1!)
    in_air_wps = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=48.111, y=2.222, z=30.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=48.115, y=2.228, z=35.0),
    ]
    in_air_plan_path = tmp_path / "in_air.plan"
    in_air_wpl_path = tmp_path / "in_air.waypoints"
    MAVLinCS.save_mission_file(in_air_plan_path, in_air_wps)
    assert MAVLinCS.load_mission_file(in_air_plan_path).waypoints == in_air_wps
    MAVLinCS.save_mission_file(in_air_wpl_path, MAVLinCS.load_mission_file(in_air_plan_path))
    assert MAVLinCS.load_mission_file(in_air_wpl_path).waypoints == in_air_wps

    # Mission Planner seq 0 command=0 quirk + comments/blank lines
    mp_quirk = tmp_path / "mp_home_zero.waypoints"
    mp_quirk.write_text(
        "QGC WPL 110\n"
        "# comment line\n\n"
        "0\t1\t0\t0\t0\t0\t0\t0\t48.1\t2.2\t100.0\t1\n"
        "1\t0\t3\t22\t15\t0\t0\t0\t48.1\t2.2\t30.0\t1\n",
        encoding="utf-8",
    )
    mp_loaded = MAVLinCS.load_mission_file(mp_quirk)
    assert len(mp_loaded.waypoints) == 1
    assert mp_loaded.waypoints[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    assert mp_loaded.home_position == HomePosition(48.1, 2.2, 100.0)

    # 3. Full roundtrip: QGroundControl .plan (waypoints + home + inclusion/exclusion polygons & circles + breachReturn + rally)
    geofence = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT, x=48.1005, y=2.2005, z=60.0, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=4.0, x=48.09, y=2.19, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=4.0, x=48.11, y=2.19, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=4.0, x=48.11, y=2.21, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=4.0, x=48.09, y=2.21, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, param1=3.0, x=48.101, y=2.201, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, param1=3.0, x=48.102, y=2.201, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, param1=3.0, x=48.1015, y=2.202, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, param1=500.0, x=48.10, y=2.20, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, param1=40.0, x=48.103, y=2.203, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
    ]
    rally_points = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT, x=48.105, y=2.205, z=65.0, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT, x=48.095, y=2.195, z=70.0, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
    ]
    plan_file = tmp_path / "full_flight.plan"
    MAVLinCS.save_mission_file(
        plan_file,
        waypoints=waypoints,
        geofence=geofence,
        rally_points=rally_points,
        home_position=HomePosition(48.100000, 2.200000, 120.0),
    )

    raw_doc = json.loads(plan_file.read_text(encoding="utf-8"))
    assert raw_doc["fileType"] == "Plan"
    assert len(raw_doc["mission"]["items"]) == 3
    assert len(raw_doc["geoFence"]["polygons"]) == 2
    assert len(raw_doc["geoFence"]["circles"]) == 2
    assert len(raw_doc["rallyPoints"]["points"]) == 2

    reloaded_plan = MAVLinCS.load_mission_file(plan_file)
    assert reloaded_plan.waypoints == waypoints
    assert reloaded_plan.geofence == geofence
    assert reloaded_plan.rally_points == rally_points
    assert reloaded_plan.home_position == HomePosition(48.100000, 2.200000, 120.0)

    # Save directly from MissionPlan object -> identical reload
    plan_file_copy = tmp_path / "copy.plan"
    MAVLinCS.save_mission_file(plan_file_copy, reloaded_plan)
    assert MAVLinCS.load_mission_file(plan_file_copy) == reloaded_plan

    # NaN param4 (default yaw) serializes cleanly to JSON null (strict RFC 8259 JSON) and 0.0 in WPL 110
    nan_yaw_wp = [MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=48.1, y=2.2, z=25.0, param4=math.nan)]
    nan_plan_path = tmp_path / "nan_yaw.plan"
    nan_wpl_path = tmp_path / "nan_yaw.waypoints"
    MAVLinCS.save_mission_file(nan_plan_path, nan_yaw_wp)
    assert json.loads(nan_plan_path.read_text(encoding="utf-8"))["mission"]["items"][0]["params"][3] is None
    MAVLinCS.save_mission_file(nan_wpl_path, nan_yaw_wp)
    assert len(MAVLinCS.load_mission_file(nan_wpl_path).waypoints) == 1

    # 4. ComplexItems (TransectStyleComplexItem.Items, fwLandingPattern, vtolLandingPattern, unexpanded error) + DO_JUMP doJumpId remapping
    complex_plan = tmp_path / "complex.plan"
    complex_plan.write_text(json.dumps({
        "fileType": "Plan",
        "version": 1,
        "mission": {
            "version": 2,
            "items": [
                {"type": "SimpleItem", "doJumpId": 1, "command": 22, "frame": 3, "params": [15, 0, 0, None, 48.1, 2.2, 50]},
                {
                    "type": "ComplexItem",
                    "doJumpId": 5,  # Non-contiguous doJumpId (e.g. items 2..4 deleted in QGC)
                    "complexItemType": "survey",
                    "TransectStyleComplexItem": {
                        "Items": [
                            {"type": "SimpleItem", "command": 16, "frame": 3, "params": [0, 0, 0, None, 48.11, 2.21, 50]},
                            {"type": "SimpleItem", "command": 16, "frame": 3, "params": [0, 0, 0, None, 48.12, 2.22, 50]},
                        ]
                    },
                },
                {"type": "SimpleItem", "doJumpId": 9, "command": 177, "frame": 2, "params": [5, 2, 0, 0, 0, 0, 0]},
                {
                    "type": "ComplexItem",
                    "complexItemType": "fwLandingPattern",
                    "altitudesAreRelative": True,
                    "landingApproachCoordinate": [48.13, 2.23, 40],
                    "landCoordinate": [48.10, 2.20, 0],
                    "loiterRadius": 75.0,
                    "loiterClockwise": False,
                },
            ],
        },
        "geoFence": {"version": 1, "polygons": [[[48.0, 2.0], [48.1, 2.0], [48.1, 2.1]]]},
        "rallyPoints": {"version": 2, "points": []},
    }), encoding="utf-8")
    parsed_complex = MAVLinCS.load_mission_file(complex_plan)
    # 1 TAKEOFF + 2 survey WPs + 1 DO_JUMP + 3 fwLandingPattern items (DO_LAND_START, LOITER_TO_ALT, NAV_LAND) = 7
    assert len(parsed_complex.waypoints) == 7
    assert parsed_complex.waypoints[3].command == mavutil.mavlink.MAV_CMD_DO_JUMP
    assert parsed_complex.waypoints[3].param1 == 2.0  # doJumpId 5 -> 2nd item in waypoints!
    assert parsed_complex.waypoints[4].command == mavutil.mavlink.MAV_CMD_DO_LAND_START
    assert parsed_complex.waypoints[5].command == mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT
    assert parsed_complex.waypoints[5].param2 == -75.0  # CCW
    assert parsed_complex.waypoints[6].command == mavutil.mavlink.MAV_CMD_NAV_LAND
    assert len(parsed_complex.geofence) == 3

    # Unexpanded ComplexItem raises ValueError instead of silently truncating mission
    unexpanded = tmp_path / "unexpanded.plan"
    unexpanded.write_text(json.dumps({
        "fileType": "Plan",
        "version": 1,
        "mission": {"version": 2, "items": [{"type": "ComplexItem", "complexItemType": "survey"}]},
    }), encoding="utf-8")
    with pytest.raises(ValueError, match="ComplexItem"):
        MAVLinCS.load_mission_file(unexpanded)

    # Invalid / unsupported inputs
    with pytest.raises(FileNotFoundError):
        MAVLinCS.load_mission_file(tmp_path / "does_not_exist.plan")
    bad_file = tmp_path / "bad.waypoints"
    bad_file.write_text("INVALID HEADER\n", encoding="utf-8")
    with pytest.raises(ValueError):
        MAVLinCS.load_mission_file(bad_file)
    with pytest.raises(ValueError):
        MAVLinCS.save_mission_file(tmp_path / "unknown.xyz", waypoints)

    # 5. Offline upload_mission_file() & download_mission_file() across BOTH ArduPilot (drifted HOME) and PX4!
    mav = MAVLinCS(address="udpin:127.0.0.1:0", timeout_heartbeat=0, pos_rate=0, mavlogfile=None)
    try:
        mav.master.target_system = 1
        ap_mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        storage: dict[int, list] = {}
        expected_count: dict[int, int] = {}
        is_px4 = [False]

        def push(msg):
            pkt = msg.pack(ap_mav)
            decoded = mav.master.mav.decode(bytearray(pkt))
            decoded._timestamp = time.time()
            mav.master.post_message(decoded)
            with mav.mavthread._lock:
                mav.mavthread._queue.append(decoded)
                mav.mavthread._msg_received.set()

        def on_send(m):
            mtype = m.get_type()
            mt = getattr(m, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            if mtype == "MISSION_CLEAR_ALL":
                storage[mt] = []
                push(ap_mav.mission_ack_encode(
                    mav.master.source_system, mav.master.source_component,
                    mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type=mt,
                ))
            elif mtype == "MISSION_COUNT":
                expected_count[mt] = m.count
                storage[mt] = []
                if m.count > 0:
                    push(ap_mav.mission_request_int_encode(
                        mav.master.source_system, mav.master.source_component, 0, mission_type=mt,
                    ))
                else:
                    push(ap_mav.mission_ack_encode(
                        mav.master.source_system, mav.master.source_component,
                        mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type=mt,
                    ))
            elif mtype == "MISSION_ITEM_INT":
                # Simulate ArduPilot overwriting wire seq 0 with its actual drifted GPS HOME (48.1008, 2.2009, 125m)
                if not is_px4[0] and mt == mavutil.mavlink.MAV_MISSION_TYPE_MISSION and m.seq == 0:
                    m = ap_mav.mission_item_int_encode(
                        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                        0, 0, 0, 0, int(48.1008 * 1e7), int(2.2009 * 1e7), 125.0, mission_type=mt,
                    )
                storage[mt].append(m)
                if len(storage[mt]) < expected_count[mt]:
                    push(ap_mav.mission_request_int_encode(
                        mav.master.source_system, mav.master.source_component, len(storage[mt]), mission_type=mt,
                    ))
                else:
                    push(ap_mav.mission_ack_encode(
                        mav.master.source_system, mav.master.source_component,
                        mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type=mt,
                    ))
            elif mtype == "MISSION_REQUEST_LIST":
                push(ap_mav.mission_count_encode(
                    mav.master.source_system, mav.master.source_component, len(storage.get(mt, [])), mission_type=mt,
                ))
            elif mtype == "MISSION_REQUEST_INT":
                stored = storage[mt][m.seq]
                push(ap_mav.mission_item_int_encode(
                    mav.master.source_system, mav.master.source_component, m.seq,
                    stored.frame, stored.command, 0, 1,
                    stored.param1, stored.param2, stored.param3, stored.param4,
                    stored.x, stored.y, stored.z, mission_type=mt,
                ))

        mav.set_additional_send_callback(on_send)

        # 5a. ArduPilot: wire has 1 + 3 = 4 items; drifted HOME never duplicates into downloaded .plan or .waypoints!
        push(ap_mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0, 0, mavutil.mavlink.MAV_STATE_STANDBY,
        ))
        assert mav.upload_mission_file(plan_file, timeout=5.0) is True
        assert len(storage[mavutil.mavlink.MAV_MISSION_TYPE_MISSION]) == 1 + len(waypoints)
        assert len(storage[mavutil.mavlink.MAV_MISSION_TYPE_FENCE]) == len(geofence)
        assert len(storage[mavutil.mavlink.MAV_MISSION_TYPE_RALLY]) == len(rally_points)

        dl_plan_path = tmp_path / "downloaded_ap.plan"
        assert mav.download_mission_file(dl_plan_path, timeout=5.0) is True
        dl_plan = MAVLinCS.load_mission_file(dl_plan_path)
        assert len(dl_plan.waypoints) == len(waypoints)
        assert dl_plan.waypoints[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        assert len(dl_plan.geofence) == len(geofence)
        assert len(dl_plan.rally_points) == len(rally_points)
        assert math.isclose(dl_plan.home_position.latitude_home, 48.1008, abs_tol=1e-5)

        dl_wpl_path = tmp_path / "downloaded_ap.waypoints"
        assert mav.download_mission_file(dl_wpl_path, timeout=5.0) is True
        dl_wpl = MAVLinCS.load_mission_file(dl_wpl_path)
        assert len(dl_wpl.waypoints) == len(waypoints)
        assert math.isclose(dl_wpl.home_position.latitude_home, 48.1008, abs_tol=1e-5)

        # Mission Planner .fen and .ral standalone WPL 110 downloads & re-uploads
        dl_fen_path = tmp_path / "downloaded_ap.fen"
        dl_ral_path = tmp_path / "downloaded_ap.ral"
        assert mav.download_mission_file(dl_fen_path, timeout=5.0) is True
        assert mav.download_mission_file(dl_ral_path, timeout=5.0) is True
        assert MAVLinCS.load_mission_file(dl_fen_path).geofence == geofence
        assert MAVLinCS.load_mission_file(dl_ral_path).rally_points == rally_points

        # 5b. PX4: wire has exact item count (no seq 0 HOME), and DO_JUMP shifts to 0-based on wire & back to 1-based!
        is_px4[0] = True
        push(ap_mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_PX4,
            0, 0, mavutil.mavlink.MAV_STATE_STANDBY,
        ))
        assert mav.upload_mission_file(complex_plan, timeout=5.0) is True
        px4_wire = storage[mavutil.mavlink.MAV_MISSION_TYPE_MISSION]
        assert len(px4_wire) == 7  # No fake seq 0 HOME sent to PX4!
        assert px4_wire[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        assert px4_wire[3].command == mavutil.mavlink.MAV_CMD_DO_JUMP
        assert px4_wire[3].param1 == 1.0  # 0-based index 1 on PX4 wire (2nd item)
        px4_dl = mav.download_mission(timeout=5.0)
        assert len(px4_dl) == 7
        assert px4_dl[3].param1 == 2.0  # Restored to 1-based item #2!

        # Missing file / invalid extension return False gracefully
        assert mav.upload_mission_file(tmp_path / "missing.plan") is False
        assert mav.download_mission_file(tmp_path / "bad.xyz") is False
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
    assert drone.loiter_here() is False
    assert drone.change_altitude(20.0) is False
    assert drone.change_heading(90.0) is False
    assert drone.release_parachute() is False
    assert drone.set_landing_gear(True) is False
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
    assert drone.upload_rally_points([(48.0, 2.0, 50.0)]) is False
    assert drone.upload_geofence([(mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT, 48.0, 2.0, 0.0)]) is False
    assert drone.upload_mission_file("nonexistent.plan") is False
    assert drone.start_mission() is False
    assert drone.set_current_waypoint(1) is False
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
    assert drone.set_current_waypoint(1) is False
    assert drone.loiter_here() is False
    assert drone.change_altitude(20.0) is False
    assert drone.change_heading(90.0) is False
    assert drone.release_parachute() is False
    assert drone.set_landing_gear(True) is False
    assert drone.do_follow(target_system=2) is None
    assert drone.do_follow_with_takeoff(target_system=2) is None
    assert drone.send_landing_target(0.0, 0.0, 1.0) is None
    assert drone.set_home(0.0, 0.0, 0.0) is False
    assert drone.set_home_current_pos() is False
    assert drone.set_ekf_origin(0.0, 0.0, 0.0) is False
    assert math.isnan(drone.request_home_position().latitude_home)
    assert math.isnan(drone.get_parameter("SYSID_MYGCS"))
    assert drone.download_mission() is None
    assert drone.download_rally_points() is None
    assert drone.download_geofence() is None
    assert drone.upload_mission_file("nonexistent.plan") is False
    assert drone.download_mission_file("out.plan") is False
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
