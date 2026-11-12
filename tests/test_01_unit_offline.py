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
    assert mav.motors_armed() is None
    assert mav.motors_disarmed() is None
    assert mav.mode() == "UNKNOWN"
    assert mav.autopilot_str() == "UNKNOWN"
    assert math.isnan(mav.autopilot_int())
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


def test_safety_guards_and_invalid_inputs(drone):
    # PWM out of bounds [1000, 2000]
    assert drone.set_servo(servo_channel=9, pwm=950) is False
    assert drone.set_servo(servo_channel=9, pwm=2050) is False

    # NaN coordinates in set_gimbal_target
    assert drone.set_gimbal_target(math.nan, 2.0, 10.0) is False

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
    assert drone.return_to_launch() is False
    assert drone.set_mode("GUIDED") is False
    assert drone.set_gps_pos_target(0.0, 0.0, 10.0) is False
    assert drone.set_local_pos_target(0.0, 0.0, -10.0) is False
    assert drone.set_speed(1.0, 0.0, 0.0) is False
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
    assert drone.return_to_launch() is False
    assert drone.do_follow(target_system=2) is None
    assert drone.do_follow_with_takeoff(target_system=2) is None
    assert drone.send_landing_target(0.0, 0.0, 1.0) is None
    assert drone.set_home(0.0, 0.0, 0.0) is False
    assert drone.set_home_current_pos() is False
    assert drone.set_ekf_origin(0.0, 0.0, 0.0) is False
    assert math.isnan(drone.request_home_position().latitude_home)
    assert math.isnan(drone.get_parameter("SYSID_MYGCS"))
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
