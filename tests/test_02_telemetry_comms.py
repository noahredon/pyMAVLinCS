import math
import time
from pymavlink import mavutil
from pyMAVLinCS.mission_control_code import MCC

MCC_PYTEST_COMM = MCC(value=9010, name="COMM_OK", level="SUCCESS", description="Communication validated")


def test_telemetry_getters(drone):
    assert drone.dialect == "ardupilotmega"
    assert drone.time_boot >= 0
    assert drone.time_boot_ms >= 0
    assert drone.time_usec > 0

    assert drone.autopilot_int() == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
    assert drone.autopilot_str() == "MAV_AUTOPILOT_ARDUPILOTMEGA"
    assert drone.gps_3d_fix() is True
    assert drone.mode() != "UNKNOWN"
    assert not math.isnan(drone.custom_mode())
    assert not math.isnan(drone.base_mode())
    assert not math.isnan(drone.custom_main_mode())
    assert not math.isnan(drone.custom_sub_mode())

    mapping = drone.mode_mapping()
    assert isinstance(mapping, dict)
    assert "GUIDED" in mapping and "AUTO" in mapping and "RTL" in mapping

    gps = drone.position_gps()
    assert not math.isnan(gps.lat) and not math.isnan(gps.lon)
    assert math.isclose(drone.get_distance_with_gps_pos(gps.lat, gps.lon), 0.0, abs_tol=1.0)
    assert math.isclose(
        drone.get_distance_with_gps_pos(gps.lat, gps.lon, relative_alt=gps.relative_alt),
        0.0,
        abs_tol=1.0,
    )
    assert math.isclose(
        drone.get_distance_with_gps_pos(gps.lat, gps.lon, alt=gps.alt),
        0.0,
        abs_tol=1.0,
    )

    loc = drone.position_local()
    assert not math.isnan(loc.x) and not math.isnan(loc.y) and not math.isnan(loc.z)
    assert math.isclose(drone.get_distance_with_local_pos(loc.x, loc.y, loc.z), 0.0, abs_tol=1.0)
    assert math.isclose(drone.get_distance_with_local_pos(loc.x, loc.y), 0.0, abs_tol=1.0)

    angles = drone.angles()
    assert not math.isnan(angles.roll) and not math.isnan(angles.pitch) and not math.isnan(angles.yaw)

    rates = drone.angular_rates()
    assert not math.isnan(rates.rollspeed) and not math.isnan(rates.pitchspeed) and not math.isnan(rates.yawspeed)

    spd = drone.speed()
    assert not math.isnan(spd.vx) and not math.isnan(spd.vy) and not math.isnan(spd.vz)
    assert not math.isnan(drone.speed_module())

    assert drone.battery_voltage() > 0.0
    assert not math.isnan(drone.battery_percentage())

    gimbal = drone.gimbal_angles()
    assert hasattr(gimbal, "roll") and hasattr(gimbal, "pitch") and hasattr(gimbal, "yaw")


def test_get_msg_recv_match_and_modes(drone):
    # get_msg with valid max_age vs expired max_age vs unknown system ID
    hb = drone.get_msg("HEARTBEAT", max_age=5.0)
    assert hb is not None
    assert drone.get_msg("HEARTBEAT", max_age=0.0) is None
    assert drone.get_msg("HEARTBEAT", source_system=250) is None

    # recv_match for an incoming telemetry packet
    matched = drone.recv_match(type=["HEARTBEAT", "ATTITUDE"], blocking=True, timeout=3.0)
    assert matched is not None
    assert matched.get_type() in ("HEARTBEAT", "ATTITUDE")

    # Mode switching via string and wait_mode_changed
    assert drone.set_mode("STABILIZE", timeout_ack=3, timeout_set_mode=0) is True
    assert drone.wait_mode_changed("STABILIZE", timeout=4) is True
    assert drone.mode() == "STABILIZE"

    # Switch back to GUIDED using integer mode ID from mode_mapping
    guided_id = drone.mode_mapping()["GUIDED"]
    assert drone.set_mode(guided_id, timeout_ack=3, timeout_set_mode=4) is True
    assert drone.mode() == "GUIDED"


def test_home_and_ekf_origin(drone):
    home = drone.request_home_position(timeout_ack=3, timeout_home_reception=5)
    assert not math.isnan(home.latitude_home)
    assert not math.isnan(home.longitude_home)

    cached_home = drone.home_position()
    assert math.isclose(cached_home.latitude_home, home.latitude_home, abs_tol=1e-4)

    # Test set_home with custom coordinates and wait_for_home
    assert drone.set_home(
        latitude=home.latitude_home,
        longitude=home.longitude_home,
        altitude=home.altitude_home,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    ) is True
    updated_home = drone.wait_for_home(timeout=5)
    assert not math.isnan(updated_home.latitude_home)

    # Test set_home_current_pos
    assert drone.set_home_current_pos() is True

    # Test ekf_origin and set_ekf_origin
    origin = drone.ekf_origin()
    assert hasattr(origin, "latitude")
    assert drone.set_ekf_origin(
        latitude=home.latitude_home,
        longitude=home.longitude_home,
        altitude=home.altitude_home,
    ) is True


def test_timesync_and_msg_rate(drone):
    assert drone.set_msg_rate("ATTITUDE", rate=5, timeout_ack=3) is True
    assert drone.set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, rate=5, timeout_ack=3) is True
    assert drone.send_timesync() is True
    time.sleep(0.5)
    key = (drone.master.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
    latency = drone.get_latence_ms_with_sysidcompid(key)
    assert not math.isnan(latency) and latency >= 0


def test_callbacks_mcc_and_statustext(drone):
    received_mcc = []
    received_statustexts = []
    sent_types = []
    recv_types = []

    drone.set_mcc_callback(lambda m: received_mcc.append(m))
    drone.set_statustext_callback(lambda s: received_statustexts.append(s))
    drone.set_additional_send_callback(lambda m: sent_types.append(m.get_type()))
    drone.set_additional_recv_msg_callback(lambda m: recv_types.append(m.get_type()))

    # Send a generic NAMED_VALUE_INT (including name > 10 chars truncation)
    assert drone.send_named_value_int(value=42, name="VERY_LONG_METRIC_NAME") is True

    # Send an MCC
    assert drone.send_mcc(MCC_PYTEST_COMM) is True
    assert drone.check_mcc(MCC_PYTEST_COMM) is True
    drone.delete_mcc(MCC_PYTEST_COMM)
    assert drone.check_mcc(MCC_PYTEST_COMM) is False

    # Send a long STATUSTEXT (> 50 characters -> multi-chunks)
    long_msg = "Pytest verification of multi-chunk STATUSTEXT transmission in pyMAVLinCS!"
    assert drone.send_statustext(long_msg) is True

    # Query last statustext from autopilot key (None or MAVLink_message)
    ap_key = (drone.master.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
    last_st = drone.last_statustext_from_sysidcompid(ap_key)
    assert last_st is None or hasattr(last_st, "text")

    # Write a harmless parameter
    assert drone.write_parameter("SYSID_MYGCS", 255, mavutil.mavlink.MAV_PARAM_TYPE_INT16) is True

    time.sleep(0.5)
    assert "NAMED_VALUE_INT" in sent_types
    assert "STATUSTEXT" in sent_types
    assert len(recv_types) > 0
