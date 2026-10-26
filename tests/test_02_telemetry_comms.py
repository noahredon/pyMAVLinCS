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
    assert drone.mode() != "UNKNOWN"
    assert not math.isnan(drone.custom_mode())
    assert not math.isnan(drone.base_mode())

    gps = drone.position_gps()
    assert not math.isnan(gps.lat) and not math.isnan(gps.lon)
    assert math.isclose(drone.get_distance_with_gps_pos(gps.lat, gps.lon), 0.0, abs_tol=1.0)

    loc = drone.position_local()
    assert not math.isnan(loc.x) and not math.isnan(loc.y) and not math.isnan(loc.z)
    assert math.isclose(drone.get_distance_with_local_pos(loc.x, loc.y, loc.z), 0.0, abs_tol=1.0)

    angles = drone.angles()
    assert not math.isnan(angles.roll) and not math.isnan(angles.pitch) and not math.isnan(angles.yaw)

    rates = drone.angular_rates()
    assert not math.isnan(rates.rollspeed)

    spd = drone.speed()
    assert not math.isnan(spd.vx)
    assert not math.isnan(drone.speed_module())

    assert drone.battery_voltage() > 0.0


def test_home_and_ekf_origin(drone):
    home = drone.request_home_position(timeout_ack=3, timeout_home_reception=5)
    assert not math.isnan(home.latitude_home)
    assert not math.isnan(home.longitude_home)

    # Test set_home_current_pos
    assert drone.set_home_current_pos() is True


def test_timesync_and_msg_rate(drone):
    assert drone.set_msg_rate("ATTITUDE", rate=5, timeout_ack=3) is True
    assert drone.send_timesync() is True
    time.sleep(0.5)
    key = (drone.master.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
    latency = drone.get_latence_ms_with_sysidcompid(key)
    assert not math.isnan(latency) and latency >= 0


def test_callbacks_mcc_and_statustext(drone):
    received_mcc = []
    sent_types = []
    recv_types = []

    drone.set_mcc_callback(lambda m: received_mcc.append(m))
    drone.set_additional_send_callback(lambda m: sent_types.append(m.get_type()))
    drone.set_additional_recv_msg_callback(lambda m: recv_types.append(m.get_type()))

    # Send an MCC
    assert drone.send_mcc(MCC_PYTEST_COMM) is True
    assert drone.check_mcc(MCC_PYTEST_COMM) is True
    drone.delete_mcc(MCC_PYTEST_COMM)
    assert drone.check_mcc(MCC_PYTEST_COMM) is False

    # Send a long STATUSTEXT (> 50 characters -> multi-chunks)
    long_msg = "Pytest verification of multi-chunk STATUSTEXT transmission in pyMAVLinCS!"
    assert drone.send_statustext(long_msg) is True

    # Write a harmless parameter
    assert drone.write_parameter("SYSID_MYGCS", 255, mavutil.mavlink.MAV_PARAM_TYPE_INT16) is True

    time.sleep(0.5)
    assert "NAMED_VALUE_INT" in sent_types
    assert "STATUSTEXT" in sent_types
    assert len(recv_types) > 0
