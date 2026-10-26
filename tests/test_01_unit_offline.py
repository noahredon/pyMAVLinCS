import math
import pytest
from pyMAVLinCS import MAVLinCS
from pyMAVLinCS.mission_control_code import MCC, EMPTY


def test_quaternion_to_euler():
    # Identity quaternion (w=1, x=0, y=0, z=0) -> roll=0, pitch=0, yaw=0
    angles = MAVLinCS.quaternion_to_euler([1.0, 0.0, 0.0, 0.0])
    assert math.isclose(angles.roll, 0.0, abs_tol=1e-6)
    assert math.isclose(angles.pitch, 0.0, abs_tol=1e-6)
    assert math.isclose(angles.yaw, 0.0, abs_tol=1e-6)


def test_distance_between_two_local_points():
    # 3-4-5 triangle in 2D (z ignored)
    d2d = MAVLinCS.distance_between_two_local_points(0.0, 3.0, 0.0, 4.0)
    assert math.isclose(d2d, 5.0, rel_tol=1e-5)

    # In 3D
    d3d = MAVLinCS.distance_between_two_local_points(0.0, 3.0, 0.0, 4.0, 0.0, 12.0)
    assert math.isclose(d3d, 13.0, rel_tol=1e-5)


def test_geographic_projections_roundtrip():
    lat0, lon0 = -35.363261, 149.165237
    # Cartesian projection: +100m North, +50m East
    pt = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=100.0, east_displacement=50.0)

    # Inverse calculation
    dn, de = MAVLinCS.geographic_to_cartesian_point(lat0, lon0, pt.latitude, pt.longitude)
    assert math.isclose(dn, 100.0, rel_tol=1e-2)
    assert math.isclose(de, 50.0, rel_tol=1e-2)

    # Direct distance between the two GPS points (2D and 3D)
    dist_2d = MAVLinCS.distance_between_two_gps_points(lat0, pt.latitude, lon0, pt.longitude)
    expected_2d = math.hypot(100.0, 50.0)
    assert math.isclose(dist_2d, expected_2d, rel_tol=1e-2)

    dist_3d = MAVLinCS.distance_between_two_gps_points(
        lat0, pt.latitude, lon0, pt.longitude, alt_i=10.0, alt_f=60.0
    )
    assert math.isclose(dist_3d, math.hypot(expected_2d, 50.0), rel_tol=1e-2)


def test_polar_projection():
    lat0, lon0 = 48.7098, 2.1355
    pt_north = MAVLinCS.polar_to_geographic_point(lat0, lon0, distance_m=50.0, bearing=0.0)
    assert pt_north.latitude > lat0
    assert math.isclose(pt_north.longitude, lon0, abs_tol=1e-5)


def test_mcc_registration_and_lookup():
    code = MCC(value=9001, name="PYTEST_MCC_1", level="INFO", description="Unit test")
    assert MCC.get(9001) == code
    assert MCC.get_name("PYTEST_MCC_1") == code
    assert MCC.get(999999) == EMPTY
    assert int(code) == 9001
    assert code == 9001
    assert code == "PYTEST_MCC_1"

    # Check duplicate prevention
    with pytest.raises(ValueError):
        MCC(value=9001, name="OTHER_NAME", level="INFO", description="Duplicate value")
    with pytest.raises(ValueError):
        MCC(value=9002, name="PYTEST_MCC_1", level="INFO", description="Duplicate name")


def test_safety_guards_and_invalid_inputs(drone):
    # PWM out of bounds [1000, 2000]
    assert drone.set_servo(servo_channel=9, pwm=950) is False
    assert drone.set_servo(servo_channel=9, pwm=2050) is False

    # Negative or zero message frequency
    assert drone.set_msg_rate("ATTITUDE", rate=0) is False
    assert drone.set_msg_rate("UNKNOWN_MAV_MSG_XYZ", rate=2) is False

    # Test blocking via allow_actions = False
    drone.allow_actions = False
    assert drone.arm() is False
    assert drone.takeoff(10.0) is False
    assert drone.set_mode("GUIDED") is False
    assert drone.set_servo(9, 1500) is False

    # Test blocking via mission_ended = True
    drone.allow_actions = True
    drone.mission_ended = True
    assert drone.arm() is False
    assert drone.send_statustext("Should fail") is False
    assert drone.wait_command_ack(command=400, timeout=1) is None
