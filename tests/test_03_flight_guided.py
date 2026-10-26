import math
import time
from pyMAVLinCS import MAVLinCS


def test_guided_full_flight_sequence(drone):
    # 1. Arm and Takeoff to 10m
    assert drone.arm(timeout_ack=3, timeout_arming=10) is True
    assert drone.motors_armed() is True
    assert drone.motors_disarmed() is False

    assert drone.takeoff(altitude=10.0, precision=1.0, timeout_takeoff=35) is True
    assert abs(drone.position_gps().relative_alt - 10.0) < 1.5

    # 2. Vertical repositioning via takeoff() while the drone is already in the air (14m)
    assert drone.takeoff(altitude=14.0, precision=1.0, timeout_takeoff=25) is True
    assert abs(drone.position_gps().relative_alt - 14.0) < 1.5

    # 3. Movement in NED local coordinates (+12m North, +8m East, -14m Down)
    loc_start = drone.position_local()
    tx, ty, tz = loc_start.x + 12.0, loc_start.y + 8.0, -14.0
    assert drone.set_local_pos_target(x=tx, y=ty, z=tz, yaw=math.radians(45)) is True
    assert drone.wait_proximity_with_local_pos(x=tx, y=ty, z=tz, radius_acceptance=1.8, timeout=30) is True

    # 4. GPS movement via polar projection (20m bearing 135°)
    gps_now = drone.position_gps()
    target_gps = MAVLinCS.polar_to_geographic_point(
        lat_origine=gps_now.lat, lon_origine=gps_now.lon, distance_m=20.0, bearing=135.0
    )
    assert drone.set_gps_pos_target(
        latitude=target_gps.latitude,
        longitude=target_gps.longitude,
        relative_altitude=14.0,
        yaw=math.radians(135),
    ) is True
    assert drone.wait_proximity_with_gps_pos(
        lat=target_gps.latitude,
        lon=target_gps.longitude,
        relative_alt=14.0,
        radius_acceptance=1.8,
        timeout=30,
    ) is True

    # 5. Servo PWM and Gimbal Target (ROI) control
    assert drone.set_servo(servo_channel=9, pwm=1850) is True
    time.sleep(0.4)
    assert math.isclose(drone.servo_pwm(9), 1850, abs_tol=5)
    assert drone.set_servo(servo_channel=9, pwm=1150) is True

    home = drone.home_position()
    if not math.isnan(home.latitude_home):
        assert drone.set_gimbal_target(
            latitude=home.latitude_home,
            longitude=home.longitude_home,
            altitude=home.altitude_home,
        ) is True

    # 6. Speed control (set_speed) for 6 seconds at 3.5 m/s
    target_speed = 3.5
    max_measured_speed = 0.0
    for i in range(30):
        angle = 2 * math.pi * (i / 30)
        vx = target_speed * math.cos(angle)
        vy = target_speed * math.sin(angle)
        assert drone.set_speed(vx=vx, vy=vy, vz=0.0, yaw=angle) is True
        time.sleep(0.2)
        spd = drone.speed_module()
        if not math.isnan(spd):
            max_measured_speed = max(max_measured_speed, spd)

    assert max_measured_speed > 2.0, f"The drone did not accelerate as expected (max={max_measured_speed} m/s)"
    assert drone.set_speed(vx=0.0, vy=0.0, vz=0.0) is True
    time.sleep(1.5)

    # 7. Landing and disarmament check
    assert drone.land(timeout_land=60) is True
    assert drone.motors_disarmed() is True
