import pytest
import time

def test_speed_control(drone):
    """Test speed setting commands."""
    # set_speed(vx: float, vy: float, vz: float, is_relative: bool = False)
    drone.set_speed(vx=5.0, vy=0.0, vz=0.0)

def test_servo_control(drone):
    """Test raw servo commands."""
    drone.set_servo(servo_channel=9, pwm=1500)
    time.sleep(0.5)

def test_gimbal_control(drone):
    """Test gimbal control commands."""
    drone.set_gimbal_neutral()
    drone.set_gimbal_retract()
    drone.set_gimbal_angles(pitch=-45, yaw=10)
    drone.set_gimbal_target(latitude=45.0, longitude=5.0, altitude=0.0)

def test_statustext_and_mcc(drone):
    """Test sending statustext and mcc."""
    from pyMAVLinCS.mission_control_code import MCC
    
    # Make sure we have an MCC to send
    try:
        test_mcc = MCC(value=201, name="CTRL_TEST", level="INFO", description="Test MCC")
    except ValueError:
        test_mcc = MCC.get(201)
        
    assert drone.send_mcc(test_mcc)
    
    # send_statustext
    from pymavlink.mavutil import mavlink
    drone.send_statustext("Hello from pyMAVLinCS", severity=mavlink.MAV_SEVERITY_INFO)
