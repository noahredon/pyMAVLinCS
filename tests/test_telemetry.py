import pytest
import time
import math
from pyMAVLinCS import mavtypes

def test_time_methods(drone):
    """Test time_boot and time_usec."""
    t_boot = drone.time_boot
    t_ms = drone.time_boot_ms
    t_us = drone.time_usec
    
    assert isinstance(t_boot, int)
    assert isinstance(t_ms, int)
    assert isinstance(t_us, int)

def test_gps_position(drone):
    """Test GPS position."""
    pos = drone.position_gps()
    assert isinstance(pos, mavtypes.GPSPosition)
    assert isinstance(pos.lat, float)
    assert isinstance(pos.lon, float)
    assert isinstance(pos.relative_alt, float)

def test_local_position(drone):
    """Test local position."""
    pos = drone.position_local()
    assert isinstance(pos, mavtypes.LocalPosition)
    assert isinstance(pos.x, float)
    assert isinstance(pos.y, float)
    assert isinstance(pos.z, float)

def test_angles(drone):
    """Test attitude angles."""
    ang = drone.angles()
    assert isinstance(ang, mavtypes.Angles)
    assert isinstance(ang.roll, float)
    assert isinstance(ang.pitch, float)
    assert isinstance(ang.yaw, float)

def test_angular_rates(drone):
    """Test angular rates."""
    rates = drone.angular_rates()
    assert isinstance(rates, mavtypes.AnglesRates)
    assert isinstance(rates.rollspeed, float)
    assert isinstance(rates.pitchspeed, float)
    assert isinstance(rates.yawspeed, float)

def test_speeds(drone):
    """Test speed metrics."""
    spd = drone.speed()
    assert isinstance(spd, mavtypes.Speed)
    assert isinstance(spd.vx, float)
    assert isinstance(spd.vy, float)
    assert isinstance(spd.vz, float)
    
    spd_mod = drone.speed_module()
    assert isinstance(spd_mod, float)

def test_system_status(drone):
    """Test system status like battery and mode."""
    mode = drone.mode()
    assert isinstance(mode, str)
    
    base_m = drone.base_mode()
    assert isinstance(base_m, int)
    
    cust_m = drone.custom_mode()
    assert isinstance(cust_m, int)
    
    auto_str = drone.autopilot_str()
    assert isinstance(auto_str, str)
    
    auto_int = drone.autopilot_int()
    assert isinstance(auto_int, int)

def test_home_and_ekf(drone):
    """Test home position and EKF origin."""
    # This might return NaNs if not fully initialized, but we just check the type
    home = drone.home_position()
    assert isinstance(home, mavtypes.HomePosition)
    
    ekf = drone.ekf_origin()
    assert isinstance(ekf, mavtypes.EkfOrigin)

def test_gimbal_and_servos(drone):
    """Test gimbal and servos."""
    gimbal = drone.gimbal_angles()
    assert isinstance(gimbal, mavtypes.GimbalAngles)
    
    pwm = drone.servo_pwm(1)
    assert isinstance(pwm, int)
