import pytest
import time

def test_connection(drone):
    """Test that we can connect and heartbeat is received."""
    assert drone is not None
    # Assuming connection works if we got here

def test_modes(drone):
    """Test setting and getting modes."""
    # Try setting to GUIDED
    drone.set_mode("GUIDED")
    time.sleep(1) # wait for mode to change
    
    mode = drone.mode()
    assert mode == "GUIDED"
    
    drone.set_mode("STABILIZE")
    time.sleep(1)
    
    assert drone.mode() == "STABILIZE"

def test_battery_and_telemetry(drone):
    """Test basic telemetry data."""
    voltage = drone.battery_voltage()
    assert isinstance(voltage, float)
    
    armed = drone.motors_armed()
    assert isinstance(armed, bool)
    assert not armed # shouldn't be armed initially


def test_parameter_write(drone):
    """Test modifying a flight controller parameter."""
    from pymavlink.mavutil import mavlink
    
    # Disable arming checks
    success = drone.write_parameter("ARMING_CHECK", 0, mavlink.MAV_PARAM_TYPE_REAL32)
    assert success is True
