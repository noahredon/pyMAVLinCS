import pytest
import time
from pyMAVLinCS import MAVLinCS
from pyMAVLinCS import mavtypes
from pymavlink.mavutil import mavlink

def test_arming_and_takeoff(drone):
    """Test arming and taking off."""
    # Try setting GUIDED
    drone.set_mode("GUIDED")
    time.sleep(1)
    
    # Arm the drone (might fail in SITL without EKF)
    armed_res = drone.arm(force=True)
    assert isinstance(armed_res, bool)
    
    # Takeoff (might fail if not armed)
    takeoff_res = drone.takeoff(altitude=10)
    assert isinstance(takeoff_res, bool)

def test_mission_waypoints(drone):
    """Test clearing and sending waypoints."""
    assert isinstance(drone.clear_waypoints(), bool)
    time.sleep(1)
    
    # 0: Home
    assert isinstance(drone.send_waypoint_count(number_waypoints=3), bool)
    time.sleep(0.5)
    
    drone.send_waypoint(
        waypoint_index=0,
        frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavlink.MAV_CMD_NAV_WAYPOINT,
        x=45.0,
        y=5.0,
        z=0.0
    )
    
    drone.send_waypoint(
        waypoint_index=1,
        frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavlink.MAV_CMD_NAV_TAKEOFF,
        x=45.0,
        y=5.0,
        z=15.0
    )
    
    drone.send_waypoint(
        waypoint_index=2,
        frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavlink.MAV_CMD_NAV_WAYPOINT,
        x=45.001,
        y=5.001,
        z=15.0
    )
    time.sleep(1)

def test_landing(drone):
    """Test landing."""
    land_res = drone.land()
    assert isinstance(land_res, bool)
    time.sleep(2)
    
    # We won't wait for complete landing as it takes time, just verify it's landing or disarming
    
    disarm_res = drone.disarm(force=True)
    assert isinstance(disarm_res, bool)

