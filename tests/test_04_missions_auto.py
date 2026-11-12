import time
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS, MissionItem


def _upload_mission(drone: MAVLinCS, lat0: float, lon0: float, include_takeoff_and_rtl: bool):
    p1 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=15.0, east_displacement=0.0)
    p2 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=15.0, east_displacement=15.0)

    wps = [MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=lat0, y=lon0, z=0.0)]  # seq 0 = HOME
    if include_takeoff_and_rtl:
        wps.append(MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, x=lat0, y=lon0, z=10.0))

    wps.append(MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=p1.latitude, y=p1.longitude, z=10.0))
    wps.append(MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=p2.latitude, y=p2.longitude, z=10.0))

    if include_takeoff_and_rtl:
        wps.append(MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH))

    assert drone.upload_mission(waypoints=wps, clear_existing=True, timeout=15.0) is True


def _wait_mission_completion(drone: MAVLinCS, timeout: float = 90.0) -> bool:
    start = time.time()
    saw_valid_seq = False
    while time.time() - start < timeout:
        seq = drone.current_waypoint_seq()
        if seq is not None and seq >= 0:
            saw_valid_seq = True
        if drone.is_waypoint_mission_completed():
            assert saw_valid_seq is True
            return True
        time.sleep(0.5)
    return False


def test_mission_started_in_air(drone):
    """Part A: Takeoff in GUIDED then launch mission in flight."""
    gps0 = drone.position_gps()
    _upload_mission(drone, gps0.lat, gps0.lon, include_takeoff_and_rtl=False)

    assert drone.arm(timeout_arming=10) is True
    assert drone.takeoff(altitude=10.0, timeout_takeoff=30) is True

    assert drone.start_mission(timeout=5) is True
    assert _wait_mission_completion(drone, timeout=90) is True

    assert drone.land(timeout_land=60) is True
    time.sleep(2.0)


def test_mission_started_from_ground_with_rtl(drone):
    """Part B: Full mission from the ground with integrated Takeoff and RTL."""
    gps0 = drone.position_gps()
    _upload_mission(drone, gps0.lat, gps0.lon, include_takeoff_and_rtl=True)

    assert drone.start_mission(timeout=10) is True
    assert _wait_mission_completion(drone, timeout=120) is True
    assert drone.wait_motors_disarmed(timeout=30) is True
