import time
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS


import time
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS


def _upload_mission(drone: MAVLinCS, lat0: float, lon0: float, include_takeoff_and_rtl: bool):
    assert drone.clear_waypoints(timeout=3) is True

    p1 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=15.0, east_displacement=0.0)
    p2 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=15.0, east_displacement=15.0)

    wps = [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat0, lon0, 0.0)]  # seq 0 = HOME
    if include_takeoff_and_rtl:
        wps.append((mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, lat0, lon0, 10.0))

    wps.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, p1.latitude, p1.longitude, 10.0))
    wps.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, p2.latitude, p2.longitude, 10.0))

    if include_takeoff_and_rtl:
        wps.append((mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0, 0.0))

    assert drone.send_waypoint_count(len(wps)) is True
    for seq, (cmd, lat, lon, alt) in enumerate(wps):
        time.sleep(0.05)
        assert drone.send_waypoint(
            waypoint_index=seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=cmd,
            x=lat, y=lon, z=alt,
        ) is True

    assert drone.wait_mission_ack(timeout=5) is True


def _wait_mission_completion(drone: MAVLinCS, timeout: float = 90.0) -> bool:
    start = time.time()
    while time.time() - start < timeout:
        if drone.is_waypoint_mission_completed():
            return True
        time.sleep(0.5)
    return False


def test_mission_started_in_air(drone):
    """Part A: Takeoff in GUIDED then launch mission in flight (first_wp_tkoff=False)."""
    gps0 = drone.position_gps()
    _upload_mission(drone, gps0.lat, gps0.lon, include_takeoff_and_rtl=False)

    assert drone.arm(timeout_arming=10) is True
    assert drone.takeoff(altitude=10.0, timeout_takeoff=30) is True

    assert drone.start_mission(first_wp_tkoff=False, timeout=5) is True
    assert _wait_mission_completion(drone, timeout=90) is True

    assert drone.land(timeout_land=60) is True
    time.sleep(2.0)


def test_mission_started_from_ground_with_rtl(drone):
    """Part B: Full mission from the ground with integrated Takeoff and RTL (first_wp_tkoff=True)."""
    gps0 = drone.position_gps()
    _upload_mission(drone, gps0.lat, gps0.lon, include_takeoff_and_rtl=True)

    assert drone.start_mission(first_wp_tkoff=True, timeout=10) is True
    assert _wait_mission_completion(drone, timeout=120) is True
    assert drone.wait_motors_disarmed(timeout=30) is True
