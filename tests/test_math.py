import pytest
import math
from pyMAVLinCS import MAVLinCS
from pyMAVLinCS import mavtypes

def test_quaternion_to_euler():
    # Identity quaternion
    q = [1.0, 0.0, 0.0, 0.0]
    angles = MAVLinCS.quaternion_to_euler(q)
    assert math.isclose(angles.roll, 0.0, abs_tol=1e-5)
    assert math.isclose(angles.pitch, 0.0, abs_tol=1e-5)
    assert math.isclose(angles.yaw, 0.0, abs_tol=1e-5)

def test_distance_between_two_local_points():
    dist = MAVLinCS.distance_between_two_local_points(0, 3, 0, 4)
    assert math.isclose(dist, 5.0)

    dist_3d = MAVLinCS.distance_between_two_local_points(0, 0, 0, 0, 0, 10)
    assert math.isclose(dist_3d, 10.0)

def test_distance_between_two_gps_points():
    # Paris and London rough coords
    dist = MAVLinCS.distance_between_two_gps_points(48.8566, 51.5072, 2.3522, -0.1276)
    assert dist > 300000 and dist < 400000 # ~344km

def test_cartesian_to_geographic_point():
    lat = 45.0
    lon = 5.0
    point = MAVLinCS.cartesian_to_geographic_point(lat, lon, 1000, 0) # 1km North
    assert point.latitude > lat
    assert math.isclose(point.longitude, lon, abs_tol=1e-3)

def test_polar_to_geographic_point():
    lat = 45.0
    lon = 5.0
    point = MAVLinCS.polar_to_geographic_point(lat, lon, 1000, 90) # 1km East (bearing 90)
    assert point.longitude > lon
    assert math.isclose(point.latitude, lat, abs_tol=1e-3)

def test_geographic_to_cartesian_point():
    lat1, lon1 = 45.0, 5.0
    lat2, lon2 = 45.01, 5.01
    
    dn, de = MAVLinCS.geographic_to_cartesian_point(lat1, lon1, lat2, lon2)
    assert dn > 0 # Moved North
    assert de > 0 # Moved East
