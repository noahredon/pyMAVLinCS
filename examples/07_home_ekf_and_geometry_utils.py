"""
Example 7: Home Position, EKF Origin, and Geodesic / Math Utilities.

Demonstrates how to query and modify the vehicle's HOME position and EKF
origin, compute live 2D/3D distances to GPS or local NED coordinates, and use
all built-in static mathematical and geographic conversion utilities.
"""

import math
import time
from pyMAVLinCS import MAVLinCS


def demonstrate_offline_math_and_geodesy():
    """Demonstrates static geometric and geodesic helper methods (no drone required)."""
    print("=== 1. Offline Mathematical & Geodesic Utilities ===")

    # Convert a quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw in radians)
    q_sample = [0.9238795, 0.0, 0.0, 0.3826834]  # ~45° yaw rotation
    euler = MAVLinCS.quaternion_to_euler(q=q_sample)
    print(
        f"Quaternion -> Euler : roll={math.degrees(euler.roll):.1f}°, "
        f"pitch={math.degrees(euler.pitch):.1f}°, yaw={math.degrees(euler.yaw):.1f}°"
    )

    # Local 2D and 3D Euclidean distances
    d2d_local = MAVLinCS.distance_between_two_local_points(x_i=0.0, x_f=30.0, y_i=0.0, y_f=40.0)
    d3d_local = MAVLinCS.distance_between_two_local_points(
        x_i=0.0, x_f=30.0, y_i=0.0, y_f=40.0, z_i=0.0, z_f=-12.0
    )
    print(f"Local distance      : 2D={d2d_local:.2f} m | 3D={d3d_local:.2f} m")

    # Project a GPS coordinate using Cartesian (North/East) and Polar (distance/bearing) offsets
    lat0, lon0 = 48.709800, 2.135500
    pt_cart = MAVLinCS.cartesian_to_geographic_point(
        lat_origine=lat0, lon_origine=lon0, north_displacement=100.0, east_displacement=50.0
    )
    pt_polar = MAVLinCS.polar_to_geographic_point(
        lat_origine=lat0, lon_origine=lon0, distance_m=100.0, bearing=90.0
    )
    print(f"Cartesian target    : ({pt_cart.latitude:.6f}°, {pt_cart.longitude:.6f}°)")
    print(f"Polar target (East) : ({pt_polar.latitude:.6f}°, {pt_polar.longitude:.6f}°)")

    # Recover Cartesian displacements (North, East) between two GPS coordinates
    delta_north, delta_east = MAVLinCS.geographic_to_cartesian_point(
        lat_i=lat0, lon_i=lon0, lat_f=pt_cart.latitude, lon_f=pt_cart.longitude
    )
    print(f"Inverse Cartesian   : dNorth={delta_north:.2f} m, dEast={delta_east:.2f} m")

    # Compute 2D and 3D distances between two GPS points
    dist_gps_2d = MAVLinCS.distance_between_two_gps_points(
        lat_i=lat0, lat_f=pt_cart.latitude, lon_i=lon0, lon_f=pt_cart.longitude
    )
    dist_gps_3d = MAVLinCS.distance_between_two_gps_points(
        lat_i=lat0,
        lat_f=pt_cart.latitude,
        lon_i=lon0,
        lon_f=pt_cart.longitude,
        relative_alt_i=0.0,
        relative_alt_f=25.0,
    )
    print(f"GPS distance        : 2D={dist_gps_2d:.2f} m | 3D={dist_gps_3d:.2f} m\n")


def main():
    demonstrate_offline_math_and_geodesy()

    print("=== 2. Online HOME Position, EKF Origin & Live Distances ===")
    drone = MAVLinCS(
        address="127.0.0.1:14550",
        mavlogfile="flight_07_home_and_geometry.tlog",
        send_automatic_home_request=True,
        home_request_delay=3,
    )

    try:
        drone.wait_gps_3d_fix(timeout=30)

        # Actively request HOME position from the autopilot and read cached value
        requested_home = drone.request_home_position(timeout_ack=2, timeout_home_reception=5)
        cached_home = drone.home_position()
        print(f"Requested HOME      : {requested_home}")
        print(f"Cached HOME         : {cached_home}")

        # Set HOME to the drone's current location, then set a custom HOME offset
        drone.set_home_current_pos()
        if not math.isnan(cached_home.latitude_home):
            new_home_pt = MAVLinCS.cartesian_to_geographic_point(
                lat_origine=cached_home.latitude_home,
                lon_origine=cached_home.longitude_home,
                north_displacement=5.0,
                east_displacement=5.0,
            )
            drone.set_home(
                latitude=new_home_pt.latitude,
                longitude=new_home_pt.longitude,
                altitude=cached_home.altitude_home,
                yaw=0.0,
            )
            updated_home = drone.wait_for_home(timeout=5)
            print(f"Updated HOME        : {updated_home}")

        # Inspect or initialize the EKF global origin
        origin = drone.ekf_origin()
        print(f"Current EKF Origin  : {origin}")
        if math.isnan(origin.latitude) and not math.isnan(cached_home.latitude_home):
            drone.set_ekf_origin(
                latitude=cached_home.latitude_home,
                longitude=cached_home.longitude_home,
                altitude=cached_home.altitude_home,
            )
            time.sleep(0.5)
            print(f"EKF Origin after set: {drone.ekf_origin()}")

        # Measure live distance from the drone to a GPS point and a local NED point
        gps_now = drone.position_gps()
        waypoint_ahead = MAVLinCS.polar_to_geographic_point(
            lat_origine=gps_now.lat, lon_origine=gps_now.lon, distance_m=50.0, bearing=0.0
        )
        dist_to_wp = drone.get_distance_with_gps_pos(
            lat=waypoint_ahead.latitude,
            lon=waypoint_ahead.longitude,
            relative_alt=gps_now.relative_alt,
        )
        dist_to_ned_origin = drone.get_distance_with_local_pos(x=0.0, y=0.0, z=0.0)
        print(f"Distance to +50m North WP : {dist_to_wp:.2f} m")
        print(f"Distance to NED (0, 0, 0) : {dist_to_ned_origin:.2f} m")

    except KeyboardInterrupt:
        drone.mission_ended = True
    finally:
        drone.close()


if __name__ == "__main__":
    main()
