"""
Example 2: Guided Navigation (Local NED & GPS) and Velocity Control.

Demonstrates how to fly to local and global coordinates, use built-in
geometric projections, control vehicle velocity along a circular path,
aim a gimbal at the HOME position, and trigger Return-To-Launch (RTL).
"""

import math
import time
from pyMAVLinCS import MAVLinCS


def main():
    drone = MAVLinCS(
        address="127.0.0.1:14550",
        mavlogfile="flight_02_navigation.tlog",
        pos_rate=10
    )

    try:
        drone.wait_gps_3d_fix(timeout=30)
        home = drone.request_home_position(timeout_ack=2, timeout_home_reception=5)

        if not drone.arm() or not drone.takeoff(altitude=15.0):
            return

        # ------------------------------------------------------------------
        # 1. Local NED Navigation: Fly 20m North, 10m East, at 15m altitude
        # Note: In NED frame, Z points downwards (-15.0 = 15m above origin)
        # ------------------------------------------------------------------
        print("Flying to local NED target (+20m North, +10m East)...")
        start_local = drone.position_local()
        target_x = start_local.x + 20.0
        target_y = start_local.y + 10.0
        target_z = -15.0

        drone.set_local_pos_target(x=target_x, y=target_y, z=target_z, yaw=math.radians(30))
        drone.wait_proximity_with_local_pos(
            x=target_x, y=target_y, z=target_z, radius_acceptance=1.5, timeout=30
        )

        # ------------------------------------------------------------------
        # 2. Global GPS Navigation using Polar Projection
        # Compute a GPS point 30 meters away at a 135° bearing (South-East)
        # ------------------------------------------------------------------
        print("Flying to projected GPS target (30m at 135° bearing)...")
        current_gps = drone.position_gps()
        target_gps = MAVLinCS.polar_to_geographic_point(
            lat_origine=current_gps.lat,
            lon_origine=current_gps.lon,
            distance_m=30.0,
            bearing=135.0,
        )

        drone.set_gps_pos_target(
            latitude=target_gps.latitude,
            longitude=target_gps.longitude,
            relative_altitude=15.0,
        )
        drone.wait_proximity_with_gps_pos(
            lat=target_gps.latitude,
            lon=target_gps.longitude,
            relative_alt=15.0,
            radius_acceptance=1.5,
            timeout=30,
        )

        # ------------------------------------------------------------------
        # 3. Point Gimbal (ROI) towards HOME while flying a velocity circle
        # ------------------------------------------------------------------
        if not math.isnan(home.latitude_home):
            drone.set_gimbal_target(
                latitude=home.latitude_home,
                longitude=home.longitude_home,
                altitude=home.altitude_home,
            )

        print("Flying a horizontal circle using velocity control (4 m/s)...")
        duration_s = 12.0
        dt = 0.2
        speed_ms = 4.0
        steps = int(duration_s / dt)

        for step in range(steps):
            angle = 2.0 * math.pi * (step / steps)
            vx = speed_ms * math.cos(angle)
            vy = speed_ms * math.sin(angle)
            drone.set_speed(vx=vx, vy=vy, vz=0.0, yaw=angle)
            time.sleep(dt)

        # Stop movement
        drone.set_speed(vx=0.0, vy=0.0, vz=0.0)
        time.sleep(2)

        # ------------------------------------------------------------------
        # 4. Return To Launch (RTL) and wait for landing
        # ------------------------------------------------------------------
        print("Returning to launch (RTL)...")
        drone.return_to_launch()
        drone.wait_motors_disarmed(timeout=90)
        print("RTL completed and motors disarmed!")

    except KeyboardInterrupt:
        print("\nInterrupted! Triggering RTL...")
        drone.mission_ended = False
        drone.return_to_launch()
        drone.mission_ended = True
    finally:
        drone.close()


if __name__ == "__main__":
    main()
