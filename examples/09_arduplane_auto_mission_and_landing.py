"""
Example 9: ArduPlane Autonomous Mission (Takeoff, Return Path, Landing Sequence & Go-Around).

Demonstrates how to build, upload, and execute a complete fixed-wing AUTO mission
on ArduPlane using MissionItem, upload_mission(), start_mission(),
do_return_path_start(), do_land_start(), and do_go_around():
  - Seq 0: HOME position
  - Seq 1: Fixed-wing runway takeoff (MAV_CMD_NAV_TAKEOFF with 15 deg minimum pitch)
  - Seq 2: Outbound navigation waypoint (350m North, 60m altitude)
  - Seq 3: MAV_CMD_DO_RETURN_PATH_START marker (safe inbound corridor entry)
  - Seq 4: Return / base leg waypoint (300m South, 150m East, 45m altitude)
  - Seq 5: MAV_CMD_DO_LAND_START marker (landing pattern entry)
  - Seq 6: Final approach waypoint (180m South, 20m altitude, aligned with runway)
  - Seq 7: Autonomous fixed-wing runway landing (MAV_CMD_NAV_LAND at HOME, 0m altitude)

Note:
    Unlike multirotors (ArduCopter), a pure fixed-wing aircraft on ArduPlane does not
    have a standalone "LAND" flight mode; autonomous landings are triggered via
    MAV_CMD_DO_LAND_START (executed through a MAV_CMD_NAV_LAND mission item in AUTO mode)
    or ArduPlane's AUTOLAND mode. Calling ``plane.land()`` automatically uses
    ``do_land_start()`` (with fallback to ``AUTOLAND``) on fixed-wing aircraft, and
    ``QLAND`` on VTOL / QuadPlane vehicles.
"""

import time
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS, MissionItem


def main():
    with MAVLinCS(
        address="127.0.0.1:14550",
        mavlogfile="flight_09_arduplane_mission.tlog",
    ) as plane:
        try:
            print("Waiting for 3D GPS fix on ArduPlane...")
            if not plane.wait_gps_3d_fix(timeout=45):
                print("Failed to acquire 3D GPS fix.")
                return

            gps_home = plane.position_gps()
            print(f"Runway / HOME coordinates: lat={gps_home.lat:.6f}, lon={gps_home.lon:.6f}")

            # Compute fixed-wing circuit waypoints relative to the runway
            wp_upwind = MAVLinCS.cartesian_to_geographic_point(
                lat_origine=gps_home.lat,
                lon_origine=gps_home.lon,
                north_displacement=350.0,
                east_displacement=0.0,
            )
            wp_base = MAVLinCS.cartesian_to_geographic_point(
                lat_origine=gps_home.lat,
                lon_origine=gps_home.lon,
                north_displacement=-300.0,
                east_displacement=150.0,
            )
            wp_final = MAVLinCS.cartesian_to_geographic_point(
                lat_origine=gps_home.lat,
                lon_origine=gps_home.lon,
                north_displacement=-180.0,
                east_displacement=0.0,
            )

            mission_items = [
                # Seq 0: HOME location
                MissionItem(
                    command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    x=gps_home.lat,
                    y=gps_home.lon,
                    z=0.0,
                ),
                # Seq 1: Fixed-wing takeoff (param1 = minimum pitch in degrees, z = climb altitude in meters)
                MissionItem(
                    command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    param1=15.0,
                    x=gps_home.lat,
                    y=gps_home.lon,
                    z=50.0,
                ),
                # Seq 2: Outbound waypoint at 60m
                MissionItem(
                    command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    x=wp_upwind.latitude,
                    y=wp_upwind.longitude,
                    z=60.0,
                ),
                # Seq 3: Return-path segment marker (used by do_return_path_start())
                MissionItem(command=mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START),
                # Seq 4: Base leg waypoint descending to 45m
                MissionItem(
                    command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    x=wp_base.latitude,
                    y=wp_base.longitude,
                    z=45.0,
                ),
                # Seq 5: Landing sequence marker (used by do_land_start())
                MissionItem(command=mavutil.mavlink.MAV_CMD_DO_LAND_START),
                # Seq 6: Final approach waypoint at 20m aligned with the runway
                MissionItem(
                    command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    x=wp_final.latitude,
                    y=wp_final.longitude,
                    z=20.0,
                ),
                # Seq 7: Fixed-wing runway touchdown at HOME
                MissionItem(
                    command=mavutil.mavlink.MAV_CMD_NAV_LAND,
                    x=gps_home.lat,
                    y=gps_home.lon,
                    z=0.0,
                ),
            ]

            # Allow arming with DO_LAND_START / DO_RETURN_PATH_START on ArduPlane (RTL_AUTOLAND=0 blocks arming)
            plane.write_parameter("RTL_AUTOLAND", 3.0, timeout_ack=2.0)

            print(f"Uploading fixed-wing mission ({len(mission_items)} items)...")
            if not plane.upload_mission(waypoints=mission_items, clear_existing=True, timeout=15.0):
                print("Mission upload failed.")
                return

            # start_mission() automatically arms ArduPlane and switches to AUTO mode
            print("Starting AUTO mission from the runway...")
            if not plane.start_mission(timeout=10):
                print("Failed to start mission.")
                return

            # Wait until the aircraft climbs above 40m and reaches the outbound leg
            while plane.position_gps().relative_alt < 40.0:
                time.sleep(1.0)

            # Example: cut the outbound leg short by joining the safe return path via do_return_path_start()
            print("Joining safe return path via do_return_path_start()...")
            plane.do_return_path_start(timeout_ack=3)
            time.sleep(3.0)

            # Example: jump directly into the mission landing pattern via do_land_start()
            print("Jumping to mission landing sequence via do_land_start()...")
            plane.do_land_start(timeout_ack=3)

            go_around_tested = False
            while not plane.is_waypoint_mission_completed():
                seq = plane.current_waypoint_seq()
                pos = plane.position_gps()
                hud = plane.vfr_hud()
                wnd = plane.wind()
                print(
                    f"[AUTO] WP #{seq} | "
                    f"Alt: {pos.relative_alt:.1f}m (Climb: {hud.climb:+.1f}m/s) | "
                    f"Airspeed: {hud.airspeed:.1f}m/s (GS: {hud.groundspeed:.1f}m/s) | "
                    f"Hdg: {hud.heading}° | Thr: {hud.throttle}% | "
                    f"Wind: {wnd.speed:.1f}m/s @ {wnd.direction:.0f}°"
                )
                # Demonstrate a single go-around once on final (WP #7 = MAV_CMD_NAV_LAND), then land normally
                if seq == 7 and not go_around_tested:
                    print("Runway check failed -> aborting landing via do_go_around(altitude=45.0)!")
                    if plane.do_go_around(altitude=45.0, timeout_ack=3):
                        go_around_tested = True
                time.sleep(2.0)

            print("Mission completed! Waiting for aircraft to disarm after landing roll...")
            plane.wait_motors_disarmed(timeout=60)
            print("ArduPlane disarmed safely on the runway.")

        except KeyboardInterrupt:
            print("\nMission interrupted by user! Switching to RTL...")
            plane.mission_ended = False
            plane.return_to_launch()
            plane.mission_ended = True


if __name__ == "__main__":
    main()
