"""
Example 10: ArduPlane Fixed-Wing Modes (TAKEOFF, LOITER, GUIDED via do_reposition, RTL) & Parameters.

Demonstrates how to operate a fixed-wing aircraft (ArduPlane) without a pre-loaded
waypoint mission:
  1. Inspecting vehicle type (vehicle_type_str / vehicle_type_int) and fixed-wing
     flight modes via mode_mapping().
  2. Reading and configuring ArduPlane parameters (WP_LOITER_RAD).
  3. Performing an automatic climb-out using ArduPlane's built-in TAKEOFF mode.
  4. Switching to LOITER, then commanding a 3D GPS reposition in flight using
     do_reposition() (with force_guided=True, custom loiter_radius, and ccw=True),
     and finally returning home via return_to_launch() (RTL).
"""

import math
import time
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS


def main():
    with MAVLinCS(
        address="127.0.0.1:14550",
        mavlogfile="flight_10_arduplane_modes.tlog",
    ) as plane:
        try:
            print("Waiting for 3D GPS fix on ArduPlane...")
            if not plane.wait_gps_3d_fix(timeout=45):
                print("Failed to acquire 3D GPS fix.")
                return

            # 1. Inspect vehicle type and available ArduPlane modes
            print(f"Vehicle Type : {plane.vehicle_type_str()} (ID: {plane.vehicle_type_int()})")
            modes = plane.mode_mapping()
            if modes:
                print(f"ArduPlane modes ({len(modes)}): {list(modes.keys())}")

            # 2. Read and adjust the fixed-wing loiter radius (WP_LOITER_RAD)
            loiter_rad = plane.get_parameter("WP_LOITER_RAD", timeout=2.0)
            if not math.isnan(loiter_rad):
                print(f"Current WP_LOITER_RAD: {loiter_rad:.1f}m")
                plane.write_parameter("WP_LOITER_RAD", 60.0, timeout_ack=2.0)

            # 3. Take off without a mission using ArduPlane's dedicated TAKEOFF mode
            print("\nSwitching to TAKEOFF mode and arming...")
            if not plane.set_mode("TAKEOFF", timeout_ack=3, timeout_set_mode=4):
                print("Failed to switch to TAKEOFF mode.")
                return

            if not plane.arm(timeout_arming=10):
                print("Failed to arm ArduPlane.")
                return

            print("Climbing out in TAKEOFF mode until relative altitude >= 40m...")
            start_t = time.time()
            while time.time() - start_t < 60:
                pos = plane.position_gps()
                print(f"  Alt: {pos.relative_alt:.1f}m | Speed: {plane.speed_module():.1f}m/s")
                if pos.relative_alt >= 40.0:
                    break
                time.sleep(1.5)

            # 4. Switch to LOITER mode to circle around current position
            print("\nSwitching to LOITER mode for 5 seconds...")
            plane.set_mode("LOITER", timeout_ack=3, timeout_set_mode=4)
            time.sleep(5.0)

            # 5. Reposition 250m North, 200m East at 70m relative altitude using do_reposition()
            cur_pos = plane.position_gps()
            target_pt = MAVLinCS.cartesian_to_geographic_point(
                lat_origine=cur_pos.lat,
                lon_origine=cur_pos.lon,
                north_displacement=250.0,
                east_displacement=200.0,
            )
            print("Repositioning to new 3D GPS loiter point via do_reposition(force_guided=True)...")
            if plane.do_reposition(
                latitude=target_pt.latitude,
                longitude=target_pt.longitude,
                altitude=70.0,
                loiter_radius=65.0,
                ccw=True,
                force_guided=True,
                timeout_ack=3,
            ):
                # Adjust target airspeed to 18 m/s while flying in GUIDED mode
                plane.change_speed(
                    speed=18.0,
                    speed_type=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
                    timeout_ack=2,
                )
                plane.wait_proximity_with_gps_pos(
                    lat=target_pt.latitude,
                    lon=target_pt.longitude,
                    radius_acceptance=90.0,
                    timeout=60,
                )
                print(f"Reached target loiter circle! Alt: {plane.position_gps().relative_alt:.1f}m")
                # Restore default airspeed (-2.0)
                plane.change_speed(
                    speed=-2.0,
                    speed_type=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
                    timeout_ack=2,
                )

            # 6. Return To Launch (RTL) — ArduPlane flies back and loiters above HOME
            print("\nTriggering Return-To-Launch (RTL)...")
            plane.return_to_launch(timeout_ack=3)
            plane.wait_mode_changed("RTL", timeout=5)

            home = plane.home_position()
            if not math.isnan(home.latitude_home):
                print("Waiting for aircraft to reach HOME loiter circle...")
                plane.wait_proximity_with_gps_pos(
                    lat=home.latitude_home,
                    lon=home.longitude_home,
                    radius_acceptance=100.0,
                    timeout=90,
                )
                print("Aircraft is loitering overhead HOME.")

        except KeyboardInterrupt:
            print("\nFlight interrupted by user! Triggering RTL...")
            plane.mission_ended = False
            plane.return_to_launch()
            plane.mission_ended = True


if __name__ == "__main__":
    main()
