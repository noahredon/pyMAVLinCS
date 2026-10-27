"""
Example 3: Autonomous Waypoint Mission (AUTO Mode).

Demonstrates how to generate a triangular waypoint mission around the
starting location, upload it to the flight controller, start the mission,
and monitor waypoint progression until completion.
"""

import time
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS


def build_and_upload_mission(drone: MAVLinCS, lat0: float, lon0: float):
    """Clears existing waypoints and uploads a triangular mission with Takeoff and RTL."""
    print("Clearing previous mission...")
    if not drone.clear_waypoints():
        raise RuntimeError("Could not clear waypoints")

    # Compute 3 triangle vertices around the origin using Cartesian offsets (meters)
    wp1 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=25.0, east_displacement=0.0)
    wp2 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=25.0, east_displacement=25.0)
    wp3 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=0.0, east_displacement=25.0)

    # Define mission items: (MAV_CMD, lat, lon, relative_alt)
    # Note: Sequence 0 on ArduPilot is reserved for the HOME position
    mission_items = [
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat0, lon0, 0.0),               # Seq 0: HOME
        (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, lat0, lon0, 12.0),               # Seq 1: Takeoff to 12m
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, wp1.latitude, wp1.longitude, 12.0),  # Seq 2: Corner 1
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, wp2.latitude, wp2.longitude, 18.0),  # Seq 3: Corner 2 (climb to 18m)
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, wp3.latitude, wp3.longitude, 12.0),  # Seq 4: Corner 3 (descend to 12m)
        (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0, 0.0),         # Seq 5: RTL
    ]

    print(f"Uploading {len(mission_items)} waypoints...")
    drone.send_waypoint_count(len(mission_items))

    for seq, (cmd, lat, lon, alt) in enumerate(mission_items):
        time.sleep(0.05)
        drone.send_waypoint(
            waypoint_index=seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=cmd,
            x=lat,
            y=lon,
            z=alt,
        )

    if not drone.wait_mission_ack(timeout=5):
        raise RuntimeError("Mission upload was not acknowledged by the flight controller")
    print("Mission uploaded successfully!")


def main():
    drone = MAVLinCS(
        address="127.0.0.1:14550",
        mavlogfile="flight_03_mission.tlog"
    )

    try:
        drone.wait_gps_3d_fix(timeout=30)
        start_gps = drone.position_gps()

        # 1. Upload the waypoint mission
        build_and_upload_mission(drone, start_gps.lat, start_gps.lon)

        # 2. Arm and start the mission from the ground
        print("Starting autonomous mission...")
        if not drone.start_mission(timeout=10):
            print("Failed to start mission.")
            return

        # 3. Monitor mission progress
        last_seq = -1
        while not drone.is_waypoint_mission_completed():
            seq = drone.current_waypoint_seq()
            if seq != last_seq:
                print(f" -> Flying to Waypoint #{seq}")
                last_seq = seq
            time.sleep(0.5)

        print("Mission completed! Waiting for disarm...")
        drone.wait_motors_disarmed(timeout=30)
        print("Drone disarmed safely.")

    except KeyboardInterrupt:
        print("\nMission interrupted by user! Switching to RTL...")
        drone.mission_ended = False
        drone.return_to_launch()
        drone.mission_ended = True
    finally:
        drone.close()


if __name__ == "__main__":
    main()
