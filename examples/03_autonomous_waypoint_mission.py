"""
Example 3: Autonomous Waypoint Mission (AUTO Mode).

Demonstrates how to generate a triangular waypoint mission around the
starting location, upload it reliably to the flight controller using
upload_mission() (which handles the full MAVLink Mission Protocol
MISSION_COUNT -> MISSION_REQUEST_INT / MISSION_REQUEST -> MISSION_ITEM_INT ->
MISSION_ACK handshake with automatic retries), start the mission, and monitor
waypoint progression until completion.
"""

import time
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS, MissionItem


def build_and_upload_mission(drone: MAVLinCS, lat0: float, lon0: float):
    """Builds and uploads a triangular mission with Takeoff and RTL via upload_mission()."""
    # Compute 3 triangle vertices around the origin using Cartesian offsets (meters)
    wp1 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=25.0, east_displacement=0.0)
    wp2 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=25.0, east_displacement=25.0)
    wp3 = MAVLinCS.cartesian_to_geographic_point(lat0, lon0, north_displacement=0.0, east_displacement=25.0)

    # Define mission items using MissionItem (tuples (cmd, lat, lon, alt) or dicts are also supported)
    # Note: Sequence 0 on ArduPilot is reserved for the HOME position
    mission_items = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=lat0, y=lon0, z=0.0),               # Seq 0: HOME
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, x=lat0, y=lon0, z=12.0),               # Seq 1: Takeoff to 12m
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=wp1.latitude, y=wp1.longitude, z=12.0),  # Seq 2: Corner 1
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=wp2.latitude, y=wp2.longitude, z=18.0),  # Seq 3: Corner 2 (climb to 18m)
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=wp3.latitude, y=wp3.longitude, z=12.0),  # Seq 4: Corner 3 (descend to 12m)
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH),                              # Seq 5: RTL
    ]

    print(f"Uploading {len(mission_items)} waypoints via MAVLink Mission Protocol...")
    if not drone.upload_mission(waypoints=mission_items, clear_existing=True, timeout=15.0):
        raise RuntimeError("Mission upload failed or was not acknowledged by the flight controller")
    print("Mission uploaded successfully!")


def upload_mission_low_level_example(drone: MAVLinCS, lat0: float, lon0: float):
    """Alternative low-level manual upload using clear_waypoints, send_waypoint_count, and send_waypoint."""
    if not drone.clear_waypoints():
        raise RuntimeError("Could not clear waypoints")

    wps = [
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat0, lon0, 0.0),
        (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, lat0, lon0, 10.0),
        (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0, 0.0),
    ]
    drone.send_waypoint_count(len(wps))
    for seq, (cmd, lat, lon, alt) in enumerate(wps):
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
        raise RuntimeError("Low-level mission upload was not acknowledged")


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
