"""
Example 8: Autonomous Multi-Vehicle Following (FOLLOW Mode).

Demonstrates how a follower drone running ArduPilot can automatically take off
and track another moving MAVLink vehicle (leader) on the same communication link
using do_follow_with_takeoff() and do_follow().
"""

import threading
from pyMAVLinCS import MAVLinCS


def main():
    # System ID of the follower drone we control (e.g., sysid 1)
    follower_sysid = 1
    # System ID of the target/leader vehicle to follow on the same link (e.g., sysid 2)
    leader_sysid = 2

    drone = MAVLinCS(
        address="127.0.0.1:14550",
        target_system=follower_sysid,
        mavlogfile="flight_08_follow.tlog",
        pos_rate=10,
    )

    try:
        print("Waiting for 3D GPS fix on follower drone...")
        if not drone.wait_gps_3d_fix(timeout=30):
            print("Failed to acquire 3D GPS fix.")
            return

        if not drone.arm(timeout_arming=10):
            print("Failed to arm follower drone.")
            return

        # Automatically stop the blocking follow loop after 20 seconds for this demo
        def stop_following():
            print("\nFollow duration elapsed: stopping FOLLOW mode...")
            drone.mission_ended = True

        # Phase 1: Take off to 5m and enter FOLLOW mode for 10 seconds
        stop_timer_1 = threading.Timer(10.0, stop_following)
        stop_timer_1.daemon = True
        stop_timer_1.start()

        print(f"Phase 1: Taking off and following leader (sysid={leader_sysid})...")
        drone.do_follow_with_takeoff(target_system=leader_sysid)
        stop_timer_1.cancel()

        # Phase 2: Already airborne — resume following directly via do_follow()
        drone.mission_ended = False
        stop_timer_2 = threading.Timer(10.0, stop_following)
        stop_timer_2.daemon = True
        stop_timer_2.start()

        print(f"Phase 2: Resuming in-air follow of leader (sysid={leader_sysid})...")
        drone.do_follow(target_system=leader_sysid)
        stop_timer_2.cancel()

        # Reset mission_ended flag so we can command a safe landing
        drone.mission_ended = False
        print("Landing follower drone...")
        drone.land(timeout_land=60)

    except KeyboardInterrupt:
        print("\nFollow interrupted by user! Landing...")
        drone.mission_ended = False
        drone.land(timeout_land=60)
        drone.mission_ended = True
    finally:
        drone.close()


if __name__ == "__main__":
    main()
