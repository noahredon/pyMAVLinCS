"""
Example 5: Flight Modes, Parameters, Low-Level Messaging, and Connection Lifecycle.

Demonstrates how to inspect system timing and dialect properties, query flight
mode mappings and PX4/ArduPilot mode numbers, adjust message stream rates,
write flight controller parameters, access raw MAVLink messages via get_msg()
and recv_match(), synchronize explicit mode changes and arm/disarm states,
and manage the connection lifecycle (close / open / reboot).
"""

import time
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS


def main():
    address = "127.0.0.1:14550"
    drone = MAVLinCS(
        address=address,
        mavlogfile="flight_05_modes_and_low_level.tlog",
        pos_rate=4,
    )

    try:
        # ------------------------------------------------------------------
        # 1. System Timing, Dialect, and Detailed Mode Inspection
        # ------------------------------------------------------------------
        print("--- System & Mode Diagnostics ---")
        print(f"MAVLink Dialect  : {drone.dialect}")
        print(f"Time since boot  : {drone.time_boot} s ({drone.time_boot_ms} ms)")
        print(f"Unix Timestamp   : {drone.time_usec} us")
        print(f"Autopilot        : {drone.autopilot_str()} (int={drone.autopilot_int()})")
        print(f"Current Mode     : {drone.mode()} (custom_mode={drone.custom_mode()})")
        print(
            f"Mode Bitfields   : base_mode={drone.base_mode()}, "
            f"main={drone.custom_main_mode()}, sub={drone.custom_sub_mode()}"
        )

        available_modes = drone.mode_mapping()
        if available_modes:
            print(f"Available Modes  : {list(available_modes.keys())}")

        # ------------------------------------------------------------------
        # 2. Message Rates, Flight Controller Parameters & Raw Messages
        # ------------------------------------------------------------------
        print("\nConfiguring ATTITUDE stream rate to 10 Hz...")
        drone.set_msg_rate(msg_type="ATTITUDE", rate=10.0, timeout_ack=2)

        print("Writing parameter SYSID_MYGCS = 255...")
        drone.write_parameter(
            parameter="SYSID_MYGCS",
            value=255,
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        )

        # Retrieve the latest cached HEARTBEAT message (at most 2 seconds old)
        hb_msg = drone.get_msg(msg_type="HEARTBEAT", max_age=2.0)
        if hb_msg is not None:
            print(f"Cached HEARTBEAT : system_status={hb_msg.system_status}")

        # Wait for the next incoming SYS_STATUS packet using recv_match()
        sys_msg = drone.recv_match(type="SYS_STATUS", blocking=True, timeout=3.0)
        if sys_msg is not None:
            print(f"Matched SYS_STATUS: load={sys_msg.load / 10.0:.1f}%")

        # Send a custom MAV_CMD_REQUEST_MESSAGE for AUTOPILOT_VERSION and wait for ACK
        drone.master.mav.command_long_send(
            drone.master.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            1,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0, 0, 0, 0, 0, 0,
        )
        ack_ok = drone.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            timeout=2.0,
        )
        print(f"AUTOPILOT_VERSION request ACK accepted: {ack_ok}")

        # ------------------------------------------------------------------
        # 3. Explicit Flight Mode Transitions & Synchronized Arm / Disarm
        # ------------------------------------------------------------------
        drone.wait_gps_3d_fix(timeout=30)

        # Switch to GUIDED mode without blocking inside set_mode, then wait explicitly
        print("\nSwitching to GUIDED mode...")
        drone.set_mode(mode="GUIDED", timeout_ack=2, timeout_set_mode=0)
        if drone.wait_mode_changed(mode="GUIDED", timeout=5):
            print(f"Confirmed mode change to {drone.mode()}")

        # Arm on the ground without taking off, wait for arming confirmation, then disarm
        print("Arming motors on the ground (timeout_arming=0)...")
        if drone.arm(force=False, timeout_ack=2, timeout_arming=0):
            if drone.wait_motors_armed(timeout=5):
                print("Motors confirmed ARMED!")
                time.sleep(2.0)

                print("Disarming motors...")
                if drone.disarm(force=False, timeout_ack=2, timeout_disarming=5):
                    drone.wait_motors_disarmed(timeout=5)
                    print("Motors confirmed DISARMED!")

        # ------------------------------------------------------------------
        # 4. Connection Re-opening & Optional Autopilot Reboot
        # ------------------------------------------------------------------
        print("\nClosing and re-opening MAVLink connection...")
        drone.close()
        time.sleep(1.0)
        drone.open(address=address, timeout_heartbeat=15, pos_rate=4)
        print(f"Reconnected! Mode is still: {drone.mode()}")

        # Set to True when a modified parameter requires a flight controller reboot:
        reboot_after_config = False
        if reboot_after_config:
            drone.reboot(force=False, timeout_ack=2)

    except KeyboardInterrupt:
        drone.mission_ended = True
    finally:
        drone.close()


if __name__ == "__main__":
    main()
