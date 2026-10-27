"""
Example 4: Custom Mission Control Codes (MCC), Callbacks, and Logging.

Demonstrates how to define custom MCCs for ground/companion-computer
synchronization, register asynchronous callbacks for STATUSTEXT and MCC
events, measure link latency via TIMESYNC, and configure custom logging.
"""

import logging
import time
from pyMAVLinCS import MAVLinCS, MCC, setup_logger

# 1. Define custom Mission Control Codes (MCC)
MCC_PAYLOAD_READY = MCC(
    value=101,
    name="PAYLOAD_READY",
    level="INFO",
    description="Payload sensor initialized and ready",
)
MCC_TARGET_DETECTED = MCC(
    value=102,
    name="TARGET_DETECTED",
    level="SUCCESS",
    description="Visual target detected by onboard computer",
)


# 2. Define asynchronous event callbacks (keep callbacks non-blocking)
def on_autopilot_statustext(statustext_msg):
    print(f"[Autopilot Status] Severity={statustext_msg.severity} | Text='{statustext_msg.text}'")


def on_mcc_received(mcc: MCC):
    print(f"[MCC Event] Received code {int(mcc)}: {mcc.description}")


def main():
    # Set up a custom logger with console timestamps and file output
    custom_logger = logging.getLogger("my_drone_app")
    setup_logger(
        logger=custom_logger,
        log_file="app_mission.log",
        debug_mode=False,
        add_timestamp_to_console=True,
        add_timestamp_to_file=True,
    )

    drone = MAVLinCS(
        address="127.0.0.1:14550",
        logger=custom_logger,
        mavlogfile="flight_04_callbacks.tlog"
    )

    try:
        # Register callbacks
        drone.set_statustext_callback(on_autopilot_statustext)
        drone.set_mcc_callback(on_mcc_received)

        # Send a status text message to the GCS / flight log
        drone.send_statustext("Companion computer online and connected via pyMAVLinCS")

        # Broadcast a TIMESYNC and inspect link latency
        drone.send_timesync()
        time.sleep(1.0)
        autopilot_key = (drone.master.target_system, drone.master.target_component)
        latency_ms = drone.get_latence_ms_with_sysidcompid(autopilot_key)
        print(f"Estimated one-way link latency: {latency_ms} ms")

        # Send and verify custom MCC states
        drone.send_mcc(MCC_PAYLOAD_READY)
        if drone.check_mcc(MCC_PAYLOAD_READY):
            print("Confirmed: PAYLOAD_READY state is active.")
            drone.delete_mcc(MCC_PAYLOAD_READY)

        # Actuate a payload servo on channel 9
        drone.set_servo(servo_channel=9, pwm=1500)
        time.sleep(1.0)

    except KeyboardInterrupt:
        drone.mission_ended = True
    finally:
        drone.close()


if __name__ == "__main__":
    main()
