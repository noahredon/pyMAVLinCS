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
        # Look up registered MCCs by numeric value or by name
        print(f"Lookup MCC 101 -> {MCC.get(101)}")
        print(f"Lookup 'TARGET_DETECTED' -> {MCC.get_name('TARGET_DETECTED')}")

        # Register high-level and packet-level callbacks
        drone.set_statustext_callback(on_autopilot_statustext)
        drone.set_mcc_callback(on_mcc_received)

        sent_packet_count = 0
        recv_packet_count = 0

        def on_msg_sent(msg):
            nonlocal sent_packet_count
            sent_packet_count += 1

        def on_msg_received(msg):
            nonlocal recv_packet_count
            recv_packet_count += 1

        drone.set_additional_send_callback(on_msg_sent)
        drone.set_additional_recv_msg_callback(on_msg_received)

        # Send a status text message to the GCS / flight log
        drone.send_statustext("Companion computer online and connected via pyMAVLinCS")

        # Broadcast a TIMESYNC and inspect link latency
        drone.send_timesync()
        time.sleep(1.0)
        autopilot_key = (drone.master.target_system, drone.master.target_component)
        latency_ms = drone.get_latence_ms_with_sysidcompid(autopilot_key)
        print(f"Estimated one-way link latency: {latency_ms} ms")

        # Retrieve the last complete STATUSTEXT received from the autopilot
        last_status = drone.last_statustext_from_sysidcompid(autopilot_key)
        if last_status is not None:
            print(f"Last autopilot STATUSTEXT: {last_status.text}")

        # Send a generic NAMED_VALUE_INT metric and custom MCC states
        drone.send_named_value_int(value=42, name="CPU_TEMP")
        drone.send_mcc(MCC_PAYLOAD_READY)
        if drone.check_mcc(MCC_PAYLOAD_READY):
            print("Confirmed: PAYLOAD_READY state is active.")
            drone.delete_mcc(MCC_PAYLOAD_READY)

        # Actuate a payload servo on channel 9 and read back its PWM output
        drone.set_servo(servo_channel=9, pwm=1500)
        time.sleep(0.5)
        print(f"Servo 9 PWM: {drone.servo_pwm(servo_channel=9)}")
        print(f"Packets observed by callbacks -> Sent: {sent_packet_count}, Received: {recv_packet_count}")

    except KeyboardInterrupt:
        drone.mission_ended = True
    finally:
        drone.close()


if __name__ == "__main__":
    main()
