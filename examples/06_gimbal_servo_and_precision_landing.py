"""
Example 6: Gimbal Stabilization, Payload Servos, and Precision Landing Target.

Demonstrates how to control a camera gimbal (neutral, pitch/yaw angles & rates,
GPS ROI targeting, and retract mode), actuate and read back servo PWM channels,
and stream visual/IR landing target offsets in the vehicle's body frame
(MAV_FRAME_BODY_FRD) during precision landing.
"""

import time
from pyMAVLinCS import MAVLinCS


def main():
    drone = MAVLinCS(
        address="127.0.0.1:14550",
        mavlogfile="flight_06_gimbal_and_landing.tlog",
        pos_rate=10,
    )

    try:
        drone.wait_gps_3d_fix(timeout=30)

        # Request the GIMBAL_DEVICE_ATTITUDE_STATUS message stream at 4 Hz
        drone.set_msg_rate(msg_type="GIMBAL_DEVICE_ATTITUDE_STATUS", rate=4.0)
        drone.set_msg_rate(msg_type="SERVO_OUTPUT_RAW", rate=4.0)

        # ------------------------------------------------------------------
        # 1. Gimbal Mode & Attitude Control (Pre-flight check)
        # ------------------------------------------------------------------
        print("Setting gimbal to neutral position (roll=0, pitch=0, yaw=0)...")
        drone.set_gimbal_neutral(timeout_ack=3)
        time.sleep(1.0)

        print("Orienting gimbal to -45° pitch (downward tilt) and +30° yaw...")
        drone.set_gimbal_angles(
            pitch=-45.0,
            yaw=30.0,
            pitch_rate=15.0,
            yaw_rate=15.0,
            lock_pitch=True,
            lock_yaw=False,
            yaw_in_earth_frame=False,
            timeout_ack=3,
        )
        time.sleep(1.5)

        gimbal_att = drone.gimbal_angles()
        print(
            f"Gimbal Attitude -> Roll: {gimbal_att.roll:.1f}°, "
            f"Pitch: {gimbal_att.pitch:.1f}°, Yaw: {gimbal_att.yaw:.1f}°"
        )

        # ------------------------------------------------------------------
        # 2. Payload Servo Actuation & PWM Readback
        # ------------------------------------------------------------------
        print("Testing payload release servo on channel 9...")
        drone.set_servo(servo_channel=9, pwm=1900, timeout_ack=2)
        time.sleep(0.5)
        print(f"Channel 9 live PWM (open)  : {drone.servo_pwm(servo_channel=9)}")

        drone.set_servo(servo_channel=9, pwm=1100, timeout_ack=2)
        time.sleep(0.5)
        print(f"Channel 9 live PWM (closed): {drone.servo_pwm(servo_channel=9)}")

        # ------------------------------------------------------------------
        # 3. Takeoff, ROI Tracking, and Precision Landing Target Streaming
        # ------------------------------------------------------------------
        if not drone.arm() or not drone.takeoff(altitude=10.0, precision=0.5, timeout_takeoff=30):
            return

        # Lock the gimbal onto the takeoff GPS coordinates
        current_gps = drone.position_gps()
        drone.set_gimbal_target(
            latitude=current_gps.lat,
            longitude=current_gps.lon,
            altitude=current_gps.alt,
            gimbal_device_id=0,
        )

        # Point gimbal straight down (-90°) to track a landing pad beneath the drone
        drone.set_gimbal_angles(pitch=-90.0, yaw=0.0, yaw_in_earth_frame=False)

        # Switch to LAND mode and stream relative landing pad offsets in BODY_FRD frame
        print("Initiating precision landing and streaming LANDING_TARGET offsets...")
        drone.set_mode("LAND")

        while not drone.motors_disarmed():
            alt = drone.position_gps().relative_alt
            # Simulated vision sensor detecting the pad slightly ahead (+0.2m) and right (+0.1m)
            drone.send_landing_target(x=0.2, y=0.1, z=max(alt, 0.5))
            time.sleep(0.1)  # 10 Hz update rate

        print("Touchdown complete! Retracting gimbal...")
        drone.set_gimbal_retract(timeout_ack=3)

    except KeyboardInterrupt:
        drone.mission_ended = False
        drone.land(timeout_land=60)
        drone.mission_ended = True
    finally:
        drone.close()


if __name__ == "__main__":
    main()
