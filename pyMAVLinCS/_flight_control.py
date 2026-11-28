# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import time
import math
from pymavlink import mavutil

from pyMAVLinCS import mavtypes
from pyMAVLinCS._system import _System


class _FlightControl(_System):
    """Flight control, navigation, VTOL transitions, HOME/EKF origin, gimbal, servo, parachute, and landing gear."""

    def set_servo(self,
            servo_channel: int,
            pwm: int,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Sets the position of a servo.

        Args:
            servo_channel (int): Servo channel.
            pwm (int): PWM value (between 1000 and 2000. 1000=closed, 2000=open).
            target_system (int|None): Target MAVLink system ID for the command. Defaults to the master's ID.
            **timeout_ack (float|None): Acknowledgment wait time. Default: 2.

        Returns:
            bool: Result of the operation.
        """
        self.logger.debug("Setting servo..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set servo")
            return False
        if not self.allow_actions:
            self.logger.warning("Actions not allowed: Impossible to set servo")
            return False
        if pwm < 1000 or pwm > 2000:
            self.logger.critical("PWM: %s must be between 1000 and 2000", pwm)
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system
        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # Command to control a servo
            1, # confirmation
            servo_channel, # param1 (Instance)
            pwm, # param2 (PWM)
            0,0,0,0,0 # param3-7 (unused)
        )
        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Servo set")
        elif accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Servo setting timed out")
        else: # Refused
            self.logger.error("Servo setting failed")
        return bool(accepted)

    def release_parachute(self,
            action: int = mavutil.mavlink.PARACHUTE_RELEASE,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Controls or releases the vehicle's parachute.

        Args:
            action (int): ``PARACHUTE_ACTION``:

                - ``PARACHUTE_DISABLE``: Disable automatic parachute release.
                - ``PARACHUTE_ENABLE``: Enable automatic parachute release.
                - ``PARACHUTE_RELEASE``: Immediately release the parachute (default).
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK waiting timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the command was accepted by the flight controller, False otherwise.

        Note:
            - Supported on **ArduPilot** (requires firmware built with ``HAL_PARACHUTE_ENABLED``).
            - On ArduPlane, ``PARACHUTE_RELEASE`` is rejected (returns False) if the parachute is disabled (``CHUTE_ENABLED = 0``), already released, or below ``CHUTE_ALT_MIN`` once airborne.
            - On ArduPilot, releasing the parachute also automatically deploys the landing gear if ``AP_LANDINGGEAR_ENABLED``.
        """
        self.logger.info("Sending DO_PARACHUTE (action=%s)..", action)
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to send parachute command")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to send parachute command")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        valid_actions = (
            mavutil.mavlink.PARACHUTE_DISABLE,
            mavutil.mavlink.PARACHUTE_ENABLE,
            mavutil.mavlink.PARACHUTE_RELEASE,
        )
        if isinstance(action, bool) or not isinstance(action, int) or action not in valid_actions:
            self.logger.error(
                "Invalid parachute action: %s (must be PARACHUTE_DISABLE/0, PARACHUTE_ENABLE/1, or PARACHUTE_RELEASE/2)",
                action,
            )
            return False

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE, # command
            1, # confirmation
            float(action), # param1 (PARACHUTE_ACTION)
            0,0,0,0,0,0 # param2-7 (unused)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Parachute command accepted (action=%s)", action)
        elif accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Parachute command timed out")
        else: # Refused
            self.logger.error("Parachute command failed or refused (action=%s)", action)
        return bool(accepted)

    def set_landing_gear(self,
            deploy: bool,
            landing_gear_id: int = -1,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Deploys or retracts the vehicle's landing gear.

        Args:
            deploy (bool): Target landing gear state: True to deploy (down), False to retract (up).
            landing_gear_id (int): Landing gear instance ID (-1 for all landing gear, >= 0 for a specific gear). Default: -1.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK waiting timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the command was accepted by the flight controller, False otherwise.

        Note:
            - Supported across ArduPilot vehicles when compiled with ``AP_LANDINGGEAR_ENABLED``.
            - Physical servo actuation on ArduPilot requires ``LGR_ENABLE = 1`` and a servo channel assigned to landing gear control (``SERVOn_FUNCTION = 29``).
        """
        self.logger.info("Setting landing gear (deploy=%s, id=%s)..", deploy, landing_gear_id)
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to set landing gear")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to set landing gear")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False
        if not isinstance(deploy, bool):
            self.logger.error("Invalid deploy value: %s (must be a bool: True=deploy, False=retract)", deploy)
            return False
        if isinstance(landing_gear_id, bool) or not isinstance(landing_gear_id, int) or landing_gear_id < -1:
            self.logger.error("Invalid landing_gear_id: %s (must be an int >= -1)", landing_gear_id)
            return False

        if target_system is None:
            target_system = self.master.target_system

        # MAV_CMD_AIRFRAME_CONFIGURATION param2: 0 = Down/Deploy, 1 = Up/Retract
        gear_position = 0.0 if deploy else 1.0

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, # command
            1, # confirmation
            float(landing_gear_id), # param1 (Landing gear ID: -1 for all)
            gear_position, # param2 (Landing gear position: 0=Deploy/Down, 1=Retract/Up)
            0,0,0,0,0 # param3-7 (reserved)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Landing gear command accepted (deploy=%s)", deploy)
        elif accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Landing gear command timed out")
        else: # Refused
            self.logger.error("Landing gear command failed or refused (deploy=%s)", deploy)
        return bool(accepted)

    def set_gimbal_target(self,
            latitude: float,
            longitude: float,
            altitude: float,
            gimbal_device_id: int = 0,
            **kwargs
        ) -> bool:
        """Sends a target GPS position.

        Args:
            latitude (float): Latitude in degrees.
            longitude (float): Longitude in degrees.
            altitude (float): Absolute altitude (MSL) in meters.
            gimbal_device_id (int): Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components.
            **timeout_ack (float): ACK wait timeout in seconds. Default: 3.

        Returns:
            bool: Result of the operation.
        """
        # Function not tested but should work by changing the gimbal mode (function to implement)
        self.logger.debug("Setting Gimbal Target..")
        timeout_ack = kwargs.get('timeout_ack', 3)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set Gimbal target")
            return False
        if not self.allow_actions:
            self.logger.warning("Actions not allowed: Impossible to set Gimbal target")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False
        if math.isnan(latitude) or math.isnan(longitude) or math.isnan(altitude):
            self.logger.error("Invalid NaN coordinates for Gimbal target!")
            return False
        self.master.mav.command_int_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            0, # current
            0, # autocontinue
            gimbal_device_id, # param1 (Gimbal device ID)
            0,0,0, # param2-4 (unused)
            int(latitude*1e7),
            int(longitude*1e7),
            float(altitude)
        )
        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Gimbal Target set")
        elif accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Gimbal Target setting timed out")
        else: # Refused
            self.logger.error("Gimbal Target setting failed")
        return bool(accepted)

    def set_gimbal_angles(self, pitch: float, yaw: float,
            pitch_rate: float = 0.0, yaw_rate: float = 0.0,
            lock_pitch: bool = True, lock_yaw: bool = True,
            yaw_in_earth_frame: bool = True,
            **kwargs
        ) -> bool:
        """Orients the gimbal.

        Args:
            pitch (float): Pitch angle (deg).
            yaw (float): Yaw angle (deg).
            pitch_rate (float): Pitch rotation speed (deg/s).
            yaw_rate (float): Yaw rotation speed (deg/s).
            lock_pitch (bool): True = lock pitch relative to the horizon.
            lock_yaw (bool): True = lock yaw relative to North (earth frame).
            yaw_in_earth_frame (bool): True = yaw expressed in Earth frame, otherwise vehicle frame.
            **timeout_ack (float): Maximum wait time for ACK (seconds). Default: 3.

        Returns:
            bool: True if ACK received (command accepted), False otherwise.
        """
        self.logger.debug("Setting Gimbal Angles..")
        timeout_ack = kwargs.get('timeout_ack', 3)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set Gimbal angles")
            return False
        if not self.allow_actions:
            self.logger.warning("Actions not allowed: Impossible to set Gimbal angles")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        # Construct bitmask flags
        flags = 0
        # Roll lock is often enabled by default on stabilized gimbals
        flags |= mavutil.mavlink.GIMBAL_MANAGER_FLAGS_ROLL_LOCK
        if lock_pitch:
            flags |= mavutil.mavlink.GIMBAL_MANAGER_FLAGS_PITCH_LOCK
        if lock_yaw:
            flags |= mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK
        # Choose the reference frame for yaw
        if yaw_in_earth_frame:
            flags |= mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME
        else:
            flags |= mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            1, pitch, yaw, pitch_rate, yaw_rate,    # Speeds (deg/s)
            flags, 0, 0
        )
        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Gimbal Angles set")
        elif accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Gimbal Angles setting timed out")
        else: # Refused
            self.logger.error("Gimbal Angles setting failed")
        return bool(accepted)

    def set_gimbal_retract(self, **kwargs) -> bool:
        """Sets the gimbal to retract position (no stabilization).

        Args:
            **timeout_ack (float): Maximum wait time for ACK (seconds). Default: 3.

        Returns:
            bool: True if ACK received (command accepted), False otherwise.
        """
        self.logger.debug("Setting Gimbal Retract..")
        timeout_ack = kwargs.get('timeout_ack', 3)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set Gimbal Retract")
            return False
        if not self.allow_actions:
            self.logger.warning("Actions not allowed: Impossible to set Gimbal Retract")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            1, 0, 0, 0, 0,
            mavutil.mavlink.GIMBAL_MANAGER_FLAGS_RETRACT, 0, 0
        )
        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Gimbal Retract set")
        elif accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Gimbal Retract setting timed out")
        else: # Refused
            self.logger.error("Gimbal Retract setting failed")
        return bool(accepted)

    def set_gimbal_neutral(self, **kwargs) -> bool:
        """Sets the gimbal to neutral position (roll=pitch=yaw=0).

        Args:
            **timeout_ack (float): Maximum wait time for ACK (seconds). Default: 3.

        Returns:
            bool: True if ACK received (command accepted), False otherwise.
        """
        self.logger.debug("Setting Gimbal Neutral..")
        timeout_ack = kwargs.get('timeout_ack', 3)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set Gimbal Neutral")
            return False
        if not self.allow_actions:
            self.logger.warning("Actions not allowed: Impossible to set Gimbal Neutral")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            1, 0, 0, 0, 0,
            mavutil.mavlink.GIMBAL_MANAGER_FLAGS_NEUTRAL, 0, 0
        )
        accepted = self.wait_command_ack(
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Gimbal Neutral set")
        elif accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Gimbal Neutral setting timed out")
        else: # Refused
            self.logger.error("Gimbal Neutral setting failed")
        return bool(accepted)

    def request_home_position(self,
            target_system: int|None = None,
            **kwargs
        ) -> mavtypes.HomePosition:
        """Requests and waits to receive the HOME position.

        Args:
            target_system (int|None): Target MAVLink system ID for the request. Defaults to the master's ID.
            **timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns NaN, NaN, NaN). None if no timeout. Default: 2.
            **timeout_home_reception (float|None): Timeout for receiving the HOME position after ACK. 0 if no wait (returns NaN, NaN, NaN). None if no timeout. Default: 2.

        Returns:
            mavtypes.HomePosition:
                - latitude_home (float): HOME latitude.
                - longitude_home (float): HOME longitude.
                - altitude_home (float): HOME altitude (MSL).

        Note:
            All returned values are NaN if no data has been received.
        """
        self.logger.info("Requesting HOME..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        timeout_home_reception = kwargs.get('timeout_home_reception', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to request HOME")
            return mavtypes.HomePosition(latitude_home=math.nan, longitude_home=math.nan, altitude_home=math.nan)
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return mavtypes.HomePosition(latitude_home=math.nan, longitude_home=math.nan, altitude_home=math.nan)

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, # Command to request a message
            1, # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, # param1 (Message ID)
            0,0,0,0,0,0 # param2-7 (unused)
        )
        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted is None:
            self.logger.critical("Unable to obtain HOME: ACK timeout")
            return mavtypes.HomePosition(latitude_home=math.nan, longitude_home=math.nan, altitude_home=math.nan)
        elif not accepted:
            self.logger.error("Unable to obtain HOME: refused")
            return mavtypes.HomePosition(latitude_home=math.nan, longitude_home=math.nan, altitude_home=math.nan)

        home = self.wait_for_home(
            source_system=target_system,
            timeout=timeout_home_reception
        )
        if math.isnan(home.latitude_home) or math.isnan(home.longitude_home) or math.isnan(home.altitude_home):
            self.logger.critical("Unable to receive HOME")
        return home

    def set_home(self,
            latitude: float|None,
            longitude: float|None,
            altitude: float|None,
            roll: float|None = None,
            pitch: float|None = None,
            yaw: float|None = None,
            target_system: int|None = None
        ) -> bool:
        """Sets the HOME position.

        Args:
            latitude (float): HOME latitude in degrees.
            longitude (float): HOME longitude in degrees.
            altitude (float): HOME altitude (MSL) in meters.
            roll (float|None): Roll (of the surface) in degrees [-180°; +180°].
            pitch (float|None): Pitch (of the surface) in degrees [-90°; +90°].
            yaw (float|None): Yaw in degrees [-180°; +180°].
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: True if the command was sent, False otherwise.

        Note:
            - Set latitude/longitude/altitude to None if you do not want to specify a certain characteristic of the HOME position (uses the current drone value).
            - Set roll/pitch/yaw to None if you do not want to specify a certain angle (uses the current drone value).
        """
        self.logger.debug('Setting HOME..')
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set HOME")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        if latitude is None:
            latitude = 2147483647 # INT32_MAX
        else:
            latitude = int(latitude * 1e7)
        if longitude is None:
            longitude = 2147483647 # INT32_MAX
        else:
            longitude = int(longitude * 1e7)
        if altitude is None:
            altitude = math.nan

        if roll is None:
            roll = math.nan
        elif roll == 0:
            roll = 0.01
        if pitch is None:
            pitch = math.nan
        elif pitch == 0:
            pitch = 0.01
        if yaw is None:
            yaw = math.nan

        self.master.mav.command_int_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL, # frame
            mavutil.mavlink.MAV_CMD_DO_SET_HOME, # command
            0, # current (unused)
            0, # autocontinue (unused)
            0, # param1 (Use Current)
            roll, # param2 (Roll)
            pitch, # param3 (Pitch)
            yaw, # param4 (Yaw)
            latitude, # x
            longitude, # y
            altitude # z
        )

        self.logger.info("HOME set")
        return True

    def set_home_current_pos(self, target_system: int|None = None) -> bool:
        """Sets the HOME position to the current position.

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: True if the command was sent, False otherwise.
        """
        self.logger.debug('Setting HOME..')
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set HOME")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.command_int_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL, # frame
            mavutil.mavlink.MAV_CMD_DO_SET_HOME, # command
            0, # current (unused)
            0, # autocontinue (unused)
            1, # param1 (Use Current)
            0, # param2 (Roll)
            0, # param3 (Pitch)
            0, # param4 (Yaw)
            0, # x
            0, # y
            0 # z
        )

        self.logger.info("HOME set")
        return True

    def set_ekf_origin(self,
            latitude: float,
            longitude: float,
            altitude: float,
            target_system: int|None = None
        ) -> bool:
        """Sets the drone's EKF origin.

        Args:
            latitude (float): EKF origin latitude in degrees.
            longitude (float): EKF origin longitude in degrees.
            altitude (float): EKF origin altitude (MSL) in meters.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: True if the command was sent, False otherwise.
        """
        self.logger.debug('Setting EKF origin..')
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set EKF origin")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        latitude = int(latitude * 1e7)
        longitude = int(longitude * 1e7)
        altitude = int(altitude * 1e3)

        self.master.mav.set_gps_global_origin_send(
            target_system, # target_system
            latitude, # latitude
            longitude, # longitude
            altitude, # altitude
            self.time_usec # time_usec
        )

        self.logger.debug('EKF origin set')
        return True

    def arm(self,
            force: bool = False,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Arms the drone.

        Args:
            force (bool): Whether to force arming or not.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns False). None if no timeout. Default: 2.
            **timeout_arming (float|None): Arming wait timeout. 0 if no wait after ACK (returns the ACK wait result as bool). None if no timeout. Default: 4.

        Returns:
            bool: Whether the drone was armed or not.
        """
        self.logger.info("Arming..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        timeout_arming = kwargs.get('timeout_arming', 4)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to arm")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Impossible to arm")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        if not force and self.motors_armed(source_system=target_system):
            self.logger.info("Vehicle already armed: arming process stopped")
            return True

        if self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            # Check current flight mode
            custom_mode = self.mode(source_system=target_system)
            if not force and custom_mode in ("INITIALIZING", "INITIALISING", "PREFLIGHT", "CALIBRATION"):
                self.logger.error("Initializing: impossible to arm")
                return False

            non_armable_modes = {
                "LAND", "QLAND", "TEST", "FOLLOW", "RTL", "QRTL",
                "FLOWHOLD", "BRAKE", "CIRCLE", "SMART_RTL", "SMARTRTL",
                "AUTO", "AUTO_RTL", "FLIP", "SYSTEMID", "AVOID_ADSB", "AUTOROTATE",
            }
            if self.vehicle_type_int(source_system=target_system) in self._COPTER_MAV_TYPES:
                non_armable_modes.add("AUTOTUNE")

            if custom_mode in non_armable_modes:
                self.logger.warning("Mode %s not armable: trying to change mode to GUIDED", custom_mode)
                mod_changed = self.set_mode(
                    mode="GUIDED",
                    target_system=target_system
                )
                if not force and not mod_changed:
                    self.logger.critical("Not the right mode, impossible to arm")
                    return False
        else:
            self.logger.info("Mode check before arming not implemented outside ArduPilot")

        # Send arming request
        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
            1, # confirmation
            1, # param1 (Arm)
            21196 if force else 0, # param2 (Force)
            0,0,0,0,0 # param3-7 (unused)
        )
        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Arming timed out")
            return False
        elif not accepted: # Refused
            self.logger.error("Arming failed")
            return False

        if timeout_arming == 0:
            self.logger.warning("Vehicle sent an ACK but isn't armed yet")
            return True

        armed = self.wait_motors_armed(
            source_system=target_system,
            timeout=timeout_arming
        )
        return bool(armed)

    def disarm(self,
            force: bool = False,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Disarms the drone.

        Args:
            force (bool): Whether to force disarming or not.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns False). None if no timeout. Default: 2.
            **timeout_disarming (float|None): Disarming wait timeout. 0 if no wait after ACK (returns the ACK wait result as bool). None if no timeout. Default: 4.

        Returns:
            bool: Whether the drone was disarmed or not.
        """
        self.logger.info("Disarming..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        timeout_disarming = kwargs.get('timeout_disarming', 4)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to disarm")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Impossible to disarm")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
            1, # confirmation
            0, # param1 (Disarm)
            21196 if force else 0, # param2 (Force)
            0,0,0,0,0 # param3-7 (unused)
        )
        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Disarming timed out")
            return False
        elif not accepted: # Refused
            self.logger.error("Disarming failed")
            return False

        if timeout_disarming == 0:
            self.logger.warning("Vehicle sent an ACK but isn't disarmed yet")
            return True

        disarmed = self.wait_motors_disarmed(
            source_system=target_system,
            timeout=timeout_disarming
        )
        return bool(disarmed)

    def set_mode(self,
            mode: str|int|tuple[int, int, int],
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Changes the flight mode.

        Args:
            mode (str|int|tuple[int, int, int]): Flight mode. str: mode name. int: mode ID on ArduPilot. tuple[int, int, int]: (MAV_MODE_FLAG, Custom Main Mode, Custom Sub Mode) on PX4.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns False). None if no timeout. Default: 2.
            **timeout_set_mode (float|None): Mode change wait timeout. 0 if no wait (returns the ACK wait result as bool). None if no timeout. Default: 4.

        Returns:
            bool: Whether the mode was changed or not.
        """
        self.logger.info("Setting Mode..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        timeout_set_mode = kwargs.get('timeout_set_mode', 4)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to change mode")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Impossible to change mode")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        # Look up IDs associated with the flight mode string
        if isinstance(mode, str):
            mode_mapping = self.mode_mapping(source_system=target_system)
            if mode_mapping is not None and mode in mode_mapping:
                key = mode_mapping[mode]
            else:
                self.logger.error("Trying to change mode. Unknown mode %s", mode)
                self.logger.debug("mode_mapping(%s): %s", target_system, mode_mapping)
                return False
        else:
            key = mode

        # Decompose the IDs
        if isinstance(key, int):
            base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            custom_main_mode = key
            custom_sub_mode = 0
        elif isinstance(key, tuple):
            if len(key) != 3:
                self.logger.error("Trying to change mode. Invalid mode: %s", key)
                return False
            base_mode = key[0]
            custom_main_mode = key[1]
            custom_sub_mode = key[2]
        else:
            self.logger.critical("Trying to change mode. Invalid mode: %s", key)
            return False

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, # command
            1, # confirmation
            base_mode, # param1 (Mode)
            custom_main_mode, # param2 (Custom Mode)
            custom_sub_mode, # param3 (Custom Submode)
            0,0,0,0  # param4-7 (unused)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Setting Mode timed out")
            return False
        elif not accepted: # Refused
            self.logger.error("Setting Mode failed")
            return False

        if timeout_set_mode == 0:
            self.logger.warning("Vehicle sent an ACK but mode isn't changed yet")
            return True

        mod_changed = self.wait_mode_changed(
            mode=mode,
            source_system=target_system,
            timeout=timeout_set_mode
        )
        return bool(mod_changed)

    def set_gps_pos_target(self,
            latitude: float,
            longitude: float,
            relative_altitude: float,
            yaw: float = math.nan,
            target_system: int|None = None
        ) -> bool:
        """Sets the GPS target position.

        Args:
            latitude (float): Latitude (deg).
            longitude (float): Longitude (deg).
            relative_altitude (float): Relative altitude (m).
            yaw (float): Target position yaw (rad).
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: True if the command was sent, False otherwise.

        Note:
            Set yaw to NaN if no yaw is specified.
        """
        self.logger.debug("Setting GPS position target..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set GPS position target")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Impossible to set GPS position target")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        type_mask = 0b000000000000
        for b in [
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ]:
            type_mask |= b
        if math.isnan(yaw):
            yaw = 0
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

        self.master.mav.set_position_target_global_int_send(
            self.time_boot_ms, # time_boot_ms
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # coordinate_frame
            type_mask, # type_mask
            int(latitude * 1e7), # lat_int
            int(longitude * 1e7), # lon_int
            relative_altitude, # relative_alt
            0,0,0, # vx, vy, vz
            0,0,0, # afx, afy, afz
            yaw,0 # yaw, yaw_rate
        )
        self.logger.info("GPS position target set")
        return True

    def set_local_pos_target(self,
            x: float,
            y: float,
            z: float,
            yaw: float = math.nan,
            target_system: int|None = None
        ) -> bool:
        """Sets the local target position.

        Args:
            x (float): North (m).
            y (float): East (m).
            z (float): Down (m).
            yaw (float): Target position yaw (rad).
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: True if the command was sent, False otherwise.

        Note:
            Set yaw to NaN if no yaw is specified.
        """
        self.logger.debug("Setting local position target..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set local position target")
            return False
        if not self.allow_actions:
            self.logger.error("[FAIL] Actions not allowed: Impossible to set local position target")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        type_mask = 0b000000000000
        for b in [
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ]:
            type_mask |= b
        if math.isnan(yaw):
            yaw = 0
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

        self.master.mav.set_position_target_local_ned_send(
            self.time_boot_ms, # time_boot_ms
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # coordinate_frame
            type_mask, # type_mask
            x, y, z, # x, y, z
            0,0,0, # vx, vy, vz
            0,0,0, # afx, afy, afz
            yaw,0 # yaw, yaw_rate
        )
        self.logger.info("Local position target set")
        return True

    def set_speed(self,
            vx: float,
            vy: float,
            vz: float,
            yaw: float = math.nan,
            target_system: int|None = None
        ) -> bool:
        """Sets the drone's velocity.

        Args:
            vx (float): Velocity towards North in m/s.
            vy (float): Velocity towards East in m/s.
            vz (float): Velocity downwards in m/s.
            yaw (float): Target position yaw (rad).
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: True if the command was sent, False otherwise.
        """
        self.logger.debug("Setting speed..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set speed")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Impossible to set speed")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system
        type_mask = 0b000000000000
        for b in [
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE,
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ]:
            type_mask |= b
        if math.isnan(yaw):
            yaw = 0
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

        self.master.mav.set_position_target_local_ned_send(
            self.time_boot_ms, # time_boot_ms
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # coordinate_frame
            type_mask, # type_mask
            0,0,0, # x, y, z
            vx, vy, vz, # vx, vy, vz
            0,0,0, # afx, afy, afz
            yaw if yaw is not None else 0,0 # yaw, yaw_rate
        )
        self.logger.debug("Speed set")
        return True

    def change_speed(self,
            speed: float = -1.0,
            speed_type: int = mavutil.mavlink.SPEED_TYPE_GROUNDSPEED,
            throttle: float = -1.0,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Changes the vehicle's target speed and/or throttle setpoint using MAV_CMD_DO_CHANGE_SPEED.

        The value persists until it is overridden or there is a flight mode change.

        Args:
            speed (float): Target speed in m/s (-1.0 indicates no change, -2.0 on ArduPlane restores default airspeed).

                - On ArduCopter: must be strictly positive (> 0).
                - On ArduPlane (``SPEED_TYPE_AIRSPEED``): must be within ``[AIRSPEED_MIN, AIRSPEED_MAX]``, or -2.0.
            speed_type (int): ``SPEED_TYPE`` | Type of speed controlled by ``speed``:

                - ``SPEED_TYPE_AIRSPEED`` (0): Airspeed (treated as groundspeed on ArduCopter for GCS compatibility).
                - ``SPEED_TYPE_GROUNDSPEED`` (1): Groundspeed (horizontal NE speed on ArduCopter, min groundspeed on ArduPlane).
                - ``SPEED_TYPE_CLIMB_SPEED`` (2): Climb speed (ArduCopter only).
                - ``SPEED_TYPE_DESCENT_SPEED`` (3): Descent speed (ArduCopter only).
            throttle (float): Target throttle percentage in (0, 100] (-1.0 indicates no change). Used on ArduPlane when speed is not changed.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK wait timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the command was accepted by the flight controller, False otherwise.

        Note:
            On ArduPlane, the vehicle must be in GUIDED or AUTO mode for this command to be accepted.
        """
        self.logger.debug("Sending DO_CHANGE_SPEED..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to change speed")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to change speed")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False
        if math.isnan(speed) or math.isinf(speed) or math.isnan(throttle) or math.isinf(throttle):
            self.logger.error("Invalid NaN/Inf value for change_speed (speed=%s, throttle=%s)", speed, throttle)
            return False
        if speed_type not in (
            mavutil.mavlink.SPEED_TYPE_AIRSPEED,
            mavutil.mavlink.SPEED_TYPE_GROUNDSPEED,
            mavutil.mavlink.SPEED_TYPE_CLIMB_SPEED,
            mavutil.mavlink.SPEED_TYPE_DESCENT_SPEED,
        ):
            self.logger.error("Invalid speed_type: %s", speed_type)
            return False
        if speed <= 0 and speed not in (-1.0, -2.0):
            self.logger.error("Invalid speed value %s: must be > 0 (or -1 for no change, -2 for default airspeed)", speed)
            return False
        if speed == -2.0 and speed_type != mavutil.mavlink.SPEED_TYPE_AIRSPEED:
            self.logger.error("speed=-2.0 (restore default) is only supported with SPEED_TYPE_AIRSPEED")
            return False

        if target_system is None:
            target_system = self.master.target_system

        is_on_ardupilot = bool(self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
        is_copter = bool(self.vehicle_type_int(source_system=target_system) in self._COPTER_MAV_TYPES)

        if is_on_ardupilot and is_copter:
            if throttle != -1.0:
                self.logger.warning("Throttle parameter (param3) is ignored on ArduCopter")
                throttle = -1.0
            if speed <= 0:
                self.logger.error("On ArduCopter, target speed must be strictly positive (got %s)", speed)
                return False

        # On ArduPlane, SPEED_TYPE_GROUNDSPEED does not ignore speed=-1.0 before checking throttle;
        # using SPEED_TYPE_AIRSPEED when speed==-1.0 ensures only throttle_cruise is updated.
        effective_speed_type = mavutil.mavlink.SPEED_TYPE_AIRSPEED if speed == -1.0 else speed_type

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
            1, # confirmation
            float(effective_speed_type), # param1 (Speed type)
            float(speed), # param2 (Speed in m/s)
            float(throttle), # param3 (Throttle %)
            0,0,0,0 # param4-7 (Reserved)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Speed change command accepted")
        elif accepted is None: # Mission end or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Speed change command timed out")
        else: # Refused
            self.logger.error("Speed change command failed or denied")
        return bool(accepted)

    def loiter_here(self,
            radius: float = 0.0,
            ccw: bool = False,
            altitude: float|None = None,
            frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Commands the vehicle to hold / loiter immediately at its current position.

        - When called with default parameters, sends ``MAV_CMD_NAV_LOITER_UNLIM``
          (switches ArduPilot to ``LOITER`` mode using the autopilot's default loiter radius and current altitude).
        - When ``radius > 0``, ``ccw=True``, or ``altitude`` is specified, delegates to ``do_reposition()`` at the
          vehicle's current GPS coordinates (``force_guided=True``) so the exact orbit radius, direction (CW/CCW),
          and target altitude are applied in ``GUIDED`` mode.

        Args:
            radius (float): Fixed-wing loiter radius in meters (0.0 uses autopilot default ``WP_LOITER_RAD``). Must be >= 0. Default: 0.0.
            ccw (bool): Fixed-wing orbit direction: ``False`` for clockwise, ``True`` for counter-clockwise (requires ``radius > 0`` on ArduPlane). Default: False.
            altitude (float|None): Optional target altitude in meters (according to ``frame``). ``None`` keeps current altitude. Default: None.
            frame (int): ``MAV_FRAME`` coordinate frame used when delegating to ``do_reposition()``. Default: ``MAV_FRAME_GLOBAL_RELATIVE_ALT_INT``.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK waiting timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the loiter command was accepted by the flight controller, False otherwise.

        Note:
            - On ArduPlane, specifying ``ccw=True`` with ``radius=0.0`` will not change orbit direction.
        """
        self.logger.info("Commanding loiter at current position (radius=%s, ccw=%s, alt=%s)..", radius, ccw, altitude)
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to loiter")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to loiter")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if not math.isfinite(radius) or radius < 0:
            self.logger.error("Invalid loiter radius: %s (must be >= 0)", radius)
            return False
        has_alt = altitude is not None and not math.isnan(altitude)
        if altitude is not None and not math.isnan(altitude) and not math.isfinite(altitude):
            self.logger.error("Invalid loiter altitude: %s", altitude)
            return False

        if target_system is None:
            target_system = self.master.target_system

        # Default hold: MAV_CMD_NAV_LOITER_UNLIM
        if radius == 0.0 and not ccw and not has_alt:
            self.master.mav.command_long_send(
                target_system, # target_system
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, # command
                1, # confirmation
                0,0,0,0,0,0,0 # param1-7 (unused by ArduPilot)
            )
            accepted = self.wait_command_ack(
                command=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                source_system=target_system,
                timeout=timeout_ack
            )
            if accepted:
                self.logger.info("Loiter command accepted")
            elif accepted is None:
                if self.mission_ended:
                    self.logger.critical("Mission ended")
                else:
                    self.logger.critical("NAV_LOITER_UNLIM timed out")
            else:
                self.logger.error("NAV_LOITER_UNLIM failed or refused")
            return bool(accepted)

        # Parameterized loiter at current GPS position via DO_REPOSITION
        gps = self.position_gps(source_system=target_system)
        if math.isnan(gps.lat) or math.isnan(gps.lon):
            self.logger.error("No GPS position received: unable to loiter with custom parameters")
            return False

        if ccw and radius == 0.0:
            self.logger.warning("ArduPlane DO_REPOSITION only applies ccw direction when radius > 0")

        if has_alt:
            target_alt = float(altitude)
        elif frame in (mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_FRAME_GLOBAL_INT):
            target_alt = gps.alt
        else:
            target_alt = gps.relative_alt

        if math.isnan(target_alt):
            self.logger.error("No altitude available: unable to loiter with custom parameters")
            return False

        return self.do_reposition(
            latitude=gps.lat,
            longitude=gps.lon,
            altitude=target_alt,
            frame=frame,
            loiter_radius=float(radius),
            ccw=ccw,
            force_guided=True,
            target_system=target_system,
            timeout_ack=timeout_ack,
        )

    def change_altitude(self,
            altitude: float,
            frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            rate: float|None = None,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Changes the vehicle's target altitude in flight without changing its horizontal target.

        - On **ArduPlane** with ``rate=None``: sends ``MAV_CMD_DO_CHANGE_ALTITUDE`` (works in ``GUIDED``, ``LOITER``, and ``AUTO`` modes).
        - On **ArduPlane** with ``rate`` specified (``>= 0``): sends ``MAV_CMD_GUIDED_CHANGE_ALTITUDE``.
        - On **ArduCopter**: updates target altitude at the current horizontal GPS position via ``do_reposition()`` in ``GUIDED`` mode.

        Args:
            altitude (float): Target altitude in meters (according to ``frame``).
            frame (int): Global ``MAV_FRAME`` for ``altitude``. Default: ``MAV_FRAME_GLOBAL_RELATIVE_ALT``.
            rate (float|None): Optional vertical slew rate in m/s (``0.0`` = maximum rate) using
                ``MAV_CMD_GUIDED_CHANGE_ALTITUDE`` (ArduPlane ``GUIDED`` mode only; ignored with a warning on ArduCopter).
                ``None`` uses ``MAV_CMD_DO_CHANGE_ALTITUDE``. Default: None.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK waiting timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the command was accepted by the flight controller, False otherwise.

        Note:
            - When using ``rate`` on ArduPlane, it strictly denies ``altitude == 0`` or ``altitude == -1`` and requires ``GUIDED`` mode.
        """
        self.logger.info("Changing target altitude to %s m (frame=%s, rate=%s)..", altitude, frame, rate)
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to change altitude")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to change altitude")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if not math.isfinite(altitude):
            self.logger.error("Invalid target altitude: %s", altitude)
            return False
        valid_frames = (
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
        )
        if frame not in valid_frames:
            self.logger.error("Unsupported altitude frame: %s (must be a global MAV_FRAME_GLOBAL*)", frame)
            return False
        if rate is not None and (not math.isfinite(rate) or rate < 0):
            self.logger.error("Invalid altitude change rate: %s (must be >= 0 or None)", rate)
            return False

        if target_system is None:
            target_system = self.master.target_system

        is_ardupilot = (self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
        is_copter = (self.vehicle_type_int(source_system=target_system) in self._COPTER_MAV_TYPES)

        # ArduCopter does not implement DO_CHANGE_ALTITUDE / GUIDED_CHANGE_ALTITUDE
        if is_ardupilot and is_copter:
            if rate is not None:
                self.logger.warning("Altitude slew rate parameter is ignored on ArduCopter")
            gps = self.position_gps(source_system=target_system)
            if math.isnan(gps.lat) or math.isnan(gps.lon):
                self.logger.error("No GPS position received: unable to change altitude on ArduCopter")
                return False
            return self.do_reposition(
                latitude=gps.lat,
                longitude=gps.lon,
                altitude=float(altitude),
                frame=frame,
                force_guided=False,
                target_system=target_system,
                timeout_ack=timeout_ack,
            )

        if rate is not None:
            if altitude == 0.0 or altitude == -1.0:
                self.logger.error("MAV_CMD_GUIDED_CHANGE_ALTITUDE does not allow altitude=0 or -1 on ArduPlane")
                return False
            self.master.mav.command_int_send(
                target_system, # target_system
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                frame, # coordinate frame
                mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE, # command
                0, # current (unused)
                0, # autocontinue (unused)
                0.0, # param1 (reserved)
                0.0, # param2 (reserved)
                float(rate), # param3 (rate of change in m/s; 0 = max rate)
                0.0, # param4 (reserved)
                0, # x (unused)
                0, # y (unused)
                float(altitude) # z (target altitude)
            )
            cmd_id = mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE
        else:
            self.master.mav.command_long_send(
                target_system, # target_system
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE, # command
                1, # confirmation
                float(altitude), # param1 (altitude)
                float(frame), # param2 (MAV_FRAME)
                0,0,0,0,0 # param3-7 (unused)
            )
            cmd_id = mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE

        accepted = self.wait_command_ack(
            command=cmd_id,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Altitude change accepted")
        elif accepted is None:
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("Altitude change timed out")
        else:
            self.logger.error("Altitude change failed or denied")
        return bool(accepted)

    def change_heading(self,
            heading: float = 0.0,
            heading_type: int = mavutil.mavlink.HEADING_TYPE_COURSE_OVER_GROUND,
            max_accel: float = 0.0,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Commands a target heading or course-over-ground in ``GUIDED`` mode using ``MAV_CMD_GUIDED_CHANGE_HEADING``.

        Slews the aircraft smoothly toward the requested heading or ground course at up to ``max_accel`` centripetal
        acceleration. Pass ``heading_type=mavutil.mavlink.HEADING_TYPE_DEFAULT`` to cancel the guided heading override
        and return to normal ``GUIDED`` navigation / loitering.

        Args:
            heading (float): Target heading / course in degrees, strictly in ``[0.0, 360.0)`` (0 = North, 90 = East). Default: 0.0.
            heading_type (int): ``HEADING_TYPE``:

                - ``HEADING_TYPE_COURSE_OVER_GROUND``: Track ground course (wind-compensated).
                - ``HEADING_TYPE_HEADING``: Vehicle magnetic/true nose heading.
                - ``HEADING_TYPE_DEFAULT``: Cancel guided heading override.

                Default: ``HEADING_TYPE_COURSE_OVER_GROUND``.
            max_accel (float): Maximum centripetal acceleration in m/s² (turn rate limit; ``>= 0``). ArduPlane clamps values below 0.05 to 0.05 m/s². Default: 0.0.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK waiting timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the command was accepted by the flight controller, False otherwise.

        Note:
            - Supported on **ArduPlane** in ``GUIDED`` mode (requires ``AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED``, enabled by default).
            - Fails (returns False) if the aircraft is not in ``GUIDED`` mode.
        """
        self.logger.info("Changing target heading to %s deg (type=%s, max_accel=%s)..", heading, heading_type, max_accel)
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to change heading")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to change heading")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        valid_types = (
            mavutil.mavlink.HEADING_TYPE_COURSE_OVER_GROUND,
            mavutil.mavlink.HEADING_TYPE_HEADING,
            mavutil.mavlink.HEADING_TYPE_DEFAULT,
        )
        if isinstance(heading_type, bool) or not isinstance(heading_type, int) or heading_type not in valid_types:
            self.logger.error("Invalid heading_type: %s (must be HEADING_TYPE_COURSE_OVER_GROUND/0, HEADING_TYPE_HEADING/1, or HEADING_TYPE_DEFAULT/2)", heading_type)
            return False
        if not math.isfinite(heading) or heading < 0.0 or heading >= 360.0:
            self.logger.error("Invalid target heading: %s (must be in [0.0, 360.0))", heading)
            return False
        if not math.isfinite(max_accel) or max_accel < 0.0:
            self.logger.error("Invalid max_accel: %s (must be >= 0)", max_accel)
            return False

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.command_int_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL, # frame (unused)
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING, # command
            0, # current (unused)
            0, # autocontinue (unused)
            float(heading_type), # param1 (HEADING_TYPE)
            float(heading), # param2 (target heading in degrees [0, 360))
            float(max_accel), # param3 (max centripetal acceleration in m/s^2)
            0.0, # param4 (reserved)
            0, # x (unused)
            0, # y (unused)
            0.0 # z (unused)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Heading change accepted")
        elif accepted is None:
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("GUIDED_CHANGE_HEADING timed out")
        else:
            self.logger.error("GUIDED_CHANGE_HEADING failed or denied")
        return bool(accepted)

    def do_reposition(self,
            latitude: float = math.nan,
            longitude: float = math.nan,
            altitude: float = math.nan,
            frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            speed: float = -1.0,
            loiter_radius: float = 0.0,
            ccw: bool = False,
            yaw: float = math.nan,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Commands the vehicle to reposition to a specific 3D GPS location using MAV_CMD_DO_REPOSITION.

        Args:
            latitude (float): Target latitude in degrees. Set to 0 or NaN to hold the current latitude.
            longitude (float): Target longitude in degrees. Set to 0 or NaN to hold the current longitude.
            altitude (float): Target altitude in meters (interpreted according to frame). Set to 0 or NaN to hold the current altitude.
            frame (int): MAV_FRAME | Coordinate and altitude reference frame:

                - ``MAV_FRAME_GLOBAL`` (0): Altitude (MSL).
                - ``MAV_FRAME_GLOBAL_RELATIVE_ALT`` (3): Altitude relative to the HOME position.
                - ``MAV_FRAME_GLOBAL_TERRAIN_ALT`` (10): Altitude relative to terrain height (requires rangefinder or terrain database).
            speed (float): Desired ground speed in m/s (-1.0 or 0.0 uses the vehicle's default waypoint speed).
            loiter_radius (float): [Fixed-Wing only] Loiter radius in meters around the target point (0.0 uses the default Flight Controller parameter).
            ccw (bool): [Fixed-Wing only] Counter-clockwise loiter direction (False = clockwise, True = counter-clockwise). Note: If ccw is used on ArduPlane, it overrides yaw parameter.
            yaw (float): Target yaw in radians (NaN to let the autopilot control heading). Ignored on ArduPlane if ccw is used.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **force_guided (bool): Switch to Guided/Hold mode before sending the command. Default: False.
            **timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns False). None if no timeout. Default: 2.

        Returns:
            bool: True if the command was accepted, False otherwise.
        """
        self.logger.debug("Sending DO_REPOSITION..")
        force_guided = kwargs.get('force_guided', False)
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to reposition")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to reposition")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        is_on_ardupilot = bool(self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
        is_copter = bool(self.vehicle_type_int(source_system=target_system) in self._COPTER_MAV_TYPES)
        is_fixed_wing = bool(self.vehicle_type_int(source_system=target_system) in (
            mavutil.mavlink.MAV_TYPE_FIXED_WING,
            *self._VTOL_MAV_TYPES,
        ))

        # ArduCopter requires already being in GUIDED mode before accepting DO_REPOSITION
        if force_guided and is_on_ardupilot and is_copter:
            if self.mode(source_system=target_system) != "GUIDED":
                self.logger.info("Switching to GUIDED mode before DO_REPOSITION..")
                if not self.set_mode(mode="GUIDED", target_system=target_system, timeout_ack=timeout_ack):
                    self.logger.error("Failed to switch to GUIDED mode: aborting reposition")
                    return False

        # Set MAV_DO_REPOSITION_FLAGS_CHANGE_MODE (bit 0 = 1) if force_guided is enabled
        flags = mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE if force_guided else 0

        # Sanitize coordinates: NaN or 0.0 means "keep current" (0 in int32 degE7)
        lat_int = 0 if (latitude is None or math.isnan(latitude)) else int(latitude * 1e7)
        lon_int = 0 if (longitude is None or math.isnan(longitude)) else int(longitude * 1e7)
        alt_float = math.nan if (altitude is None or math.isnan(altitude)) else float(altitude)

        # Determine param4: ArduPlane uses param4 for loiter direction (0=CW, 1=CCW),
        # while Copter/PX4 use param4 for Yaw (in radians, or NaN for default)
        if is_fixed_wing:
            param4 = 1.0 if ccw else 0.0
        else:
            param4 = float(yaw) if yaw is not None else math.nan

        self.master.mav.command_int_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            frame, # coordinate frame
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, # command
            0, # current (unused)
            0, # autocontinue (unused)
            float(speed), # param1: Ground speed in m/s (-1 for default)
            float(flags), # param2: Bitmask (MAV_DO_REPOSITION_FLAGS)
            float(loiter_radius), # param3: Loiter radius in m for Planes (0 for default)
            param4, # param4: Loiter direction (Plane) or Yaw in rad (Copter/PX4)
            lat_int, # x: Latitude * 1e7
            lon_int, # y: Longitude * 1e7
            alt_float # z: Altitude in meters
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Reposition command accepted")
        elif accepted is None: # Mission end or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Reposition command timed out")
        else: # Refused
            self.logger.error("Reposition command failed or denied")
        return bool(accepted)

    def _is_vtol_and_fixed_wing(self, target_system: int) -> tuple[bool, bool]:
        """Detects whether the target vehicle is a VTOL/QuadPlane or a pure Fixed-Wing."""
        vtype = self.vehicle_type_int(source_system=target_system)
        vs = self.vtol_state(source_system=target_system)
        has_vtol_state = bool(not math.isnan(vs) and vs != mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED)
        is_vtol = (vtype in self._VTOL_MAV_TYPES) or (
            vtype == mavutil.mavlink.MAV_TYPE_FIXED_WING and has_vtol_state
        )
        is_fixed_wing = (vtype == mavutil.mavlink.MAV_TYPE_FIXED_WING) and not is_vtol
        return is_vtol, is_fixed_wing

    def takeoff(self,
            altitude: float,
            pitch: float = 0,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Perform an automated takeoff. Make sure to arm the vehicle beforehand.

        Automatically adapts the takeoff strategy on ArduPilot to the vehicle type:

        - **Multirotors / VTOL / QuadPlane (ArduPilot)**: Takeoff in ``GUIDED`` mode
          (vertical takeoff; if already airborne, updates target altitude).
        - **Fixed-Wing (ArduPlane)**: Switches to ``TAKEOFF`` flight mode
          (using the autopilot's configured ``TKOFF_ALT``, ``TKOFF_LVL_PITCH``, and ``TKOFF_DIST`` parameters by default).

        Args:
            altitude (float): Target takeoff altitude relative to HOME (m). Must be strictly positive.
            pitch (float): Minimum takeoff pitch angle (deg). Ignored on VTOL vertical takeoff, and only written to ``TKOFF_LVL_PITCH`` on Fixed-Wing if ``update_tkoff_params=True`` and ``pitch > 0``. Default: 0.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **precision (float): Required altitude precision in meters to consider the takeoff successful. Default: 1.
            **timeout_ack (float|None): Timeout while waiting for command/parameter ACKs. 0 means no waiting (returns False). None means no timeout. Default: 2.
            **timeout_set_mode (float|None): Timeout while waiting for the mode change. Default: 4.
            **timeout_takeoff (float|None): Timeout while waiting for the takeoff to complete. 0 means no waiting (returns the ack/mode result as bool). None means no timeout. Default: None.
            **update_tkoff_params (bool): Fixed-Wing (ArduPlane) only. If True, overwrites ``TKOFF_ALT`` (and ``TKOFF_LVL_PITCH`` when ``pitch > 0``) on the autopilot before entering ``TAKEOFF`` mode. Default: False (does not modify autopilot parameters).

        Returns:
            bool: Takeoff result.

        Note:
            - On Fixed-Wing (ArduPlane), ``TAKEOFF`` mode climbs at ``TKOFF_LVL_PITCH`` to ``TKOFF_ALT`` and then
              loiters at ``TKOFF_DIST`` meters ahead of the takeoff point. By default (``update_tkoff_params=False``),
              ``takeoff()`` does **not** modify these parameters on the flight controller: make sure ``altitude`` does
              not exceed the aircraft's configured ``TKOFF_ALT`` (ArduPlane default: 50 m), otherwise waiting for
              ``altitude`` will time out once the aircraft levels off at ``TKOFF_ALT``. Pass
              ``update_tkoff_params=True`` if you explicitly want ``takeoff()`` to write ``TKOFF_ALT`` / ``TKOFF_LVL_PITCH``
              (note that ArduPilot persists parameter writes to EEPROM).
            - On Fixed-Wing (ArduPlane), make sure the aircraft is armed before calling ``takeoff()``
              (or arm right after if calling with ``timeout_takeoff=0``).
            - On VTOL / QuadPlane (ArduPlane), vertical takeoff in ``GUIDED`` mode requires the vehicle to be
              armed beforehand; ``pitch`` is ignored during vertical VTOL climb.
        """
        self.logger.debug("Starting takeoff..")
        precision = kwargs.get('precision', 1)
        timeout_ack = kwargs.get('timeout_ack', 2)
        timeout_set_mode = kwargs.get('timeout_set_mode', 4)
        timeout_takeoff = kwargs.get('timeout_takeoff', None)
        update_tkoff_params = kwargs.get('update_tkoff_params', False)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to takeoff")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to takeoff")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False
        if not math.isfinite(altitude) or altitude <= 0:
            self.logger.error("Invalid takeoff altitude: %s (must be > 0)", altitude)
            return False
        if not math.isfinite(pitch):
            self.logger.error("Invalid takeoff pitch: %s", pitch)
            return False

        if target_system is None:
            target_system = self.master.target_system

        _, _, relative_alt, _ = self.position_gps(source_system=target_system)
        if math.isnan(relative_alt):
            self.logger.error("No position received: impossible to takeoff with safety")
            return False

        is_ardupilot = (self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
        is_vtol, is_fixed_wing = self._is_vtol_and_fixed_wing(target_system)

        goto_sent = False

        if is_ardupilot and is_fixed_wing:
            if update_tkoff_params:
                if pitch < 0:
                    self.logger.error("Invalid Fixed-Wing takeoff pitch: %s (must be >= 0)", pitch)
                    return False

                if not self.write_parameter(
                    parameter="TKOFF_ALT",
                    value=float(altitude),
                    target_system=target_system,
                    timeout_ack=timeout_ack
                ):
                    self.logger.error("Failed to set TKOFF_ALT parameter: takeoff aborted")
                    return False

                if pitch > 0:
                    if not self.write_parameter(
                        parameter="TKOFF_LVL_PITCH",
                        value=float(pitch),
                        target_system=target_system,
                        timeout_ack=timeout_ack
                    ):
                        self.logger.error("Failed to set TKOFF_LVL_PITCH parameter: takeoff aborted")
                        return False
            elif pitch != 0:
                self.logger.warning(
                    "Fixed-Wing takeoff uses the autopilot's configured TKOFF_ALT and TKOFF_LVL_PITCH "
                    "(pitch=%s not written; pass update_tkoff_params=True to overwrite them)",
                    pitch,
                )

            # ArduPlane fixed-wing rejects COMMAND_LONG(MAV_CMD_NAV_TAKEOFF); use TAKEOFF mode
            expected_mode = "TAKEOFF"
            if self.mode(source_system=target_system) != expected_mode:
                self.logger.info("Switching to TAKEOFF mode for Fixed-Wing takeoff..")
                mod_changed = self.set_mode(
                    mode=expected_mode,
                    target_system=target_system,
                    timeout_ack=timeout_ack,
                    timeout_set_mode=timeout_set_mode
                )
                if not mod_changed:
                    self.logger.critical("Takeoff aborted")
                    return False

            goto_sent = True
        else:
            expected_mode = "GUIDED"
            # Find the correct flight mode
            if is_ardupilot:
                if is_vtol and pitch != 0:
                    self.logger.warning("Pitch parameter (%s) is ignored for VTOL vertical takeoff", pitch)
                if self.mode(source_system=target_system) != expected_mode:
                    self.logger.warning("Mode %s != GUIDED", self.mode(source_system=target_system))
                    mod_changed = self.set_mode(
                        mode=expected_mode,
                        target_system=target_system,
                        timeout_ack=timeout_ack,
                        timeout_set_mode=timeout_set_mode
                    )
                    if not mod_changed:
                        self.logger.critical("Takeoff aborted")
                        return False
            else:
                self.logger.info("No mode check: device is not running ArduPilot")

            m = self.get_msg(
                msg_type="EXTENDED_SYS_STATE",
                source_system=target_system
            )
            already_airborne = bool(
                m and (
                    m.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
                    or m.landed_state == mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF
                )
            )
            if already_airborne and (is_vtol or pitch == 0):
                # Send an upward goto when already airborne (vertical climb on Copter / VTOL)
                self.logger.info("Vehicle already in the air: sending goto request instead of normal takeoff..")
                current_lat, current_lon, _, _ = self.position_gps(source_system=target_system)
                self.master.mav.set_position_target_global_int_send(
                    self.time_boot_ms, # time_boot_ms
                    target_system, # target_system
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # coordinate_frame
                    0b110111111000, # type_mask
                    int(current_lat * 1e7), # lat_int
                    int(current_lon * 1e7), # lon_int
                    altitude, # relative_alt
                    0,0,0, # vx, vy, vz
                    0,0,0, # afx, afy, afz
                    0,0 # yaw, yaw_rate
                )
                goto_sent = True

            else:
                effective_pitch = 0 if (is_ardupilot and is_vtol) else pitch
                self.logger.info("Sending takeoff request to %s: (alt, pitch): (%s, %s)..", target_system, altitude, effective_pitch)
                self.master.mav.command_long_send(
                    target_system, # target_system
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
                    1, # confirmation
                    effective_pitch, # param1 (Pitch)
                    0,0, # params2-3 (unused)
                    0, #param4 (Yaw)
                    0, # param5 (Latitude)
                    0, # param6 (Longitude)
                    altitude # param7 (Altitude)
                )
                accepted = self.wait_command_ack(
                    command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    source_system=target_system,
                    timeout=timeout_ack
                )
                if accepted is None: # End of mission or Timeout
                    if self.mission_ended:
                        self.logger.critical("End of mission")
                    else:
                        self.logger.critical("Takeoff failed: ack timeout")
                    return False
                elif not accepted: # Refused
                    self.logger.error("Takeoff failed: refused")
                    return False

        def _altitude_reached(rel_alt: float) -> bool:
            if is_ardupilot and is_fixed_wing:
                return rel_alt >= altitude - precision
            return abs(rel_alt - altitude) < precision

        _, _, relative_alt, _ = self.position_gps(source_system=target_system)
        if _altitude_reached(relative_alt):
            self.logger.info("Takeoff succeeded")
            return True

        if timeout_takeoff == 0:
            self.logger.warning("Vehicle accepted the takeoff but has not taken off yet")
            return True

        # Wait for correct positioning
        start_time = time.time()
        while True:
            if self.mission_ended:
                self.logger.critical("Mission stopped: stopping wait for takeoff completion")
                return False

            _, _, relative_alt, _ = self.position_gps(source_system=target_system)
            if _altitude_reached(relative_alt):
                self.logger.info("Altitude reached: stopping wait for takeoff completion")
                return True

            if not goto_sent and not is_vtol:
                if not self.mission_ended and self.allow_actions:
                    m = self.get_msg(
                        msg_type="EXTENDED_SYS_STATE",
                        source_system=target_system
                    )
                    # pitch==0 means that you want to takeoff vertically, MAV_LANDED_STATE_IN_AIR means takeoff is complete
                    if pitch == 0 and m and m.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
                        self.logger.info("Sending goto repositioning request to %s. alt: %s..", target_system, altitude)
                        current_lat, current_lon, _, _ = self.position_gps(source_system=target_system)
                        self.master.mav.set_position_target_global_int_send(
                            self.time_boot_ms, # time_boot_ms
                            target_system, # target_system
                            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # coordinate_frame
                            0b110111111000, # type_mask
                            int(current_lat * 1e7), # lat_int
                            int(current_lon * 1e7), # lon_int
                            altitude, # relative_alt
                            0,0,0, # vx, vy, vz
                            0,0,0, # afx, afy, afz
                            0,0 # yaw, yaw_rate
                        )
                        goto_sent = True

            if timeout_takeoff is not None and start_time + timeout_takeoff < time.time():
                self.logger.critical("Timeout: stopping wait for takeoff completion")
                return False

            if is_ardupilot:
                if self.mode(source_system=target_system) != expected_mode:
                    self.logger.warning("Mode changed: stopping wait for takeoff completion")
                    return False

            time.sleep(0.05)

    def land(self,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Perform an automated landing. Currently implemented only for ArduPilot.

        Automatically adapts the landing strategy to the vehicle type:

        - **Multirotors (ArduCopter)**: Switches to ``LAND`` flight mode and waits for touchdown and disarm.
        - **VTOL / QuadPlane (ArduPlane)**: Switches to ``QLAND`` flight mode (vertical descent) and waits for touchdown and disarm.
          Detected via ``MAV_TYPE_VTOL_*`` or a non-undefined ``EXTENDED_SYS_STATE.vtol_state`` (for QuadPlanes reporting ``MAV_TYPE_FIXED_WING``).
        - **Fixed-Wing (ArduPlane)**: Triggers the mission landing sequence via ``do_land_start()`` (``MAV_CMD_DO_LAND_START`` -> ``AUTO`` mode).
          If no ``MAV_CMD_DO_LAND_START`` sequence is present in the mission, falls back to ``AUTOLAND`` flight mode (ArduPlane 4.6+), then waits for touchdown and disarm.

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): Timeout while waiting for the ack. Default: 2.
            **timeout_set_mode (float|None): Timeout while waiting for the mode change. Default: 3.
            **timeout_land (float|None): Timeout while waiting for the landing (motors disarmed) to complete. 0 skips waiting (returns False). None means no timeout. Default: None.

        Returns:
            bool: Result of the operation.

        Note:
            - On Fixed-Wing (ArduPlane), mission-based landing requires a ``MAV_CMD_DO_LAND_START`` item followed by
              a landing sequence (ending with ``MAV_CMD_NAV_LAND``) in the uploaded mission. Without one, ``land()``
              falls back to ArduPlane's ``AUTOLAND`` mode (requires ArduPlane 4.6+, aircraft airborne, and a captured
              takeoff direction).
            - On Fixed-Wing (ArduPlane), automatic disarm after touchdown depends on the ``LAND_DISARMDELAY`` parameter
              (default: 20 s). If ``LAND_DISARMDELAY`` is set to 0, the aircraft will not auto-disarm on landing and
              waiting for disarm will time out unless handled accordingly.
            - On VTOL / QuadPlane, if you prefer to execute a planned mission landing (e.g., ``NAV_VTOL_LAND``)
              instead of vertical ``QLAND`` at the current position, call ``do_land_start()`` directly.
        """
        self.logger.info("Landing..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        timeout_set_mode = kwargs.get('timeout_set_mode', 3)
        timeout_land = kwargs.get('timeout_land', None)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to land")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to land")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        if self.autopilot_int(source_system=target_system) != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            self.logger.error("Landing not implemented outside ArduPilot")
            return False

        is_vtol, is_fixed_wing = self._is_vtol_and_fixed_wing(target_system)

        if is_vtol:
            expected_mode = "QLAND"
            if not self.set_mode(
                mode=expected_mode,
                target_system=target_system,
                timeout_ack=timeout_ack,
                timeout_set_mode=timeout_set_mode
            ):
                self.logger.critical("Landing failed")
                return False
        elif is_fixed_wing:
            if self.do_land_start(target_system=target_system, timeout_ack=timeout_ack):
                expected_mode = "AUTO"
                if timeout_set_mode != 0:
                    if not self.wait_mode_changed(
                        mode=expected_mode,
                        source_system=target_system,
                        timeout=timeout_set_mode
                    ):
                        self.logger.critical("Landing failed: vehicle did not enter AUTO mode after DO_LAND_START")
                        return False
            else:
                if self.mission_ended:
                    self.logger.critical("Mission stopped: Unable to land")
                    return False
                self.logger.warning("DO_LAND_START unavailable or refused: falling back to AUTOLAND mode..")
                expected_mode = "AUTOLAND"
                if not self.set_mode(
                    mode=expected_mode,
                    target_system=target_system,
                    timeout_ack=timeout_ack,
                    timeout_set_mode=timeout_set_mode
                ):
                    self.logger.critical("Landing failed")
                    return False
        else:
            expected_mode = "LAND"
            if not self.set_mode(
                mode=expected_mode,
                target_system=target_system,
                timeout_ack=timeout_ack,
                timeout_set_mode=timeout_set_mode
            ):
                self.logger.critical("Landing failed")
                return False

        if timeout_land == 0:
            self.logger.warning("Null timeout: not waiting for landing")
            return False

        self.logger.info("Waiting for landing (mode: %s)..", expected_mode)
        start_time = time.time()
        while True:
            disarmed = self.motors_disarmed(source_system=target_system)
            if self.mission_ended:
                self.logger.critical("Mission stopped: stopping wait for landing")
                return False

            if disarmed:
                self.logger.info("Drone disarmed: stopping wait for landing")
                return True

            if self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                if self.mode(source_system=target_system) != expected_mode:
                    self.logger.error("Mode changed: stopping wait for landing")
                    return False

            if timeout_land is not None and time.time() > start_time + timeout_land:
                self.logger.error("Timeout: stopping wait for landing")
                return bool(disarmed)
            time.sleep(0.05)

    def vtol_transition(self,
            state: int,
            force_immediate: bool = False,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Requests a VTOL transition between Multicopter (MC) and Fixed-Wing (FW) states using ``MAV_CMD_DO_VTOL_TRANSITION``.

        Args:
            state (int): Target ``MAV_VTOL_STATE``. Only ``MAV_VTOL_STATE_MC`` or ``MAV_VTOL_STATE_FW`` are allowed.
            force_immediate (bool): Sets ``param2 = 1`` to force an immediate/emergency transition (PX4 quadchute). Ignored on ArduPilot QuadPlane. Default: False.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK wait timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.
            **timeout_transition (float|None): Timeout in seconds while waiting for ``vtol_state()`` to reach ``state`` after ACK acceptance. 0 skips waiting (returns True once ACK is accepted). None means no timeout. Default: 0.

        Returns:
            bool: True if the transition command was accepted (and completed if ``timeout_transition != 0``), False otherwise.

        Note:
            - On ArduPlane (QuadPlane), this command is only accepted while in ``AUTO`` flight mode.
              Outside ``AUTO`` mode, QuadPlane transitions are performed by switching between Q-modes (e.g., ``QLOITER``)
              and fixed-wing modes (e.g., ``FBWA``, ``CRUISE``, ``GUIDED``).
            - On PX4, this command is supported across modes and ``force_immediate=True`` triggers an immediate transition (use with caution depending on airspeed and flight envelope).
        """
        self.logger.info("Requesting VTOL transition (state=%s)..", state)
        timeout_ack = kwargs.get('timeout_ack', 2)
        timeout_transition = kwargs.get('timeout_transition', 0)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to perform VTOL transition")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to perform VTOL transition")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if timeout_transition is not None and (not math.isfinite(timeout_transition) or timeout_transition < 0):
            self.logger.error("Invalid timeout_transition: %s (must be >= 0 or None)", timeout_transition)
            return False

        # Strictly validate target VTOL state (only MC=3 and FW=4 int enum values are valid)
        if isinstance(state, bool) or not isinstance(state, int) or state not in (
            mavutil.mavlink.MAV_VTOL_STATE_MC,
            mavutil.mavlink.MAV_VTOL_STATE_FW,
        ):
            self.logger.error(
                "Invalid VTOL transition target state: %s (only MAV_VTOL_STATE_MC or MAV_VTOL_STATE_FW are allowed)",
                state,
            )
            return False
        target_state = state

        if target_system is None:
            target_system = self.master.target_system

        is_ardupilot = (self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
        param2 = 1 if force_immediate else 0
        if is_ardupilot:
            if force_immediate:
                self.logger.warning("force_immediate (param2) is ignored on ArduPilot QuadPlane")
                param2 = 0
            cur_mode = self.mode(source_system=target_system)
            if cur_mode != "AUTO":
                self.logger.warning(
                    "ArduPlane QuadPlane only accepts MAV_CMD_DO_VTOL_TRANSITION in AUTO mode (current mode: %s)",
                    cur_mode,
                )

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, # command
            1, # confirmation
            float(target_state), # param1 (MAV_VTOL_STATE_MC or MAV_VTOL_STATE_FW)
            float(param2), # param2 (0=normal, 1=force immediate on PX4)
            0,0,0,0,0 # param3-7 (unused)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted is None: # Mission end or timeout
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("DO_VTOL_TRANSITION timed out")
            return False
        elif not accepted: # Refused
            self.logger.error("DO_VTOL_TRANSITION failed or refused")
            return False

        self.logger.info("VTOL transition accepted (target state: %s)", mavutil.mavlink.enums["MAV_VTOL_STATE"][target_state].name)

        if timeout_transition == 0:
            return True

        self.logger.info("Waiting for VTOL transition to complete..")
        start_time = time.monotonic()
        while True:
            if self.mission_ended:
                self.logger.critical("Mission stopped: stopping wait for VTOL transition")
                return False

            if self.vtol_state(source_system=target_system) == target_state:
                self.logger.info("VTOL transition completed")
                return True

            if timeout_transition is not None and time.monotonic() > start_time + timeout_transition:
                self.logger.error("Timeout: stopping wait for VTOL transition")
                return False

            time.sleep(0.05)

    def return_to_launch(self,
            target_system: int|None = None,
            use_qrtl: bool = True,
            **kwargs
        ) -> bool:
        """Send a Return-To-Launch (RTL / QRTL) request.

        - **VTOL / QuadPlane (ArduPlane)** (``use_qrtl=True``): Switches to ``QRTL`` flight mode
          so the vehicle flies back to HOME (or the nearest Rally Point) and performs an automatic vertical VTOL landing.
        - **Multirotors (ArduCopter), Fixed-Wing (ArduPlane), PX4, or QuadPlane** (``use_qrtl=False``): standard RTL.

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            use_qrtl (bool): On ArduPlane VTOL / QuadPlane, use ``QRTL`` mode instead of ``MAV_CMD_NAV_RETURN_TO_LAUNCH``. Default: True.
            **timeout_ack (float|None): ACK waiting timeout. Default: 2.
            **timeout_set_mode (float|None): Mode change wait timeout used when switching to ``QRTL`` on QuadPlane. Default: 4.

        Returns:
            bool: Result of the request.

        Note:
            - On ArduPlane QuadPlane, standard RTL (``use_qrtl=False``) circles indefinitely above HOME unless
              ``Q_RTL_MODE`` is configured otherwise, whereas ``QRTL`` (``use_qrtl=True``) returns and lands vertically.
        """
        self.logger.debug("RTL..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        timeout_set_mode = kwargs.get('timeout_set_mode', 4)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to RTL")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to RTL")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        is_ardupilot = (self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
        is_vtol, _ = self._is_vtol_and_fixed_wing(target_system)

        if is_ardupilot and is_vtol and use_qrtl:
            self.logger.info("Switching VTOL/QuadPlane to QRTL mode..")
            if self.set_mode(
                mode="QRTL",
                target_system=target_system,
                timeout_ack=timeout_ack,
                timeout_set_mode=timeout_set_mode
            ):
                self.logger.info("QRTL started")
                return True
            self.logger.error("QRTL failed")
            return False

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, # command
            1, # confirmation
            0,0,0,0,0,0,0 # param1-7 (unused)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("RTL started")
        elif accepted is None: # Mission end or timeout
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("RTL timed out")
        else: # Refused
            self.logger.error("RTL failed")
        return bool(accepted)

    def do_follow_with_takeoff(self, target_system: int):
        """Follow a moving vehicle (+ takeoff). Works only on ArduPilot.

        Args:
            target_system (int): MAVLink ID of the vehicle to follow.
        """
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to follow")
            return
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to follow")
            return
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return

        if self.autopilot_int() != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            self.logger.error("FOLLOW not possible outside ArduPilot")
            return
        takeoff_succeded = self.takeoff(altitude=5)
        if not takeoff_succeded:
            return
        self.do_follow(target_system=target_system)

    def do_follow(self, target_system: int):
        """Follow a moving vehicle. Works only on ArduPilot.

        Args:
            target_system (int): MAVLink ID of the vehicle to follow.
        """
        self.logger.debug("Following..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to follow")
            return
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to follow")
            return
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return

        if self.autopilot_int() != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            self.logger.critical("FOLLOW not possible outside ArduPilot")
            return
        self.logger.info("Initializing FOLLOW..")

        rate = max(10, self.data_init["pos_rate"]) # Minimum 10 Hz
        self.set_msg_rate(
            msg_type="GLOBAL_POSITION_INT",
            rate=rate,
            target_system=target_system
        )
        self.set_msg_rate(
            msg_type="ATTITUDE_QUATERNION",
            rate=rate,
            target_system=target_system
        )
        self.set_msg_rate(
            msg_type="ATTITUDE",
            rate=rate,
            target_system=target_system
        )

        self.logger.info("Waiting for GPS..")
        while self.get_msg(
            msg_type="GLOBAL_POSITION_INT",
            source_system=target_system
        ) is None:
            if self.mission_ended:
                self.logger.critical("Mission stopped: follow stopped")
                return
            time.sleep(0.05)
        self.logger.info("GPS received")

        if self.mission_ended:
            self.logger.critical("Mission stopped: follow stopped")
            return

        mode_mapping = self.mode_mapping()
        if mode_mapping is None or "FOLLOW" not in mode_mapping:
            mode_id = 23 # Do not change; works on ArduCopter
        else:
            mode_id = mode_mapping["FOLLOW"]

        mod_changed = self.set_mode(mode=mode_id)
        if not mod_changed:
            self.logger.critical("Mission stopped: Mode isn't FOLLOW")
            return

        self.logger.info("Follow started")

        while True:
            if self.mission_ended or not self.allow_actions:
                self.logger.critical("Mission stopped: follow stopped")
                self.set_mode(mode="GUIDED")
                return
            if self.custom_mode() != mode_id:
                self.logger.error("Mode != FOLLOW: follow stopped")
                return

            global_position_int = self.get_msg(
                msg_type="GLOBAL_POSITION_INT",
                source_system=target_system,
                max_age=1.0
            )
            if global_position_int is None:
                self.logger.warning("Target GPS data lost or stale (>1s): skipping follow_target_send")
                time.sleep(0.1)
                continue

            lat = global_position_int.lat
            lon = global_position_int.lon
            alt = global_position_int.alt / 1000
            vx = global_position_int.vx / 100
            vy = global_position_int.vy / 100
            vz = global_position_int.vz / 100

            est_capabilities = 1 | 2

            scaled_imu = self.get_msg(
                msg_type="SCALED_IMU",
                source_system=target_system,
                max_age=0.5
            )
            if scaled_imu is None:
                scaled_imu = self.get_msg(
                    msg_type="SCALED_IMU2",
                    source_system=target_system,
                    max_age=0.5
                )
            if scaled_imu is None:
                scaled_imu = self.get_msg(
                    msg_type="SCALED_IMU3",
                    source_system=target_system,
                    max_age=0.5
                )

            if scaled_imu:
                xacc = scaled_imu.xacc / 1000.0 * 9.80665
                yacc = scaled_imu.yacc / 1000.0 * 9.80665
                zacc = scaled_imu.zacc / 1000.0 * 9.80665
                est_capabilities |= 4
            else:
                xacc, yacc, zacc = 0, 0, 0

            attitude_quaternion = self.get_msg(
                msg_type="ATTITUDE_QUATERNION",
                source_system=target_system,
                max_age=0.5
            )
            attitude = self.get_msg(
                msg_type="ATTITUDE",
                source_system=target_system,
                max_age=0.5
            )

            if attitude_quaternion and attitude:
                q1 = attitude_quaternion.q1
                q2 = attitude_quaternion.q2
                q3 = attitude_quaternion.q3
                q4 = attitude_quaternion.q4
                rollspeed = attitude.rollspeed
                pitchspeed = attitude.pitchspeed
                yawspeed = attitude.yawspeed
                est_capabilities |= 8
            else:
                q1, q2, q3, q4 = 0, 0, 0, 0
                rollspeed, pitchspeed, yawspeed = 0, 0, 0

            self.master.mav.follow_target_send(
                self.time_boot_ms,  # timestamp (ms)
                est_capabilities, # est_capabilities (indicating which data is sent)
                lat, # Latitude (WGS84) (degE7)
                lon, # Longitude (WGS84) (degE7)
                alt, # Altitude (MSL) (m)
                [vx, vy, vz], # vel (m/s)
                [xacc, yacc, zacc], # acc (m/s²)
                [q1, q2, q3, q4], # attitude_q
                [rollspeed, pitchspeed, yawspeed], # rates
                [0, 0, 0], # position cov
                0 # custom_state
            )
            time.sleep(0.1) # 10 Hz

    def send_landing_target(self, x: float, y: float, z: float):
        """Send a landing target request. Movements are in the drone's BODY frame.

        Args:
            x (float): Front (+) / Back (-) (m)
            y (float): Right (+) / Left (-) (m)
            z (float): Down (+) / Up (-) (m)
        """
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to send a landing target")
            return
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to send a landing target")
            return
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return

        self.logger.debug("Sending landing target..")
        if self.autopilot_int() == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            distance_m = math.sqrt(x**2 + y**2 + z**2)
        else:
            distance_m = 0
        self.master.mav.landing_target_send(
            self.time_usec, # time_usec
            0, # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD, # frame
            0,0, # angle_x, angle_y (ignored)
            distance_m, # distance
            0,0, # size_x, size_y (ignored)
            x, y, z, # x, y, z
            [1, 0, 0, 0], # q
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_OTHER, # type
            1 # position_valid
        )
        self.logger.debug("Landing Target sent")

    def do_land_start(self,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Triggers an autonomous mission landing sequence using ``MAV_CMD_DO_LAND_START``.

        Commands the flight controller to jump to the closest ``MAV_CMD_DO_LAND_START`` marker in the loaded mission and switch to AUTO mode to execute the landing pattern.

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK waiting timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the command was accepted by the flight controller, False otherwise.

        Note:
            - Supported on ArduPlane and ArduCopter (4.2+, switches to AUTO / AUTO_RTL).
            - Fails (returns False) if no ``MAV_CMD_DO_LAND_START`` marker is present in the uploaded mission.
            - If multiple ``MAV_CMD_DO_LAND_START`` markers exist in the mission, ArduPilot chooses the closest one
              in 3D (latitude, longitude, altitude) to the vehicle's current location (or uses the
              coordinates of the following waypoint if the marker's coordinates are 0).
        """
        self.logger.info("Starting mission landing sequence (DO_LAND_START)..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to start landing sequence")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to start landing sequence")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_DO_LAND_START, # command
            1, # confirmation
            0,0,0,0,0,0,0 # param1-7 (ignored)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_LAND_START,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Landing sequence started")
        elif accepted is None: # Mission end or timeout
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("DO_LAND_START timed out")
        else: # Refused
            self.logger.error("DO_LAND_START failed or refused")
        return bool(accepted)
