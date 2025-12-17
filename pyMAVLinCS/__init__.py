# pyMAVLinCS/__init__.py
# Copyright (C) 2025 Noah Redon
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

import os
import time
import logging
import math
from typing import Callable
import unicodedata
import geopy.distance
from pymavlink import mavutil

from pyMAVLinCS.mavecstra import MavThread, TimesyncHandler, StatustextReceiver, MavLogger
from pyMAVLinCS import mavtypes
from pyMAVLinCS.mission_control_code import MCC, EMPTY
from pyMAVLinCS.setup_logger import default_logger

class MAVLinCS:
    """Allows connection to one or more flight controllers on one link."""
    def __init__(self,
            address: str,
            source_system: int = 255,
            source_component: int = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            target_system: int|None = None,
            baud: int = 57600,
            timeout_heartbeat: float|None = None,
            send_automatic_timesync: bool = True,
            timesync_delay: int = 3,
            send_automatic_home_request: bool = True,
            home_request_delay: int = 3,
            sysid_to_request_home: None|int|set|list[int] = None,
            pos_rate: float = 4,
            dialect: str = "ardupilotmega",
            logger: logging.Logger = default_logger,
            mavlogfile: str | None = 'mav.tlog',
            relative_path_to_cwd: bool = True,
            command_ack_to_hide: None|int|set|list[int] = None,
            mcc_class = MCC
        ):
        """Initializes the connection.

        Args:
            address (str): Connection address.
            source_system (int): Ground station MAVLink system ID.
            source_component (int): Ground station MAV_COMPONENT.
            target_system (int|None): MAVLink system ID of the target flight controller.
            baud (int): Connection baudrate.
            timeout_heartbeat (float): Timeout (in seconds) while waiting for a HEARTBEAT before considering the connection impossible (raises an Exception).
            send_automatic_timesync (bool): Whether to send TIMESYNC messages at regular intervals.
            timesync_delay (int): Delay (in seconds) between two automatic TIMESYNC messages.
            send_automatic_home_request (bool): Whether to request the HOME position at regular intervals.
            home_request_delay (int): Delay (in seconds) between two automatic HOME position requests.
            sysid_to_request_home (None|int|set|list[int]): List of system IDs to which HOME position requests should be sent regularly.
            pos_rate (float): Rate at which the flight controller sends position updates to the ground station.
            dialect (str): MAVLink dialect, essential if timeout_heartbeat == 0. 'ardupilotmega' for ArduPilot, 'common' for PX4.
            logger (logging.Logger): Logger to use.
            mavlogfile (str|None): Filename for MAVLink message logging.
            relative_path_to_cwd (bool): Whether the tlog path is relative to the current working directory.
            command_ack_to_hide (None|int|set|list[int]|): List of COMMAND_ACK (MAV_CMD) to show only in debug mode.
            mcc_class: MCC base class used in your code (gives access to your custom MCCs).

        Note:
            - If target_system is None, the MAVLink ID of the first detected vehicle is used as target.
            - If timeout_heartbeat is None, blocking wait until a valid HEARTBEAT is received (does not raise an Exception).
            - If timeout_heartbeat is 0, no HEARTBEAT wait is performed. Requires the correct MAVLink dialect and assumes MAVLink 2.
            - If sysid_to_request_home is None, HOME position requests are sent only to the target flight controller.
            - If mavlogfile is None, no .tlog file will be generated.
            - If pos_rate<=0, doesn't request position.
        """
        logger.debug("Initializing MAVLinCS..")
        # -----------------------
        # User-provided variables
        # -----------------------

        self.logger: logging.Logger = logger
        """Logger in use."""

        self.data_init: mavtypes.DataInit = {
            "address": address,
            "target_system": target_system,
            "source_system": source_system,
            "source_component": source_component,
            "baud": baud,
            "sysid_to_request_home": sysid_to_request_home,
            "pos_rate": pos_rate,
            "dialect": dialect,
            "command_ack_to_hide": command_ack_to_hide
        }
        """Initialization data."""

        # -----------------
        # General variables
        # -----------------

        self.mavthread: MavThread = MavThread(
            master=None,
            logger=logger
        )
        """Thread management object."""

        self.statustext_receiver: StatustextReceiver = StatustextReceiver()
        """STATUSTEXT management object."""

        self.timesync_handler: TimesyncHandler = TimesyncHandler(
            master=None,
            send_automatic_timesync=send_automatic_timesync,
            timesync_delay=timesync_delay,
            logger=logger
        )
        """TIMESYNC management object."""

        self.mcc_class = mcc_class
        """MCC base class used in your code (gives access to your custom MCCs)."""

        self.mcc: set = set()
        """Set containing received and sent MCCs."""

        self.master: mavutil.mavfile|None = None 
        """Master object used to communicate via MAVLink."""

        self.mission_ended: bool = False
        """Indicates whether the code should stop (prevents further actions and stops blocking functions)."""

        self.allow_actions: bool = True
        """Indicates whether flight command sending is allowed (arming, disarming, mode switching, etc.)."""

        self._send_automatic_home_request: bool = send_automatic_home_request
        self._home_request_delay: int = home_request_delay
        self._sysid_to_request_home: set = set()
        self._command_ack_to_hide: set = set()

        self._mavlogger: MavLogger = MavLogger(file=mavlogfile, relative_path_to_cwd=relative_path_to_cwd)
        self._time_boot: float = time.time()
        self._nb_heartbeat_since_last_home_request: int = home_request_delay

        self._additional_recv_msg_callback: Callable|None = None     
        self._additional_send_callback: Callable|None = None
        self._mcc_callback: Callable|None = None
        self._statustext_callback: Callable|None = None

        # Open connection
        self.open(
            address=address,
            source_system=source_system,
            source_component=source_component,
            target_system=target_system,
            baud=baud,
            timeout_heartbeat=timeout_heartbeat,
            pos_rate=pos_rate,
            dialect=dialect
        )

    # ----------------------------------------------
    # Time since the creation of the MAVLinCS object
    # ----------------------------------------------

    @property
    def time_boot(self) -> int:
        """Time since startup in seconds."""
        return int(time.time() - self._time_boot)

    @property
    def time_boot_ms(self) -> int:
        """Time since startup in milliseconds."""
        return int((time.time() - self._time_boot) * 1e3)

    @property
    def time_usec(self) -> int:
        """Unix timestamp in microseconds."""
        return int(time.time() * 1e6)

    # ----------------------
    # MAVLink dialect in use
    # ----------------------

    @property
    def dialect(self) -> str|None:
        """Returns the MAVLink dialect in use.

        Returns:
            str|None: MAVLink dialect (e.g., 'ardupilotmega').

        Note:
            The case dialect = None is not supposed to occur.
        """
        return getattr(mavutil, 'current_dialect', None)

    # ----
    # Data
    # ----

    def position_gps(self, source_system: int|None = None) -> mavtypes.GPSPosition:
        """GPS position.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            mavtypes.GPSPosition:
                - lat (float): latitude in degrees.
                - lon (float): longitude in degrees.
                - relative_alt (float): relative altitude in meters.
                - alt (float): absolute altitude (MSL) in meters.

        Note:
            All returned values are NaN if no data has been received.
        """ 
        m = self.get_msg(
            msg_type="GLOBAL_POSITION_INT",
            source_system=source_system
        )
        if m:
            lat, lon, relative_alt, alt = m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1e3, m.alt / 1e3
        else:
            lat, lon, relative_alt, alt = math.nan, math.nan, math.nan, math.nan
        return mavtypes.GPSPosition(lat=lat, lon=lon, relative_alt=relative_alt, alt=alt)

    def position_local(self, source_system: int|None = None) -> mavtypes.LocalPosition:
        """Local position.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            mavtypes.LocalPosition:
                - x (float): North (m).
                - y (float): East (m).
                - z (float): Down (m).

        Note:
            All returned values are NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="LOCAL_POSITION_NED",
            source_system=source_system
        )
        if m:
            x, y, z = m.x, m.y, m.z
        else:
            x, y, z = math.nan, math.nan, math.nan
        return mavtypes.LocalPosition(x=x, y=y, z=z)

    def angles(self, source_system: int|None = None) -> mavtypes.Angles:
        """Attitude angles.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            mavtypes.Angles:
                - roll (float): Rotation around the longitudinal axis (front-back) ([-pi ; +pi] rad).
                - pitch (float): Rotation around the lateral axis (left-right) ([-pi ; +pi] rad).
                - yaw (float): Rotation around the vertical axis (up-down) ([-pi ; +pi] rad).

        Note:
            All returned values are NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="ATTITUDE",
            source_system=source_system
        )
        if m:
            roll, pitch, yaw = m.roll, m.pitch, m.yaw
        else:
            roll, pitch, yaw = math.nan, math.nan, math.nan
        return mavtypes.Angles(roll=roll, pitch=pitch, yaw=yaw)

    def angular_rates(self, source_system: int|None = None) -> mavtypes.AnglesRates:
        """Angular velocities.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            mavtypes.AnglesRates:
                - rollspeed (float): Rotation speed around the longitudinal axis (front-back) (rad/s).
                - pitchspeed (float): Rotation speed around the lateral axis (left-right) (rad/s).
                - yawspeed (float): Rotation speed around the vertical axis (up-down) (rad/s).

        Note:
            All returned values are NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="ATTITUDE",
            source_system=source_system
        )
        if m:
            rollspeed, pitchspeed, yawspeed = m.rollspeed, m.pitchspeed, m.yawspeed
        else:
            rollspeed, pitchspeed, yawspeed = math.nan, math.nan, math.nan
        return mavtypes.AnglesRates(rollspeed=rollspeed, pitchspeed=pitchspeed, yawspeed=yawspeed)

    def speed(self, source_system: int|None = None) -> mavtypes.Speed:
        """Velocities.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            mavtypes.Speed:
                - vx (float): Velocity in m/s towards North.
                - vy (float): Velocity in m/s towards East.
                - vz (float): Velocity in m/s downwards.

        Note:
            All returned values are NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="GLOBAL_POSITION_INT",
            source_system=source_system
        )
        if m:
            vx, vy, vz = m.vx / 1e2, m.vy / 1e2, m.vz / 1e2
        else:
            vx, vy, vz = math.nan, math.nan, math.nan
        return mavtypes.Speed(vx=vx, vy=vy, vz=vz)

    def speed_module(self, source_system: int|None = None) -> float:
        """Absolute speed.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            float: Speed in m/s.

        Note:
            Returns NaN if no data has been received.
        """
        vx, vy, vz = self.speed(source_system=source_system)
        if math.isnan(vx) or math.isnan(vy) or math.isnan(vz):
            return math.nan
        return math.sqrt(vx**2+vy**2+vz**2)

    def motors_armed(self, source_system: int|None = None) -> bool|None:
        """Arming status.

        Args:
            source_system (int|None): MAVLink system ID to check. Defaults to the master's ID.

        Returns:
            bool|None: True if the drone is armed, False otherwise.

        Note:
            Returns None if no data has been received.
        """
        hb = self.get_msg(
            msg_type="HEARTBEAT",
            source_system=source_system
        )
        if hb is None:
            return None
        return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def motors_disarmed(self, source_system: int|None = None) -> bool|None:
        """Disarming status.

        Args:
            source_system (int|None): MAVLink system ID to check. Defaults to the master's ID.

        Returns:
            bool|None: True if the drone is disarmed, False otherwise.

        Note:
            Returns None if no data has been received.
        """
        hb = self.get_msg(
            msg_type="HEARTBEAT",
            source_system=source_system
        )
        if hb is None:
            return None
        return not bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def mode(self, source_system: int|None = None) -> str:
        """Flight mode.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            str: Flight controller mode.

        Note:
            Returns 'UNKNOWN' if no valid data has been received.
        """
        if float(mavutil.mavlink.WIRE_PROTOCOL_VERSION) >= 1:
            # MAVLink 1 or MAVLink 2
            hb = self.get_msg(
                msg_type="HEARTBEAT",
                source_system=source_system
            )
            if hb:
                return mavutil.mode_string_v10(hb)
            # Fallback: we check HIGH_LATENCY2
            high_latency2 = self.get_msg(
                msg_type="HIGH_LATENCY2",
                source_system=source_system
            )
            if high_latency2:
                return mavutil.mode_string_v10(high_latency2)
            return "UNKNOW"
        elif float(mavutil.mavlink.WIRE_PROTOCOL_VERSION) == '0.9':
            # MAVLink 0.9
            sys_status = self.get_msg(
                msg_type="SYS_STATUS",
                source_system=source_system
            )
            if sys_status:
                return mavutil.mode_string_v09(sys_status)
            return "UNKNOW"
        return "UNKNOW"

    def custom_mode(self, source_system: int|None = None) -> int:
        """Flight mode as an integer (ArduPilot). On PX4, allows retrieving the Custom Main Mode and Custom Sub Mode.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int: Flight controller mode.

        Note:
            Returns NaN if no data has been received.
        """
        hb = self.get_msg(
            msg_type="HEARTBEAT",
            source_system=source_system
        )
        if hb:
            return hb.custom_mode
        # Fallback: we check HIGH_LATENCY2
        high_latency2 = self.get_msg(
            msg_type="HIGH_LATENCY2",
            source_system=source_system
        )
        if high_latency2:
            return high_latency2.custom_mode
        return math.nan

    def base_mode(self, source_system: int|None = None) -> int:
        """Base Mode. Partially characterizes the mode on PX4.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int: Base Mode.

        Note:
            Returns NaN if no data has been received.
        """
        hb = self.get_msg(
            msg_type="HEARTBEAT",
            source_system=source_system
        )
        if hb:
            return hb.base_mode
        return math.nan

    def custom_main_mode(self, source_system: int|None = None) -> int:
        """Custom Main Mode (PX4). Partially characterizes the mode on PX4.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int: Flight controller Custom Main Mode.

        Note:
            Returns NaN if no data has been received.
        """
        custom_mode = self.custom_mode(source_system=source_system)
        if math.isnan(custom_mode):
            return math.nan
        return (custom_mode & 0xFF0000) >> 16

    def custom_sub_mode(self, source_system: int|None = None) -> int:
        """Custom Sub Mode (PX4). Partially characterizes the mode on PX4.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int: Flight controller Custom Sub Mode.

        Note:
            Returns NaN if no data has been received.
        """
        custom_mode = self.custom_mode(source_system=source_system)
        if math.isnan(custom_mode):
            return math.nan
        return (custom_mode & 0xFF000000) >> 24

    def autopilot_int(self, source_system: int|None = None) -> int:
        """Autopilot.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int: MAV_AUTOPILOT | Autopilot.

        Note:
            Returns NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="HEARTBEAT",
            source_system=source_system
        )
        if m:
            return m.autopilot
        return math.nan

    def autopilot_str(self, source_system: int|None = None) -> str:
        """Autopilot.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            str: Autopilot (e.g.: 'MAV_AUTOPILOT_PX4', 'MAV_AUTOPILOT_ARDUPILOTMEGA').

        Note:
            Returns 'UNKNOW' if no valid data has been received.
        """
        m = self.get_msg(
            msg_type="HEARTBEAT",
            source_system=source_system
        )
        if m:
            return mavutil.mavlink.enums["MAV_AUTOPILOT"][m.autopilot].name
        return "UNKNOW"

    def battery_percentage(self, source_system: int|None = None) -> int:
        """Remaining battery (requires proper configuration of the flight controller).

        Args:
            source_system (int|None): MAVLink source system ID. Defaults to the master's ID.

        Returns:
            int: Remaining battery percentage.

        Note:
            Returns NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="SYS_STATUS",
            source_system=source_system
        )
        if m:
            return m.battery_remaining
        return math.nan

    def battery_voltage(self, source_system: int|None = None) -> float:
        """Battery voltage.

        Args:
            source_system (int|None): MAVLink source system ID. Defaults to the master's ID.

        Returns:
            float: Battery voltage in V.

        Note:
            Returns NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="SYS_STATUS",
            source_system=source_system
        )
        if m:
            return m.voltage_battery / 1e3 if m.voltage_battery != 65535 else math.nan
        return math.nan

    def gps_3d_fix(self, source_system: int|None = None) -> bool|None:
        """3D GPS fix status.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            bool|None: Whether there is a 3D GPS fix.

        Note:
            Returns None if no data has been received.
        """
        m = self.get_msg(
            msg_type="GPS_RAW_INT",
            source_system=source_system
        )
        if m is None:
            return None
        return bool(m.fix_type>=3 and m.lat != 0)

    def servo_pwm(self, servo_channel: int, source_system: int|None = None) -> float:
        """Servo PWM.

        Args:
            servo_channel (int): Servo channel (1-16 or 1-8 for MAVLink < 2).
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            float: PWM value (between 1000 and 2000).

        Note:
            Returns NaN if no data has been received.
        """
        m = self.get_msg(
           msg_type="SERVO_OUTPUT_RAW",
           source_system=source_system
        )
        if m is None:
            return math.nan
        return getattr(m, f"servo{servo_channel}_raw", math.nan)

    def gimbal_angles(self) -> mavtypes.GimbalAngles:
        """Gimbal angles in degrees.

        Returns:
            mavtypes.GimbalAngles:
                - roll (float): Roll in degrees.
                - pitch (float): Pitch in degrees.
                - yaw (float): Yaw in degrees.

        Note:
            All returned values are NaN if no data has been received.
        """
        m = self.get_msg(msg_type="GIMBAL_DEVICE_ATTITUDE_STATUS")
        if m is None:
            return mavtypes.GimbalAngles(roll=math.nan, pitch=math.nan, yaw=math.nan)
        roll, pitch, yaw = MAVLinCS.quaternion_to_euler(q=m.q)
        return mavtypes.GimbalAngles(roll=math.degrees(roll), pitch=math.degrees(pitch), yaw=math.degrees(yaw))

    def current_waypoint_seq(self, source_system: int|None = None) -> int:
        """Current waypoint seq.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int: Waypoint sequence number.

        Note:
            Returns NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="MISSION_CURRENT",
            source_system=source_system
        )
        if m:
            return m.seq
        return math.nan

    def is_waypoint_mission_completed(self, source_system: int|None = None) -> bool|None:
        """Checks whether the waypoint mission is completed or not.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            bool|None: True if the mission is completed, False otherwise.

        Note:
            Returns None if no data has been received.
        """
        m = self.get_msg(
            msg_type="MISSION_CURRENT",
            source_system=source_system
        )
        if m is None:
            return None
        if hasattr(m, "mission_state"):
            return bool(m.mission_state == mavutil.mavlink.MISSION_STATE_NOT_STARTED)
        return bool(m.seq == 0) # We assume being at wp 0 <=> Mission not started

    def home_position(self, source_system: int|None = None) -> mavtypes.HomePosition:
        """HOME position.

        Args:
            source_system (int|None): MAVLink source system of the HOME position. Defaults to the master's ID.

        Returns:
            mavtypes.HomePosition:
                - latitude_home (float): HOME latitude.
                - longitude_home (float): HOME longitude.
                - altitude_home (float): HOME altitude (MSL).

        Note:
            All returned values are NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="HOME_POSITION",
            source_system=source_system
        )
        if m:
            lat, lon, alt = m.latitude / 1e7, m.longitude / 1e7, m.altitude / 1e3
        else:
            lat, lon, alt = math.nan, math.nan, math.nan
        return mavtypes.HomePosition(latitude_home=lat, longitude_home=lon, altitude_home=alt)

    def ekf_origin(self, source_system: int|None = None) -> mavtypes.EkfOrigin:
        """EKF origin.

        Args:
            source_system (int|None): MAVLink source system of the EKF origin. Defaults to the master's ID.

        Returns:
            mavtypes.EkfOrigin:
                - latitude (float): EKF origin latitude.
                - longitude (float): EKF origin longitude.
                - altitude (float): EKF origin altitude (MSL).

        Note:
            All returned values are NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="GPS_GLOBAL_ORIGIN",
            source_system=source_system
        )
        if m:
            lat, lon, alt = m.latitude / 1e7, m.longitude / 1e7, m.altitude / 1e3
        else:
            lat, lon, alt = math.nan, math.nan, math.nan
        return mavtypes.EkfOrigin(latitude=lat, longitude=lon, altitude=alt)

    def get_distance_with_gps_pos(self,
            lat: float,
            lon: float,
            relative_alt: float|None = None,
            alt: float|None = None
        ) -> float:
        """Distance between the flight controller and a GPS point.

        Args:
            lat (float): Latitude of the GPS point (in degrees).
            lon (float): Longitude of the GPS point (in degrees).
            relative_alt (float): Relative altitude of the GPS point (in meters).
            alt (float): Absolute altitude (MSL) of the GPS point (in meters).

        Returns:
            float: Distance in meters.

        Note:
            - If relative_alt is None or NaN, relative altitude distance will not be considered.
            - If alt is None or NaN, altitude distance will not be considered.
            - If both relative_alt and alt are provided, alt takes priority.
            - Returns NaN if no data has been received.
        """
        gps = self.position_gps()
        if math.isnan(gps.lat) or math.isnan(gps.lon) or math.isnan(gps.relative_alt) or math.isnan(gps.alt):
            return math.nan
        return MAVLinCS.distance_between_two_gps_points(
            lat_i=gps.lat,
            lat_f=lat,
            lon_i=gps.lon,
            lon_f=lon,
            relative_alt_i=gps.relative_alt,
            relative_alt_f=relative_alt,
            alt_i=gps.alt,
            alt_f=alt
        )

    def get_distance_with_local_pos(self,
            x: float,
            y: float,
            z: float|None = None
        ) -> float:
        """Distance between the flight controller and a point in local coordinates.

        Args:
            x (float): North (m).
            y (float): East (m).
            z (float|None): Down (m).

        Returns:
            float: Distance in meters.

        Note:
            - If z is None or NaN, the Z distance will not be considered.
            - Returns NaN if no data has been received.
        """
        localpos = self.position_local()
        if math.isnan(localpos.x) or math.isnan(localpos.y) or math.isnan(localpos.z):
            return math.nan
        return MAVLinCS.distance_between_two_local_points(
            x_i=localpos.x,
            x_f=x,
            y_i=localpos.y,
            y_f=y,
            z_i=localpos.z,
            z_f=z
        )

    def get_latence_ms_with_sysidcompid(self, key: tuple[int, int]) -> int:
        """Returns the estimated one-way latency between our system and (sysid, compid) in ms.

        Args:
            key (tuple[int, int]): (sysid, compid)

        Returns:
            int: Latency in ms.

        Note:
            Returns NaN if no data has been received.
        """
        return self.timesync_handler.get_latence_ms_with_sysidcompid(key=key)

    def last_statustext_from_sysidcompid(self, key: tuple[int, int]):
        """Returns the last complete statustext received from (sysid, compid).

        Args:
            key (tuple[int, int]): (sysid, compid)

        Returns:
            MAVLink_message|None: Last complete STATUSTEXT received.

        Note:
            Returns None if no complete STATUSTEXT has been received.
        """
        return self.statustext_receiver.last_statustext_from_sysidcompid(key=key)

    # ---------
    # Commandes
    # ---------

    def set_servo(self,
            servo_channel: int,
            pwm: int,
            timeout_ack: float|None = 2,
            target_system: int|None = None
        ) -> bool:
        """Sets the position of a servo.

        Args:
            servo_channel (int): Servo channel.
            pwm (int): PWM value (between 1000 and 2000. 1000=closed, 2000=open).
            timeout_ack (float|None): Acknowledgment wait time.
            target_system (int|None): Target MAVLink system ID for the command. Defaults to the master's ID.

        Returns:
            bool: Result of the operation.
        """
        self.logger.debug("Setting servo..")
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
            timeout=timeout_ack,
            source_system=target_system
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

    def set_gimbal_target(self,
            latitude: float,
            longitude: float,
            altitude: float,
            timeout_ack: float = 3
        ) -> bool:
        """Sends a target GPS position.

        Args:
            latitude (float): Latitude in degrees.
            longitude (float): Longitude in degrees.
            altitude (float): Absolute altitude (MSL) in meters.
            timeout_ack (float): ACK wait timeout in seconds.

        Returns:
            bool: Result of the operation.
        """
        # Function not tested but should work by changing the gimbal mode (function to implement)
        self.logger.debug("Setting Gimbal Target..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set Gimbal target")
            return False
        if not self.allow_actions:
            self.logger.warning("Actions not allowed: Impossible to set Gimbal target")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            1,0,0,0,0,
            int(latitude*1e7),
            int(longitude*1e7),
            altitude
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
            timeout_ack: float = 3.0
        ) -> bool:
        """Orients the gimbal.

        Args:
            pitch (float): Pitch angle (rad).
            yaw (float): Yaw angle (rad).
            pitch_rate (float): Pitch rotation speed (rad/s).
            yaw_rate (float): Yaw rotation speed (rad/s).
            lock_pitch (bool): True = lock pitch relative to the horizon.
            lock_yaw (bool): True = lock yaw relative to North (earth frame).
            yaw_in_earth_frame (bool): True = yaw expressed in Earth frame, otherwise vehicle frame.
            timeout_ack (float): Maximum wait time for ACK (seconds).

        Returns:
            bool: True if ACK received (command accepted), False otherwise.
        """
        self.logger.debug("Setting Gimbal Angles..")
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
            1, pitch, yaw, pitch_rate, yaw_rate,    # Speeds (rad/s)
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

    def set_gimbal_retract(self, timeout_ack: float = 3.0) -> bool:
        """Sets the gimbal to retract position (no stabilization).

        Args:
            timeout_ack (float): Maximum wait time for ACK (seconds).

        Returns:
            bool: True if ACK received (command accepted), False otherwise.
        """
        self.logger.debug("Setting Gimbal Retract..")
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

    def set_gimbal_neutral(self, timeout_ack: float = 3.0) -> bool:
        """Sets the gimbal to neutral position (roll=pitch=yaw=0).

        Args:
            timeout_ack (float): Maximum wait time for ACK (seconds).

        Returns:
            bool: True if ACK received (command accepted), False otherwise.
        """
        self.logger.debug("Setting Gimbal Neutral..")
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
            timeout_ack: float|None = 2,
            timeout_home_reception: float|None = 2
        ) -> mavtypes.HomePosition:
        """Requests and waits to receive the HOME position.

        Args:
            target_system (int|None): Target MAVLink system ID for the request. Defaults to the master's ID.
            timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns NaN, NaN, NaN). None if no timeout.
            timeout_home_reception (float|None): Timeout for receiving the HOME position after ACK. 0 if no wait (returns NaN, NaN, NaN). None if no timeout.

        Returns:
            mavtypes.HomePosition:
                - latitude_home (float): HOME latitude.
                - longitude_home (float): HOME longitude.
                - altitude_home (float): HOME altitude (MSL).

        Note:
            All returned values are NaN if no data has been received.
        """
        self.logger.info("Requesting HOME..")
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

    def send_timesync(self) -> bool:
        """Sends a TIMESYNC in broadcast to obtain connection latencies.
        
        Returns:
            bool: True if the command was sent, False otherwise.
        """
        self.logger.debug("Sending TIMESYNC..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to send TIMESYNC")
            return False
        return self.timesync_handler.send_timesync()

    def set_msg_rate(self,
            msg_type: str | int,
            rate: float,
            target_system: int|None = None,
            timeout_ack: float|None = 2
        ) -> bool:
        """Changes the sending frequency of a specific flight controller message.

        Args:
            msg_type (str | int): Message type (name or ID).
            rate (float): Transmission frequency in Hz.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns False). None if no timeout.

        Returns:
            bool: True if the request was accepted by the flight controller, False otherwise.
        """
        self.logger.debug("Setting Msg Rate..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to set msg rate")
            return False
        if rate <= 0:
            self.logger.critical("Rate: %s must be greater than 0", rate)
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        # Search for message ID
        if isinstance(msg_type, str):
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{msg_type}", math.nan)
            if math.isnan(msg_id):
                self.logger.error("Wrong Msg Type: %s", msg_type)
                return False
        else:
            msg_id = msg_type

        self.master.mav.command_long_send(
            target_system, # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # command
            1, # confirmation
            msg_id, # param1 (Message ID)
            int(10**6/rate), # param2 (Interval)
            0,0,0,0,0 # param3-7 (unused)
        )
        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Msg Rate set")
        elif accepted is None: # End of mission or Timeout
            if self.mission_ended:
                self.logger.critical("End of mission")
            else:
                self.logger.critical("Msg Rate setting timed out")
        else: # Refused
            self.logger.error("Msg Rate setting failed")
        return bool(accepted)

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
            timeout_ack: float|None = 2,
            timeout_arming: float|None = 4,
            target_system: int|None = None
        ) -> bool:
        """Arms the drone.

        Args:
            force (bool): Whether to force arming or not.
            timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns False). None if no timeout.
            timeout_arming (float|None): Arming wait timeout. 0 if no wait after ACK (returns the ACK wait result as bool). None if no timeout.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: Whether the drone was armed or not.
        """
        self.logger.info("Arming..")
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
            if not force and custom_mode in ["INITIALIZING", "PREFLIGHT", "CALIBRATION"]:
                self.logger.error("Initializing: impossible to arm")
                return False

            if custom_mode in ["LAND", "TEST", "FOLLOW", "RTL", "FLOWHOLD", "BRAKE", "CIRCLE", "SMART_RTL", "SMARTRTL", "AUTO"]:
                self.logger.warning("Mode %s not armable: trying to change mode to GUIDED", custom_mode)
                mod_changed = self.set_mode(
                    mode="GUIDED",
                    target_system=target_system,
                    timeout_ack=3,
                    timeout_set_mode=5
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
            timeout=timeout_ack,
            source_system=target_system
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
            timeout_ack: float|None = 2,
            timeout_disarming: float|None = 4,
            target_system: int|None = None
        ) -> bool:
        """Disarms the drone.

        Args:
            force (bool): Whether to force disarming or not.
            timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns False). None if no timeout.
            timeout_disarming (float|None): Disarming wait timeout. 0 if no wait after ACK (returns the ACK wait result as bool). None if no timeout.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: Whether the drone was disarmed or not.
        """
        self.logger.info("Disarming..")
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
            timeout=timeout_ack,
            source_system=target_system
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
            timeout_ack: float|None = 2,
            timeout_set_mode: float|None = 4,
            target_system: int|None = None
        ) -> bool:
        """Changes the flight mode.

        Args:
            mode (str|int|tuple[int, int, int]): Flight mode. str: mode name. int: mode ID on ArduPilot. tuple[int, int, int]: (MAV_MODE_FLAG, Custom Main Mode, Custom Sub Mode) on PX4.
            timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns False). None if no timeout.
            timeout_set_mode (float|None): Mode change wait timeout. 0 if no wait (returns the ACK wait result as bool). None if no timeout.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: Whether the mode was changed or not.
        """
        self.logger.info("Setting Mode..")
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
            timeout=timeout_ack,
            source_system=target_system
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

        type_mask = 0b0000101111111000
        if math.isnan(yaw):
            yaw = 0
            type_mask += mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

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

        type_mask = 0b0000101111111000
        if math.isnan(yaw):
            yaw = 0
            type_mask += mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

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
            yaw: int|None = None,
            target_system: int|None = None
        ) -> bool:
        """Sets the drone's velocity.

        Args:
            vx (float): Velocity towards North in m/s.
            vy (float): Velocity towards East in m/s.
            vz (float): Velocity downwards in m/s.
            yaw (int|None): Desired yaw. Not defined by default.
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
        if yaw is None:
            bitmask = 0b0000111111000111
        else:
            bitmask = 0b0000101111000111

        self.master.mav.set_position_target_local_ned_send(
            self.time_boot_ms, # time_boot_ms
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # coordinate_frame
            bitmask, # type_mask
            0,0,0, # x, y, z
            vx, vy, vz, # vx, vy, vz
            0,0,0, # afx, afy, afz
            yaw if yaw is not None else 0,0 # yaw, yaw_rate
        )
        self.logger.debug("Speed set")
        return True

    def takeoff(self,
            altitude: float,
            pitch: float = 0,
            timeout_ack: float|None = 2,
            timeout_takeoff: float|None = None,
            precision: float = 1,
            target_system: int|None = None
        ) -> bool:
        """Perform a takeoff. Make sure to arm the drone beforehand.

        Args:
            altitude (float): Takeoff altitude.
            pitch (float): Enforced pitch.
            timeout_ack (float|None): Timeout while waiting for the ack. 0 means no waiting (returns False). None means no timeout.
            timeout_takeoff (float|None): Timeout while waiting for the takeoff to complete. 0 means no waiting (returns the ack result as bool). None means no timeout.
            precision (float): Required altitude precision in meters to consider the takeoff successful. Defaults to 1.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: Takeoff result.
        """
        self.logger.debug("Starting takeoff..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to takeoff")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to takeoff")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        # Find the correct flight mode
        if self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            if self.mode(source_system=target_system) != "GUIDED":
                self.logger.warning("Mode %s != GUIDED", self.mode(source_system=target_system))
                mod_changed = self.set_mode(
                    mode="GUIDED",
                    timeout_ack=3,
                    timeout_set_mode=5,
                    target_system=target_system
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
        if pitch==0 and m and (m.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR or m.landed_state == mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF):  # If already airborne or taking off
            # pitch==0 because we send an upward goto
            self.logger.info("Vehicle already in the air: sending goto request instead of normal takeoff..")
            current_lat, current_lon, _, _ = self.position_gps(source_system=target_system)
            self.master.mav.set_position_target_global_int_send(
                self.time_boot_ms, # time_boot_ms
                target_system, # target_system
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # coordinate_frame
                0b0000111111111000, # type_mask
                int(current_lat * 1e7), # lat_int
                int(current_lon * 1e7), # lon_int
                altitude, # relative_alt
                0,0,0, # vx, vy, vz
                0,0,0, # afx, afy, afz
                0,0 # yaw, yaw_rate
            )

        else:
            self.logger.info("Sending takeoff request to %s: (alt, pitch): (%s, %s)..", target_system, altitude, pitch)
            self.master.mav.command_long_send(
                target_system, # target_system
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
                1, # confirmation
                pitch, # param1 (Pitch)
                0,0, # params2-3 (unused)
                0, #param4 (Yaw)
                0, # param5 (Latitude)
                0, # param6 (Longitude)
                altitude # param7 (Altitude)
            )
            accepted = self.wait_command_ack(
                command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                timeout=timeout_ack, source_system=target_system
            )
            if accepted is None:
                self.logger.critical("Takeoff failed: insufficient data")
                return False
            elif not accepted:
                self.logger.critical("Takeoff failed: ack timeout")
                return False

        _, _, relative_alt, _ = self.position_gps(source_system=target_system)
        if abs(relative_alt - altitude) < precision:
            self.logger.info("Takeoff succeeded")
            return True

        if timeout_takeoff == 0:
            self.logger.warning("Vehicle accepted the takeoff but has not taken off yet")
            return True

        goto_sent = False

        # Wait for correct positioning
        start_time = time.time()
        while True:
            if self.mission_ended:
                self.logger.critical("Mission stopped: stopping wait for takeoff completion")
                return False

            _, _, relative_alt, _ = self.position_gps(source_system=target_system)
            if abs(relative_alt - altitude) < precision:
                self.logger.info("Altitude reached: stopping wait for takeoff completion")
                return True

            if not goto_sent:
                if not self.mission_ended and self.allow_actions:
                    m = self.get_msg(
                        msg_type="EXTENDED_SYS_STATE",
                        source_system=target_system
                    )
                    # pitch==0 means that you want to takeoff vertically, MAV_LANDED_STATE_IN_AIR means takeoff is complete
                    if pitch==0 and m and m.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
                        self.logger.info("Sending goto repositioning request to %s. alt: %s..", target_system, altitude)
                        current_lat, current_lon, _, _ = self.position_gps(source_system=target_system)
                        self.master.mav.set_position_target_global_int_send(
                            self.time_boot_ms, # time_boot_ms
                            target_system, # target_system
                            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # coordinate_frame
                            0b0000111111111000, # type_mask
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

            if self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                if self.mode(source_system=target_system) != "GUIDED":
                    self.logger.warning("Mode changed: stopping wait for takeoff completion")
                    return False

            time.sleep(0.05)

    def land(self,
            timeout_ack: float|None = 2,
            timeout_set_mode: float|None = 3,
            timeout_land: float|None = None,
            target_system: int|None = None
        ) -> bool:
        """Perform a simple landing. Currently implemented only for ArduPilot.

        Args:
            timeout_ack (float|None): Timeout while waiting for the ack.
            timeout_set_mode (float|None): Timeout while waiting for the mode change.
            timeout_land (float|None): Timeout while waiting for the landing to complete.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: Result of the operation.
        """
        self.logger.info("Landing..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to land")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to land")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if self.autopilot_int(source_system=target_system) != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            self.logger.error("Landing not implemented outside ArduPilot")
            return False

        if not self.set_mode(
            mode="LAND",
            timeout_ack=timeout_ack,
            timeout_set_mode=timeout_set_mode,
            target_system=target_system
        ):
            self.logger.critical("Landing failed")
            return False

        if timeout_land == 0:
            self.logger.warning("Null timeout: not waiting for landing")
            return False

        self.logger.info("Waiting for landing..")
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
                if self.mode(source_system=target_system) != "LAND":
                    self.logger.error("Mode changed: stopping wait for landing")
                    return False

            if timeout_land is not None and time.time() > start_time + timeout_land:
                self.logger.error("Timeout: stopping wait for landing")
                return disarmed
            time.sleep(0.05)

    def write_parameter(self,
            parameter: str, value,
            param_type: int,
            target_system: int|None = None
        ) -> bool:
        """Modify a flight controller parameter.

        Args:
            parameter (str): Parameter name.
            value: Parameter value.
            param_type (int): MAV_PARAM_TYPE | Parameter type.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: True if the command was sent, False otherwise.
        """
        # To know the associated param_type, ask ChatGPT, it works well
        # Note: some parameters require a reboot to take effect (check Mission Planner when changing them manually)
        self.logger.debug("Writing parameter..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to write parameter")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to write parameter")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system
        self.master.mav.param_set_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            parameter.encode(), # param_id
            value, # param_value
            param_type # param_type
        )
        self.logger.info("Parameter request sent")
        return True

    def send_waypoint(self,
            waypoint_index: int,
            frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command: int = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            param1: float = 0.0,
            param2: float = 0.0,
            param3: float = 0.0,
            param4: float = 0.0,
            x: float = 0.0,
            y: float = 0.0,
            z: float = 0.0,
            target_system: int|None = None
        ) -> bool:
        """Send a waypoint.

        Args:
            waypoint_index (int): Waypoint ID (sequence number). Starts at zero. Must increment for each waypoint with no gaps (0,1,2,3,4).
            frame (int): MAV_FRAME | Coordinate system of the waypoint.
            command (int): MAV_CMD | Waypoint action.
            param1 (float): param1 (depending on the command).
            param2 (float): param2 (depending on the command).
            param3 (float): param3 (depending on the command).
            param4 (float): param4 (depending on the command).
            x (float): Local: x position in meters, Global: latitude in degrees.
            y (float): Local: y position in meters, Global: longitude in degrees.
            z (float): Global: altitude in meters (relative or absolute, depending on the frame).
            target_system (int): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: True if the command was sent, False otherwise.
        """
        self.logger.debug("Sending waypoint..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to send a waypoint")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to send a waypoint")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system
        if frame in [
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
            ]:
            x = x * 1e7
            y = y * 1e7
        elif frame in [
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mavutil.mavlink.MAV_FRAME_LOCAL_ENU,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_LOCAL_FLU,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            mavutil.mavlink.MAV_FRAME_BODY_FRD
            ]:
            x = x * 1e4
            y = y * 1e4
        self.master.mav.mission_item_int_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            waypoint_index, # seq
            frame, # frame
            command, # command
            0, # current
            1, # autocontinue
            param1, # param1
            param2, # param2
            param3, # param3
            param4, # param4
            int(x), # x
            int(y), # y
            z # z
        )
        self.logger.info("Waypoint sent")
        return True

    def send_waypoint_count(self,
            number_waypoints: int,
            target_system: int|None = None
        ) -> bool:
        """Inform the flight controller of the number of waypoints that will be sent.
        Must be sent before each mission.

        Args:
            number_waypoints (int): Number of waypoints.
            target_system (int): Target MAVLink system ID.

        Returns:
            bool: True if the command was sent, False otherwise.
        """
        self.logger.debug("Sending waypoint count..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to send waypoint count")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to send waypoint count")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system
        self.master.mav.mission_count_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            number_waypoints # count
        )
        self.logger.info("Waypoint count sent")
        return True

    def clear_waypoints(self,
            timeout: float|None = 2,
            target_system: int|None = None
        ) -> bool:
        """Delete all waypoints.

        Args:
            timeout (float|None): Timeout while waiting for the ack.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: Result of the request.
        """
        self.logger.debug("Deleting waypoints..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to clear waypoints")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to clear waypoints")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system
        self.master.mav.mission_clear_all_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 # target_component
        )

        accepted = self.wait_mission_ack(
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            timeout=timeout,
            source_system=target_system
        )
        if accepted:
            self.logger.info("Waypoints cleared")
        elif accepted is None:  # Mission end or timeout
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("Waypoint clearing timed out")
        else:  # Refused
            self.logger.error("Waypoint clearing failed")
        return bool(accepted)

    def start_mission(self,
            first_wp_tkoff: bool,
            target_system: int|None = None,
            timeout: float|None = 3
        ) -> bool:
        """Start the mission. Currently implemented only for ArduPilot.

        Args:
            first_wp_tkoff (bool): True if the first waypoint is a takeoff waypoint, False otherwise.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            timeout (float|None): Waiting timeout (used in several places).

        Returns:
            bool: Whether the mission has started.
        """
        self.logger.info("Starting mission..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to start mission")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to start mission")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if self.autopilot_int(source_system=target_system) != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            self.logger.error("Start mission: Feature not implemented outside ArduPilot")
            return False

        if target_system is None:
            target_system = self.master.target_system
        if first_wp_tkoff:
            armed = self.arm(
                timeout_ack=timeout,
                timeout_arming=timeout,
                target_system=target_system
            )
            if not armed:
                self.logger.critical("Start mission failed")
                return False
            mod_auto = self.set_mode(
                mode="AUTO",
                timeout_ack=timeout,
                timeout_set_mode=timeout,
                target_system=target_system
            )
            if not mod_auto:
                self.logger.critical("Start mission failed")
                return False
            self.logger.info("Mission started")
            return True
        disarmed = self.disarm(
            timeout_ack=timeout,
            timeout_disarming=timeout,
            target_system=target_system
        )
        if not disarmed:
            self.logger.critical("Start mission failed")
            return False
        mod_auto = self.set_mode(
            mode="AUTO",
            timeout_ack=timeout,
            timeout_set_mode=timeout,
            target_system=target_system
        )
        if not mod_auto:
            self.logger.critical("Start mission failed")
            return False
        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_MISSION_START, # command
            1, # confirmation
            1, # param1 (First Item)
            0, # param2 (Last Item) (à confirmer)
            0,0,0,0,0 # param3-7 (unused)
        )
        ack = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_MISSION_START,
            source_system=target_system,
            timeout=timeout
        )
        if ack:
            self.logger.info("Mission started")
        elif ack is None: # Mission end or timeout
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("Mission start timed out")
        else: # Refused
            self.logger.error("Mission start failed")
        return bool(ack)

    def return_to_launch(self,
            timeout: float|None = 2,
            target_system: int|None = None
        ) -> bool:
        """Send a Return-To-Launch (RTL) request.

        Args:
            timeout (float|None): ACK waiting timeout.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: Result of the request.
        """
        self.logger.debug("RTL..")
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

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, # command
            1, # confirmation
            0,0,0,0,0,0,0 # param1-7 (unused)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            timeout=timeout,
            source_system=target_system
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

    def reboot(self,
            timeout_ack: float|None = 2,
            target_system: int|None = None,
            force=False
        ) -> bool:
        """Reboot the autopilot.

        Args:
            timeout_ack (float|None): ACK waiting timeout.
            target_system (int|None): Target MAVLink system ID.
            force (bool): Whether to perform a forced reboot (not recommended; for reference only).

        Returns:
            bool: Result of the operation.
        """
        self.logger.debug("Rebooting..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to reboot")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to reboot")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_ALL, # target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, # command
            1, # confirmation
            1, # param1 (Autopilot)
            0, # param2 (Companion)
            0, # param3 (Component action)
            0, # param4 (Component ID)
            0, # param5 (unused)
            20190226 if force else 0, # param6 (force)
            0 # param7 (unused)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            timeout=timeout_ack,
            source_system=target_system
        )
        if accepted:
            self.logger.info("Reboot accepted")
        elif accepted is None: # Mission end or timeout
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("Rebooting timed out")
        else: # Refused
            self.logger.error("Rebooting failed")
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
                source_system=target_system
            )

            lat = global_position_int.lat
            lon = global_position_int.lon
            alt = global_position_int.alt / 1000
            vx = global_position_int.vx / 100
            vy = global_position_int.vy / 100
            vz = global_position_int.vz / 100

            est_capabilities = 1 | 2

            scaled_imu = self.get_msg(
                msg_type="SCALED_IMU",
                source_system=target_system
            )
            if scaled_imu is None:
                scaled_imu = self.get_msg(
                    msg_type="SCALED_IMU2",
                    source_system=target_system
                )
            if scaled_imu is None:
                scaled_imu = self.get_msg(
                    msg_type="SCALED_IMU3",
                    source_system=target_system
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
                source_system=target_system
            )
            attitude = self.get_msg(
                msg_type="ATTITUDE",
                source_system=target_system
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

    # -------------
    # Communication
    # -------------

    def send_statustext(self,
            text: str,
            severity: int = 7,
            msg_id: int = 0
        ) -> bool:
        """Send a STATUSTEXT message.

        Args:
            text (str): Text to send.
            severity (int): Severity level.
            msg_id (int): STATUSTEXT ID.

        Returns:
            bool: True if the STATUSTEXT was sent, False otherwise.
        """
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to send a STATUSTEXT")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        max_chunk_size = 50
        chunks = [text[i:i+max_chunk_size] for i in range(0, len(text), max_chunk_size)]

        if len(chunks) > 1:
            self.logger.warning("Statustext truncated")

        self.logger.debug("Sending statustext: %s", text)
        for i, chunk in enumerate(chunks):
            try:
                text_encoded = chunk.ljust(50, '\0').encode('ascii')
            except Exception:
                text_encoded = unicodedata.normalize('NFKD', chunk.ljust(50, '\0')).encode('ascii', 'ignore')
            self.master.mav.statustext_send(
                severity,
                text_encoded,
                msg_id,
                i
            )
            if i == len(chunk) - 1:
                break # No need to sleep unnecessarily
            time.sleep(0.1)  # 10Hz
        self.logger.info("Statustext sent")
        return True

    def send_named_value_int(self, value: int, name = ""):
        """Send a NAMED_VALUE_INT message.

        Args:
            value (int): Value.
            name (str): Name.

        Returns:
            bool: True if the NAMED_VALUE_INT was sent, False otherwise.
        """
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to send a NAMED_VALUE_INT")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if len(name) > 10: # Limit
            name = name[:10]
            self.logger.warning("NAMED_VALUE_INT name too long: truncated")
        self.logger.debug("Sending named value int: %s", value)
        self.master.mav.named_value_int_send(
            self.time_boot_ms, # time_boot_ms
            name.encode("ascii"), # name
            value # value
        )
        return True

    def send_mcc(self, mcc: MCC) -> bool:
        """Send an MCC message.

        Args:
            mcc (MCC): MCC object.

        Returns:
            bool: True if the MCC was sent, False otherwise.
        """
        self.logger.debug("Sending MCC..")
        mcc_sent = self.send_named_value_int(value=mcc.value, name="MCC")
        if mcc_sent:
            self.logger.debug("MCC sent")
        else:
            self.logger.debug("MCC not sent")
        return mcc_sent

    def check_mcc(self, mcc: MCC) -> bool:
        """Check if an MCC has been received.

        Args:
            mcc (MCC): MCC to wait for.

        Returns:
            bool: True if the MCC is present, False otherwise.
        """
        return bool(mcc in self.mcc)

    def delete_mcc(self, mcc: MCC):
        """Delete the MCC.

        Args:
            mcc (MCC): MCC to delete.
        """
        if mcc in self.mcc:
            self.mcc.remove(mcc)

    # ------------
    # Data Waiting
    # ------------

    def wait_motors_armed(self, source_system: int|None = None, timeout: float|None = 4) -> bool|None:
        """Wait for the motors to be armed.

        Args:
            source_system (int|None): MAVLink source ID to wait for. Defaults to the master's ID.
            timeout (float|None): Armament waiting timeout. 0 means no wait (returns None). None means no timeout.

        Returns:
            bool|None: Whether the motors are armed or not.

        Note:
            Returns None if the mission ended or no data received (after timeout).
        """
        if timeout == 0:
            self.logger.warning("Zero timeout: not waiting for armament")
            return None

        self.logger.info("Waiting for armament..")
        start_time = time.time()
        while True:
            armed = self.motors_armed(source_system=source_system)
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for armament")
                return None

            if armed:
                self.logger.info("Drone armed: stop waiting for armament")
                return True

            if timeout is not None and time.time() > start_time + timeout:
                self.logger.error("Timeout: stop waiting for armament")
                return armed
            time.sleep(0.05)

    def wait_motors_disarmed(self, source_system: int|None = None, timeout: float|None = 4) -> bool|None:
        """Wait for the motors to be disarmed.

        Args:
            source_system (int|None): MAVLink source ID to wait for. Defaults to the master's ID.
            timeout (float|None): Disarm waiting timeout. 0 means no wait (returns None). None means no timeout.

        Returns:
            bool|None: Whether the motors are disarmed or not.

        Note:
            Returns None if the mission ended or no data received (after timeout).
        """
        if timeout == 0:
            self.logger.warning("Zero timeout: not waiting for disarmament")
            return None

        self.logger.info("Waiting for disarmament..")
        start_time = time.time()
        while True:
            disarmed = self.motors_disarmed(source_system=source_system)
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for disarmament")
                return None

            if disarmed:
                self.logger.info("Drone disarmed: stop waiting for disarmament")
                return True

            if timeout is not None and time.time() > start_time + timeout:
                self.logger.error("Timeout: stop waiting for disarmament")
                return disarmed
            time.sleep(0.05)

    def wait_mode_changed(self,
            mode: str | int | tuple[int, int, int],
            source_system: int|None = None,
            timeout: float|None = 4
        ) -> bool|None:
        """Wait for a flight mode change.

        Args:
            mode (str | int | tuple[int, int, int]): Flight mode. str: mode name. int: mode ID on ArduPilot. tuple[int, int, int]: (MAV_MODE_FLAG, Custom Main Mode, Custom Sub Mode) on PX4.
            source_system (int|None): MAVLink source ID to wait for. Defaults to the master's ID.
            timeout (float|None): Mode change waiting timeout. 0 means no wait (returns None). None means no timeout.

        Returns:
            bool|None: True if mode changed, False otherwise.

        Note:
            Returns None if mission ended or no data received (after timeout).
        """
        if timeout == 0:
            self.logger.warning("Zero timeout: not waiting for mode change")
            return None

        # Lookup IDs to identify the flight mode (if given as str)
        if isinstance(mode, str):
            mode_mapping = self.mode_mapping(source_system=source_system)
            if mode_mapping is not None and mode in mode_mapping:
                key = mode_mapping[mode]
            else:
                self.logger.critical("Waiting for mode change: Unknown mode %s", mode)
                self.logger.debug("mode_mapping(%s): %s", source_system, mode_mapping)
                return False
        else:
            key = mode

        # Decompose IDs
        if isinstance(key, int):
            base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            custom_main_mode = key
            custom_sub_mode = 0
        elif isinstance(key, tuple):
            if len(key) != 3:
                self.logger.error("Waiting for mode change: Invalid mode %s", key)
                return False
            for i in key:
                if not isinstance(i, int):
                    self.logger.error("Waiting for mode change: Invalid mode %s", key)
                    return False
            base_mode = key[0]
            custom_main_mode = key[1]
            custom_sub_mode = key[2]
        else:
            self.logger.error("Waiting for mode change: Invalid mode %s", key)
            return False

        self.logger.info("Waiting for mode change..")
        # Wait for mode change
        start_time = time.time()
        while True:
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for mode change")
                return None

            if self.autopilot_int(source_system=source_system) == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                if self.base_mode(source_system=source_system) == base_mode and \
                self.custom_main_mode(source_system=source_system) == custom_main_mode and \
                self.custom_sub_mode(source_system=source_system) == custom_sub_mode:
                    self.logger.info("Mode changed: stop waiting for mode change")
                    return True
            else:
                if self.custom_mode(source_system=source_system) == custom_main_mode:
                    self.logger.info("Mode changed: stop waiting for mode change")
                    return True

            if timeout is not None and time.time() > start_time + timeout:
                self.logger.error("Timeout: stop waiting for mode change")
                if math.isnan(self.base_mode(source_system=source_system)) or math.isnan(self.custom_mode(source_system=source_system)):
                    return None
                return False
            time.sleep(0.05)

    def wait_gps_3d_fix(self, source_system: int|None = None, timeout: float|None = None) -> bool|None:
        """Wait for a GPS 3D fix.

        Args:
            source_system (int|None): MAVLink source ID to wait for. Defaults to the master's ID.
            timeout (float|None): GPS fix waiting timeout. 0 means no wait (returns None). None means no timeout.

        Returns:
            bool|None: True if GPS 3D fix is achieved, False otherwise.

        Note:
            Returns None if the mission ended or no data received (after timeout).
        """
        if timeout == 0:
            self.logger.warning("Zero timeout: not waiting for GPS 3D fix")
            return None

        self.logger.info("Waiting for GPS 3D fix..")
        start_time = time.time()
        while True:
            gps_3d_fixed = self.gps_3d_fix(source_system=source_system)
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for GPS 3D fix")
                return None

            if gps_3d_fixed:
                self.logger.info("GPS 3D fix achieved: stop waiting for GPS 3D fix")
                return True

            if timeout is not None and time.time() > start_time + timeout:
                self.logger.error("Timeout: stop waiting for GPS 3D fix")
                return gps_3d_fixed
            time.sleep(0.05)

    def wait_proximity_with_gps_pos(self,
            lat: float,
            lon: float,
            relative_alt: float|None = None,
            alt: float|None = None,
            radius_acceptance: float = 1,
            timeout: float|None = None
        ) -> bool|None:
        """Wait to reach a certain distance between the flight controller and a GPS point.

        Args:
            lat (float): Latitude of the GPS point (degrees).
            lon (float): Longitude of the GPS point (degrees).
            relative_alt (float|None): Relative altitude of the GPS point (meters).
            alt (float|None): Absolute altitude (MSL) of the GPS point (meters).
            radius_acceptance (float): Distance threshold to consider reached.
            timeout (float|None): Proximity waiting timeout. 0 means no wait (returns None). None means no timeout.

        Returns:
            bool|None: True if within proximity, False otherwise.

        Note:
            Returns None if the mission ended or no data received (after timeout).
        """
        if timeout == 0:
            self.logger.warning("Zero timeout: not waiting for proximity")
            return None

        self.logger.info("Waiting for proximity..")
        start_time = time.time()
        while True:
            distance_prox = self.get_distance_with_gps_pos(
                lat=lat,
                lon=lon,
                relative_alt=relative_alt,
                alt=alt
            )
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for proximity")
                return None

            if distance_prox <= radius_acceptance:
                self.logger.info("Proximity reached: stop waiting for proximity")
                return True

            if timeout is not None and time.time() > start_time + timeout:
                self.logger.error("Timeout: stop waiting for proximity")
                if math.isnan(distance_prox):
                    return None
                return False
            time.sleep(0.05)

    def wait_proximity_with_local_pos(self,
            x: float,
            y: float,
            z: float|None = None,
            radius_acceptance: float = 1,
            timeout: float|None = None
        ) -> bool|None:
        """Wait to reach a certain distance between the flight controller and local coordinates.

        Args:
            x (float): North (m).
            y (float): East (m).
            z (float|None): Down (m).
            radius_acceptance (float): Distance threshold to consider reached.
            timeout (float|None): Proximity waiting timeout. 0 means no wait (returns None). None means no timeout.

        Returns:
            bool|None: True if within proximity, False otherwise.

        Note:
            Returns None if the mission ended or no data received (after timeout).
        """
        if timeout == 0:
            self.logger.warning("Zero timeout: not waiting for proximity")
            return None

        self.logger.info("Waiting for proximity..")
        start_time = time.time()
        while True:
            distance_prox = self.get_distance_with_local_pos(
                x=x,
                y=y,
                z=z
            )
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for proximity")
                return None

            if distance_prox <= radius_acceptance:
                self.logger.info("Proximity reached: stop waiting for proximity")
                return True

            if timeout is not None and time.time() > start_time + timeout:
                self.logger.error("Timeout: stop waiting for proximity")
                if math.isnan(distance_prox):
                    return None
                return False
            time.sleep(0.05)

    def wait_for_home(self, source_system: int|None = None, timeout: float|None = None) -> mavtypes.HomePosition:
        """Wait to receive the HOME position.

        Args:
            source_system (int|None): MAVLink source ID to wait for. Defaults to the master's ID.
            timeout (float|None): Reception waiting timeout. 0 means no wait (returns NaN, NaN, NaN). None means no timeout.

        Returns:
            mavtypes.HomePosition:
                - latitude_home (float): HOME latitude (deg).
                - longitude_home (float): HOME longitude (deg).
                - altitude_home (float): HOME altitude (MSL, m).

        Note:
            All returned values are NaN if the mission ended or no data received (after timeout).
        """
        start_time = time.time() - 0.01 # Small 10 ms tolerance

        if timeout == 0: # No result desired
            self.logger.warning("Zero timeout: not waiting for HOME")
            return math.nan, math.nan, math.nan

        self.logger.info("Waiting for HOME..")
        while True:
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for HOME")
                return math.nan, math.nan, math.nan

            m = self.get_msg(msg_type="HOME_POSITION", source_system=source_system)
            if m and m._timestamp >= start_time: # Received after search started
                self.logger.info("HOME received: stop waiting for HOME")
                return mavtypes.HomePosition(
                    latitude_home=m.latitude / 1e7,
                    longitude_home=m.longitude / 1e7,
                    altitude_home=m.altitude / 1e3
                )

            if timeout is not None and time.time() > start_time + timeout:
                self.logger.error("Timeout: stop waiting for HOME")
                return mavtypes.HomePosition(
                    latitude_home=math.nan,
                    longitude_home=math.nan,
                    altitude_home=math.nan
                )

            time.sleep(0.05)

    def wait_command_ack(self,
            command: int,
            source_system: int|None = None,
            timeout: float|None = 2
        ) -> bool|None:
        """Wait to receive a specific COMMAND_ACK and check if it was accepted.

        Args:
            command (int): MAV_CMD | Command to wait for.
            source_system (int|None): MAVLink source ID of the COMMAND_ACK. Defaults to master's target_system.
            timeout (float|None): Timeout for receiving COMMAND_ACK. 0 means no wait (returns None). None means no timeout.

        Returns:
            bool|None: True if COMMAND_ACK accepted, False if refused, None if mission ended or timeout.

        Note:
            Returns None if mission ended or no data received (after timeout).
        """
        if timeout == 0: # No result desired
            self.logger.debug("Zero timeout: not waiting for COMMAND_ACK")
            return None

        self.logger.debug("Waiting for COMMAND_ACK..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: stop waiting for COMMAND_ACK")
            return None

        if source_system is None:
            source_system = self.master.target_system

        condition = (
            f"COMMAND_ACK.command == {command} and "
            f"getattr(COMMAND_ACK, 'target_system', {self.master.source_system}) == {self.master.source_system} and "
            f"getattr(COMMAND_ACK, 'target_component', {self.master.source_component}) == {self.master.source_component}"
        )
        m = self.recv_match(
            type="COMMAND_ACK",
            condition=condition,
            blocking=True,
            timeout=timeout,
            source_system=source_system
        )
        if m:
            self.logger.debug("COMMAND_ACK received: stop waiting for COMMAND_ACK")
            return bool(m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED)
        elif self.mission_ended:
            self.logger.critical("Mission stopped: stop waiting for COMMAND_ACK")
            return None
        else:
            self.logger.error("Timeout: stop waiting for COMMAND_ACK")
            return None

    def wait_mission_ack(self,
            mission_type: int|None = None,
            source_system: int|None = None,
            timeout: float|None = 2
        ) -> bool|None:
        """Wait to receive a specific MISSION_ACK and check if it was accepted.

        Args:
            mission_type (int|None): MAV_MISSION_TYPE | Type of mission. None means any type.
            source_system (int|None): MAVLink source ID of the MISSION_ACK. Defaults to master's target_system.
            timeout (float|None): Timeout for receiving MISSION_ACK. 0 means no wait (returns None). None means no timeout.

        Returns:
            bool|None: True if MISSION_ACK accepted, False if refused, None if mission ended or timeout.

        Note:
            Returns None if mission ended or no data received (after timeout).
        """
        if timeout == 0:
            self.logger.debug("Zero timeout: not waiting for MISSION_ACK")
            return None

        self.logger.debug("Waiting for MISSION_ACK..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: stop waiting for MISSION_ACK")
            return None

        if source_system is None:
            source_system = self.master.target_system

        condition = (
            f"({mission_type} is None or getattr(MISSION_ACK, 'mission_type', {mission_type}) == {mission_type}) and "
            f"MISSION_ACK.target_system == {self.master.source_system} and "
            f"MISSION_ACK.target_component == {self.master.source_component}"
        )

        m = self.recv_match(
            type="MISSION_ACK",
            condition=condition,
            blocking=True,
            timeout=timeout,
            source_system=source_system
        )

        if m:
            self.logger.debug("MISSION_ACK received: stop waiting for MISSION_ACK")
            return bool(m.type == mavutil.mavlink.MAV_MISSION_ACCEPTED)
        elif self.mission_ended:
            self.logger.critical("Mission stopped: stop waiting for MISSION_ACK")
            return None
        else:
            self.logger.error("Timeout: stop waiting for MISSION_ACK")
            return None

    # ---------
    # Utilities
    # ---------

    def get_msg(self,
            msg_type: str,
            source_system: int|None = None
        ) -> any:
        """Return the last received message of a certain type from a given source_system.

        Args:
            msg_type (str): MAVLink message type.
            source_system (int|None): MAVLink source ID of the message. Defaults to master's target_system.

        Returns:
            msg | None: The MAVLink message if received, else None.

        Note:
            Returns None if the message hasn't been received yet.
        """
        if self.master is None:
            return None
        if source_system is None:
            source_system = self.master.target_system
        if source_system not in self.master.sysid_state:
            return None
        if msg_type not in self.master.sysid_state[source_system].messages:
            return None
        return self.master.sysid_state[source_system].messages.get(msg_type)

    def mode_mapping(self, source_system: int|None = None) -> dict | None:
        """Returns a dictionary mapping flight mode names to their IDs, or None if unknown.

        Args:
            source_system (int|None): MAVLink ID of the vehicle. Defaults to master's target_system.

        Returns:
            dict | None: Dictionary mapping mode names (str) to IDs (int for ArduPilot, tuple[int,int,int] for PX4), or None if no valid data.

        Note:
            Returns None if no valid HEARTBEAT message is received.
        """
        hb = self.get_msg(msg_type="HEARTBEAT", source_system=source_system)
        if hb is None:
            return None

        mav_type = hb.type
        mav_autopilot = hb.autopilot
        if mav_autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            return mavutil.px4_map
        if mav_type is None:
            return None
        return mavutil.mode_mapping_byname(mav_type)

    # ----------
    # Connection
    # ----------

    def open(self,
            address: str,
            source_system: int = 255,
            source_component: int = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            target_system: int|None = None,
            baud: int = 57600,
            timeout_heartbeat: float|None = None,
            pos_rate: float = 4,
            dialect: str = "ardupilotmega"
        ):
        """Opens a MAVLink connection. Ensure any previous connection is closed.

        Args:
            address (str): Connection address (e.g., serial port or UDP endpoint).
            source_system (int): MAVLink system ID for the ground station.
            source_component (int): MAVLink component ID for the ground station.
            target_system (int|None): MAVLink system ID of the flight controller. If None, first detected vehicle is used.
            baud (int): Baud rate for serial connection.
            timeout_heartbeat (float|None): Time to wait for a HEARTBEAT before considering connection impossible. None = wait indefinitely, 0 = skip waiting.
            pos_rate (float): Frequency (Hz) at which the flight controller sends position messages to the ground station.
            dialect (str): MAVLink dialect, required if timeout_heartbeat==0. "ardupilotmega" for ArduPilot, "common" for PX4.

        Note:
            - Sets up callbacks for sending and receiving MAVLink messages.
            - Initializes MAVLink version and dialect if timeout_heartbeat==0.
            - Starts background MAV thread for message handling.
            - If pos_rate<=0, doesn't request position.
        """
        self.logger.debug("Creating master object..")
        self.master: mavutil.mavfile = mavutil.mavlink_connection(
            device=address,
            baud=baud,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True
        )

        try:
            self.logger.debug("Master object created")
            self.timesync_handler.set_master(self.master)

            if timeout_heartbeat != 0:
                self.master.mav.set_callback(self._recv_msg_callback)
                self.logger.info("Waiting for HEARTBEAT%s", f" from ({target_system}:1)" if target_system else "")
                start_time = time.time()
                while True:
                    timeout_left = None
                    if timeout_heartbeat is not None:
                        timeout_left = start_time + timeout_heartbeat - time.time()
                        if timeout_left <= 0:
                            raise TimeoutError("No HEARTBEAT received")

                    hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout_left)
                    if not hb:
                        raise TimeoutError("No HEARTBEAT received")

                    self.logger.info("HEARTBEAT received from (%s:%s)", hb.get_srcSystem(), hb.get_srcComponent())
                    if (target_system is None or hb.get_srcSystem() == target_system) and hb.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                        self.master.target_system = hb.get_srcSystem()
                        self.master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                        self.logger.info("Stop waiting for HEARTBEAT")
                        break
            else:
                self.logger.info("No timeout for HEARTBEAT specified")
                self.master.first_byte = False
                self.master.WIRE_PROTOCOL_VERSION = "2.0"
                os.environ['MAVLINK20'] = '1'
                mavutil.set_dialect(dialect=dialect)
                self.master.mav = mavutil.mavlink.MAVLink(self.master, srcSystem=self.master.source_system, srcComponent=self.master.source_component)
                self.master.mav.robust_parsing = self.master.robust_parsing
                self.master.WIRE_PROTOCOL_VERSION = mavutil.mavlink.WIRE_PROTOCOL_VERSION
                self.master.mav.set_callback(self._recv_msg_callback)
                self.master.target_system = 1 if target_system is None else target_system
                self.master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

            self.master.mav.set_send_callback(self._send_callback)

            self._sysid_to_request_home.clear()
            sysid_to_request_home = self.data_init.get("sysid_to_request_home", None)
            if isinstance(sysid_to_request_home, int):
                self._sysid_to_request_home.add(sysid_to_request_home)
            elif isinstance(sysid_to_request_home, (list, set)):
                for sysid in sysid_to_request_home:
                    if isinstance(sysid, int):
                        self._sysid_to_request_home.add(sysid)
                    else:
                        self.logger.warning("Wrong sysid to request home : %s", sysid)
            elif sysid_to_request_home is None:
                self._sysid_to_request_home.add(self.master.target_system)
            else:
                self.logger.warning("Wrong sysid to request home : %s", sysid_to_request_home)

            command_ack_to_hide = self.data_init["command_ack_to_hide"]
            self._command_ack_to_hide.clear()
            if isinstance(command_ack_to_hide, int):
                if command_ack_to_hide in mavutil.mavlink.enums["MAV_CMD"]:
                    self._command_ack_to_hide.add(command_ack_to_hide)
                else:
                    self.logger.warning("%s is not a MAV_CMD", command_ack_to_hide)
            elif isinstance(command_ack_to_hide, (list, set)):
                for cmd in command_ack_to_hide:
                    if isinstance(cmd, int):
                        if cmd in mavutil.mavlink.enums["MAV_CMD"]:
                            self._command_ack_to_hide.add(cmd)
                        else:
                            self.logger.warning("%s is not a MAV_CMD", cmd)
                    else:
                        self.logger.warning("%s is not a MAV_CMD", cmd)
            elif command_ack_to_hide is not None:
                self.logger.warning("%s is not a MAV_CMD", cmd)

            # Start MAV thread
            self.mavthread.set_master(self.master)
            self.mavthread.start_thread()

            if timeout_heartbeat != 0 and pos_rate>0:
                for msg_type, rate in [("GLOBAL_POSITION_INT", pos_rate), ("LOCAL_POSITION_NED", pos_rate), ("ATTITUDE", pos_rate), ("EXTENDED_SYS_STATE", 2)]:
                    self.set_msg_rate(msg_type=msg_type, rate=rate)

            self.logger.info("Connection initialized")
        except (KeyboardInterrupt, Exception) as e:
            try:
                self.master.close()
            except Exception:
                pass
            raise e

    def close(self):
        """Closes the connection with the flight controller and stops all related threads."""
        self.logger.info("Closing connection..")
        
        # Stop background MAV thread
        self.mavthread.stop_thread()
        
        # Close master MAVLink connection
        try:
            if self.master is not None:
                self.master.close()
        except Exception:
            self.logger.exception("Error while closing MAVLink connection")
        
        # Close internal MAV logger
        self._mavlogger.close()
        
        self.logger.info("Connection closed")

    # --------------------------------------------------------------------------
    # Reception of MAVLink messages that do not involve any physical interaction
    # --------------------------------------------------------------------------

    def recv_match(self, condition=None, type=None, blocking=False, timeout=None, source_system=None, source_component=None):
        '''receive the next MAVLink message that matches the given condition
        type can be a string or a list of strings (almost same logic as the pymavlink one)'''
        if type is not None and not isinstance(type, list) and not isinstance(type, set):
            type = [type]
        if source_system is not None and not isinstance(source_system, list) and not isinstance(source_system, set):
            source_system = [source_system]
        if source_component is not None and not isinstance(source_component, list) and not isinstance(source_component, set):
            source_component = [source_component]
        start_time = time.time()
        while True:
            if self.mission_ended:
                return None
            if timeout is not None:
                now = time.time()
                if now < start_time:
                    start_time = now # If an external process rolls back system time, we should not spin forever.
                if start_time + timeout < time.time():
                    return None
            m = self.mavthread.recv_msg()
            if m is None:
                if blocking:
                    self.mavthread.select(0.05)
                    continue
                return None
            if type is not None and not m.get_type() in type:
                continue
            if source_system is not None and not m.get_srcSystem() in source_system:
                continue
            if source_component is not None and not m.get_srcComponent() in source_component:
                continue
            src_system = None
            if hasattr(m, 'get_srcSystem'):
                src_system = m.get_srcSystem()
            messages = getattr(self.master.sysid_state.get(src_system), 'messages', None)
            if not mavutil.evaluate_condition(condition, messages if messages is not None else self.master.messages):
                continue
            return m

    # ---------
    # Callbacks
    # ---------

    def _recv_msg_callback(self, m):
        try:
            self._mavlogger.log(m)
            # Analysis
            if m.get_type() == "COMMAND_ACK" and getattr(m, "target_system", self.master.source_system) == self.master.source_system and getattr(m, "target_component", self.master.source_component) == self.master.source_component:
                # Check that the COMMAND_ACK is intended for us
                command = mavutil.mavlink.enums["MAV_CMD"][m.command].name[8:]
                result = mavutil.mavlink.enums["MAV_RESULT"][m.result].name[11:]
                if m.command not in self._command_ack_to_hide:
                    self.logger.info("Got COMMAND_ACK: %s: %s", command, result)
                else:
                    self.logger.debug("Got COMMAND_ACK: %s: %s", command, result)
            elif m.get_type() == "MISSION_ACK" and m.target_system == self.master.source_system and m.target_component == self.master.source_component:
                # Check that the MISSION_ACK is intended for us
                t = mavutil.mavlink.enums["MAV_MISSION_TYPE"][m.mission_type].name[12:]
                res = mavutil.mavlink.enums["MAV_MISSION_RESULT"][m.type].name[12:]
                self.logger.info("Got MISSION_ACK: %s: %s", t, res)
            elif m.get_type() == "STATUSTEXT":
                result = self.statustext_receiver.parse(m)
                if result is not None: # A STATUSTEXT sequence has been completed
                    self.logger.info("AP: %s", result.text)
                    if self._statustext_callback is not None:
                        self._statustext_callback(result)
            elif m.get_type() == "NAMED_VALUE_INT":
                if m.name == "MCC": # This is an MCC
                    mcc = self.mcc_class.get(value=m.value)
                    self.logger.info(mcc.description)
                    self.mcc.add(mcc)
                    # MCC analysis
                    if self._mcc_callback is not None:
                        self._mcc_callback(mcc)
            elif m.get_type() == "TIMESYNC":
                self.timesync_handler.parse(m)
            if self._additional_recv_msg_callback is not None:
                self._additional_recv_msg_callback(m)
        except Exception:
            self.logger.exception('Error in recv_msg_callback')

    def set_additional_recv_msg_callback(self, callback: Callable | None):
        """Defines an additional callback to execute.

        Args:
            callback (Callable | None): Callback with 1 function argument: the MAVLink message.

        Note:
            Blocks actual message reception during this time (do not use a highly blocking function).
        """
        self._additional_recv_msg_callback = callback

    def set_mcc_callback(self, callback: Callable | None):
        """Defines a callback to execute when an MCC is received.

        Args:
            callback (Callable | None): Callback with 1 function argument: the received MCC (MCC class).

        Note:
            Blocks actual message reception during this time (do not use a highly blocking function).
        """
        self._mcc_callback = callback

    def set_statustext_callback(self, callback: Callable | None):
        """Defines a callback to execute when a complete STATUSTEXT is received.

        Args:
            callback (Callable | None): Callback with 1 function argument: the complete received STATUSTEXT message.

        Note:
            Blocks actual message reception during this time (do not use a highly blocking function).
        """
        self._statustext_callback = callback

    def _send_callback(self, m):
        try:
            self._mavlogger.log(m)
            if m.get_type() == "NAMED_VALUE_INT":
                if m.name == "MCC": # MCC
                    mcc = self.mcc_class.get(value=m.value)
                    self.logger.info(mcc.description)
                    if mcc != EMPTY:
                        self.mcc.add(mcc)
            elif m.get_type() == "TIMESYNC":
                self.timesync_handler.timesync_sent_callback(m)
            elif m.get_type() == "HEARTBEAT":
                self.timesync_handler.heartbeat_sent_callback()
                if self._send_automatic_home_request:
                    if self._nb_heartbeat_since_last_home_request >= self._home_request_delay:
                        for sysid in self._sysid_to_request_home:
                            self.master.mav.command_long_send(
                                sysid, # target_system
                                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
                                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, # Command to request a message
                                0, # confirmation
                                mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, # param1 (Message ID)
                                0,0,0,0,0,0 # param2-7 (unused)
                            )
                        self._nb_heartbeat_since_last_home_request = 1
                    else:
                        self._nb_heartbeat_since_last_home_request += 1
            if self._additional_send_callback is not None:
                self._additional_send_callback(m)
        except Exception:
            self.logger.exception('Error in send_callback')

    def set_additional_send_callback(self, callback: Callable | None):
        """Defines an additional callback to execute.

        Args:
            callback (Callable | None): Callback with 1 function argument: the MAVLink message.
        """
        self._additional_send_callback = callback

    # --------------
    # Static methods
    # --------------

    @staticmethod
    def quaternion_to_euler(q: list) -> mavtypes.Angles:
        """Conversion from Quaternion to Euler.

        Args:
            q (list): Quaternions.

        Returns:
            mavtypes.Angles: Euler angles.
        """
        w, x, y, z = q

        # Standard formulas for quaternion -> Euler conversion (Z-Y-X convention)
        sinp = 2 * (w * y - z * x)
        sinp = max(min(sinp, 1.0), -1.0)

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(sinp)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return mavtypes.Angles(roll=roll, pitch=pitch, yaw=yaw)

    @staticmethod
    def distance_between_two_gps_points(
            lat_i: float,
            lat_f: float,
            lon_i: float,
            lon_f: float,
            relative_alt_i: float|None = None,
            relative_alt_f: float|None = None,
            alt_i: float|None = None,
            alt_f: float|None = None
        ) -> float:
        """Distance between two GPS points.

        Args:
            lat_i (float): Latitude of the initial GPS point (in degrees).
            lat_f (float): Latitude of the final GPS point (in degrees).
            lon_i (float): Longitude of the initial GPS point (in degrees).
            lon_f (float): Longitude of the final GPS point (in degrees).
            relative_alt_i (float): Relative altitude of the initial GPS point (in meters).
            relative_alt_f (float): Relative altitude of the final GPS point (in meters).
            alt_i (float): Absolute altitude (MSL) of the initial GPS point (in meters).
            alt_f (float): Absolute altitude (MSL) of the final GPS point (in meters).

        Returns:
            float: Distance in meters.

        Note:
            - If relative_alt_(i or f) is None or NaN, the relative altitude difference will not be considered.
            - If alt_(i or f) is None or NaN, the altitude difference will not be considered.
            - If both relative_alt and alt fields are provided, the alt field takes priority.
        """
        lat1 = math.radians(lat_f)
        lon1 = math.radians(lon_f)
        lat2 = math.radians(lat_i)
        lon2 = math.radians(lon_i)
        if abs(lat2-lat1) < 1.0e-15:
            q = math.cos(lat1)
        else:
            q = (lat2-lat1)/math.log(math.tan(lat2/2+math.pi/4)/math.tan(lat1/2+math.pi/4))
        d = math.sqrt((lat2-lat1)**2 + q**2 * (lon2-lon1)**2)
        dist = d * 6378100.0
        if alt_i is not None and not math.isnan(alt_i) and alt_f is not None and not math.isnan(alt_f):
            return math.sqrt(dist**2 + (alt_f - alt_i)**2) # Return the 3D distance
        if relative_alt_i is not None and not math.isnan(relative_alt_i) and relative_alt_f is not None and not math.isnan(relative_alt_f):
            return math.sqrt(dist**2 + (relative_alt_f-relative_alt_i)**2)
        return dist

    @staticmethod
    def distance_between_two_local_points(
            x_i: float,
            x_f: float,
            y_i: float,
            y_f: float,
            z_i: float|None = None,
            z_f: float|None = None
        ) -> float:
        """Distance between two points in local coordinates.

        Args:
            x_i (float): North coordinate of the initial point (m).
            x_f (float): North coordinate of the final point (m).
            y_i (float): East coordinate of the initial point (m).
            y_f (float): East coordinate of the final point (m).
            z_i (float|None): Down coordinate of the initial point (m).
            z_f (float|None): Down coordinate of the final point (m).

        Returns:
            float: Distance in meters.

        Note:
            If z_(i or f) is None or NaN, the Down distance will not be considered.
        """
        if z_i is not None and not math.isnan(z_i) and z_f is not None and not math.isnan(z_f):
            return math.sqrt((x_f-x_i)**2+(y_f-y_i)**2+(z_f-z_i)**2)
        return math.sqrt((x_f-x_i)**2+(y_f-y_i)**2)

    @staticmethod
    def cartesian_to_geographic_point(
            lat_origine: float,
            lon_origine: float,
            north_displacement: float,
            east_displacement: float
        ) -> mavtypes.GeographicPoint:
        """Returns the GPS coordinates of a point from a Cartesian definition
        (with the drone as the origin).

        Args:
            lat_origine (float): Latitude of the origin point (deg).
            lon_origine (float): Longitude of the origin point (deg).
            north_displacement (float): North displacement (m).
            east_displacement (float): East displacement (m).

        Returns:
            mavtypes.GeographicPoint:
                - latitude (float): Latitude of the GPS point (deg).
                - longitude (float): Longitude of the GPS point (deg).

        Note:
            All returned values are NaN if no GPS data has been received.
        """
        start_point = (lat_origine, lon_origine)
        distance_meters = math.sqrt(north_displacement**2 + east_displacement**2)
        bearing = math.degrees(math.atan2(east_displacement, north_displacement))
        new_point = geopy.distance.distance(meters=distance_meters).destination(start_point, bearing)
        return mavtypes.GeographicPoint(latitude=new_point.latitude, longitude=new_point.longitude)

    @staticmethod
    def polar_to_geographic_point(
            lat_origine: float,
            lon_origine: float,
            distance_m: float,
            bearing: float
        ) -> mavtypes.GeographicPoint:
        """Returns the GPS coordinates of a point from a polar definition
        (with the drone as the origin).

        Args:
            lat_origine (float): Latitude of the origin point (deg).
            lon_origine (float): Longitude of the origin point (deg).
            distance_m (float): Distance (m).
            bearing (float): Bearing (deg).

        Returns:
            mavtypes.GeographicPoint:
                - latitude (float): Latitude of the GPS point (deg).
                - longitude (float): Longitude of the GPS point (deg).

        Note:
            All returned values are NaN if no GPS data has been received.
        """
        start_point = (lat_origine, lon_origine)
        new_point = geopy.distance.distance(meters=distance_m).destination(start_point, bearing)
        return mavtypes.GeographicPoint(latitude=new_point.latitude, longitude=new_point.longitude)

    @staticmethod
    def geographic_to_cartesian_point(
            lat_i: float,
            lon_i: float,
            lat_f: float,
            lon_f: float
        ) -> tuple[float, float]:
        """Computes the north and east displacements in meters
        to go from point i to point f.

        Args:
            lat_i (float): Latitude of the initial point (deg).
            lon_i (float): Longitude of the initial point (deg).
            lat_f (float): Latitude of the final point (deg).
            lon_f (float): Longitude of the final point (deg).

        Returns:
            tuple:
                - delta_north (float): North displacement (m).
                - delta_east (float): East displacement (m).
        """
        # North displacement (latitude difference, same longitude)
        delta_north = MAVLinCS.distance_between_two_gps_points(
            lat_i=lat_i,
            lat_f=lat_f,
            lon_i=lon_i,
            lon_f=lon_i
        )
        if lat_f < lat_i:
            delta_north *= -1

        # East displacement (longitude difference, same latitude)
        delta_east = MAVLinCS.distance_between_two_gps_points(
            lat_i=lat_i,
            lat_f=lat_i,
            lon_i=lon_i,
            lon_f=lon_f
        )
        if lon_f < lon_i:
            delta_east *= -1

        return delta_north, delta_east
