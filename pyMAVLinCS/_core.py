# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import os
import time
import logging
import math
from typing import Callable
import geopy
from geopy import Point
from geopy.distance import geodesic
from pymavlink import mavutil

from pyMAVLinCS.mavecstra import MavThread, TimesyncHandler, StatustextReceiver, MavLogger
from pyMAVLinCS import mavtypes
from pyMAVLinCS.mission_control_code import MCC, EMPTY
from pyMAVLinCS.setup_logger import default_logger


class _Core:
    """Core connection lifecycle, message I/O, telemetry getters, and geometry helpers."""

    _COPTER_MAV_TYPES = tuple(dict.fromkeys(
        getattr(mavutil.mavlink, name)
        for name in (
            "MAV_TYPE_QUADROTOR",
            "MAV_TYPE_HEXAROTOR",
            "MAV_TYPE_OCTOROTOR",
            "MAV_TYPE_TRICOPTER",
            "MAV_TYPE_HELICOPTER",
            "MAV_TYPE_COAXIAL",
            "MAV_TYPE_DECAROTOR",
            "MAV_TYPE_DODECAROTOR",
            "MAV_TYPE_GENERIC_MULTIROTOR",
        )
        if hasattr(mavutil.mavlink, name)
    ))

    _VTOL_MAV_TYPES = tuple(dict.fromkeys(
        getattr(mavutil.mavlink, name)
        for name in (
            "MAV_TYPE_VTOL_DUOROTOR",
            "MAV_TYPE_VTOL_TAILSITTER_DUOROTOR",
            "MAV_TYPE_VTOL_QUADROTOR",
            "MAV_TYPE_VTOL_TAILSITTER_QUADROTOR",
            "MAV_TYPE_VTOL_TILTROTOR",
            "MAV_TYPE_VTOL_TAILSITTER",
            "MAV_TYPE_VTOL_RESERVED2",
            "MAV_TYPE_VTOL_FIXEDROTOR",
            "MAV_TYPE_VTOL_RESERVED3",
            "MAV_TYPE_VTOL_RESERVED4",
            "MAV_TYPE_VTOL_RESERVED5",
            "MAV_TYPE_VTOL_GYRODYNE",
        )
        if hasattr(mavutil.mavlink, name)
    ))

    def __init__(self,
            address: str,
            baud: int = 57600,
            source_system: int = 255,
            source_component: int = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            target_system: int|None = None,
            dialect: str = "ardupilotmega",
            mcc_class = MCC,
            logger: logging.Logger = default_logger,
            mavlogfile: str|None = 'mav.tlog',
            relative_path_to_cwd: bool = True,
            **kwargs
        ):
        """Initializes the connection.

        Args:
            address (str): Connection address.
            baud (int): Connection baudrate.
            source_system (int): Ground station MAVLink system ID.
            source_component (int): Ground station MAV_COMPONENT.
            target_system (int|None): MAVLink system ID of the target flight controller.
            dialect (str): MAVLink dialect, essential if timeout_heartbeat == 0. 'ardupilotmega' for ArduPilot, 'common' for PX4.
            mcc_class (type[MCC]): MCC base class used in your code (gives access to your custom MCCs).
            logger (logging.Logger): Logger to use.
            mavlogfile (str|None): Filename for MAVLink message logging.
            relative_path_to_cwd (bool): Whether the tlog path is relative to the current working directory.
            **timeout_heartbeat (float): Timeout (in seconds) while waiting for a HEARTBEAT before considering the connection impossible (raises an Exception). Default: None.
            **pos_rate (float): Rate at which the flight controller sends position updates to the ground station. Default: 4.
            **send_automatic_timesync (bool): Whether to send TIMESYNC messages at regular intervals. Default: False.
            **timesync_delay (int): Delay (in seconds) between two automatic TIMESYNC messages. Default: 3.
            **send_automatic_home_request (bool): Whether to request the HOME position at regular intervals. Default: False.
            **home_request_delay (int): Delay (in seconds) between two automatic HOME position requests. Default: 3.
            **sysid_to_request_home (None|int|set|list[int]|tuple[int]): List of system IDs to which HOME position requests should be sent regularly. Default: None.
            **command_ack_to_hide (None|int|set|list[int]|tuple[int]): List of COMMAND_ACK (MAV_CMD) to show only in debug mode. Default: None.

        Note:
            - If ``target_system`` is None, the MAVLink ID of the first detected vehicle is used as target.
            - If ``mavlogfile`` is None, no ``.tlog`` file will be generated.
            - If ``timeout_heartbeat`` is None, blocking wait until a valid HEARTBEAT is received (does not raise an Exception).
            - If ``timeout_heartbeat`` is 0, no HEARTBEAT wait is performed. Requires the correct MAVLink dialect and assumes MAVLink 2.
            - If ``sysid_to_request_home`` is None, HOME position requests are sent only to the target flight controller.
            - If ``pos_rate <= 0``, doesn't request position.
        """
        logger.debug("Initializing MAVLinCS..")

        # ------
        # kwargs
        # ------

        timeout_heartbeat = kwargs.get('timeout_heartbeat', None)
        pos_rate = kwargs.get('pos_rate', 4)
        send_automatic_timesync = kwargs.get('send_automatic_timesync', False)
        timesync_delay = kwargs.get('timesync_delay', 3)
        send_automatic_home_request = kwargs.get('send_automatic_home_request', False)
        home_request_delay = kwargs.get('home_request_delay', 3)
        sysid_to_request_home = kwargs.get('sysid_to_request_home', None)
        command_ack_to_hide = kwargs.get('command_ack_to_hide', mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE)

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

        self._additional_recv_msg_callback: Callable|None = None     
        self._additional_send_callback: Callable|None = None
        self._mcc_callback: Callable|None = None
        self._statustext_callback: Callable|None = None

        self.mavthread: MavThread = MavThread(
            mavlincs=self
        )
        """Thread management object."""

        # Open connection
        self.open(
            address=address,
            baud=baud,
            source_system=source_system,
            source_component=source_component,
            target_system=target_system,
            dialect=dialect,
            timeout_heartbeat=timeout_heartbeat,
            pos_rate=pos_rate
        )

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

    @property
    def dialect(self) -> str|None:
        """Returns the MAVLink dialect in use.

        Returns:
            str|None: MAVLink dialect (e.g., 'ardupilotmega').

        Note:
            The case dialect = None is not supposed to occur.
        """
        return getattr(mavutil, 'current_dialect', None)

    def get_msg(self,
            msg_type: str,
            source_system: int|None = None,
            max_age: float|None = None
        ) -> any:
        """Return the last received message of a certain type from a given source_system.

        Args:
            msg_type (str): MAVLink message type.
            source_system (int|None): MAVLink source ID of the message. Defaults to master's target_system.
            max_age (float|None): Maximum age (in s) of the last message to be considered valid.

        Returns:
            MAVLink_message|None: The MAVLink message if received, else None.

        Note:
            Returns None if the message hasn't been received yet.
        """
        if self.master is None:
            return None
        if source_system is None:
            source_system = self.master.target_system
        if source_system not in self.master.sysid_state:
            return None
        m = self.master.sysid_state[source_system].messages.get(msg_type, None)
        if m is not None and max_age is not None:
            if time.time() - getattr(m, "_timestamp", 0) > max_age:
                return None
        return m

    def _resolve_ardupilot_mode_map(self, mav_type: int, by_name: bool = False) -> dict|None:
        """Resolves ArduPilot mode mapping with fallbacks for MAV_TYPEs missing from pymavlink's AP_MAV_TYPE_MODE_MAP."""
        lookup = mavutil.mode_mapping_byname if by_name else mavutil.mode_mapping_bynumber
        mapping = lookup(mav_type)
        if mapping is not None:
            return mapping
        if mav_type in self._VTOL_MAV_TYPES:
            return lookup(mavutil.mavlink.MAV_TYPE_FIXED_WING)
        if mav_type in self._COPTER_MAV_TYPES:
            return lookup(mavutil.mavlink.MAV_TYPE_QUADROTOR)
        return None

    def mode_mapping(self, source_system: int|None = None) -> dict|None:
        """Returns a dictionary mapping flight mode names to their IDs, or None if unknown.

        Args:
            source_system (int|None): MAVLink ID of the vehicle. Defaults to master's target_system.

        Returns:
            dict|None: Dictionary mapping mode names (str) to IDs (int for ArduPilot, tuple[int,int,int] for PX4), or None if no valid data.

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
        if mav_autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            return self._resolve_ardupilot_mode_map(mav_type, by_name=True)
        return mavutil.mode_mapping_byname(mav_type)

    def open(self,
            address: str,
            baud: int = 57600,
            source_system: int = 255,
            source_component: int = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            target_system: int|None = None,
            dialect: str = "ardupilotmega",
            **kwargs
        ):
        """Opens a MAVLink connection. Ensure any previous connection is closed.

        Args:
            address (str): Connection address (e.g., serial port or UDP endpoint).
            baud (int): Baud rate for serial connection.
            source_system (int): MAVLink system ID for the ground station.
            source_component (int): MAVLink component ID for the ground station.
            target_system (int|None): MAVLink system ID of the flight controller. If None, first detected vehicle is used.
            dialect (str): MAVLink 2 dialect. Example: "ardupilotmega" for ArduPilot, "common" for PX4.
            **timeout_heartbeat (float|None): Time to wait for a HEARTBEAT before considering connection impossible. None = wait indefinitely, 0 = skip waiting. Default: None.
            **pos_rate (float): Frequency (Hz) at which the flight controller sends position messages to the ground station. Default: 4.

        Note:
            - Initializes MAVLink version and dialect.
            - Sets up callbacks for sending and receiving MAVLink messages.
            - Starts background MAV thread for message handling.
            - If ``pos_rate <= 0``, doesn't request position.
        """
        timeout_heartbeat = kwargs.get('timeout_heartbeat', None)
        pos_rate = kwargs.get('pos_rate', 4)
        self.logger.debug("Loading MAVLink version and dialect..")
        os.environ['MAVLINK20'] = '1'
        os.environ.pop('MAVLINK09', None)
        mavutil.set_dialect(dialect=dialect)
        self.logger.debug("MAVLink version and dialect loaded")

        self.logger.debug("Creating master object..")
        self.master: mavutil.mavfile = mavutil.mavlink_connection(
            device=address,
            baud=baud,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True
        )

        try:
            self.master.first_byte = False
            self.master.mav.set_callback(self._recv_msg_callback)
            self.master.mav.set_send_callback(self._send_callback)
            self.timesync_handler.set_master(self.master)
            self.mavthread.set_master()

            self.logger.debug("Master object created")

            if timeout_heartbeat != 0:
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
                self.master.target_system = 1 if target_system is None else target_system
                self.master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

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
            elif isinstance(command_ack_to_hide, (list, set, tuple)):
                for cmd in command_ack_to_hide:
                    if isinstance(cmd, int):
                        if cmd in mavutil.mavlink.enums["MAV_CMD"]:
                            self._command_ack_to_hide.add(cmd)
                        else:
                            self.logger.warning("%s is not a MAV_CMD", cmd)
                    else:
                        self.logger.warning("%s is not a MAV_CMD", cmd)
            elif command_ack_to_hide is not None:
                self.logger.warning("%s does not represent any MAV_CMD", command_ack_to_hide)

            # Start MAV thread
            self.mavthread.start_thread()

            if timeout_heartbeat != 0 and pos_rate>0:
                for msg_type, rate in [("GLOBAL_POSITION_INT", pos_rate), ("LOCAL_POSITION_NED", pos_rate), ("ATTITUDE", pos_rate), ("VFR_HUD", pos_rate), ("EXTENDED_SYS_STATE", 2)]:
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

    def __enter__(self) -> "MAVLinCS":
        """Enters the runtime context related to this object.

        Returns:
            MAVLinCS: The active MAVLinCS instance.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        """Exits the runtime context and ensures the connection and threads are properly closed.

        Args:
            exc_type: Exception type if an exception was raised in the with block, else None.
            exc_val: Exception instance if an exception was raised, else None.
            exc_tb: Traceback if an exception was raised, else None.

        Returns:
            bool: False so that any exception raised inside the with block is propagated.
        """
        self.close()
        return False

    def recv_match(self, condition=None, type=None, blocking=False, timeout=None, source_system=None, source_component=None):
        """Receives the next MAVLink message matching the given filters (similar to pymavlink's ``recv_match``).

        Args:
            condition (str|None): Optional pymavlink condition expression evaluated against received messages. Default: None.
            type (str|list[str]|set[str]|None): Message type name or collection of type names to match. Default: None.
            blocking (bool): If True, waits until a matching message arrives or ``timeout`` expires. Default: False.
            timeout (float|None): Maximum wait time in seconds when ``blocking=True`` (None waits indefinitely). Default: None.
            source_system (int|list[int]|set[int]|None): Filter by source MAVLink system ID(s). Default: None.
            source_component (int|list[int]|set[int]|None): Filter by source MAVLink component ID(s). Default: None.

        Returns:
            MAVLink_message|None: The matching MAVLink message, or None if none was received before timeout or mission end.
        """
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
            if self.master is None:
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
            if condition is not None:
                base_msgs = getattr(self.master.sysid_state.get(m.get_srcSystem()), 'messages', {'MAV' : self.master}) if self.master else {}
                try:
                    eval_context = base_msgs.copy()
                except Exception:
                    eval_context = {'MAV' : self.master}
                eval_context[m.get_type()] = m
                if not mavutil.evaluate_condition(condition, eval_context):
                    continue
            return m

    def set_msg_rate(self,
            msg_type: str|int,
            rate: float,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Changes the sending frequency of a specific flight controller message.

        Args:
            msg_type (str|int): Message type (name or ID).
            rate (float): Transmission frequency in Hz.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK wait timeout. 0 if no wait (returns False). None if no timeout. Default: 2.

        Returns:
            bool: True if the request was accepted by the flight controller, False otherwise.
        """
        self.logger.debug("Setting Msg Rate..")
        timeout_ack = kwargs.get('timeout_ack', 2)
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
        start_time = time.monotonic()
        while True:
            remaining = None
            if timeout is not None:
                remaining = (start_time + timeout) - time.monotonic()
                if remaining <= 0:
                    self.logger.error("Timeout: stop waiting for COMMAND_ACK")
                    return None

            m = self.recv_match(
                type="COMMAND_ACK",
                condition=condition,
                blocking=True,
                timeout=remaining,
                source_system=source_system
            )
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for COMMAND_ACK")
                return None
            if m is None:
                self.logger.error("Timeout: stop waiting for COMMAND_ACK")
                return None
            if m.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                self.logger.debug("COMMAND_ACK in progress, continuing wait..")
                continue
            self.logger.debug(f"COMMAND_ACK received with code {m.result}: stop waiting for COMMAND_ACK")
            return bool(m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED)

    def _recv_msg_callback(self, m):
        try:
            self._mavlogger.log(m)
            # Analysis
            if m.get_type() == "COMMAND_ACK" and getattr(m, "target_system", self.master.source_system) == self.master.source_system and getattr(m, "target_component", self.master.source_component) == self.master.source_component:
                # Check that the COMMAND_ACK is intended for us
                if m.command in mavutil.mavlink.enums["MAV_CMD"]:
                    command = mavutil.mavlink.enums["MAV_CMD"][m.command].name[8:]
                else:
                    command = f"UNKNOWN MAV_CMD ({m.command})"
                if m.result in mavutil.mavlink.enums["MAV_RESULT"]:
                    result = mavutil.mavlink.enums["MAV_RESULT"][m.result].name[11:]
                else:
                    result = f"UNKNOWN MAV_RESULT ({m.result})"
                if m.command not in self._command_ack_to_hide:
                    self.logger.info("Got COMMAND_ACK: %s: %s", command, result)
                else:
                    self.logger.debug("Got COMMAND_ACK: %s: %s", command, result)
            elif m.get_type() == "MISSION_ACK" and m.target_system == self.master.source_system and m.target_component == self.master.source_component:
                # Check that the MISSION_ACK is intended for us
                if m.mission_type in mavutil.mavlink.enums["MAV_MISSION_TYPE"]:
                    t = mavutil.mavlink.enums["MAV_MISSION_TYPE"][m.mission_type].name[12:]
                else:
                    t = f"UNKNOWN MAV_MISSION_TYPE ({m.mission_type})"
                if m.type in mavutil.mavlink.enums["MAV_MISSION_RESULT"]:
                    res = mavutil.mavlink.enums["MAV_MISSION_RESULT"][m.type].name[12:]
                else:
                    res = f"UNKNOWN MAV_MISSION_RESULT ({m.type})"
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

    def set_additional_recv_msg_callback(self, callback: Callable|None):
        """Defines an additional callback to execute.

        Args:
            callback (Callable|None): Callback with 1 function argument: the MAVLink message.

        Note:
            Blocks actual message reception during this time (do not use a highly blocking function).
        """
        self._additional_recv_msg_callback = callback

    def set_mcc_callback(self, callback: Callable|None):
        """Defines a callback to execute when an MCC is received.

        Args:
            callback (Callable|None): Callback with 1 function argument: the received MCC (MCC class).

        Note:
            Blocks actual message reception during this time (do not use a highly blocking function).
        """
        self._mcc_callback = callback

    def set_statustext_callback(self, callback: Callable|None):
        """Defines a callback to execute when a complete STATUSTEXT is received.

        Args:
            callback (Callable|None): Callback with 1 function argument: the complete received STATUSTEXT message.

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
            if self._additional_send_callback is not None:
                self._additional_send_callback(m)
        except Exception:
            self.logger.exception('Error in send_callback')

    def set_additional_send_callback(self, callback: Callable|None):
        """Defines an additional callback to execute.

        Args:
            callback (Callable|None): Callback with 1 function argument: the MAVLink message.
        """
        self._additional_send_callback = callback

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

    def vfr_hud(self, source_system: int|None = None) -> mavtypes.VfrHud:
        """Aeronautical HUD metrics (VFR_HUD).

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            mavtypes.VfrHud:
                - airspeed (float): Current airspeed in m/s.
                - groundspeed (float): Current horizontal ground speed in m/s.
                - heading (int|float): Current compass heading in degrees (0-360, 0=North).
                - throttle (int|float): Current throttle setting in percent (0-100).
                - alt (float): Current altitude (MSL) in meters.
                - climb (float): Current climb rate in m/s.

        Note:
            All returned values are NaN if no VFR_HUD message has been received.
        """
        m = self.get_msg(
            msg_type="VFR_HUD",
            source_system=source_system
        )
        if m:
            return mavtypes.VfrHud(
                airspeed=float(m.airspeed),
                groundspeed=float(m.groundspeed),
                heading=int(m.heading),
                throttle=int(m.throttle),
                alt=float(m.alt),
                climb=float(m.climb)
            )
        return mavtypes.VfrHud(
            airspeed=math.nan,
            groundspeed=math.nan,
            heading=math.nan,
            throttle=math.nan,
            alt=math.nan,
            climb=math.nan
        )

    def airspeed(self, source_system: int|None = None) -> float:
        """Current airspeed (calibrated/indicated airspeed on fixed-wing aircraft).

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            float: Airspeed in m/s.

        Note:
            Returns NaN if no VFR_HUD data has been received.
        """
        m = self.get_msg(
            msg_type="VFR_HUD",
            source_system=source_system
        )
        if m:
            return float(m.airspeed)
        return math.nan

    def groundspeed(self, source_system: int|None = None) -> float:
        """Current horizontal ground speed from VFR_HUD.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            float: Ground speed in m/s.

        Note:
            Returns NaN if no VFR_HUD data has been received.
        """
        m = self.get_msg(
            msg_type="VFR_HUD",
            source_system=source_system
        )
        if m:
            return float(m.groundspeed)
        return math.nan

    def heading(self, source_system: int|None = None) -> int|float:
        """Current compass heading from VFR_HUD.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: Heading in degrees (0-360, 0=North).

        Note:
            Returns NaN if no VFR_HUD data has been received.
        """
        m = self.get_msg(
            msg_type="VFR_HUD",
            source_system=source_system
        )
        if m:
            return int(m.heading)
        return math.nan

    def throttle(self, source_system: int|None = None) -> int|float:
        """Current throttle percentage from VFR_HUD.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: Throttle in percent (0-100).

        Note:
            Returns NaN if no VFR_HUD data has been received.
        """
        m = self.get_msg(
            msg_type="VFR_HUD",
            source_system=source_system
        )
        if m:
            return int(m.throttle)
        return math.nan

    def climb_rate(self, source_system: int|None = None) -> float:
        """Current vertical climb rate (variometer) from VFR_HUD.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            float: Climb rate in m/s.

        Note:
            Returns NaN if no VFR_HUD data has been received.
        """
        m = self.get_msg(
            msg_type="VFR_HUD",
            source_system=source_system
        )
        if m:
            return float(m.climb)
        return math.nan

    def wind(self, source_system: int|None = None) -> mavtypes.Wind:
        """Wind estimation (ArduPilot WIND message).

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            mavtypes.Wind:
                - direction (float): Wind direction (that wind is coming from) in degrees.
                - speed (float): Horizontal wind speed in ground plane in m/s.
                - speed_z (float): Vertical wind speed in m/s.

        Note:
            All returned values are NaN if no WIND message has been received.
        """
        m = self.get_msg(
            msg_type="WIND",
            source_system=source_system
        )
        if m:
            return mavtypes.Wind(
                direction=float(m.direction),
                speed=float(m.speed),
                speed_z=float(m.speed_z)
            )
        return mavtypes.Wind(
            direction=math.nan,
            speed=math.nan,
            speed_z=math.nan
        )

    def nav_controller_output(self, source_system: int|None = None) -> mavtypes.NavControllerOutput:
        """Navigation controller output.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            mavtypes.NavControllerOutput:
                - nav_roll (float): Current desired roll in degrees.
                - nav_pitch (float): Current desired pitch in degrees.
                - nav_bearing (int|float): Current desired heading in degrees.
                - target_bearing (int|float): Bearing to current waypoint/target in degrees.
                - wp_dist (int|float): Distance to active waypoint in meters.
                - alt_error (float): Current altitude error in meters.
                - aspd_error (float): Current airspeed error in m/s.
                - xtrack_error (float): Current crosstrack error on x-y plane in meters.

        Note:
            All returned values are NaN if no ``NAV_CONTROLLER_OUTPUT`` message has been received.
            Use ``set_msg_rate("NAV_CONTROLLER_OUTPUT", rate)`` if your flight controller does not stream it by default.
        """
        m = self.get_msg(
            msg_type="NAV_CONTROLLER_OUTPUT",
            source_system=source_system
        )
        if m:
            return mavtypes.NavControllerOutput(
                nav_roll=float(m.nav_roll),
                nav_pitch=float(m.nav_pitch),
                nav_bearing=int(m.nav_bearing),
                target_bearing=int(m.target_bearing),
                wp_dist=int(m.wp_dist),
                alt_error=float(m.alt_error),
                aspd_error=float(m.aspd_error),
                xtrack_error=float(m.xtrack_error),
            )
        return mavtypes.NavControllerOutput(
            nav_roll=math.nan,
            nav_pitch=math.nan,
            nav_bearing=math.nan,
            target_bearing=math.nan,
            wp_dist=math.nan,
            alt_error=math.nan,
            aspd_error=math.nan,
            xtrack_error=math.nan,
        )

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
                if (
                    hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                    and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
                ):
                    mode_map = self._resolve_ardupilot_mode_map(hb.type, by_name=False)
                    if mode_map and hb.custom_mode in mode_map:
                        return mode_map[hb.custom_mode]
                return mavutil.mode_string_v10(hb)
            # Fallback: we check HIGH_LATENCY2
            high_latency2 = self.get_msg(
                msg_type="HIGH_LATENCY2",
                source_system=source_system
            )
            if high_latency2:
                if high_latency2.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                    mode_map = self._resolve_ardupilot_mode_map(high_latency2.type, by_name=False)
                    if mode_map and high_latency2.custom_mode in mode_map:
                        return mode_map[high_latency2.custom_mode]
                return mavutil.mode_string_v10(high_latency2)
            return "UNKNOWN"
        elif float(mavutil.mavlink.WIRE_PROTOCOL_VERSION) == 0.9:
            # MAVLink 0.9
            sys_status = self.get_msg(
                msg_type="SYS_STATUS",
                source_system=source_system
            )
            if sys_status:
                return mavutil.mode_string_v09(sys_status)
            return "UNKNOWN"
        return "UNKNOWN"

    def custom_mode(self, source_system: int|None = None) -> int|float:
        """Flight mode as an integer (ArduPilot). On PX4, allows retrieving the Custom Main Mode and Custom Sub Mode.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: Flight controller mode.

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

    def base_mode(self, source_system: int|None = None) -> int|float:
        """Base Mode. Partially characterizes the mode on PX4.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: Base Mode.

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

    def custom_main_mode(self, source_system: int|None = None) -> int|float:
        """Custom Main Mode (PX4). Partially characterizes the mode on PX4.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: Flight controller Custom Main Mode.

        Note:
            Returns NaN if no data has been received.
        """
        custom_mode = self.custom_mode(source_system=source_system)
        if math.isnan(custom_mode):
            return math.nan
        return (custom_mode & 0xFF0000) >> 16

    def custom_sub_mode(self, source_system: int|None = None) -> int|float:
        """Custom Sub Mode (PX4). Partially characterizes the mode on PX4.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: Flight controller Custom Sub Mode.

        Note:
            Returns NaN if no data has been received.
        """
        custom_mode = self.custom_mode(source_system=source_system)
        if math.isnan(custom_mode):
            return math.nan
        return (custom_mode & 0xFF000000) >> 24

    def autopilot_int(self, source_system: int|None = None) -> int|float:
        """Autopilot.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: MAV_AUTOPILOT | Autopilot.

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
            Returns 'UNKNOWN' if no valid data has been received.
        """
        m = self.get_msg(
            msg_type="HEARTBEAT",
            source_system=source_system
        )
        if m:
            if m.autopilot not in mavutil.mavlink.enums["MAV_AUTOPILOT"]:
                return f"UNKNOWN: {m.autopilot}"
            return mavutil.mavlink.enums["MAV_AUTOPILOT"][m.autopilot].name
        return "UNKNOWN"

    def vehicle_type_int(self, source_system: int|None = None) -> int|float:
        """Vehicle type.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: MAV_TYPE | Vehicle type.

        Note:
            Returns NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="HEARTBEAT",
            source_system=source_system
        )
        if m:
            return m.type
        return math.nan

    def vehicle_type_str(self, source_system: int|None = None) -> str:
        """Vehicle type.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            str: Vehicle type (e.g.: 'MAV_TYPE_FIXED_WING', 'MAV_TYPE_QUADROTOR').

        Note:
            Returns 'UNKNOWN' if no valid data has been received.
        """
        m = self.get_msg(
            msg_type="HEARTBEAT",
            source_system=source_system
        )
        if m:
            if m.type not in mavutil.mavlink.enums["MAV_TYPE"]:
                return f"UNKNOWN: {m.type}"
            return mavutil.mavlink.enums["MAV_TYPE"][m.type].name
        return "UNKNOWN"

    def vtol_state(self, source_system: int|None = None) -> int|float:
        """Current VTOL state.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: ``MAV_VTOL_STATE``. NaN if no data has been received.

        Note:
            - Returns ``MAV_VTOL_STATE_UNDEFINED`` on non-VTOL Fixed-Wing ArduPlane aircraft.
            - Returns ``MAV_VTOL_STATE_MC`` on ArduCopter multirotors.
        """
        m = self.get_msg(
            msg_type="EXTENDED_SYS_STATE",
            source_system=source_system
        )
        if m:
            return m.vtol_state
        return math.nan

    def vtol_state_str(self, source_system: int|None = None) -> str:
        """Current VTOL state.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            str: ``MAV_VTOL_STATE``. ``UNKNOWN`` if no valid data has been received.
        """
        m = self.get_msg(
            msg_type="EXTENDED_SYS_STATE",
            source_system=source_system
        )
        if m:
            if m.vtol_state not in mavutil.mavlink.enums["MAV_VTOL_STATE"]:
                return f"UNKNOWN: {m.vtol_state}"
            return mavutil.mavlink.enums["MAV_VTOL_STATE"][m.vtol_state].name
        return "UNKNOWN"

    def landed_state(self, source_system: int|None = None) -> int|float:
        """Current landed state.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: ``MAV_LANDED_STATE``. NaN if no data has been received.
        """
        m = self.get_msg(
            msg_type="EXTENDED_SYS_STATE",
            source_system=source_system
        )
        if m:
            return m.landed_state
        return math.nan

    def landed_state_str(self, source_system: int|None = None) -> str:
        """Current landed state.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            str: ``MAV_LANDED_STATE``. ``UNKNOWN`` if no valid data has been received.
        """
        m = self.get_msg(
            msg_type="EXTENDED_SYS_STATE",
            source_system=source_system
        )
        if m:
            if m.landed_state not in mavutil.mavlink.enums["MAV_LANDED_STATE"]:
                return f"UNKNOWN: {m.landed_state}"
            return mavutil.mavlink.enums["MAV_LANDED_STATE"][m.landed_state].name
        return "UNKNOWN"

    def battery_percentage(self, source_system: int|None = None) -> int|float:
        """Remaining battery (requires proper configuration of the flight controller).

        Args:
            source_system (int|None): MAVLink source system ID. Defaults to the master's ID.

        Returns:
            int|float: Remaining battery percentage.

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

    def airspeed_healthy(self, source_system: int|None = None) -> bool|None:
        """Airspeed / differential-pressure (Pitot) sensor health.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            bool|None:
                - True if an airspeed sensor is present and healthy.
                - False if an airspeed sensor is present but unhealthy / failed.
                - None if no ``SYS_STATUS`` message has been received or no airspeed sensor is present on the vehicle.
        """
        m = self.get_msg(
            msg_type="SYS_STATUS",
            source_system=source_system
        )
        if m is None:
            return None
        bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
        if not (m.onboard_control_sensors_present & bit):
            return None
        return bool(m.onboard_control_sensors_health & bit)

    def ekf_status(self, source_system: int|None = None) -> mavtypes.EkfStatus:
        """EKF health flags and innovation variances.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            mavtypes.EkfStatus:
                - flags (int|float): ``EKF_STATUS_FLAGS`` bitmask.
                - velocity_variance (float): Velocity variance ratio.
                - pos_horiz_variance (float): Horizontal position variance ratio.
                - pos_vert_variance (float): Vertical position variance ratio.
                - compass_variance (float): Compass variance ratio.
                - terrain_alt_variance (float): Terrain altitude variance ratio.

        Note:
            All returned values are NaN if no ``EKF_STATUS_REPORT`` message has been received.
            Use ``set_msg_rate("EKF_STATUS_REPORT", rate)`` if your flight controller does not stream it by default.
        """
        m = self.get_msg(
            msg_type="EKF_STATUS_REPORT",
            source_system=source_system
        )
        if m:
            return mavtypes.EkfStatus(
                flags=int(m.flags),
                velocity_variance=float(m.velocity_variance),
                pos_horiz_variance=float(m.pos_horiz_variance),
                pos_vert_variance=float(m.pos_vert_variance),
                compass_variance=float(m.compass_variance),
                terrain_alt_variance=float(m.terrain_alt_variance),
            )
        return mavtypes.EkfStatus(
            flags=math.nan,
            velocity_variance=math.nan,
            pos_horiz_variance=math.nan,
            pos_vert_variance=math.nan,
            compass_variance=math.nan,
            terrain_alt_variance=math.nan,
        )

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
        roll, pitch, yaw = _Core.quaternion_to_euler(q=m.q)
        return mavtypes.GimbalAngles(roll=math.degrees(roll), pitch=math.degrees(pitch), yaw=math.degrees(yaw))

    def current_waypoint_seq(self, source_system: int|None = None) -> int|float:
        """Current waypoint seq.

        Args:
            source_system (int|None): MAVLink system ID of the device. Defaults to the master's ID.

        Returns:
            int|float: Waypoint sequence number.

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
        if hasattr(m, "mission_state") and m.mission_state != mavutil.mavlink.MISSION_STATE_UNKNOWN:
            return bool(m.mission_state == mavutil.mavlink.MISSION_STATE_COMPLETE)
        return False

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
        return _Core.distance_between_two_gps_points(
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
        return _Core.distance_between_two_local_points(
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
        delta_north = _Core.distance_between_two_gps_points(
            lat_i=lat_i,
            lat_f=lat_f,
            lon_i=lon_i,
            lon_f=lon_i
        )
        if lat_f < lat_i:
            delta_north *= -1

        # East displacement (longitude difference, same latitude)
        delta_east = _Core.distance_between_two_gps_points(
            lat_i=lat_i,
            lat_f=lat_i,
            lon_i=lon_i,
            lon_f=lon_f
        )
        if lon_f < lon_i:
            delta_east *= -1

        return delta_north, delta_east
