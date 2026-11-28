# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import time
import math
import unicodedata
from pymavlink import mavutil

from pyMAVLinCS import mavtypes
from pyMAVLinCS.mission_control_code import MCC, EMPTY
from pyMAVLinCS._core import _Core


class _System(_Core):
    """System configuration, parameters, MCC/STATUSTEXT/TIMESYNC messaging, and wait helpers."""

    def reboot(self,
            force=False,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Reboot the autopilot.

        Args:
            force (bool): Whether to perform a forced reboot (not recommended; for reference only).
            target_system (int|None): Target MAVLink system ID.
            **timeout_ack (float|None): ACK waiting timeout. Default: 2.

        Returns:
            bool: Result of the operation.
        """
        self.logger.debug("Rebooting..")
        timeout_ack = kwargs.get('timeout_ack', 2)
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
            source_system=target_system,
            timeout=timeout_ack
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

    def get_parameter(self,
            parameter: str,
            use_cache: bool = False,
            target_system: int|None = None,
            **kwargs
        ) -> float:
        """Requests and returns the value of a flight controller parameter.

        Args:
            parameter (str): Parameter name (1 to 16 ASCII characters, e.g., 'PILOT_THR_FILT' or 'WPNAV_SPEED').
            use_cache (bool): If True and the parameter has already been received, returns the cached value immediately without sending a network request. Default: False.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout (float|None): Overall timeout in seconds while waiting for PARAM_VALUE. 0 means no wait (returns cached value if use_cache=True, else NaN). None means no overall timeout. Default: 2.0.
            **retry_timeout (float): Timeout in seconds before re-sending PARAM_REQUEST_READ if no response is received. Default: 0.5.
            **max_retries (int): Maximum consecutive retries on timeout. Default: 3.

        Returns:
            float: Parameter value, or NaN if unavailable or timed out.
        """
        self.logger.debug("Getting parameter %s..", parameter)
        timeout = kwargs.get('timeout', 2.0)
        retry_timeout = float(kwargs.get('retry_timeout', 0.5))
        max_retries = int(kwargs.get('max_retries', 3))

        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to get parameter")
            return math.nan
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return math.nan
        if not isinstance(parameter, str) or not parameter or len(parameter) > 16:
            self.logger.error("Invalid parameter name '%s': must be 1 to 16 characters", parameter)
            return math.nan

        if target_system is None:
            target_system = self.master.target_system

        param_key = (target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
        pstate = self.master.param_state.get(param_key)

        if use_cache and pstate is not None and parameter in pstate.params:
            return float(pstate.params[parameter])

        if timeout == 0:
            self.logger.warning("Zero timeout: not waiting for parameter %s", parameter)
            return math.nan

        if pstate is not None:
            pstate.params.pop(parameter, None)

        start_time = time.time()
        last_send_time = start_time
        self.master.mav.param_request_read_send(
            target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            parameter.encode("ascii"),
            -1
        )

        retries = 0
        while True:
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for parameter %s", parameter)
                return math.nan

            pstate = self.master.param_state.get(param_key)
            if pstate is not None and parameter in pstate.params:
                val = float(pstate.params[parameter])
                self.logger.info("Got parameter %s = %s", parameter, val)
                return val

            now = time.time()
            if timeout is not None and now - start_time >= timeout:
                self.logger.error("Timeout: unable to read parameter %s", parameter)
                return math.nan

            if now - last_send_time >= retry_timeout:
                retries += 1
                if retries > max_retries:
                    self.logger.error("Failed to read parameter %s: max retries (%s) exceeded", parameter, max_retries)
                    return math.nan

                self.logger.warning(
                    "No PARAM_VALUE received for %s: retrying PARAM_REQUEST_READ (attempt %s/%s)..",
                    parameter, retries, max_retries
                )
                last_send_time = time.time()
                self.master.mav.param_request_read_send(
                    target_system,
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                    parameter.encode("ascii"),
                    -1
                )

            time.sleep(0.02)

    def request_all_parameters(self, target_system: int|None = None) -> bool:
        """Requests the flight controller to stream all its parameters so they are cached.

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.

        Returns:
            bool: True if the request was sent, False otherwise.
        """
        self.logger.debug("Requesting all parameters..")
        if self.mission_ended:
            self.logger.critical("Mission stopped: Impossible to request all parameters")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.param_request_list_send(
            target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        )
        self.logger.info("All parameters requested")
        return True

    def write_parameter(self,
            parameter: str,
            value: float|int,
            param_type: int = mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Modifies a flight controller parameter and waits for PARAM_VALUE confirmation.

        Args:
            parameter (str): Parameter name (1 to 16 ASCII characters).
            value (float|int): Desired parameter value.
            param_type (int): MAV_PARAM_TYPE | Parameter type (default: MAV_PARAM_TYPE_REAL32).
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): Timeout in seconds while waiting for the confirming PARAM_VALUE. 0 means no wait after sending (returns True immediately). None means no overall timeout. Default: 2.0.
            **retry_timeout (float): Timeout in seconds before re-sending PARAM_SET if no PARAM_VALUE is received. Default: 0.5.
            **max_retries (int): Maximum consecutive retries on timeout. Default: 3.
            **tolerance (float): Relative/absolute tolerance when verifying the confirmed parameter value. Default: 1e-4.

        Returns:
            bool: True if the parameter was set and confirmed (or sent when timeout_ack=0), False otherwise.
        """
        # Note: some parameters require a reboot to take effect (check Mission Planner when changing them manually)
        self.logger.debug("Writing parameter %s = %s..", parameter, value)
        timeout_ack = kwargs.get('timeout_ack', 2.0)
        retry_timeout = float(kwargs.get('retry_timeout', 0.5))
        max_retries = int(kwargs.get('max_retries', 3))
        tolerance = float(kwargs.get('tolerance', 1e-4))

        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to write parameter")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to write parameter")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False
        if not isinstance(parameter, str) or not parameter or len(parameter) > 16:
            self.logger.error("Invalid parameter name '%s': must be 1 to 16 characters", parameter)
            return False
        if math.isnan(value) or math.isinf(value):
            self.logger.error("Invalid parameter value for %s: %s", parameter, value)
            return False

        if target_system is None:
            target_system = self.master.target_system

        param_key = (target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
        pstate = self.master.param_state.get(param_key)
        if pstate is not None:
            pstate.params.pop(parameter, None)

        start_time = time.time()
        last_send_time = start_time
        self.master.mav.param_set_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            parameter.encode("ascii"), # param_id
            float(value), # param_value
            param_type # param_type
        )

        if timeout_ack == 0:
            self.logger.info("Parameter request sent (no confirmation wait)")
            return True

        retries = 0
        while True:
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for parameter %s confirmation", parameter)
                return False

            pstate = self.master.param_state.get(param_key)
            if pstate is not None and parameter in pstate.params:
                confirmed_val = float(pstate.params[parameter])
                if math.isclose(confirmed_val, float(value), rel_tol=tolerance, abs_tol=tolerance):
                    self.logger.info("Parameter %s set to %s", parameter, confirmed_val)
                    return True
                self.logger.error(
                    "Parameter %s write refused (expected %s, got %s)",
                    parameter, value, confirmed_val
                )
                return False

            now = time.time()
            if timeout_ack is not None and now - start_time >= timeout_ack:
                self.logger.critical("Setting parameter %s timed out", parameter)
                return False

            if now - last_send_time >= retry_timeout:
                retries += 1
                if retries > max_retries:
                    self.logger.critical(
                        "Setting parameter %s failed: max retries (%s) exceeded",
                        parameter, max_retries
                    )
                    return False

                self.logger.warning(
                    "No PARAM_VALUE confirmation for %s: retrying PARAM_SET (attempt %s/%s)..",
                    parameter, retries, max_retries
                )
                last_send_time = time.time()
                self.master.mav.param_set_send(
                    target_system,
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                    parameter.encode("ascii"),
                    float(value),
                    param_type
                )

            time.sleep(0.02)

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
            if i == len(chunks) - 1:
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
        self.mcc.discard(mcc)

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
            mode: str|int|tuple[int, int, int],
            source_system: int|None = None,
            timeout: float|None = 4
        ) -> bool|None:
        """Wait for a flight mode change.

        Args:
            mode (str|int|tuple[int, int, int]): Flight mode. str: mode name. int: mode ID on ArduPilot. tuple[int, int, int]: (MAV_MODE_FLAG, Custom Main Mode, Custom Sub Mode) on PX4.
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
                current_base = self.base_mode(source_system=source_system)
                if not math.isnan(current_base) and \
                (current_base & ~mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == (base_mode & ~mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and \
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
            return mavtypes.HomePosition(
                latitude_home=math.nan,
                longitude_home=math.nan,
                altitude_home=math.nan
            )

        self.logger.info("Waiting for HOME..")
        while True:
            if self.mission_ended:
                self.logger.critical("Mission stopped: stop waiting for HOME")
                return mavtypes.HomePosition(
                    latitude_home=math.nan,
                    longitude_home=math.nan,
                    altitude_home=math.nan
                )

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
            f"({mission_type is None} or getattr(MISSION_ACK, 'mission_type', {mission_type}) == {mission_type}) and "
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
