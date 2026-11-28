# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import os
import json
import time
import math
from pathlib import Path
from pymavlink import mavutil

from pyMAVLinCS import mavtypes
from pyMAVLinCS._flight_control import _FlightControl


class _Missions(_FlightControl):
    """MAVLink Mission, Geofence, and Rally Point protocol handling and mission file I/O."""

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
            target_system: int|None = None,
            mission_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        ) -> bool:
        """Send a waypoint (or fence/rally item).

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
            mission_type (int): MAV_MISSION_TYPE. Default: MAV_MISSION_TYPE_MISSION.

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
            int(round(x)), # x
            int(round(y)), # y
            z, # z
            mission_type=mission_type
        )
        self.logger.info("Waypoint sent")
        return True

    def send_waypoint_count(self,
            number_waypoints: int,
            target_system: int|None = None,
            mission_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        ) -> bool:
        """Inform the flight controller of the number of mission/fence/rally items that will be sent.
        Must be sent before each mission.

        Args:
            number_waypoints (int): Number of waypoints.
            target_system (int): Target MAVLink system ID.
            mission_type (int): MAV_MISSION_TYPE. Default: MAV_MISSION_TYPE_MISSION.

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
            number_waypoints, # count
            mission_type # mission_type
        )
        self.logger.info("Waypoint count sent")
        return True

    def clear_waypoints(self,
            target_system: int|None = None,
            mission_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            **kwargs
        ) -> bool:
        """Delete all stored items of a given mission type (mission, geofence, or rally points).

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            mission_type (int): MAV_MISSION_TYPE. Default: MAV_MISSION_TYPE_MISSION.
            **timeout (float|None): Timeout while waiting for the ack. Default: 2.

        Returns:
            bool: Result of the request.
        """
        self.logger.debug("Deleting waypoints..")
        timeout = kwargs.get('timeout', 2)
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
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mission_type # mission_type
        )

        accepted = self.wait_mission_ack(
            mission_type=mission_type,
            source_system=target_system,
            timeout=timeout
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

    def _uses_seq0_home(self, system_id: int|None = None) -> bool:
        """Returns True if the target autopilot reserves mission sequence 0 for HOME (ArduPilot)."""
        ap = self.autopilot_int(source_system=system_id)
        if not math.isnan(ap):
            return int(ap) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        return self.data_init.get("dialect", "ardupilotmega") == "ardupilotmega"

    def upload_mission(self,
            waypoints: list[mavtypes.MissionItem|dict|tuple]|mavtypes.MissionPlan,
            clear_existing: bool = True,
            target_system: int|None = None,
            mission_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            **kwargs
        ) -> bool:
        """Uploads a complete mission, geofence, or rally-point list using the MAVLink Mission Protocol handshake.

        Args:
            waypoints (list[mavtypes.MissionItem|dict|tuple]|mavtypes.MissionPlan): Ordered list of mission items (or a ``MissionPlan``).
                Each element can be:

                - a ``mavtypes.MissionItem`` instance
                - a ``dict`` with keys: ``command``, ``x`` (or ``lat``/``latitude``), ``y`` (or ``lon``/``longitude``),
                  ``z`` (or ``alt``/``altitude``/``relative_alt``), and optional ``frame``, ``param1``, ``param2``, ``param3``, ``param4``
                - a 4-tuple ``(command, x, y, z)``, 5-tuple ``(command, x, y, z, frame)``, or 9-tuple
                  ``(command, x, y, z, frame, param1, param2, param3, param4)``.
            clear_existing (bool): Whether to clear existing items of ``mission_type`` on the flight controller before uploading. Default: True.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            mission_type (int): MAV_MISSION_TYPE. Default: MAV_MISSION_TYPE_MISSION.
            **timeout (float|None): Overall timeout in seconds for the upload handshake. 0 means no wait (returns False). None means no overall timeout. Default: 15.0.
            **retry_timeout (float): Timeout in seconds before re-sending the last packet if no request/ack is received. Default: 1.5.
            **max_retries (int): Maximum consecutive retries per step on timeout. Default: 5.

        Returns:
            bool: True if the mission was uploaded and accepted by the flight controller, False otherwise.

        Note:
            - Do not manually insert a dummy HOME waypoint at index 0: ``upload_mission()`` handles ArduPilot's sequence 0 HOME slot automatically.
            - For ``MAV_CMD_DO_JUMP`` items in ``waypoints``, ``param1`` uses 1-based item numbering (``1`` = 1st mission
              item, matching Mission Planner, QGroundControl, and ArduPilot ``current_waypoint_seq()``), and is
              automatically converted to 0-based sequence indexing when uploading to PX4.
            - Tested on ArduPilot SITL (Copter, Plane, QuadPlane); PX4 behavior has only been verified with offline
              unit tests and still needs real PX4 SITL / hardware testing.
        """
        self.logger.info("Uploading mission..")
        timeout = kwargs.get('timeout', 15.0)
        retry_timeout = float(kwargs.get('retry_timeout', 1.5))
        max_retries = int(kwargs.get('max_retries', 5))

        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to upload mission")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to upload mission")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False
        if timeout == 0:
            self.logger.warning("Zero timeout: not uploading mission")
            return False

        plan_home: mavtypes.HomePosition|None = kwargs.get('_home_position', None)
        if isinstance(waypoints, mavtypes.MissionPlan):
            if plan_home is None:
                plan_home = waypoints.home_position
            raw_waypoints = waypoints.waypoints
        else:
            raw_waypoints = waypoints

        if not raw_waypoints:
            self.logger.error("Empty waypoint list: Unable to upload mission")
            return False

        # Normalize all waypoint definitions into mavtypes.MissionItem objects
        normalized_wps: list[mavtypes.MissionItem] = []
        default_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        for idx, item in enumerate(raw_waypoints):
            if isinstance(item, mavtypes.MissionItem):
                normalized_wps.append(item)
            elif isinstance(item, dict):
                if "command" not in item:
                    self.logger.error("Waypoint #%s dict is missing required 'command' key", idx)
                    return False
                x_val = item.get("x", item.get("lat", item.get("latitude", 0.0)))
                y_val = item.get("y", item.get("lon", item.get("longitude", 0.0)))
                z_val = item.get("z", item.get("alt", item.get("altitude", item.get("relative_alt", 0.0))))
                normalized_wps.append(mavtypes.MissionItem(
                    command=int(item["command"]),
                    x=float(x_val),
                    y=float(y_val),
                    z=float(z_val),
                    frame=int(item.get("frame", default_frame)),
                    param1=float(item.get("param1", 0.0)),
                    param2=float(item.get("param2", 0.0)),
                    param3=float(item.get("param3", 0.0)),
                    param4=float(item.get("param4", 0.0)),
                ))
            elif isinstance(item, (tuple, list)):
                if len(item) == 4:
                    cmd, x_val, y_val, z_val = item
                    normalized_wps.append(mavtypes.MissionItem(
                        command=int(cmd), x=float(x_val), y=float(y_val), z=float(z_val), frame=default_frame
                    ))
                elif len(item) == 5:
                    cmd, x_val, y_val, z_val, frm = item
                    normalized_wps.append(mavtypes.MissionItem(
                        command=int(cmd), x=float(x_val), y=float(y_val), z=float(z_val), frame=int(frm)
                    ))
                elif len(item) == 9:
                    cmd, x_val, y_val, z_val, frm, p1, p2, p3, p4 = item
                    normalized_wps.append(mavtypes.MissionItem(
                        command=int(cmd), x=float(x_val), y=float(y_val), z=float(z_val),
                        frame=int(frm), param1=float(p1), param2=float(p2), param3=float(p3), param4=float(p4)
                    ))
                else:
                    self.logger.error(
                        "Invalid tuple length (%s) for waypoint #%s: expected 4, 5, or 9 elements",
                        len(item), idx
                    )
                    return False
            else:
                self.logger.error("Invalid waypoint type at index %s: %s", idx, type(item))
                return False

            last_wp = normalized_wps[-1]
            if not (math.isfinite(last_wp.x) and math.isfinite(last_wp.y) and math.isfinite(last_wp.z)):
                self.logger.error("Waypoint #%s has non-finite coordinates (x=%s, y=%s, z=%s)", idx, last_wp.x, last_wp.y, last_wp.z)
                return False
            if any(math.isinf(p) for p in (last_wp.param1, last_wp.param2, last_wp.param3, last_wp.param4)):
                self.logger.error("Waypoint #%s has infinite parameter value", idx)
                return False

        if target_system is None:
            target_system = self.master.target_system

        # Build wire-level item list according to autopilot HOME seq-0 convention
        wire_wps: list[mavtypes.MissionItem] = list(normalized_wps)
        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            if self._uses_seq0_home(target_system):
                hp_lat, hp_lon, hp_alt = 0.0, 0.0, 0.0
                if plan_home is not None and math.isfinite(plan_home.latitude_home) and math.isfinite(plan_home.longitude_home):
                    hp_lat = plan_home.latitude_home
                    hp_lon = plan_home.longitude_home
                    hp_alt = plan_home.altitude_home if math.isfinite(plan_home.altitude_home) else 0.0
                else:
                    veh_home = self.home_position(source_system=target_system)
                    if not math.isnan(veh_home.latitude_home) and not math.isnan(veh_home.longitude_home):
                        hp_lat = veh_home.latitude_home
                        hp_lon = veh_home.longitude_home
                        hp_alt = 0.0 if math.isnan(veh_home.altitude_home) else veh_home.altitude_home
                wire_wps.insert(0, mavtypes.MissionItem(
                    command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    x=hp_lat,
                    y=hp_lon,
                    z=hp_alt,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
                ))
            else:
                # PX4 uses 0-based sequence numbers without a seq 0 HOME slot: shift 1-based DO_JUMP targets by -1
                wire_wps = [
                    wp._replace(param1=max(0.0, wp.param1 - 1.0))
                    if wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP and wp.param1 >= 1.0
                    else wp
                    for wp in wire_wps
                ]

        start_time = time.monotonic()

        if clear_existing:
            clear_timeout = min(timeout, 3.0) if timeout is not None else 3.0
            if not self.clear_waypoints(target_system=target_system, mission_type=mission_type, timeout=clear_timeout):
                return False

        if not self.send_waypoint_count(number_waypoints=len(wire_wps), target_system=target_system, mission_type=mission_type):
            return False

        last_requested_seq: int|None = None
        retries = 0

        while True:
            if self.mission_ended:
                self.logger.critical("Mission stopped: aborting mission upload")
                return False

            wait_time = retry_timeout
            if timeout is not None:
                remaining = (start_time + timeout) - time.monotonic()
                if remaining <= 0:
                    self.logger.critical("Mission upload timed out")
                    return False
                wait_time = min(retry_timeout, remaining)

            m = self.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True,
                timeout=wait_time,
                source_system=target_system
            )

            if self.mission_ended:
                self.logger.critical("Mission stopped: aborting mission upload")
                return False

            if m is None:
                # Check if overall timeout expired
                if timeout is not None and (start_time + timeout) - time.monotonic() <= 0:
                    self.logger.critical("Mission upload timed out")
                    return False

                retries += 1
                if retries > max_retries:
                    self.logger.critical(
                        "Mission upload failed: max retries (%s) exceeded", max_retries
                    )
                    return False

                if last_requested_seq is None:
                    self.logger.warning(
                        "No MISSION_REQUEST received: retrying MISSION_COUNT (attempt %s/%s)..",
                        retries, max_retries
                    )
                    if not self.send_waypoint_count(number_waypoints=len(wire_wps), target_system=target_system, mission_type=mission_type):
                        return False
                else:
                    wp = wire_wps[last_requested_seq]
                    self.logger.warning(
                        "No response after waypoint #%s: retrying send_waypoint (attempt %s/%s)..",
                        last_requested_seq, retries, max_retries
                    )
                    if not self.send_waypoint(
                        waypoint_index=last_requested_seq,
                        frame=wp.frame,
                        command=wp.command,
                        param1=wp.param1,
                        param2=wp.param2,
                        param3=wp.param3,
                        param4=wp.param4,
                        x=wp.x,
                        y=wp.y,
                        z=wp.z,
                        target_system=target_system,
                        mission_type=mission_type
                    ):
                        return False
                continue

            # Ensure the message is addressed to our system/component and matches mission_type
            if getattr(m, "target_system", self.master.source_system) != self.master.source_system:
                continue
            if getattr(m, "target_component", self.master.source_component) != self.master.source_component:
                continue
            if getattr(m, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION) != mission_type:
                continue

            msg_type = m.get_type()
            if msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                seq = m.seq
                if seq < 0 or seq >= len(wire_wps):
                    self.logger.error(
                        "Autopilot requested out-of-range waypoint seq %s (count=%s)",
                        seq, len(wire_wps)
                    )
                    return False
                wp = wire_wps[seq]
                last_requested_seq = seq
                retries = 0
                if not self.send_waypoint(
                    waypoint_index=seq,
                    frame=wp.frame,
                    command=wp.command,
                    param1=wp.param1,
                    param2=wp.param2,
                    param3=wp.param3,
                    param4=wp.param4,
                    x=wp.x,
                    y=wp.y,
                    z=wp.z,
                    target_system=target_system,
                    mission_type=mission_type
                ):
                    return False

            elif msg_type == "MISSION_ACK":
                if m.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    self.logger.info("Mission uploaded successfully (%s items, type=%s)", len(normalized_wps), mission_type)
                    return True
                self.logger.error("Mission upload failed (MISSION_ACK result=%s)", m.type)
                return False

    def _download_mission_internal(self,
            target_system: int|None = None,
            mission_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            **kwargs
        ) -> tuple[list[mavtypes.MissionItem]|None, mavtypes.HomePosition|None]:
        """Internal download helper returning ``(items, wire_seq0_home)``."""
        self.logger.info("Downloading mission..")
        timeout = kwargs.get('timeout', 15.0)
        retry_timeout = kwargs.get('retry_timeout', 1.0)
        max_retries = kwargs.get('max_retries', 3)

        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to download mission")
            return None, None
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return None, None
        if timeout == 0:
            self.logger.warning("Zero timeout: not downloading mission")
            return None, None

        if target_system is None:
            target_system = self.master.target_system

        global_frames = (
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
        )
        local_frames = (
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mavutil.mavlink.MAV_FRAME_LOCAL_ENU,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_LOCAL_FLU,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
        )

        start_time = time.monotonic()

        # Step 1: Request mission count (MISSION_REQUEST_LIST -> MISSION_COUNT)
        self.master.mav.mission_request_list_send(
            target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, mission_type
        )
        count: int|None = None
        retries = 0
        while count is None:
            if self.mission_ended:
                self.logger.critical("Mission stopped: aborting mission download")
                return None, None

            wait_time = retry_timeout
            if timeout is not None:
                remaining = (start_time + timeout) - time.monotonic()
                if remaining <= 0:
                    self.logger.critical("Mission download timed out waiting for MISSION_COUNT")
                    return None, None
                wait_time = min(retry_timeout, remaining)

            m = self.recv_match(
                type=["MISSION_COUNT", "MISSION_ACK"],
                blocking=True,
                timeout=wait_time,
                source_system=target_system,
            )

            if self.mission_ended:
                self.logger.critical("Mission stopped: aborting mission download")
                return None, None

            if m is None:
                if timeout is not None and (start_time + timeout) - time.monotonic() <= 0:
                    self.logger.critical("Mission download timed out waiting for MISSION_COUNT")
                    return None, None
                retries += 1
                if retries > max_retries:
                    self.logger.critical("Mission download failed: max retries (%s) exceeded for MISSION_COUNT", max_retries)
                    return None, None
                self.logger.warning(
                    "No MISSION_COUNT received: retrying MISSION_REQUEST_LIST (attempt %s/%s)..",
                    retries, max_retries,
                )
                self.master.mav.mission_request_list_send(
                    target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, mission_type
                )
                continue

            if getattr(m, "target_system", self.master.source_system) != self.master.source_system:
                continue
            if getattr(m, "target_component", self.master.source_component) != self.master.source_component:
                continue
            if getattr(m, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION) != mission_type:
                continue

            if m.get_type() == "MISSION_ACK":
                if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    self.logger.error("Mission download rejected (MISSION_ACK result=%s)", m.type)
                    return None, None
                continue

            count = int(m.count)

        # Step 2: Request each waypoint sequentially (MISSION_REQUEST_INT -> MISSION_ITEM_INT / MISSION_ITEM)
        items: list[mavtypes.MissionItem] = []
        for seq in range(count):
            self.master.mav.mission_request_int_send(
                target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, seq, mission_type
            )
            retries = 0
            received_item: mavtypes.MissionItem|None = None

            while received_item is None:
                if self.mission_ended:
                    self.logger.critical("Mission stopped: aborting mission download")
                    return None, None

                wait_time = retry_timeout
                if timeout is not None:
                    remaining = (start_time + timeout) - time.monotonic()
                    if remaining <= 0:
                        self.logger.critical("Mission download timed out at waypoint #%s", seq)
                        return None, None
                    wait_time = min(retry_timeout, remaining)

                m = self.recv_match(
                    type=["MISSION_ITEM_INT", "MISSION_ITEM", "MISSION_ACK"],
                    blocking=True,
                    timeout=wait_time,
                    source_system=target_system,
                )

                if self.mission_ended:
                    self.logger.critical("Mission stopped: aborting mission download")
                    return None, None

                if m is None:
                    if timeout is not None and (start_time + timeout) - time.monotonic() <= 0:
                        self.logger.critical("Mission download timed out at waypoint #%s", seq)
                        return None, None
                    retries += 1
                    if retries > max_retries:
                        self.logger.critical(
                            "Mission download failed: max retries (%s) exceeded at waypoint #%s",
                            max_retries, seq,
                        )
                        return None, None
                    self.logger.warning(
                        "No response for waypoint #%s: retrying MISSION_REQUEST_INT (attempt %s/%s)..",
                        seq, retries, max_retries,
                    )
                    self.master.mav.mission_request_int_send(
                        target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, seq, mission_type
                    )
                    continue

                if getattr(m, "target_system", self.master.source_system) != self.master.source_system:
                    continue
                if getattr(m, "target_component", self.master.source_component) != self.master.source_component:
                    continue
                if getattr(m, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION) != mission_type:
                    continue

                msg_type = m.get_type()
                if msg_type == "MISSION_ACK":
                    if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        self.logger.error("Mission download failed at waypoint #%s (MISSION_ACK result=%s)", seq, m.type)
                        return None, None
                    continue

                if int(m.seq) != seq:
                    continue

                x_val = float(m.x)
                y_val = float(m.y)
                if msg_type == "MISSION_ITEM_INT":
                    if m.frame in global_frames:
                        x_val /= 1e7
                        y_val /= 1e7
                    elif m.frame in local_frames:
                        x_val /= 1e4
                        y_val /= 1e4

                received_item = mavtypes.MissionItem(
                    command=int(m.command),
                    x=x_val,
                    y=y_val,
                    z=float(m.z),
                    frame=int(m.frame),
                    param1=float(m.param1),
                    param2=float(m.param2),
                    param3=float(m.param3),
                    param4=float(m.param4),
                )

            items.append(received_item)

        # Step 3: Acknowledge mission download completion
        self.master.mav.mission_ack_send(
            target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_MISSION_ACCEPTED,
            mission_type
        )

        wire_home: mavtypes.HomePosition|None = None
        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            if self._uses_seq0_home(target_system):
                if items:
                    wp0 = items[0]
                    if wp0.x != 0.0 or wp0.y != 0.0:
                        wire_home = mavtypes.HomePosition(
                            latitude_home=wp0.x,
                            longitude_home=wp0.y,
                            altitude_home=wp0.z,
                        )
                    items = items[1:]
            else:
                # Convert PX4 0-based DO_JUMP targets back to 1-based item numbering
                items = [
                    wp._replace(param1=wp.param1 + 1.0)
                    if wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP and wp.param1 >= 0.0
                    else wp
                    for wp in items
                ]

        self.logger.info("Mission downloaded successfully (%s items, type=%s)", len(items), mission_type)
        return items, wire_home

    def download_mission(self,
            target_system: int|None = None,
            mission_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            **kwargs
        ) -> list[mavtypes.MissionItem]|None:
        """Downloads the active mission, geofence, or rally-point list from the flight controller using the MAVLink Mission Protocol handshake.

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            mission_type (int): MAV_MISSION_TYPE. Default: MAV_MISSION_TYPE_MISSION.
            **timeout (float|None): Overall timeout in seconds for the entire download. 0 means no wait (returns None). None means no timeout. Default: 15.
            **retry_timeout (float): Seconds to wait before retrying ``MISSION_REQUEST_LIST`` or ``MISSION_REQUEST_INT``. Default: 1.0.
            **max_retries (int): Maximum number of retries per protocol step before aborting. Default: 3.

        Returns:
            list[mavtypes.MissionItem]|None: Ordered list of flight ``MissionItem`` objects (excluding ArduPilot's
            protocol-level sequence 0 HOME slot). ``[]`` if no mission items are stored, or ``None`` if the download
            failed, timed out, or was aborted.

        Note:
            - Tested on ArduPilot SITL (Copter, Plane, QuadPlane); PX4 behavior has only been verified with offline
              unit tests and still needs real PX4 SITL / hardware testing.
        """
        items, _ = self._download_mission_internal(
            target_system=target_system,
            mission_type=mission_type,
            **kwargs,
        )
        return items

    def upload_rally_points(self,
            rally_points: list[mavtypes.MissionItem|dict|tuple],
            clear_existing: bool = True,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Uploads a list of Rally Points using the MAVLink Mission Protocol.

        Rally Points provide safe alternate return/landing locations for ``RTL`` / ``QRTL`` (especially valuable on
        long-range fixed-wing and VTOL aircraft).

        Args:
            rally_points (list[mavtypes.MissionItem|dict|tuple]): Ordered list of rally points. Each element can be:

                - a ``mavtypes.MissionItem`` with ``command=MAV_CMD_NAV_RALLY_POINT``
                - a ``dict`` with keys ``lat``/``latitude``/``x``, ``lon``/``longitude``/``y``, ``alt``/``altitude``/``relative_alt``/``z``,
                  and optional ``frame`` (default: ``MAV_FRAME_GLOBAL_RELATIVE_ALT_INT``)
                - a 3-tuple ``(lat, lon, alt)`` or 4-tuple ``(lat, lon, alt, frame)``.
            clear_existing (bool): Whether to clear existing rally points before uploading. Default: True.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout (float|None): Overall upload timeout in seconds. Default: 15.
            **retry_timeout (float): Retry timeout in seconds. Default: 1.0.
            **max_retries (int): Max retries per step. Default: 3.

        Returns:
            bool: True if the rally points were accepted by the flight controller, False otherwise.

        Note:
            - All rally points must use a global coordinate frame.
            - To remove all rally points, call ``clear_waypoints(mission_type=mavutil.mavlink.MAV_MISSION_TYPE_RALLY)``.
        """
        if not rally_points:
            self.logger.error("Empty rally_points list: Unable to upload rally points")
            return False

        default_frame = kwargs.get('default_frame', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
        valid_frames = (
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
        )

        normalized: list[mavtypes.MissionItem] = []
        for idx, item in enumerate(rally_points):
            if isinstance(item, mavtypes.MissionItem):
                wp = item
            elif isinstance(item, dict):
                cmd = int(item.get("command", mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT))
                x_val = item.get("x", item.get("lat", item.get("latitude", 0.0)))
                y_val = item.get("y", item.get("lon", item.get("longitude", 0.0)))
                z_val = item.get("z", item.get("alt", item.get("altitude", item.get("relative_alt", 0.0))))
                wp = mavtypes.MissionItem(
                    command=cmd,
                    x=float(x_val),
                    y=float(y_val),
                    z=float(z_val),
                    frame=int(item.get("frame", default_frame)),
                    param1=float(item.get("param1", 0.0)),
                    param2=float(item.get("param2", 0.0)),
                    param3=float(item.get("param3", 0.0)),
                    param4=float(item.get("param4", 0.0)),
                )
            elif isinstance(item, (tuple, list)):
                if len(item) == 3:
                    lat, lon, alt = item
                    wp = mavtypes.MissionItem(
                        command=mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                        x=float(lat), y=float(lon), z=float(alt), frame=default_frame,
                    )
                elif len(item) == 4:
                    lat, lon, alt, frm = item
                    wp = mavtypes.MissionItem(
                        command=mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                        x=float(lat), y=float(lon), z=float(alt), frame=int(frm),
                    )
                else:
                    self.logger.error("Invalid tuple length (%s) for rally point #%s: expected 3 or 4 elements", len(item), idx)
                    return False
            else:
                self.logger.error("Invalid rally point type at index %s: %s", idx, type(item))
                return False

            if wp.command != mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT:
                self.logger.error("Rally point #%s has invalid command %s (expected MAV_CMD_NAV_RALLY_POINT)", idx, wp.command)
                return False
            if wp.frame not in valid_frames:
                self.logger.error("Rally point #%s has unsupported frame %s (must be a global MAV_FRAME_GLOBAL*)", idx, wp.frame)
                return False
            normalized.append(wp)

        return self.upload_mission(
            waypoints=normalized,
            clear_existing=clear_existing,
            target_system=target_system,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
            **kwargs,
        )

    def download_rally_points(self,
            target_system: int|None = None,
            **kwargs
        ) -> list[mavtypes.MissionItem]|None:
        """Downloads all stored Rally Points from the flight controller.

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout (float|None): Overall download timeout in seconds. Default: 15.
            **retry_timeout (float): Retry timeout in seconds. Default: 1.0.
            **max_retries (int): Max retries per step. Default: 3.

        Returns:
            list[mavtypes.MissionItem]|None: List of ``MAV_CMD_NAV_RALLY_POINT`` items. [] if none stored, or None on error/timeout.
        """
        return self.download_mission(
            target_system=target_system,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
            **kwargs,
        )

    def upload_geofence(self,
            fence_items: list[mavtypes.MissionItem|dict|tuple],
            clear_existing: bool = True,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Uploads a Geofence definition (inclusion/exclusion polygons, circles, and return point).

        Args:
            fence_items (list[mavtypes.MissionItem|dict|tuple]): Ordered list of geofence items.
                Each element must use one of the MAVLink geofence commands:

                - ``MAV_CMD_NAV_FENCE_RETURN_POINT``
                - ``MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION`` (``param1`` = vertex count)
                - ``MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION`` (``param1`` = vertex count)
                - ``MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION`` (``param1`` = radius in meters)
                - ``MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION`` (``param1`` = radius in meters)

                Accepted item formats match ``upload_mission()`` (``MissionItem``, ``dict``, or 4/5/9-element ``tuple``).
            clear_existing (bool): Whether to clear existing geofence items before uploading. Default: True.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout (float|None): Overall upload timeout in seconds. Default: 15.
            **retry_timeout (float): Retry timeout in seconds. Default: 1.0.
            **max_retries (int): Max retries per step. Default: 3.

        Returns:
            bool: True if the geofence was accepted by the flight controller, False otherwise.

        Note:
            - All geofence items must use a global coordinate frame.
            - To remove all geofence items, call ``clear_waypoints(mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE)``.
        """
        if not fence_items:
            self.logger.error("Empty fence_items list: Unable to upload geofence")
            return False

        default_frame = kwargs.get('default_frame', mavutil.mavlink.MAV_FRAME_GLOBAL_INT)
        valid_frames = (
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
        )
        valid_fence_cmds = (
            mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
        )
        polygon_cmds = (
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
        )
        circle_cmds = (
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
        )

        normalized: list[mavtypes.MissionItem] = []
        for idx, item in enumerate(fence_items):
            if isinstance(item, mavtypes.MissionItem):
                wp = item
            elif isinstance(item, dict):
                if "command" not in item:
                    self.logger.error("Fence item #%s dict is missing required 'command' key", idx)
                    return False
                x_val = item.get("x", item.get("lat", item.get("latitude", 0.0)))
                y_val = item.get("y", item.get("lon", item.get("longitude", 0.0)))
                z_val = item.get("z", item.get("alt", item.get("altitude", 0.0)))
                wp = mavtypes.MissionItem(
                    command=int(item["command"]),
                    x=float(x_val),
                    y=float(y_val),
                    z=float(z_val),
                    frame=int(item.get("frame", default_frame)),
                    param1=float(item.get("param1", 0.0)),
                    param2=float(item.get("param2", 0.0)),
                    param3=float(item.get("param3", 0.0)),
                    param4=float(item.get("param4", 0.0)),
                )
            elif isinstance(item, (tuple, list)):
                if len(item) == 4:
                    cmd, x_val, y_val, z_val = item
                    wp = mavtypes.MissionItem(
                        command=int(cmd), x=float(x_val), y=float(y_val), z=float(z_val), frame=default_frame
                    )
                elif len(item) == 5:
                    cmd, x_val, y_val, z_val, frm = item
                    wp = mavtypes.MissionItem(
                        command=int(cmd), x=float(x_val), y=float(y_val), z=float(z_val), frame=int(frm)
                    )
                elif len(item) == 9:
                    cmd, x_val, y_val, z_val, frm, p1, p2, p3, p4 = item
                    wp = mavtypes.MissionItem(
                        command=int(cmd), x=float(x_val), y=float(y_val), z=float(z_val),
                        frame=int(frm), param1=float(p1), param2=float(p2), param3=float(p3), param4=float(p4)
                    )
                else:
                    self.logger.error("Invalid tuple length (%s) for fence item #%s: expected 4, 5, or 9 elements", len(item), idx)
                    return False
            else:
                self.logger.error("Invalid fence item type at index %s: %s", idx, type(item))
                return False

            if wp.command not in valid_fence_cmds:
                self.logger.error("Fence item #%s has unsupported command %s", idx, wp.command)
                return False
            if wp.frame not in valid_frames:
                self.logger.error("Fence item #%s has unsupported frame %s (must be a global MAV_FRAME_GLOBAL*)", idx, wp.frame)
                return False
            if wp.command in polygon_cmds and (wp.param1 < 3 or wp.param1 > 255):
                self.logger.error("Fence polygon item #%s has invalid vertex count param1=%s (must be in [3, 255])", idx, wp.param1)
                return False
            if wp.command in circle_cmds and (not math.isfinite(wp.param1) or wp.param1 <= 0):
                self.logger.error("Fence circle item #%s has invalid radius param1=%s (must be > 0)", idx, wp.param1)
                return False
            normalized.append(wp)

        return self.upload_mission(
            waypoints=normalized,
            clear_existing=clear_existing,
            target_system=target_system,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            **kwargs,
        )

    def download_geofence(self,
            target_system: int|None = None,
            **kwargs
        ) -> list[mavtypes.MissionItem]|None:
        """Downloads all stored Geofence items from the flight controller.

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout (float|None): Overall download timeout in seconds. Default: 15.
            **retry_timeout (float): Retry timeout in seconds. Default: 1.0.
            **max_retries (int): Max retries per step. Default: 3.

        Returns:
            list[mavtypes.MissionItem]|None: List of geofence ``MissionItem`` objects. [] if none stored, or None on error/timeout.
        """
        return self.download_mission(
            target_system=target_system,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            **kwargs,
        )

    def upload_mission_file(self,
            filepath: str|os.PathLike,
            clear_existing: bool = True,
            upload_geofence: bool = True,
            upload_rally_points: bool = True,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Loads a Mission Planner (``.waypoints`` / WPL 110) or QGroundControl (``.plan``) file and uploads it to the vehicle.

        Args:
            filepath (str|os.PathLike): Path to the ``.waypoints`` / ``.txt`` (WPL 110) or ``.plan`` (QGroundControl JSON) file.
            clear_existing (bool): Whether to clear existing items on the flight controller before uploading. Default: True.
            upload_geofence (bool): Whether to upload geofence items contained in the file. Default: True.
            upload_rally_points (bool): Whether to upload rally points contained in the file. Default: True.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout (float|None): Overall timeout in seconds per upload handshake. Default: 15.0.
            **retry_timeout (float): Retry timeout in seconds. Default: 1.5.
            **max_retries (int): Maximum consecutive retries per step. Default: 5.

        Returns:
            bool: True if all items from the file were uploaded and accepted, False otherwise.

        Note:
            - Tested with ArduPilot SITL and offline unit tests; real-world testing with files freshly exported from
              the QGroundControl / Mission Planner GUIs and against real PX4 hardware/SITL is still needed.
        """
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to upload mission file")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to upload mission file")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        try:
            plan = self.load_mission_file(filepath=filepath)
        except (OSError, ValueError) as exc:
            self.logger.error("Failed to load mission file '%s': %s", filepath, exc)
            return False

        uploaded_any = False
        if plan.waypoints:
            if not self.upload_mission(
                waypoints=plan,
                clear_existing=clear_existing,
                target_system=target_system,
                **kwargs,
            ):
                return False
            uploaded_any = True

        if upload_geofence and plan.geofence:
            if not self.upload_geofence(
                fence_items=list(plan.geofence),
                clear_existing=clear_existing,
                target_system=target_system,
                **kwargs,
            ):
                return False
            uploaded_any = True

        if upload_rally_points and plan.rally_points:
            if not self.upload_rally_points(
                rally_points=list(plan.rally_points),
                clear_existing=clear_existing,
                target_system=target_system,
                **kwargs,
            ):
                return False
            uploaded_any = True

        if not uploaded_any:
            self.logger.error("Mission file '%s' contains no items to upload", filepath)
            return False
        return True

    def download_mission_file(self,
            filepath: str|os.PathLike,
            file_format: str|None = None,
            include_geofence: bool = True,
            include_rally_points: bool = True,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Downloads the mission (and optionally geofence & rally points) from the vehicle and saves it to a file.
        Supports both Mission Planner (``.waypoints`` / QGC WPL 110) and QGroundControl (``.plan`` JSON) formats.

        Args:
            filepath (str|os.PathLike): Destination file path. Format is inferred from extension if ``file_format`` is None.
            file_format (str|None): Explicit format (``"waypoints"`` or ``"plan"``). Default: None (infer from extension).
            include_geofence (bool): When saving as ``.plan``, also download and include geofence items. Default: True.
            include_rally_points (bool): When saving as ``.plan``, also download and include rally points. Default: True.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout (float|None): Overall timeout in seconds per download handshake. Default: 15.0.
            **retry_timeout (float): Retry timeout in seconds. Default: 1.5.
            **max_retries (int): Maximum consecutive retries per step. Default: 5.

        Returns:
            bool: True if the download and file export succeeded, False otherwise.

        Note:
            - Tested with ArduPilot SITL and offline unit tests; opening exported ``.plan`` / ``.waypoints`` files
              directly in the QGroundControl and Mission Planner desktop GUIs and testing on real PX4 hardware/SITL
              is still needed.
        """
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to download mission file")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        try:
            resolved_fmt = self._resolve_mission_file_format(filepath=filepath, file_format=file_format)
        except ValueError as exc:
            self.logger.error("Invalid mission file format for '%s': %s", filepath, exc)
            return False

        if target_system is None:
            target_system = self.master.target_system

        suffix = Path(filepath).suffix.lower()
        if resolved_fmt == "waypoints" and suffix == ".fen":
            dl_fence = self.download_geofence(target_system=target_system, **kwargs)
            if dl_fence is None:
                return False
            try:
                self.save_mission_file(filepath=filepath, waypoints=[], file_format="waypoints", geofence=dl_fence)
            except (OSError, ValueError) as exc:
                self.logger.error("Failed to save geofence file '%s': %s", filepath, exc)
                return False
            return True

        if resolved_fmt == "waypoints" and suffix == ".ral":
            dl_rally = self.download_rally_points(target_system=target_system, **kwargs)
            if dl_rally is None:
                return False
            try:
                self.save_mission_file(filepath=filepath, waypoints=[], file_format="waypoints", rally_points=dl_rally)
            except (OSError, ValueError) as exc:
                self.logger.error("Failed to save rally file '%s': %s", filepath, exc)
                return False
            return True

        waypoints, wire_home = self._download_mission_internal(
            target_system=target_system,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            **kwargs,
        )
        if waypoints is None:
            return False

        hp = self.home_position(source_system=target_system)
        if not math.isnan(hp.latitude_home) and not math.isnan(hp.longitude_home):
            home_pos: mavtypes.HomePosition|None = hp
        else:
            home_pos = wire_home

        geofence: list[mavtypes.MissionItem] = []
        rally_points: list[mavtypes.MissionItem] = []
        firmware_type = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        vehicle_type = mavutil.mavlink.MAV_TYPE_QUADROTOR

        if resolved_fmt == "plan":
            if include_geofence:
                dl_fence = self.download_geofence(target_system=target_system, **kwargs)
                if dl_fence is None:
                    return False
                geofence = dl_fence
            if include_rally_points:
                dl_rally = self.download_rally_points(target_system=target_system, **kwargs)
                if dl_rally is None:
                    return False
                rally_points = dl_rally

            ap = self.autopilot_int(source_system=target_system)
            if not math.isnan(ap):
                firmware_type = int(ap)
            vt = self.vehicle_type_int(source_system=target_system)
            if not math.isnan(vt):
                vehicle_type = int(vt)

        try:
            self.save_mission_file(
                filepath=filepath,
                waypoints=waypoints,
                file_format=resolved_fmt,
                geofence=geofence,
                rally_points=rally_points,
                home_position=home_pos,
                firmware_type=firmware_type,
                vehicle_type=vehicle_type,
            )
        except (OSError, ValueError) as exc:
            self.logger.error("Failed to save mission file '%s': %s", filepath, exc)
            return False
        return True

    def start_mission(self,
            first_wp_tkoff: bool = None,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Start the mission. Currently implemented only for ArduPilot.

        Args:
            first_wp_tkoff (bool): Deprecated (will be removed soon, so do not use it). Still here for questions regarding backward compatibility.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout (float|None): Waiting timeout (used in several places). Default: 3.

        Returns:
            bool: Whether the mission has started.
        """
        self.logger.info("Starting mission..")
        if first_wp_tkoff is not None:
            self.logger.warning("first_wp_tkoff option in start_mission() is deprecated and will be removed soon, so do not use it!!")
        timeout = kwargs.get('timeout', 3)
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

        if not self.motors_armed(source_system=target_system):
            armed = self.arm(
                target_system=target_system,
                timeout_ack=timeout,
                timeout_arming=timeout
            )
            if not armed:
                self.logger.critical("Start mission failed")
                return False

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_MISSION_START, # command
            1, # confirmation
            0, # param1 (First Item)
            0, # param2 (Last Item)
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

    def set_current_waypoint(self,
            seq: int,
            reset: bool = False,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Sets the active mission item (waypoint jump) using ``MAV_CMD_DO_SET_MISSION_CURRENT``.

        If a mission is currently running in ``AUTO`` mode, the vehicle immediately navigates to the specified
        sequence number, skipping intermediate mission items. This command does not switch the vehicle to ``AUTO`` mode.

        Args:
            seq (int): Mission item sequence number to jump to (must be >= 0 on ArduPilot; -1 with ``reset=True`` is supported on PX4 to reset jump counters without changing the current item).
            reset (bool): If True, sets ``param2 = 1`` to reset ``MAV_CMD_DO_JUMP`` repeat counters and reactivate a completed mission. Default: False.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK waiting timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the command was accepted by the flight controller, False otherwise.

        Note:
            - Returns False if seq is out of range for the currently uploaded mission.
            - On ArduPilot, ``seq = -1`` is not supported; pass an explicit non-negative index.
            - On ArduPlane, jumping to a new waypoint clears crosstrack for the next leg and resumes a stopped ``AUTO`` mission.
        """
        self.logger.info("Setting current waypoint to seq=%s (reset=%s)..", seq, reset)
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to set current waypoint")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to set current waypoint")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if isinstance(seq, bool) or not isinstance(seq, int) or seq < -1:
            self.logger.error("Invalid waypoint sequence number: %s (must be an integer >= 0)", seq)
            return False

        if target_system is None:
            target_system = self.master.target_system

        is_ardupilot = (self.autopilot_int(source_system=target_system) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
        if seq == -1:
            if not reset or is_ardupilot:
                self.logger.error(
                    "Invalid waypoint sequence number: -1 is only valid with reset=True outside ArduPilot"
                )
                return False

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, # command
            1, # confirmation
            float(seq), # param1 (Mission sequence number)
            1.0 if reset else 0.0, # param2 (Reset mission)
            0,0,0,0,0 # param3-7 (unused)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Current waypoint set to seq=%s", seq)
        elif accepted is None: # Mission end or timeout
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("DO_SET_MISSION_CURRENT timed out")
        else: # Refused / Denied
            self.logger.error("DO_SET_MISSION_CURRENT failed or denied (seq=%s)", seq)
        return bool(accepted)

    def do_go_around(self,
            altitude: float = 0.0,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Aborts an autonomous landing and initiates a go-around maneuver using ``MAV_CMD_DO_GO_AROUND``.

        If the loaded mission contains a planned ``MAV_CMD_DO_GO_AROUND`` abort sequence, ArduPlane jumps to the closest one.
        Otherwise it commands a pitch-up climb-out to the specified relative altitude (or default abort altitude if altitude==0 before repeating the landing approach.

        Args:
            altitude (float): Target climb-out altitude relative to HOME in meters. 0.0 uses the flight controller's default go-around/abort altitude. Default: 0.0.
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK waiting timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the go-around command was accepted by the flight controller, False otherwise.

        Note:
            - Supported on ArduPlane (fixed-wing and QuadPlane).
            - On ArduPlane, the aircraft must be in AUTO mode and actively executing a
              landing command (e.g., ``MAV_CMD_NAV_LAND`` or ``MAV_CMD_NAV_VTOL_LAND``) for
              this command to be accepted.
        """
        self.logger.info("Triggering go-around (DO_GO_AROUND)..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to trigger go-around")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to trigger go-around")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False
        if math.isnan(altitude) or math.isinf(altitude) or altitude < 0:
            self.logger.error("Invalid altitude for go-around: %s (must be >= 0)", altitude)
            return False

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_DO_GO_AROUND, # command
            1, # confirmation
            float(altitude), # param1 (Climb-out altitude in meters, 0 = default)
            0,0,0,0,0,0 # param2-7 (unused)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_GO_AROUND,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Go-around accepted")
        elif accepted is None: # Mission end or timeout
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("DO_GO_AROUND timed out")
        else: # Refused
            self.logger.error("DO_GO_AROUND failed or refused")
        return bool(accepted)

    def do_return_path_start(self,
            target_system: int|None = None,
            **kwargs
        ) -> bool:
        """Joins the closest leg of a mission return-path segment using ``MAV_CMD_DO_RETURN_PATH_START``.

        Commands the flight controller to find the closest waypoint/leg following a ``MAV_CMD_DO_RETURN_PATH_START`` marker (and preceding any landing sequence)
        in the loaded mission, and switches to AUTO mode to follow that return corridor.

        Args:
            target_system (int|None): Target MAVLink system ID. Defaults to the master's ID.
            **timeout_ack (float|None): ACK waiting timeout in seconds. 0 means no wait (returns False). None means no timeout. Default: 2.

        Returns:
            bool: True if the command was accepted by the flight controller, False otherwise.

        Note:
            - Supported on ArduPlane and ArduCopter.
            - Fails (returns False) if no ``MAV_CMD_DO_RETURN_PATH_START`` segment with valid waypoints is present in the uploaded mission.
        """
        self.logger.info("Starting mission return path (DO_RETURN_PATH_START)..")
        timeout_ack = kwargs.get('timeout_ack', 2)
        if self.mission_ended:
            self.logger.critical("Mission stopped: Unable to start return path")
            return False
        if not self.allow_actions:
            self.logger.error("Actions not allowed: Unable to start return path")
            return False
        if self.master is None:
            self.logger.error("Master object is not defined!")
            return False

        if target_system is None:
            target_system = self.master.target_system

        self.master.mav.command_long_send(
            target_system, # target_system
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, # target_component
            mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START, # command
            1, # confirmation
            0,0,0,0,0,0,0 # param1-7 (ignored)
        )

        accepted = self.wait_command_ack(
            command=mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START,
            source_system=target_system,
            timeout=timeout_ack
        )
        if accepted:
            self.logger.info("Return path started")
        elif accepted is None: # Mission end or timeout
            if self.mission_ended:
                self.logger.critical("Mission ended")
            else:
                self.logger.critical("DO_RETURN_PATH_START timed out")
        else: # Refused
            self.logger.error("DO_RETURN_PATH_START failed or refused")
        return bool(accepted)

    @staticmethod
    def _resolve_mission_file_format(filepath: str|os.PathLike, file_format: str|None = None) -> str:
        """Resolves a mission file format ('waypoints' or 'plan') from explicit parameter or extension."""
        if file_format is not None:
            fmt = str(file_format).strip().lower().lstrip(".")
            if fmt in ("plan", "qgc", "json"):
                return "plan"
            if fmt in ("waypoints", "wpl", "wpl110", "txt", "fen", "ral"):
                return "waypoints"
            raise ValueError(f"Unsupported mission file format '{file_format}' (expected 'waypoints' or 'plan')")

        suffix = Path(filepath).suffix.lower()
        if suffix in (".plan", ".json"):
            return "plan"
        if suffix in (".waypoints", ".txt", ".wpl", ".fen", ".ral"):
            return "waypoints"
        raise ValueError(
            f"Cannot infer mission file format from extension '{suffix}' (pass file_format='waypoints' or 'plan')"
        )

    @staticmethod
    def _normalize_mission_items_for_file(
            items: list[mavtypes.MissionItem|dict|tuple]|tuple[mavtypes.MissionItem|dict|tuple, ...],
            default_frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            default_command: int|None = None,
        ) -> list[mavtypes.MissionItem]:
        """Normalizes MissionItem/dict/tuple sequences into a list of mavtypes.MissionItem."""
        normalized: list[mavtypes.MissionItem] = []
        for idx, item in enumerate(items):
            if isinstance(item, mavtypes.MissionItem):
                normalized.append(item)
            elif isinstance(item, dict):
                if "command" not in item and default_command is None:
                    raise ValueError(f"Mission item #{idx} dict is missing required 'command' key")
                cmd = int(item.get("command", default_command))
                x_val = item.get("x", item.get("lat", item.get("latitude", 0.0)))
                y_val = item.get("y", item.get("lon", item.get("longitude", 0.0)))
                z_val = item.get("z", item.get("alt", item.get("altitude", item.get("relative_alt", 0.0))))
                normalized.append(mavtypes.MissionItem(
                    command=cmd,
                    x=float(x_val),
                    y=float(y_val),
                    z=float(z_val),
                    frame=int(item.get("frame", default_frame)),
                    param1=float(item.get("param1", 0.0)),
                    param2=float(item.get("param2", 0.0)),
                    param3=float(item.get("param3", 0.0)),
                    param4=float(item.get("param4", 0.0)),
                ))
            elif isinstance(item, (tuple, list)):
                if len(item) == 3 and default_command is not None:
                    x_val, y_val, z_val = item
                    normalized.append(mavtypes.MissionItem(
                        command=int(default_command), x=float(x_val), y=float(y_val), z=float(z_val), frame=default_frame
                    ))
                elif len(item) == 4:
                    cmd, x_val, y_val, z_val = item
                    normalized.append(mavtypes.MissionItem(
                        command=int(cmd), x=float(x_val), y=float(y_val), z=float(z_val), frame=default_frame
                    ))
                elif len(item) == 5:
                    cmd, x_val, y_val, z_val, frm = item
                    normalized.append(mavtypes.MissionItem(
                        command=int(cmd), x=float(x_val), y=float(y_val), z=float(z_val), frame=int(frm)
                    ))
                elif len(item) == 9:
                    cmd, x_val, y_val, z_val, frm, p1, p2, p3, p4 = item
                    normalized.append(mavtypes.MissionItem(
                        command=int(cmd), x=float(x_val), y=float(y_val), z=float(z_val),
                        frame=int(frm), param1=float(p1), param2=float(p2), param3=float(p3), param4=float(p4)
                    ))
                else:
                    raise ValueError(f"Invalid tuple length ({len(item)}) for mission item #{idx}")
            else:
                raise ValueError(f"Invalid mission item type at index {idx}: {type(item)}")

            last_wp = normalized[-1]
            if not (math.isfinite(last_wp.x) and math.isfinite(last_wp.y) and math.isfinite(last_wp.z)):
                raise ValueError(f"Mission item #{idx} has non-finite coordinates: ({last_wp.x}, {last_wp.y}, {last_wp.z})")
            if any(math.isinf(p) for p in (last_wp.param1, last_wp.param2, last_wp.param3, last_wp.param4)):
                raise ValueError(f"Mission item #{idx} has infinite parameter value")
        return normalized

    @staticmethod
    def load_mission_file(filepath: str|os.PathLike) -> mavtypes.MissionPlan:
        """Loads a mission file in Mission Planner (``.waypoints`` / QGC WPL 110) or QGroundControl (``.plan`` JSON) format.

        Format is auto-detected from file content (JSON ``fileType == "Plan"`` vs ``QGC WPL 110`` header).
        Both formats produce the exact same ``MissionPlan`` representation where ``waypoints`` contains only the
        real flight commands (never ArduPilot's internal sequence 0 HOME placeholder), ready to pass directly to
        ``upload_mission()`` or ``save_mission_file()``.

        Args:
            filepath (str|os.PathLike): Path to the ``.waypoints`` / ``.txt`` / ``.fen`` / ``.ral`` (WPL 110) or ``.plan`` (QGroundControl JSON) file.

        Returns:
            mavtypes.MissionPlan:
                - waypoints (list[MissionItem]): Ordered flight mission items ready for ``upload_mission()``.
                - geofence (list[MissionItem]): Geofence items ready for ``upload_geofence()``.
                - rally_points (list[MissionItem]): Rally points ready for ``upload_rally_points()``.
                - home_position (HomePosition|None): Planned HOME coordinates if defined in the file.

        Raises:
            FileNotFoundError: If ``filepath`` does not exist.
            ValueError: If the file format or contents are invalid or contain unexpanded complex items.

        Note:
            - For QGC ``.plan`` files, ``SimpleItem`` entries, ``fwLandingPattern`` / ``vtolLandingPattern`` landing
              patterns, and ``ComplexItem`` entries embedding expanded ``Items`` (or ``TransectStyleComplexItem.Items``)
              are supported; ``MAV_CMD_DO_JUMP`` ``doJumpId`` targets are automatically remapped to 1-based item indices.
            - For WPL 110 files, Mission Planner's sequence 0 HOME entry is extracted into ``MissionPlan.home_position``,
              and standalone WPL 110 fence or rally files are automatically routed to ``geofence`` or ``rally_points``.
            - Validated against ArduPilot/QGC sample files and offline unit tests, but still needs real-world testing
              with files exported directly from recent QGroundControl and Mission Planner desktop releases.
        """
        raw_text = Path(filepath).read_text(encoding="utf-8-sig")
        stripped = raw_text.lstrip()
        if not stripped:
            raise ValueError(f"Mission file '{filepath}' is empty")

        fence_cmds = {
            mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
        }

        # ------------------------------------------------------------------
        # QGroundControl .plan JSON format
        # ------------------------------------------------------------------
        if stripped.startswith("{") or Path(filepath).suffix.lower() in (".plan", ".json"):
            try:
                data = json.loads(raw_text)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON in plan file '{filepath}': {exc}") from exc

            if not isinstance(data, dict) or data.get("fileType") != "Plan":
                raise ValueError(f"File '{filepath}' is not a valid QGroundControl Plan file (missing fileType='Plan')")
            mission_obj = data.get("mission")
            if not isinstance(mission_obj, dict):
                raise ValueError(f"Plan file '{filepath}' is missing 'mission' object")

            def _to_float(val, default: float = 0.0) -> float:
                if val is None:
                    return default
                fval = float(val)
                if not math.isfinite(fval):
                    raise ValueError(f"Non-finite numeric value in plan file: {val}")
                return fval

            home_position: mavtypes.HomePosition | None = None
            raw_home = mission_obj.get("plannedHomePosition")
            if isinstance(raw_home, (list, tuple)) and len(raw_home) >= 2 and raw_home[0] is not None and raw_home[1] is not None:
                hp_lat = _to_float(raw_home[0])
                hp_lon = _to_float(raw_home[1])
                hp_alt = _to_float(raw_home[2]) if len(raw_home) >= 3 else 0.0
                if hp_lat != 0.0 or hp_lon != 0.0:
                    home_position = mavtypes.HomePosition(
                        latitude_home=hp_lat,
                        longitude_home=hp_lon,
                        altitude_home=hp_alt,
                    )

            waypoints: list[mavtypes.MissionItem] = []
            jump_id_to_index: dict[int, int] = {}

            def _parse_qgc_items(raw_items: list) -> None:
                for entry in raw_items:
                    if not isinstance(entry, dict):
                        continue
                    item_type = entry.get("type")
                    jump_id = entry.get("doJumpId")
                    if item_type == "SimpleItem" or ("command" in entry and "params" in entry):
                        raw_params = entry.get("params")
                        if not isinstance(raw_params, list):
                            raise ValueError(f"Invalid 'params' field in SimpleItem: {entry}")
                        params = [_to_float(raw_params[i]) if i < len(raw_params) else 0.0 for i in range(7)]
                        frame = int(entry.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))
                        command = int(entry["command"])
                        waypoints.append(mavtypes.MissionItem(
                            command=command,
                            x=params[4],
                            y=params[5],
                            z=params[6],
                            frame=frame,
                            param1=params[0],
                            param2=params[1],
                            param3=params[2],
                            param4=params[3],
                        ))
                        if isinstance(jump_id, int):
                            jump_id_to_index[jump_id] = len(waypoints)
                    elif item_type == "ComplexItem":
                        complex_type = entry.get("complexItemType", "")
                        nested = entry.get("Items")
                        if not isinstance(nested, list):
                            transect = entry.get("TransectStyleComplexItem")
                            if isinstance(transect, dict):
                                nested = transect.get("Items")
                        if isinstance(nested, list):
                            first_idx = len(waypoints) + 1
                            _parse_qgc_items(nested)
                            if isinstance(jump_id, int) and len(waypoints) >= first_idx:
                                jump_id_to_index[jump_id] = first_idx
                        elif complex_type in ("fwLandingPattern", "vtolLandingPattern"):
                            app_coord = entry.get("landingApproachCoordinate", entry.get("loiterCoordinate"))
                            land_coord = entry.get("landCoordinate")
                            if (
                                not isinstance(app_coord, (list, tuple)) or len(app_coord) < 2
                                or not isinstance(land_coord, (list, tuple)) or len(land_coord) < 2
                            ):
                                raise ValueError(f"Invalid landing pattern coordinates in '{filepath}': {entry}")
                            rel_alt = bool(entry.get("altitudesAreRelative", True))
                            alt_frame = (
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                                if rel_alt
                                else mavutil.mavlink.MAV_FRAME_GLOBAL
                            )
                            app_lat, app_lon = _to_float(app_coord[0]), _to_float(app_coord[1])
                            app_alt = _to_float(app_coord[2]) if len(app_coord) >= 3 else 0.0
                            land_lat, land_lon = _to_float(land_coord[0]), _to_float(land_coord[1])
                            land_alt = _to_float(land_coord[2]) if len(land_coord) >= 3 else 0.0
                            radius = abs(_to_float(entry.get("loiterRadius", 75.0)))
                            clockwise = bool(entry.get("loiterClockwise", True))
                            signed_radius = radius if clockwise else -radius

                            first_idx = len(waypoints) + 1
                            waypoints.append(mavtypes.MissionItem(
                                command=mavutil.mavlink.MAV_CMD_DO_LAND_START,
                                x=app_lat,
                                y=app_lon,
                                z=0.0,
                                frame=alt_frame,
                            ))
                            if complex_type == "vtolLandingPattern" and not bool(entry.get("useLoiterToAlt", True)):
                                waypoints.append(mavtypes.MissionItem(
                                    command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                    x=app_lat,
                                    y=app_lon,
                                    z=app_alt,
                                    frame=alt_frame,
                                ))
                            else:
                                waypoints.append(mavtypes.MissionItem(
                                    command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
                                    x=app_lat,
                                    y=app_lon,
                                    z=app_alt,
                                    frame=alt_frame,
                                    param1=1.0,
                                    param2=signed_radius,
                                    param4=1.0,
                                ))
                            land_cmd = (
                                mavutil.mavlink.MAV_CMD_NAV_LAND
                                if complex_type == "fwLandingPattern"
                                else mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND
                            )
                            waypoints.append(mavtypes.MissionItem(
                                command=land_cmd,
                                x=land_lat,
                                y=land_lon,
                                z=land_alt,
                                frame=alt_frame,
                            ))
                            if isinstance(jump_id, int):
                                jump_id_to_index[jump_id] = first_idx
                        else:
                            raise ValueError(
                                f"Unsupported unexpanded QGroundControl ComplexItem '{complex_type}' in '{filepath}'"
                            )

            raw_mission_items = mission_obj.get("items", [])
            if not isinstance(raw_mission_items, list):
                raise ValueError(f"Plan file '{filepath}' has invalid 'mission.items' (expected list)")
            _parse_qgc_items(raw_mission_items)

            # Remap MAV_CMD_DO_JUMP target doJumpId to 1-based item index in waypoints
            if jump_id_to_index:
                waypoints = [
                    wp._replace(param1=float(jump_id_to_index[int(round(wp.param1))]))
                    if wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP and int(round(wp.param1)) in jump_id_to_index
                    else wp
                    for wp in waypoints
                ]

            # Parse geoFence (v2 polygons/circles + optional breachReturn + v1 polygon fallback)
            geofence: list[mavtypes.MissionItem] = []
            fence_obj = data.get("geoFence")
            if isinstance(fence_obj, dict):
                breach = fence_obj.get("breachReturn", fence_obj.get("breachReturnPoint"))
                if isinstance(breach, (list, tuple)) and len(breach) >= 2 and breach[0] is not None and breach[1] is not None:
                    geofence.append(mavtypes.MissionItem(
                        command=mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                        x=_to_float(breach[0]),
                        y=_to_float(breach[1]),
                        z=_to_float(breach[2]) if len(breach) >= 3 else 0.0,
                        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    ))

                for poly in fence_obj.get("polygons", []):
                    if isinstance(poly, dict):
                        inclusion = bool(poly.get("inclusion", True))
                        verts = poly.get("polygon", [])
                    elif isinstance(poly, list):
                        inclusion = True
                        verts = poly
                    else:
                        continue
                    if not isinstance(verts, list) or len(verts) < 3:
                        continue
                    cmd = (
                        mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
                        if inclusion
                        else mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
                    )
                    vertex_count = float(len(verts))
                    for vert in verts:
                        if isinstance(vert, (list, tuple)) and len(vert) >= 2:
                            geofence.append(mavtypes.MissionItem(
                                command=cmd,
                                x=_to_float(vert[0]),
                                y=_to_float(vert[1]),
                                z=0.0,
                                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                                param1=vertex_count,
                            ))

                for circ in fence_obj.get("circles", []):
                    if not isinstance(circ, dict):
                        continue
                    inclusion = bool(circ.get("inclusion", True))
                    inner = circ.get("circle", circ)
                    if not isinstance(inner, dict):
                        continue
                    center = inner.get("center")
                    radius = inner.get("radius")
                    if not isinstance(center, (list, tuple)) or len(center) < 2 or radius is None:
                        continue
                    radius_f = _to_float(radius)
                    if radius_f <= 0:
                        continue
                    cmd = (
                        mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
                        if inclusion
                        else mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
                    )
                    geofence.append(mavtypes.MissionItem(
                        command=cmd,
                        x=_to_float(center[0]),
                        y=_to_float(center[1]),
                        z=0.0,
                        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                        param1=radius_f,
                    ))

            # Parse rallyPoints
            rally_points: list[mavtypes.MissionItem] = []
            rally_obj = data.get("rallyPoints")
            if isinstance(rally_obj, dict):
                for pt in rally_obj.get("points", []):
                    if isinstance(pt, (list, tuple)) and len(pt) >= 2 and pt[0] is not None and pt[1] is not None:
                        rally_points.append(mavtypes.MissionItem(
                            command=mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                            x=_to_float(pt[0]),
                            y=_to_float(pt[1]),
                            z=_to_float(pt[2]) if len(pt) >= 3 else 0.0,
                            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        ))

            return mavtypes.MissionPlan(
                waypoints=waypoints,
                geofence=geofence,
                rally_points=rally_points,
                home_position=home_position,
            )

        # ------------------------------------------------------------------
        # Mission Planner / QGC WPL 110 text format (.waypoints / .txt / .fen / .ral)
        # ------------------------------------------------------------------
        lines = raw_text.splitlines()
        header = None
        body_start = 0
        for idx, line in enumerate(lines):
            s = line.strip()
            if s:
                header = s
                body_start = idx + 1
                break

        if header != "QGC WPL 110":
            raise ValueError(f"Unsupported waypoint file header '{header}' in '{filepath}' (expected 'QGC WPL 110' or QGC '.plan' JSON)")

        parsed_items: list[mavtypes.MissionItem] = []
        first_meta: tuple[int, int] | None = None  # (seq, raw_command)
        for line_no, raw_line in enumerate(lines[body_start:], start=body_start + 1):
            s = raw_line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) != 12:
                raise ValueError(f"Invalid WPL 110 line #{line_no} in '{filepath}': expected 12 columns, got {len(parts)}")
            try:
                seq = int(parts[0])
                int(parts[1])  # current
                frame = int(parts[2])
                raw_command = int(parts[3])
                p1, p2, p3, p4 = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                x_val, y_val, z_val = float(parts[8]), float(parts[9]), float(parts[10])
                int(parts[11])  # autocontinue
            except ValueError as exc:
                raise ValueError(f"Invalid numeric field on WPL 110 line #{line_no} in '{filepath}': {exc}") from exc

            if not all(math.isfinite(v) for v in (p1, p2, p3, p4, x_val, y_val, z_val)):
                raise ValueError(f"Non-finite numeric value on WPL 110 line #{line_no} in '{filepath}'")

            if first_meta is None:
                first_meta = (seq, raw_command)

            command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT if (seq == 0 and raw_command == 0 and not parsed_items) else raw_command

            parsed_items.append(mavtypes.MissionItem(
                command=command,
                x=x_val,
                y=y_val,
                z=z_val,
                frame=frame,
                param1=p1,
                param2=p2,
                param3=p3,
                param4=p4,
            ))

        # Auto-route standalone WPL 110 fence or rally files
        if parsed_items and all(it.command in fence_cmds for it in parsed_items):
            return mavtypes.MissionPlan(
                waypoints=[],
                geofence=parsed_items,
                rally_points=[],
                home_position=None,
            )
        if parsed_items and all(it.command == mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT for it in parsed_items):
            return mavtypes.MissionPlan(
                waypoints=[],
                geofence=[],
                rally_points=parsed_items,
                home_position=None,
            )

        # Standard WPL 110 mission file: seq 0 (command 16 or 0) is always the HOME slot
        home_position = None
        if parsed_items and first_meta is not None and first_meta[0] == 0 and first_meta[1] in (0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT):
            wp0 = parsed_items[0]
            if wp0.x != 0.0 or wp0.y != 0.0:
                home_position = mavtypes.HomePosition(
                    latitude_home=wp0.x,
                    longitude_home=wp0.y,
                    altitude_home=wp0.z,
                )
            waypoints = parsed_items[1:]
        else:
            # Hand-crafted WPL 110 without a seq 0 HOME entry: shift 0-based DO_JUMP targets to 1-based
            waypoints = [
                wp._replace(param1=wp.param1 + 1.0)
                if wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP and wp.param1 >= 0.0
                else wp
                for wp in parsed_items
            ]

        return mavtypes.MissionPlan(
            waypoints=waypoints,
            geofence=[],
            rally_points=[],
            home_position=home_position,
        )

    @staticmethod
    def save_mission_file(
            filepath: str|os.PathLike,
            waypoints: list[mavtypes.MissionItem|dict|tuple]|mavtypes.MissionPlan = (),
            file_format: str|None = None,
            geofence: list[mavtypes.MissionItem|dict|tuple]|None = None,
            rally_points: list[mavtypes.MissionItem|dict|tuple]|None = None,
            home_position: mavtypes.HomePosition|tuple[float, float, float]|None = None,
            firmware_type: int = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            vehicle_type: int = mavutil.mavlink.MAV_TYPE_QUADROTOR,
            cruise_speed: float = 15.0,
            hover_speed: float = 5.0,
        ) -> None:
        """Exports a mission (and optionally geofence & rally points) to Mission Planner (``.waypoints``) or QGroundControl (``.plan``) format.

        ``waypoints`` must contain only the actual flight mission items (never a manual sequence 0 HOME placeholder):

        - When writing ``.waypoints`` (QGC WPL 110), sequence 0 HOME is automatically written at row 0 (using
          ``home_position`` if provided, else ``(0, 0, 0)``), followed by ``waypoints`` at rows ``1..N`` (or standalone
          fence/rally rows if ``waypoints`` is empty and only ``geofence`` or ``rally_points`` is given).
        - When writing ``.plan`` (QGroundControl JSON), all ``waypoints`` are written to ``mission.items`` with
          ``plannedHomePosition``, ``geoFence``, and ``rallyPoints``.

        Args:
            filepath (str|os.PathLike): Output file path. Parent directories are created automatically.
            waypoints (list[mavtypes.MissionItem|dict|tuple]|mavtypes.MissionPlan): Flight mission items (or a ``MissionPlan`` returned by ``load_mission_file()``).
            file_format (str|None): Output format: ``"waypoints"`` (QGC WPL 110) or ``"plan"`` (QGroundControl JSON). If None, inferred from ``filepath`` extension (``.waypoints``/``.txt``/``.fen``/``.ral`` or ``.plan``/``.json``).
            geofence (list[mavtypes.MissionItem|dict|tuple]|None): Optional geofence items.
            rally_points (list[mavtypes.MissionItem|dict|tuple]|None): Optional rally points.
            home_position (mavtypes.HomePosition|tuple[float, float, float]|None): Optional planned HOME position ``(lat, lon, alt_msl)``.
            firmware_type (int): MAV_AUTOPILOT value stored in ``.plan`` metadata. Default: ``MAV_AUTOPILOT_ARDUPILOTMEGA`` (3).
            vehicle_type (int): MAV_TYPE value stored in ``.plan`` metadata. Default: ``MAV_TYPE_QUADROTOR`` (2).
            cruise_speed (float): Nominal cruise speed (m/s) stored in ``.plan`` metadata. Default: 15.0.
            hover_speed (float): Nominal hover speed (m/s) stored in ``.plan`` metadata. Default: 5.0.

        Raises:
            ValueError: If ``file_format`` cannot be resolved or mission items are invalid.
            OSError: If writing to ``filepath`` fails.

        Note:
            - Validated with ``pymavlink.mavwp`` and roundtrip unit tests, but exported ``.plan`` and ``.waypoints``
              files still need real-world testing by opening them directly in the QGroundControl and Mission Planner GUIs.
        """
        resolved_fmt = _Missions._resolve_mission_file_format(filepath=filepath, file_format=file_format)

        if isinstance(waypoints, mavtypes.MissionPlan):
            plan = waypoints
            raw_wps = plan.waypoints
            raw_fence = plan.geofence if geofence is None else geofence
            raw_rally = plan.rally_points if rally_points is None else rally_points
            if home_position is None:
                home_position = plan.home_position
        else:
            raw_wps = waypoints
            raw_fence = geofence or []
            raw_rally = rally_points or []

        norm_wps = _Missions._normalize_mission_items_for_file(
            raw_wps, default_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        )
        norm_fence = _Missions._normalize_mission_items_for_file(
            raw_fence, default_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT
        )
        norm_rally = _Missions._normalize_mission_items_for_file(
            raw_rally,
            default_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            default_command=mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
        )

        planned_home: list[float] | None = None
        if home_position is not None:
            if isinstance(home_position, mavtypes.HomePosition):
                hp_lat, hp_lon, hp_alt = home_position.latitude_home, home_position.longitude_home, home_position.altitude_home
            elif isinstance(home_position, (tuple, list)) and len(home_position) >= 2:
                hp_lat, hp_lon = float(home_position[0]), float(home_position[1])
                hp_alt = float(home_position[2]) if len(home_position) >= 3 else 0.0
            else:
                raise ValueError(f"Invalid home_position: {home_position}")
            if math.isfinite(hp_lat) and math.isfinite(hp_lon) and (hp_lat != 0.0 or hp_lon != 0.0):
                planned_home = [hp_lat, hp_lon, hp_alt if math.isfinite(hp_alt) else 0.0]

        out_path = Path(filepath)
        if out_path.parent and str(out_path.parent) not in ("", "."):
            out_path.parent.mkdir(parents=True, exist_ok=True)

        if resolved_fmt == "waypoints":
            lines = ["QGC WPL 110"]
            def _wpl_p(v: float) -> float:
                return 0.0 if math.isnan(v) else v

            # Standalone fence or rally WPL 110 export when no mission waypoints are given
            if not norm_wps and (norm_fence or norm_rally) and not (norm_fence and norm_rally):
                special_items = norm_fence if norm_fence else norm_rally
                for seq, item in enumerate(special_items):
                    lines.append(
                        f"{seq}\t0\t{item.frame}\t{item.command}\t"
                        f"{_wpl_p(item.param1):.8f}\t{_wpl_p(item.param2):.8f}\t{_wpl_p(item.param3):.8f}\t{_wpl_p(item.param4):.8f}\t"
                        f"{item.x:.8f}\t{item.y:.8f}\t{item.z:.6f}\t0"
                    )
            else:
                h_lat, h_lon, h_alt = planned_home if planned_home is not None else [0.0, 0.0, 0.0]
                lines.append(
                    f"0\t1\t{mavutil.mavlink.MAV_FRAME_GLOBAL}\t{mavutil.mavlink.MAV_CMD_NAV_WAYPOINT}\t"
                    f"0.00000000\t0.00000000\t0.00000000\t0.00000000\t"
                    f"{h_lat:.8f}\t{h_lon:.8f}\t{h_alt:.6f}\t1"
                )
                for seq, wp in enumerate(norm_wps, start=1):
                    lines.append(
                        f"{seq}\t0\t{wp.frame}\t{wp.command}\t"
                        f"{_wpl_p(wp.param1):.8f}\t{_wpl_p(wp.param2):.8f}\t{_wpl_p(wp.param3):.8f}\t{_wpl_p(wp.param4):.8f}\t"
                        f"{wp.x:.8f}\t{wp.y:.8f}\t{wp.z:.6f}\t1"
                    )
            out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            return

        # QGroundControl .plan JSON format
        # Ensure plannedHomePosition is always populated for QGC visual rendering without removing any waypoint!
        qgc_home = planned_home
        if qgc_home is None:
            for wp in norm_wps:
                if wp.x != 0.0 or wp.y != 0.0:
                    ref_alt = wp.z if wp.frame in (mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_FRAME_GLOBAL_INT) else 0.0
                    qgc_home = [wp.x, wp.y, ref_alt]
                    break
            if qgc_home is None:
                qgc_home = [0.0, 0.0, 0.0]

        def _qgc_p(v: float) -> float|None:
            return None if math.isnan(v) else v

        qgc_items = [
            {
                "autoContinue": True,
                "command": wp.command,
                "doJumpId": idx,
                "frame": wp.frame,
                "params": [_qgc_p(wp.param1), _qgc_p(wp.param2), _qgc_p(wp.param3), _qgc_p(wp.param4), wp.x, wp.y, wp.z],
                "type": "SimpleItem",
            }
            for idx, wp in enumerate(norm_wps, start=1)
        ]

        # Build geoFence structure
        polygons = []
        circles = []
        breach_return = None
        i = 0
        while i < len(norm_fence):
            item = norm_fence[i]
            if item.command == mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT:
                breach_return = [item.x, item.y, item.z]
                i += 1
            elif item.command in (
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            ):
                circles.append({
                    "circle": {
                        "center": [item.x, item.y],
                        "radius": item.param1,
                    },
                    "inclusion": item.command == mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                    "version": 1,
                })
                i += 1
            elif item.command in (
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
            ):
                expected = max(1, int(round(item.param1)))
                cmd = item.command
                verts = []
                while i < len(norm_fence) and len(verts) < expected and norm_fence[i].command == cmd:
                    verts.append([norm_fence[i].x, norm_fence[i].y])
                    i += 1
                if len(verts) >= 3:
                    polygons.append({
                        "inclusion": cmd == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                        "polygon": verts,
                        "version": 1,
                    })
            else:
                i += 1

        geo_fence_obj: dict = {
            "circles": circles,
            "polygons": polygons,
            "version": 2,
        }
        if breach_return is not None:
            geo_fence_obj["breachReturn"] = breach_return

        mission_obj: dict = {
            "cruiseSpeed": float(cruise_speed),
            "firmwareType": int(firmware_type),
            "hoverSpeed": float(hover_speed),
            "items": qgc_items,
            "plannedHomePosition": qgc_home,
            "vehicleType": int(vehicle_type),
            "version": 2,
        }

        plan_doc = {
            "fileType": "Plan",
            "geoFence": geo_fence_obj,
            "groundStation": "pyMAVLinCS",
            "mission": mission_obj,
            "rallyPoints": {
                "points": [[r.x, r.y, r.z] for r in norm_rally],
                "version": 2,
            },
            "version": 1,
        }
        out_path.write_text(json.dumps(plan_doc, indent=4, allow_nan=False) + "\n", encoding="utf-8")
