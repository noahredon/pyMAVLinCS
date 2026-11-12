# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from typing import NamedTuple, TypedDict

class DataInit(TypedDict):
    address: str
    target_system: int | None
    source_system: int
    source_component: int
    baud: int
    sysid_to_request_home: None|int|list[int]|tuple[int]
    pos_rate: float
    dialect: str
    command_ack_to_hide: None|int|list[int]|tuple[int]

class GPSPosition(NamedTuple):
    lat: float
    """latitude in degrees."""
    lon: float
    """longitude in degrees."""
    relative_alt: float
    """relative altitude in meters."""
    alt: float
    """absolute altitude (MSL) in meters."""

class LocalPosition(NamedTuple):
    x: float
    """North (m)."""
    y: float
    """East (m)."""
    z: float
    """Down (m)."""

class Angles(NamedTuple):
    roll: float
    """Rotation around the longitudinal axis (front-back) ([-pi ; +pi] rad)."""
    pitch: float
    """Rotation around the lateral axis (left-right) ([-pi ; +pi] rad)."""
    yaw: float
    """Rotation around the vertical axis (up-down) ([-pi ; +pi] rad)."""

class AnglesRates(NamedTuple):
    rollspeed: float
    """Rotation speed around the longitudinal axis (front-back) (rad/s)."""
    pitchspeed: float
    """Rotation speed around the lateral axis (left-right) (rad/s)."""
    yawspeed: float
    """Rotation speed around the vertical axis (up-down) (rad/s)."""

class Speed(NamedTuple):
    vx: float
    """Velocity in m/s towards North."""
    vy: float
    """Velocity in m/s towards East."""
    vz: float
    """Velocity in m/s downwards."""

class GimbalAngles(NamedTuple):
    roll: float
    """Roll in degrees."""
    pitch: float
    """Pitch in degrees."""
    yaw: float
    """Yaw in degrees."""

class HomePosition(NamedTuple):
    latitude_home: float
    """HOME latitude."""
    longitude_home: float
    """HOME longitude."""
    altitude_home: float
    """HOME altitude (MSL)."""

class EkfOrigin(NamedTuple):
    latitude: float
    """EKF origin latitude."""
    longitude: float
    """EKF origin longitude."""
    altitude: float
    """EKF origin altitude (MSL)."""

class GeographicPoint(NamedTuple):
    latitude: float
    """Latitude of the GPS point (deg)."""
    longitude: float
    """Longitude of the GPS point (deg)."""

class MissionItem(NamedTuple):
    command: int
    """MAV_CMD | Waypoint action (e.g., mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)."""
    x: float = 0.0
    """Global: latitude in degrees, Local: x position in meters."""
    y: float = 0.0
    """Global: longitude in degrees, Local: y position in meters."""
    z: float = 0.0
    """Altitude in meters (relative or absolute, depending on the frame)."""
    frame: int = 3
    """MAV_FRAME | Coordinate system of the waypoint (default: 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT)."""
    param1: float = 0.0
    """param1 (depending on the command)."""
    param2: float = 0.0
    """param2 (depending on the command)."""
    param3: float = 0.0
    """param3 (depending on the command)."""
    param4: float = 0.0
    """param4 (depending on the command)."""
