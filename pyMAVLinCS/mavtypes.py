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

class VfrHud(NamedTuple):
    airspeed: float
    """Current airspeed in m/s."""
    groundspeed: float
    """Current horizontal ground speed in m/s."""
    heading: int | float
    """Current compass heading in degrees (0-360, 0=North)."""
    throttle: int | float
    """Current throttle setting in percent (0-100)."""
    alt: float
    """Current altitude (MSL) in meters."""
    climb: float
    """Current climb rate in m/s."""

class Wind(NamedTuple):
    direction: float
    """Wind direction (that wind is coming from) in degrees."""
    speed: float
    """Horizontal wind speed in ground plane in m/s."""
    speed_z: float
    """Vertical wind speed in m/s."""

class NavControllerOutput(NamedTuple):
    nav_roll: float
    """Current desired roll in degrees."""
    nav_pitch: float
    """Current desired pitch in degrees."""
    nav_bearing: int | float
    """Current desired heading in degrees."""
    target_bearing: int | float
    """Bearing to current waypoint/target in degrees."""
    wp_dist: int | float
    """Distance to active waypoint in meters."""
    alt_error: float
    """Current altitude error in meters."""
    aspd_error: float
    """Current airspeed error in m/s."""
    xtrack_error: float
    """Current crosstrack error on x-y plane in meters."""

class EkfStatus(NamedTuple):
    flags: int | float
    """EKF_STATUS_FLAGS bitmask indicating active EKF estimator states."""
    velocity_variance: float
    """Velocity variance ratio (values >= 1.0 indicate poor health)."""
    pos_horiz_variance: float
    """Horizontal position variance ratio (values >= 1.0 indicate poor health)."""
    pos_vert_variance: float
    """Vertical position variance ratio (values >= 1.0 indicate poor health)."""
    compass_variance: float
    """Compass variance ratio (values >= 1.0 indicate poor health)."""
    terrain_alt_variance: float
    """Terrain altitude variance ratio (values >= 1.0 indicate poor health)."""

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
    """Latitude in degrees (global frame) or X position in meters (local frame)."""
    y: float = 0.0
    """Longitude in degrees (global frame) or Y position in meters (local frame)."""
    z: float = 0.0
    """Altitude in meters (relative or absolute, depending on the frame)."""
    frame: int = 3
    """MAV_FRAME | Coordinate system of the waypoint (default is 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT)."""
    param1: float = 0.0
    """param1 (depending on the command)."""
    param2: float = 0.0
    """param2 (depending on the command)."""
    param3: float = 0.0
    """param3 (depending on the command)."""
    param4: float = 0.0
    """param4 (depending on the command)."""

class MissionPlan(NamedTuple):
    waypoints: list[MissionItem]
    """Ordered list of flight mission items (1st element = 1st mission command; excludes ArduPilot's internal seq 0 HOME slot)."""
    geofence: list[MissionItem] | tuple[MissionItem, ...] = ()
    """List of geofence items (MAV_CMD_NAV_FENCE_*)."""
    rally_points: list[MissionItem] | tuple[MissionItem, ...] = ()
    """List of rally point items (MAV_CMD_NAV_RALLY_POINT)."""
    home_position: HomePosition | None = None
    """Planned HOME position (latitude, longitude, altitude MSL) if specified in the file."""
