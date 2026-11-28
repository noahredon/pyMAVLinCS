# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from pyMAVLinCS.mavecstra import MavThread, TimesyncHandler, StatustextReceiver, MavLogger
from pyMAVLinCS import mavtypes
from pyMAVLinCS.mavtypes import MissionItem, MissionPlan
from pyMAVLinCS.mission_control_code import MCC, EMPTY
from pyMAVLinCS.setup_logger import setup_logger, default_logger
from pyMAVLinCS._missions import _Missions


class MAVLinCS(_Missions):
    """Allows connection to one or more flight controllers on one link."""
