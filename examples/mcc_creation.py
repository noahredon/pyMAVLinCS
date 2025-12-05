from pyMAVLinCS.mission_control_code import MCC

DRONE_ARMED = MCC(
    value=1,
    name="DRONE_ARMED",
    level="SUCCESS",
    description="Drone armed."
)

DRONE_NOT_ARMED = MCC(
    value=2,
    name="DRONE_NOT_ARMED",
    level="FAIL",
    description="Drone not armed."
)
