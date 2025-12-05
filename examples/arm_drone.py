from logging import getLogger
from pyMAVLinCS.setup_logger import setup_logger # To have a simple but working logger

from pyMAVLinCS import MAVLinCS
import mcc_creation as mcc_mod # For custom MCCs

logger = getLogger("MyLogger")
setup_logger(
    logger=logger,
    log_file=None,
    debug_mode=False
)

drone = MAVLinCS(
    address="127.0.0.1:14550",
    logger=logger,
    mcc_class=mcc_mod.MCC
)

try:
    is_armed = drone.arm()
    if not is_armed:
        logger.error("Drone not armed")
        drone.send_mcc(mcc=mcc_mod.DRONE_NOT_ARMED)
    else:
        logger.info("Drone armed")
        drone.send_mcc(mcc=mcc_mod.DRONE_ARMED)
except Exception:
    logger.exception('Error in main function')
except KeyboardInterrupt:
    logger.info("Stopping..")
finally:
    drone.close()
