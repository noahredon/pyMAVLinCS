from pyMAVLinCS import MAVLinCS

drone = MAVLinCS(
    address="127.0.0.1:14550"
)

drone.logger.info("Connection successful ! Houra !")

drone.close()
