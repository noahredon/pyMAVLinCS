import pytest
import subprocess
import time
import os
import signal
from pyMAVLinCS import MAVLinCS

@pytest.fixture(scope="session")
def sitl():
    # Start arducopter using sim_vehicle.py
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sim_vehicle = os.path.join(workspace, "tests", "ardupilot", "Tools", "autotest", "sim_vehicle.py")
    
    cmd = [sim_vehicle, "-v", "ArduCopter", "-f", "quad", "-N", "--mavproxy-args", "--daemon"]
    
    process = subprocess.Popen(
        cmd,
        cwd=workspace,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    
    # sim_vehicle.py starts mavproxy which forwards to 14550. This can take a bit longer.
    time.sleep(10) 
    
    yield "udpin:127.0.0.1:14550"
    
    # Teardown
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    process.wait()

@pytest.fixture(scope="session")
def drone(sitl):
    print(f"\nConnecting to SITL at {sitl}...")
    mav = MAVLinCS(sitl, timeout_heartbeat=60)
    
    from pymavlink.mavutil import mavlink
    mav.write_parameter("ARMING_CHECK", 0, mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(1)
    
    yield mav
    
    mav.close()
