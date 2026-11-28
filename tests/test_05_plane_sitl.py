import math
import os
import signal
import subprocess
import time
import pytest
from pymavlink import mavutil
from pyMAVLinCS import MAVLinCS, MissionItem
from pyMAVLinCS.setup_logger import setup_logger, default_logger


@pytest.fixture(scope="module")
def plane_sitl():
    """Starts ArduPlane SITL on instance 1 (UDP port 14560) and tears it down after the module."""
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sim_vehicle = os.path.join(
        workspace, "tests", "ardupilot", "Tools", "autotest", "sim_vehicle.py"
    )

    cmd = [
        sim_vehicle,
        "-v", "ArduPlane",
        "-I", "1",                   # Instance 1 -> UDP port 14560 (avoids conflict with ArduCopter on 14550)
        "-N",                        # No recompilation
        "-w",                        # Wipe EEPROM parameters to default
        "--speedup", "3",            # Speed up physics
        "--mavproxy-args", "--daemon"
    ]

    process = subprocess.Popen(
        cmd,
        cwd=workspace,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    try:
        time.sleep(5)
        yield "udpin:127.0.0.1:14560"
    finally:
        try:
            pgid = os.getpgid(process.pid)
            os.killpg(pgid, signal.SIGTERM)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except ProcessLookupError:
            pass


@pytest.fixture(scope="module")
def plane_drone(plane_sitl, tmp_path_factory):
    """Initializes MAVLinCS connected to ArduPlane SITL, disables ARMING_CHECK, and waits for 3D GPS fix."""
    setup_logger(logger=default_logger, debug_mode=True, add_timestamp_to_console=True)
    tlog_file = str(tmp_path_factory.mktemp("logs") / "pytest_plane_sitl.tlog")

    print(f"\nConnecting to ArduPlane SITL at {plane_sitl}...")
    mav = MAVLinCS(
        address=plane_sitl,
        timeout_heartbeat=60,
        mavlogfile=tlog_file,
        relative_path_to_cwd=False,
        pos_rate=10,
        send_automatic_timesync=True,
        timesync_delay=2,
    )

    try:
        # Disable pre-arm checks for simulation (ARMING_SKIPCHK=-1 on ArduPilot 4.7+, ARMING_CHECK=0 on older versions)
        if not mav.write_parameter("ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32, timeout_ack=1.0):
            mav.write_parameter("ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, timeout_ack=1.0)
        time.sleep(1)

        # Wait for 3D GPS Fix
        assert mav.wait_gps_3d_fix(timeout=45) is True, "Timeout: No 3D GPS Fix on ArduPlane SITL"

        # Wait for EKF to allow switching to GUIDED mode
        start_wait = time.time()
        while time.time() - start_wait < 30:
            if mav.set_mode("GUIDED", timeout_ack=2, timeout_set_mode=2):
                break
            time.sleep(1)
        else:
            pytest.fail("ArduPlane SITL EKF did not allow GUIDED mode after 30s.")

        yield mav

    finally:
        mav.mission_ended = False
        mav.allow_actions = True
        mav.close()


def test_plane_telemetry_and_modes(plane_drone, tmp_path):
    """Verifies ArduPlane vehicle type, fixed-wing mode mapping, and mode transitions."""
    assert plane_drone.vehicle_type_int() == mavutil.mavlink.MAV_TYPE_FIXED_WING
    assert plane_drone.vehicle_type_str() == "MAV_TYPE_FIXED_WING"
    assert plane_drone.vtol_state() == mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED
    assert plane_drone.vtol_state_str() == "MAV_VTOL_STATE_UNDEFINED"
    assert plane_drone.landed_state() == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    assert plane_drone.landed_state_str() == "MAV_LANDED_STATE_ON_GROUND"
    assert plane_drone.airspeed_healthy() is True
    ekf = plane_drone.ekf_status()
    assert not math.isnan(ekf.flags) and (ekf.flags & mavutil.mavlink.EKF_ATTITUDE)
    assert ekf.velocity_variance < 1.0 and ekf.pos_horiz_variance < 1.0
    # Non-QuadPlane fixed-wing refuses MAV_CMD_DO_VTOL_TRANSITION
    assert plane_drone.vtol_transition(mavutil.mavlink.MAV_VTOL_STATE_FW, timeout_ack=2) is False

    # Test landing gear retract/deploy and parachute enable/disable/denied-release on ArduPlane SITL
    assert plane_drone.set_landing_gear(deploy=False, timeout_ack=3) is True
    assert plane_drone.set_landing_gear(deploy=True, timeout_ack=3) is True
    assert plane_drone.release_parachute(action=mavutil.mavlink.PARACHUTE_DISABLE, timeout_ack=3) is True
    # Release while disabled (CHUTE_ENABLED=0) must be rejected by ArduPlane
    assert plane_drone.release_parachute(action=mavutil.mavlink.PARACHUTE_RELEASE, timeout_ack=3) is False

    mapping = plane_drone.mode_mapping()
    assert isinstance(mapping, dict)
    for expected_mode in ("MANUAL", "FBWA", "FBWB", "CRUISE", "TAKEOFF", "LOITER", "AUTO", "RTL", "GUIDED"):
        assert expected_mode in mapping

    # Test switching between Fixed-Wing specific modes
    for target_mode in ("FBWA", "LOITER", "GUIDED"):
        assert plane_drone.set_mode(target_mode, timeout_ack=3, timeout_set_mode=4) is True
        assert plane_drone.mode() == target_mode

    # Test Rally Points & Geofence upload/download/clear roundtrip on ArduPlane SITL
    gps0 = plane_drone.position_gps()
    rp1 = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=150.0, east_displacement=100.0)
    rp2 = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=-150.0, east_displacement=-100.0)
    assert plane_drone.upload_rally_points(
        [(rp1.latitude, rp1.longitude, 60.0), {"lat": rp2.latitude, "lon": rp2.longitude, "alt": 70.0}],
        clear_existing=True,
        timeout=15.0,
    ) is True
    dl_rally = plane_drone.download_rally_points(timeout=15.0)
    assert dl_rally is not None and len(dl_rally) == 2
    assert all(r.command == mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT for r in dl_rally)
    assert math.isclose(dl_rally[0].x, rp1.latitude, abs_tol=1e-5)
    assert plane_drone.clear_waypoints(mission_type=mavutil.mavlink.MAV_MISSION_TYPE_RALLY, timeout=5.0) is True
    assert plane_drone.download_rally_points(timeout=10.0) == []

    # Geofence: 4-vertex inclusion polygon + exclusion circle, then clear before flight
    p_nw = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=500.0, east_displacement=-500.0)
    p_ne = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=500.0, east_displacement=500.0)
    p_se = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=-500.0, east_displacement=500.0)
    p_sw = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=-500.0, east_displacement=-500.0)
    p_ex = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=300.0, east_displacement=-300.0)
    fence = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=4, x=p_nw.latitude, y=p_nw.longitude),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=4, x=p_ne.latitude, y=p_ne.longitude),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=4, x=p_se.latitude, y=p_se.longitude),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1=4, x=p_sw.latitude, y=p_sw.longitude),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, param1=40.0, x=p_ex.latitude, y=p_ex.longitude),
    ]
    assert plane_drone.upload_geofence(fence, clear_existing=True, timeout=15.0) is True
    dl_fence = plane_drone.download_geofence(timeout=15.0)
    assert dl_fence is not None and len(dl_fence) == 5
    assert dl_fence[0].command == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
    assert dl_fence[4].command == mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
    assert math.isclose(dl_fence[4].param1, 40.0, rel_tol=1e-4)
    assert plane_drone.clear_waypoints(mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE, timeout=5.0) is True
    assert plane_drone.download_geofence(timeout=10.0) == []

    # Test QGC .plan & Mission Planner .waypoints file upload/download roundtrip on ArduPlane SITL
    plan_path = tmp_path / "plane_sitl.plan"
    sample_wps = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, param1=15.0, x=gps0.lat, y=gps0.lon, z=45.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=rp1.latitude, y=rp1.longitude, z=50.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH),
    ]
    MAVLinCS.save_mission_file(
        plan_path,
        waypoints=sample_wps,
        geofence=fence,
        rally_points=[(rp1.latitude, rp1.longitude, 60.0)],
    )
    assert plane_drone.upload_mission_file(plan_path, clear_existing=True, timeout=15.0) is True

    dl_plan_path = tmp_path / "plane_sitl_dl.plan"
    assert plane_drone.download_mission_file(dl_plan_path, timeout=15.0) is True
    dl_plan = MAVLinCS.load_mission_file(dl_plan_path)
    assert len(dl_plan.waypoints) == len(sample_wps)
    assert len(dl_plan.geofence) == len(fence)
    assert len(dl_plan.rally_points) == 1

    dl_wpl_path = tmp_path / "plane_sitl_dl.waypoints"
    assert plane_drone.download_mission_file(dl_wpl_path, timeout=15.0) is True
    assert len(MAVLinCS.load_mission_file(dl_wpl_path).waypoints) == len(sample_wps)

    # Clean up mission, fence, and rally points before subsequent flight tests
    assert plane_drone.clear_waypoints(mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION, timeout=5.0) is True
    assert plane_drone.clear_waypoints(mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE, timeout=5.0) is True
    assert plane_drone.clear_waypoints(mission_type=mavutil.mavlink.MAV_MISSION_TYPE_RALLY, timeout=5.0) is True


def test_plane_takeoff_from_ground(plane_drone):
    """Tests Fixed-Wing runway takeoff via arm() + takeoff() using the autopilot's configured TKOFF_ALT/TKOFF_LVL_PITCH."""
    initial_tkoff_alt = plane_drone.get_parameter("TKOFF_ALT", timeout=2.0)
    assert plane_drone.arm(timeout_arming=10) is True
    assert plane_drone.motors_armed() is True

    # Default update_tkoff_params=False: uses existing TKOFF_ALT (50m) without modifying autopilot parameters
    assert plane_drone.takeoff(altitude=35.0, precision=2.0, timeout_ack=3, timeout_set_mode=5, timeout_takeoff=45) is True
    assert plane_drone.mode() == "TAKEOFF"
    assert plane_drone.position_gps().relative_alt >= 33.0
    assert plane_drone.airspeed() > 8.0
    assert math.isclose(plane_drone.get_parameter("TKOFF_ALT", timeout=2.0), initial_tkoff_alt, rel_tol=1e-5)


def test_plane_auto_takeoff_mission_and_guided_flight(plane_drone):
    """Uploads and executes a Fixed-Wing AUTO mission, then tests do_reposition, change_speed, do_return_path_start, do_land_start, do_go_around, and RTL."""
    gps0 = plane_drone.position_gps()
    assert not math.isnan(gps0.lat) and not math.isnan(gps0.lon)

    # Project waypoints (appropriate for fixed-wing turn radius)
    wp1 = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=250.0, east_displacement=0.0)
    wp2 = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=250.0, east_displacement=200.0)
    wp_ret = MAVLinCS.cartesian_to_geographic_point(gps0.lat, gps0.lon, north_displacement=120.0, east_displacement=120.0)

    # Item 1 (ArduPlane seq 1): Fixed-Wing NAV_TAKEOFF (param1 = 15 deg pitch, z = 45m)
    # Item 2 & 3 (seq 2 & 3): Outbound waypoints
    # Item 4 (seq 4): DO_RETURN_PATH_START marker + Item 5 (seq 5): return corridor waypoint
    # Item 6 (seq 6): DO_LAND_START marker + Item 7 (seq 7): NAV_LAND at HOME
    mission = [
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, param1=15.0, x=gps0.lat, y=gps0.lon, z=45.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=wp1.latitude, y=wp1.longitude, z=50.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=wp2.latitude, y=wp2.longitude, z=50.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=wp_ret.latitude, y=wp_ret.longitude, z=45.0),
        MissionItem(command=mavutil.mavlink.MAV_CMD_DO_LAND_START),
        MissionItem(command=mavutil.mavlink.MAV_CMD_NAV_LAND, x=gps0.lat, y=gps0.lon, z=0.0),
    ]

    # Set RTL_AUTOLAND = 3 (NO_RTL_GO_AROUND) so ArduPlane pre-arm checks allow arming
    # with DO_LAND_START / DO_RETURN_PATH_START in the mission while keeping normal RTL mode
    assert plane_drone.write_parameter("RTL_AUTOLAND", 3.0) is True

    assert plane_drone.upload_mission(waypoints=mission, clear_existing=True, timeout=15.0) is True

    # Verify mission roundtrip via download_mission()
    downloaded = plane_drone.download_mission(timeout=15.0)
    assert downloaded is not None and len(downloaded) == len(mission)
    downloaded_cmds = [item.command for item in downloaded]
    assert mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START in downloaded_cmds
    assert mavutil.mavlink.MAV_CMD_DO_LAND_START in downloaded_cmds
    assert mavutil.mavlink.MAV_CMD_NAV_LAND in downloaded_cmds
    assert math.isclose(downloaded[1].x, wp1.latitude, abs_tol=1e-5)
    assert math.isclose(downloaded[1].y, wp1.longitude, abs_tol=1e-5)

    # Start mission from the ground (automatically arms and enters AUTO mode)
    assert plane_drone.start_mission(timeout=10) is True
    assert plane_drone.motors_armed() is True
    assert plane_drone.wait_mode_changed("AUTO", timeout=5) is True

    # Wait for the aircraft to take off and climb above 25m relative altitude
    start_climb = time.time()
    climbed = False
    while time.time() - start_climb < 45:
        if plane_drone.position_gps().relative_alt >= 25.0 and plane_drone.airspeed() > 10.0:
            climbed = True
            break
        time.sleep(0.5)
    assert climbed is True, f"ArduPlane did not climb as expected (alt={plane_drone.position_gps().relative_alt}m)"

    # Verify aeronautical telemetry getters in flight (VFR_HUD & WIND)
    hud = plane_drone.vfr_hud()
    assert hud.airspeed > 8.0 and plane_drone.airspeed() > 8.0
    assert hud.groundspeed > 8.0 and plane_drone.groundspeed() > 8.0
    assert 0 <= plane_drone.heading() <= 360
    assert 0 <= plane_drone.throttle() <= 100
    assert not math.isnan(plane_drone.climb_rate())
    wnd = plane_drone.wind()
    assert not math.isnan(wnd.direction) and not math.isnan(wnd.speed)
    nav = plane_drone.nav_controller_output()
    assert not math.isnan(nav.nav_roll) and not math.isnan(nav.xtrack_error)
    assert plane_drone.landed_state() in (
        mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
        mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF,
    )

    # Wait for ArduPlane to reach proximity of wp1 (fixed-wing acceptance radius ~90m)
    assert plane_drone.wait_proximity_with_gps_pos(
        lat=wp1.latitude,
        lon=wp1.longitude,
        radius_acceptance=90.0,
        timeout=60,
    ) is True

    # Reposition in flight to wp2 using do_reposition (with force_guided=True, loiter_radius=65m, ccw=True)
    assert plane_drone.do_reposition(
        latitude=wp2.latitude,
        longitude=wp2.longitude,
        altitude=60.0,
        loiter_radius=65.0,
        ccw=True,
        force_guided=True,
        timeout_ack=3,
    ) is True
    assert plane_drone.wait_mode_changed("GUIDED", timeout=5) is True

    # Test change_speed on ArduPlane in GUIDED mode (airspeed, restore default airspeed, and cruise throttle)
    assert plane_drone.change_speed(
        speed=18.0,
        speed_type=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
        timeout_ack=3,
    ) is True
    assert plane_drone.change_speed(
        speed=-2.0,
        speed_type=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
        timeout_ack=3,
    ) is True
    assert plane_drone.change_speed(
        speed=-1.0,
        throttle=65.0,
        timeout_ack=3,
    ) is True

    # Test change_altitude (DO_CHANGE_ALTITUDE and GUIDED_CHANGE_ALTITUDE with slew rate)
    assert plane_drone.change_altitude(65.0, timeout_ack=3) is True
    assert plane_drone.change_altitude(70.0, rate=3.0, timeout_ack=3) is True

    # Test change_heading (GUIDED_CHANGE_HEADING COG + reset to DEFAULT)
    assert plane_drone.change_heading(
        90.0,
        heading_type=mavutil.mavlink.HEADING_TYPE_COURSE_OVER_GROUND,
        max_accel=5.0,
        timeout_ack=3,
    ) is True
    assert plane_drone.change_heading(
        heading_type=mavutil.mavlink.HEADING_TYPE_DEFAULT,
        timeout_ack=3,
    ) is True

    # Test loiter_here() with custom radius/ccw/alt (GUIDED) and default (NAV_LOITER_UNLIM -> LOITER mode)
    assert plane_drone.loiter_here(radius=70.0, ccw=True, altitude=65.0, timeout_ack=3) is True
    assert plane_drone.mode() == "GUIDED"
    assert plane_drone.loiter_here(timeout_ack=3) is True
    assert plane_drone.wait_mode_changed("LOITER", timeout=5) is True

    # Re-issue do_reposition to wp2 and verify proximity
    assert plane_drone.do_reposition(
        latitude=wp2.latitude,
        longitude=wp2.longitude,
        altitude=60.0,
        loiter_radius=65.0,
        ccw=True,
        force_guided=True,
        timeout_ack=3,
    ) is True
    assert plane_drone.wait_mode_changed("GUIDED", timeout=5) is True

    assert plane_drone.wait_proximity_with_gps_pos(
        lat=wp2.latitude,
        lon=wp2.longitude,
        radius_acceptance=95.0,
        timeout=60,
    ) is True

    # Test do_return_path_start: joins return-path leg (Seq 5) and switches back to AUTO mode
    assert plane_drone.do_return_path_start(timeout_ack=3) is True
    assert plane_drone.wait_mode_changed("AUTO", timeout=5) is True

    # Test set_current_waypoint: jump explicitly to Seq 3 (wp2), and verify out-of-range is denied
    assert plane_drone.set_current_waypoint(99, timeout_ack=3) is False
    assert plane_drone.set_current_waypoint(3, timeout_ack=3) is True
    start_seq_wait = time.time()
    while time.time() - start_seq_wait < 5:
        if plane_drone.current_waypoint_seq() == 3:
            break
        time.sleep(0.1)
    assert plane_drone.current_waypoint_seq() == 3

    # Test do_land_start: jumps directly to the landing sequence (Seq 6 -> Seq 7 NAV_LAND)
    assert plane_drone.do_land_start(timeout_ack=3) is True
    assert plane_drone.wait_mode_changed("AUTO", timeout=5) is True

    # Wait until ArduPlane enters the active NAV_LAND item (Seq 7), then abort with do_go_around
    start_land_wait = time.time()
    while time.time() - start_land_wait < 10:
        if plane_drone.current_waypoint_seq() == 7:
            break
        time.sleep(0.1)
    assert plane_drone.current_waypoint_seq() == 7

    assert plane_drone.do_go_around(altitude=55.0, timeout_ack=3) is True

    # Trigger Return-To-Launch (RTL) and verify mode change
    assert plane_drone.return_to_launch(timeout_ack=3) is True
    assert plane_drone.wait_mode_changed("RTL", timeout=5) is True

    # Test land() on Fixed-Wing WITH a DO_LAND_START mission item: triggers DO_LAND_START -> AUTO
    assert plane_drone.land(timeout_ack=3, timeout_set_mode=5, timeout_land=0) is False
    assert plane_drone.mode() == "AUTO"

    # Test land() on Fixed-Wing WITHOUT DO_LAND_START: falls back to AUTOLAND mode (ArduPlane 4.6+)
    assert plane_drone.set_mode("GUIDED", timeout_ack=3, timeout_set_mode=5) is True
    assert plane_drone.clear_waypoints(timeout_ack=3) is True
    assert plane_drone.land(timeout_ack=3, timeout_set_mode=5, timeout_land=0) is False
    assert plane_drone.mode() == "AUTOLAND"

    # Force disarm to conclude test cleanly
    assert plane_drone.disarm(force=True, timeout_ack=3, timeout_disarming=10) is True
    assert plane_drone.motors_disarmed() is True
