#!/usr/bin/env python3
"""
wp_mark.py - INDUSTRIAL-GRADE VERSION with ACTIVE WAYPOINT CONTROL
Connects to the rover via ROS 2/MAVROS and provides two operational modes:

MODE 1 - PASSIVE (Current): Waits for waypoint reached events (MISSION_ITEM_REACHED)
MODE 2 - ACTIVE (New): Script controls waypoint navigation and spraying sequence

ACTIVE MODE FEATURES:
- Reads mission waypoints proactively
- Navigates to each waypoint systematically  
- Performs spray sequence at each waypoint with industrial safety checks
- No dependency on external MISSION_ITEM_REACHED messages
- More reliable for industrial precision agriculture

INDUSTRIAL FEATURES:
- RTK Quality Verification (fix_type >= 5, eph <= 0.15m)
- Velocity Monitoring (prevents spraying when stopped)
- Mission State Monitoring (only sprays in AUTO mode)
- Emergency Stop System
- Spray Audit Logging with GPS quality metrics

Configuration (Backend/servo_manager/config.json → wp_mark):
  - mode (str: 'passive' | 'active', default 'passive')
  - delay_before_on (float seconds, default 0.5)
  - spray_duration (float seconds, default 1.5)
  - delay_after_off (float seconds, default 0.5)
  - waypoint_proximity_threshold (float meters, default 2.0)  # For active mode
  - servo_number (int, default 10)
  - pwm_on (int, default 650)
  - pwm_off (int, default 1000)

INDUSTRIAL REQUIREMENTS:
  - RTK fix_type >= 5 (RTK Float/Fixed)
  - GPS accuracy eph <= 0.15m (15cm)
  - Velocity >= 0.3 m/s (not stopped)
  - Mission mode = AUTO and armed
  - No emergency stop active
"""

import json
import math
import os
import signal
import sys
import time
from typing import Any, Dict, Optional, Tuple, List

# Import ROS servo controller
from ros_servo import ServoController

BASE = os.path.dirname(__file__)
CFG = os.path.join(BASE, "config.json")


class SprayLogger:
    """Industrial spray audit logger with GPS quality metrics."""

    def __init__(self, log_file: str = None):
        self.log_file = log_file or os.path.join(BASE, "logs", f"wp_mark_audit_{int(time.time())}.csv")
        self.events: List[Dict[str, Any]] = []
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)

        # Initialize CSV header
        with open(self.log_file, 'w') as f:
            f.write("timestamp,waypoint_seq,lat,lon,spray_duration_s,velocity_ms,fix_type,accuracy_m\n")

    def log_spray_event(self, waypoint_seq: int, position: Tuple[float, float], spray_time: float,
                       velocity: float, fix_type: int, eph: float):
        """Log each spray event with GPS quality metrics."""
        event = {
            'timestamp': time.time(),
            'waypoint_seq': waypoint_seq,
            'lat': position[0],
            'lon': position[1],
            'spray_duration_s': spray_time,
            'velocity_ms': velocity,
            'fix_type': fix_type,
            'accuracy_m': eph
        }
        self.events.append(event)

        # Write to CSV
        with open(self.log_file, 'a') as f:
            f.write(f"{event['timestamp']:.2f},{event['waypoint_seq']},{event['lat']:.7f},{event['lon']:.7f},"
                   f"{event['spray_duration_s']:.2f},{event['velocity_ms']:.2f},"
                   f"{event['fix_type']},{event['accuracy_m']:.3f}\n")

    def generate_report(self) -> Dict[str, Any]:
        """Generate spray coverage report."""
        if not self.events:
            return {"total_events": 0, "rtk_fixed_percentage": 0, "avg_accuracy_m": 0}

        rtk_fixed_count = sum(1 for e in self.events if e['fix_type'] == 6)
        avg_accuracy = sum(e['accuracy_m'] for e in self.events) / len(self.events)

        return {
            'total_events': len(self.events),
            'rtk_fixed_percentage': round((rtk_fixed_count / len(self.events) * 100), 1),
            'avg_accuracy_m': round(avg_accuracy, 3),
            'log_file': self.log_file
        }


def verify_rtk_quality(gps_data: Dict[str, Any]) -> Tuple[bool, str]:
    """Industrial RTK quality verification."""
    fix_type = gps_data.get('fix_type', 0)
    eph = gps_data.get('eph', 999) / 100.0  # cm to meters

    if fix_type < 5:  # Not RTK
        return False, f"RTK not active (fix_type={fix_type})"

    if eph > 0.15:  # More than 15cm error
        return False, f"GPS accuracy {eph:.2f}m exceeds 15cm threshold"

    return True, "RTK OK"


def verify_velocity(velocity: float) -> Tuple[bool, str]:
    """Verify rover is moving (not stopped)."""
    min_velocity = 0.3  # m/s

    if velocity < min_velocity:
        return False, f"Velocity {velocity:.2f}m/s too low (<{min_velocity}m/s)"

    return True, f"Velocity OK ({velocity:.2f}m/s)"


def verify_mission_state() -> Tuple[bool, str]:
    """Verify mission is active and in AUTO mode."""
    global mission_active

    if not mission_active:
        return False, "Mission not active"

    return True, "Mission active"


def should_spray() -> Tuple[bool, str]:
    """Industrial-grade spray decision logic."""
    global emergency_stop, current_gps_data

    # 1. Emergency stop check
    if emergency_stop:
        return False, "EMERGENCY STOP ACTIVE"

    # 2. Mission state check
    mission_ok, mission_msg = verify_mission_state()
    if not mission_ok:
        return False, mission_msg

    # 3. RTK quality check
    rtk_ok, rtk_msg = verify_rtk_quality(current_gps_data)
    if not rtk_ok:
        return False, rtk_msg

    # 4. Velocity check
    velocity = current_gps_data.get('velocity', 0)
    vel_ok, vel_msg = verify_velocity(velocity)
    if not vel_ok:
        return False, vel_msg

    return True, "All checks passed"


def emergency_stop_trigger():
    """Trigger emergency stop - can be called from frontend."""
    global emergency_stop
    emergency_stop = True
    print("[EMERGENCY] WP Mark spray system stopped", flush=True)


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate haversine distance between two GPS coordinates in meters."""
    R = 6371000  # Earth radius in meters
    
    lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
    lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
    
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c


def read_mission_waypoints(node) -> List[Dict[str, Any]]:
    """Read mission waypoints using MAVROS mission service."""
    try:
        from mavros_msgs.srv import WaypointPull, WaypointSetCurrent
        
        # Create service clients
        pull_client = node.create_client(WaypointPull, '/mavros/mission/pull')
        set_current_client = node.create_client(WaypointSetCurrent, '/mavros/mission/set_current')
        
        # Wait for services
        if not pull_client.wait_for_service(timeout_sec=5.0):
            print("[wp_mark] Mission pull service not available", flush=True)
            return []
        
        # Pull mission
        pull_request = WaypointPull.Request()
        future = pull_client.call_async(pull_request)
        
        # Wait for response
        import rclpy
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        
        if not future.done() or not future.result():
            print("[wp_mark] Failed to pull mission", flush=True)
            return []
        
        response = future.result()
        waypoints = []
        
        # Extract NAV waypoints (command 16 = MAV_CMD_NAV_WAYPOINT)
        for wp in response.mission.waypoints:
            if wp.command == 16:  # MAV_CMD_NAV_WAYPOINT
                waypoints.append({
                    'seq': wp.seq,
                    'lat': wp.x_lat / 1e7,  # degrees
                    'lon': wp.y_long / 1e7,  # degrees
                    'alt': wp.z_alt,  # meters
                    'param1': wp.param1,  # Hold time
                })
        
        print(f"[wp_mark] Loaded {len(waypoints)} NAV waypoints from mission", flush=True)
        return waypoints
        
    except Exception as e:
        print(f"[wp_mark] Error reading mission waypoints: {e}", flush=True)
        return []


def set_current_waypoint(node, wp_seq: int) -> bool:
    """Set the current waypoint index."""
    try:
        from mavros_msgs.srv import WaypointSetCurrent
        
        set_current_client = node.create_client(WaypointSetCurrent, '/mavros/mission/set_current')
        
        if not set_current_client.wait_for_service(timeout_sec=5.0):
            print("[wp_mark] Set current waypoint service not available", flush=True)
            return False
        
        request = WaypointSetCurrent.Request()
        request.wp_seq = wp_seq
        
        future = set_current_client.call_async(request)
        
        import rclpy
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        
        if future.done() and future.result():
            print(f"[wp_mark] Set current waypoint to {wp_seq}", flush=True)
            return future.result().success
        else:
            print(f"[wp_mark] Failed to set current waypoint to {wp_seq}", flush=True)
            return False
            
    except Exception as e:
        print(f"[wp_mark] Error setting current waypoint: {e}", flush=True)
        return False


def perform_spray_sequence(controller, params: Dict[str, Any], waypoint_seq: int, 
                          spray_logger: Optional["SprayLogger"]) -> bool:
    """Perform the complete spray sequence at a waypoint with industrial safety checks."""
    
    # INDUSTRIAL: Verify all safety conditions before spraying
    can_spray, reason = should_spray()
    
    if not can_spray:
        print(f"[wp_mark] SPRAY BLOCKED @ WP{waypoint_seq} | {reason}", flush=True)
        return False
    
    print(f"[wp_mark] SPRAY APPROVED @ WP{waypoint_seq} | {reason}", flush=True)
    
    # 1) delay before ON
    t = params["delay_before_on"]
    if t > 0:
        print(f"[wp_mark] WP{waypoint_seq}: Delaying {t}s before spray start", flush=True)
        time.sleep(t)
    
    # 2) ON
    controller.set_servo(params["servo_number"], params["pwm_on"])
    print(f"[wp_mark] WP{waypoint_seq}: Servo ON (PWM {params['pwm_on']})", flush=True)
    
    # 3) spray duration
    t = params["spray_duration"]
    if t > 0:
        print(f"[wp_mark] WP{waypoint_seq}: Spraying for {t}s", flush=True)
        time.sleep(t)
    
    # 4) OFF
    controller.set_servo(params["servo_number"], params["pwm_off"])
    print(f"[wp_mark] WP{waypoint_seq}: Servo OFF (PWM {params['pwm_off']})", flush=True)
    
    # 5) delay after OFF
    t = params["delay_after_off"]
    if t > 0:
        print(f"[wp_mark] WP{waypoint_seq}: Delaying {t}s after spray end", flush=True)
        time.sleep(t)
    
    # INDUSTRIAL: Log spray event with GPS metrics
    if spray_logger and current_gps_data:
        lat = current_gps_data.get('lat', 0)
        lon = current_gps_data.get('lon', 0)
        velocity = current_gps_data.get('velocity', 0)
        fix_type = current_gps_data.get('fix_type', 0)
        eph = current_gps_data.get('eph', 999) / 100.0
        
        spray_logger.log_spray_event(waypoint_seq, (lat, lon), params["spray_duration"], 
                                   velocity, fix_type, eph)
        
        print(f"[wp_mark] LOGGED: WP{waypoint_seq} @ {lat:.7f},{lon:.7f} | Duration: {params['spray_duration']}s | GPS: fix_type={fix_type}, acc={eph:.3f}m", flush=True)
    
    return True


def run_active_mode(controller, params: Dict[str, Any], spray_logger: Optional["SprayLogger"]) -> None:
    """Run active waypoint mode - script controls waypoint navigation and spraying."""
    global running, current_gps_data
    
    print("[wp_mark] Starting ACTIVE waypoint mode with industrial control", flush=True)
    
    try:
        import rclpy
        from mavros_msgs.srv import WaypointSetCurrent
        
        node = controller.node
        
        # Read mission waypoints
        waypoints = read_mission_waypoints(node)
        if not waypoints:
            print("[wp_mark] No waypoints found in mission - exiting active mode", flush=True)
            return
        
        print(f"[wp_mark] Starting active waypoint sequence with {len(waypoints)} waypoints", flush=True)
        
        # Process each waypoint
        for i, wp in enumerate(waypoints):
            if not running:
                break
                
            print(f"[wp_mark] Processing waypoint {i+1}/{len(waypoints)}: WP{wp['seq']} @ {wp['lat']:.7f},{wp['lon']:.7f}", flush=True)
            
            # Set this waypoint as current
            if not set_current_waypoint(node, wp['seq']):
                print(f"[wp_mark] Failed to set waypoint {wp['seq']} as current - skipping", flush=True)
                continue
            
            # Wait for rover to reach waypoint proximity
            print(f"[wp_mark] Waiting to reach waypoint {wp['seq']} (threshold: {params['waypoint_proximity_threshold']}m)", flush=True)
            
            reached = False
            while running and not reached:
                if current_gps_data.get('lat') and current_gps_data.get('lon'):
                    distance = haversine_distance(
                        current_gps_data['lat'], current_gps_data['lon'],
                        wp['lat'], wp['lon']
                    )
                    
                    if distance <= params['waypoint_proximity_threshold']:
                        reached = True
                        print(f"[wp_mark] Reached waypoint {wp['seq']} (distance: {distance:.2f}m)", flush=True)
                    else:
                        print(f"[wp_mark] Distance to WP{wp['seq']}: {distance:.2f}m", flush=True)
                        time.sleep(1.0)  # Check every second
                else:
                    print("[wp_mark] Waiting for GPS data...", flush=True)
                    time.sleep(1.0)
            
            if not running:
                break
            
            # Perform spray sequence at waypoint
            if not perform_spray_sequence(controller, params, wp['seq'], spray_logger):
                print(f"[wp_mark] Spray sequence failed at waypoint {wp['seq']}", flush=True)
            
            # Move to next waypoint (if not the last one)
            if i < len(waypoints) - 1:
                next_wp = waypoints[i + 1]
                print(f"[wp_mark] Moving to next waypoint: WP{next_wp['seq']}", flush=True)
                if not set_current_waypoint(node, next_wp['seq']):
                    print(f"[wp_mark] Failed to advance to waypoint {next_wp['seq']}", flush=True)
        
        print("[wp_mark] Active waypoint sequence completed", flush=True)
        
    except Exception as e:
        print(f"[wp_mark] Error in active mode: {e}", flush=True)


def gps_callback(msg):
    """Callback for GPS data updates."""
    global current_gps_data
    try:
        current_gps_data = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'fix_type': msg.fix_type,
            'eph': msg.eph,  # cm
            'epv': msg.epv,  # cm
            'vel': msg.vel,  # cm/s
            'cog': msg.cog,  # centidegrees
            'satellites_visible': msg.satellites_visible,
            'velocity': msg.vel / 100.0 if msg.vel else 0  # m/s
        }
    except Exception as e:
        print(f"[wp_mark] GPS callback error: {e}", flush=True)


def state_callback(msg):
    """Callback for MAVROS state updates."""
    global mission_active
    try:
        # Check if armed and in AUTO mode
        armed = msg.armed
        mode = msg.mode

        # Mission is active if armed and in AUTO mode
        mission_active = armed and mode == "AUTO"

        if mission_active:
            print(f"[wp_mark] Mission active: ARMED + AUTO mode", flush=True)
        else:
            print(f"[wp_mark] Mission inactive: armed={armed}, mode={mode}", flush=True)

    except Exception as e:
        print(f"[wp_mark] State callback error: {e}", flush=True)


running = True
last_seq = -1
emergency_stop = False
mission_active = False
current_gps_data: Dict[str, Any] = {}
spray_logger: Optional["SprayLogger"] = None
emergency_stop = False
mission_active = False
current_gps_data: Dict[str, Any] = {}
spray_logger: Optional["SprayLogger"] = None


def _sigterm(_sig, _frm):
    global running
    running = False


signal.signal(signal.SIGTERM, _sigterm)


def load_cfg() -> Dict[str, Any]:
    try:
        with open(CFG, "r") as f:
            return json.load(f)
    except Exception:
        return {}


def get_wp_mark_params() -> Dict[str, Any]:
    cfg = load_cfg().get("wp_mark", {})
    return {
        "mode": str(cfg.get("mode", "passive") or "passive"),  # 'passive' or 'active'
        "delay_before_on": float(cfg.get("delay_before_on", 0.5) or 0.5),
        "spray_duration": float(cfg.get("spray_duration", 1.5) or 1.5),
        "delay_after_off": float(cfg.get("delay_after_off", 0.5) or 0.5),
        "waypoint_proximity_threshold": float(cfg.get("waypoint_proximity_threshold", 2.0) or 2.0),
        "servo_number": int(cfg.get("servo_number", 10) or 10),
        "pwm_on": int(cfg.get("pwm_on", 650) or 650),
        "pwm_off": int(cfg.get("pwm_off", 1000) or 1000),
    }


def waypoint_callback(msg):
    """Callback for waypoint reached messages with industrial safety checks."""
    global last_seq, running, spray_logger
    
    if not running:
        return
    
    try:
        seq = int(msg.wp_seq)
    except Exception:
        seq = -1
    
    if seq < 0 or seq == last_seq:
        # Ignore invalid/duplicate
        return
    last_seq = seq
    
    params = get_wp_mark_params()
    print(
        f"[wp_mark] Msg Arrived: WP seq={seq} | delays=({params['delay_before_on']}, {params['spray_duration']}, {params['delay_after_off']}) | servo={params['servo_number']} pwm_on={params['pwm_on']} pwm_off={params['pwm_off']}",
        flush=True,
    )
    
    controller = getattr(waypoint_callback, 'controller', None)
    if not controller:
        print("[wp_mark] ERROR: Controller not initialized", flush=True)
        return
    
    # INDUSTRIAL: Verify all safety conditions before spraying
    can_spray, reason = should_spray()
    
    if not can_spray:
        print(f"[wp_mark] SPRAY BLOCKED @ WP{seq} | {reason}", flush=True)
        return
    
    print(f"[wp_mark] SPRAY APPROVED @ WP{seq} | {reason}", flush=True)
    
    # 2) delay before ON
    t = params["delay_before_on"]
    if t > 0:
        time.sleep(t)
    
    # 3) ON
    controller.set_servo(params["servo_number"], params["pwm_on"])
    
    # 4) spray duration
    t = params["spray_duration"]
    if t > 0:
        time.sleep(t)
    
    # 5) OFF
    controller.set_servo(params["servo_number"], params["pwm_off"])
    
    # 6) delay after OFF
    t = params["delay_after_off"]
    if t > 0:
        time.sleep(t)
    
    # INDUSTRIAL: Log spray event with GPS metrics
    if spray_logger and current_gps_data:
        lat = current_gps_data.get('lat', 0)
        lon = current_gps_data.get('lon', 0)
        velocity = current_gps_data.get('velocity', 0)
        fix_type = current_gps_data.get('fix_type', 0)
        eph = current_gps_data.get('eph', 999) / 100.0
        
        spray_logger.log_spray_event(seq, (lat, lon), params["spray_duration"], 
                                   velocity, fix_type, eph)
        
        print(f"[wp_mark] LOGGED: WP{seq} @ {lat:.7f},{lon:.7f} | Duration: {params['spray_duration']}s | GPS: fix_type={fix_type}, acc={eph:.3f}m", flush=True)


def main():
    global running, spray_logger
    
    params = get_wp_mark_params()
    mode = params.get("mode", "passive")
    
    print(f"[wp_mark] Starting INDUSTRIAL-GRADE ROS 2 servo controller (MODE: {mode.upper()})", flush=True)
    
    try:
        import rclpy
        from mavros_msgs.msg import WaypointReached, State
        from sensor_msgs.msg import NavSatFix
        
        if not rclpy.ok():
            rclpy.init()
        
        # Create servo controller
        controller = ServoController(use_ros=True)
        
        # Initialize spray logger
        spray_logger = SprayLogger()
        print(f"[wp_mark] Spray logger initialized: {spray_logger.log_file}", flush=True)
        
        # Store controller in callback function
        waypoint_callback.controller = controller
        
        if mode == "active":
            # ACTIVE MODE: Script controls waypoint navigation
            print("[wp_mark] Running in ACTIVE mode - script controls waypoint sequence", flush=True)
            
            # Subscribe to GPS and state for monitoring
            gps_subscription = controller.node.create_subscription(
                NavSatFix,
                '/mavros/global_position/global',
                gps_callback,
                10
            )
            
            state_subscription = controller.node.create_subscription(
                State,
                '/mavros/state',
                state_callback,
                10
            )
            
            # Run active waypoint control
            run_active_mode(controller, params, spray_logger)
            
        else:
            # PASSIVE MODE: Wait for waypoint reached events (original behavior)
            print("[wp_mark] Running in PASSIVE mode - waiting for waypoint reached events", flush=True)
            
            # Subscribe to waypoint reached topic
            wp_subscription = controller.node.create_subscription(
                WaypointReached,
                '/mavros/mission/reached',
                waypoint_callback,
                10
            )
            
            # INDUSTRIAL: Subscribe to GPS data for quality monitoring
            gps_subscription = controller.node.create_subscription(
                NavSatFix,
                '/mavros/global_position/global',
                gps_callback,
                10
            )
            
            # INDUSTRIAL: Subscribe to MAVROS state for mission monitoring
            state_subscription = controller.node.create_subscription(
                State,
                '/mavros/state',
                state_callback,
                10
            )
            
            print("[wp_mark] Listening for waypoint reached events with industrial safety checks...", flush=True)
            
            # Spin in loop
            while running and rclpy.ok():
                rclpy.spin_once(controller.node, timeout_sec=0.5)
        
        # Generate final report
        if spray_logger:
            report = spray_logger.generate_report()
            print(f"[wp_mark] FINAL REPORT: {report['total_events']} spray events, "
                  f"{report['rtk_fixed_percentage']}% RTK fixed, "
                  f"avg accuracy {report['avg_accuracy_m']:.3f}m", flush=True)
        
        print("[wp_mark] Shutting down...", flush=True)
        controller.shutdown()
        
    except ImportError as e:
        print(f"[wp_mark] ROS 2 not available: {e}", flush=True)
        print("[wp_mark] Falling back to MAVLink mode...", flush=True)
        main_mavlink()
    except Exception as e:
        print(f"[wp_mark] ERROR: {e}", flush=True)
        raise
def main_mavlink():
    """Fallback to MAVLink if ROS is not available - with industrial safety checks."""
    global running, last_seq, spray_logger
    
    params = get_wp_mark_params()
    mode = params.get("mode", "passive")
    
    from pymavlink import mavutil
    
    CONNECTION_STRING = os.environ.get("WP_MARK_CONNECTION", "udp:127.0.0.1:14550")
    
    print(f"[wp_mark] Starting INDUSTRIAL-GRADE MAVLink mode (MODE: {mode.upper()})...", flush=True)
    
    # Initialize spray logger
    spray_logger = SprayLogger()
    print(f"[wp_mark] Spray logger initialized: {spray_logger.log_file}", flush=True)
    
    master = None
    
    # Try to connect and wait for heartbeat
    while running and master is None:
        try:
            master = mavutil.mavlink_connection(CONNECTION_STRING)
            master.wait_heartbeat(timeout=10)
            print(
                f"[wp_mark] Heartbeat received from system {master.target_system} component {master.target_component}",
                flush=True,
            )
        except Exception as e:
            print(f"[wp_mark] Connection failed: {e} (retrying in 2s)", flush=True)
            master = None
            time.sleep(2)
    
    if not running or master is None:
        print("[wp_mark] Exiting (not running or no connection)", flush=True)
        return
    
    if mode == "active":
        print("[wp_mark] ACTIVE mode not fully supported in MAVLink fallback - using PASSIVE mode", flush=True)
        mode = "passive"  # Fallback to passive mode for MAVLink
    
    print(f"[wp_mark] Listening for MISSION_ITEM_REACHED messages with industrial safety checks (PASSIVE mode)...", flush=True)
    
    while running:
        try:
            # Use a short timeout so we can notice SIGTERM quickly
            msg = master.recv_match(type=["MISSION_ITEM_REACHED", "GPS_RAW_INT", "HEARTBEAT"], blocking=True, timeout=1.0)
        except Exception:
            msg = None
        
        if msg is None:
            continue
        
        # INDUSTRIAL: Handle GPS and heartbeat messages for safety monitoring
        if msg.get_type() == "GPS_RAW_INT":
            # Update GPS data
            global current_gps_data
            try:
                current_gps_data = {
                    'lat': msg.lat / 1e7,  # degrees
                    'lon': msg.lon / 1e7,  # degrees
                    'alt': msg.alt / 1000.0,  # meters
                    'fix_type': msg.fix_type,
                    'eph': msg.eph,  # cm
                    'epv': msg.epv,  # cm
                    'vel': msg.vel / 100.0,  # m/s
                    'cog': msg.cog,  # centidegrees
                    'satellites_visible': msg.satellites_visible,
                    'velocity': msg.vel / 100.0 if msg.vel else 0  # m/s
                }
            except Exception as e:
                print(f"[wp_mark] GPS parse error: {e}", flush=True)
            continue
            
        elif msg.get_type() == "HEARTBEAT":
            # Update mission state
            global mission_active
            try:
                armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                mode = msg.custom_mode
                # For ArduPilot, AUTO mode is typically mode 3
                auto_mode = mode == 3
                mission_active = armed and auto_mode
                
                if mission_active:
                    print(f"[wp_mark] Mission active: ARMED + AUTO mode", flush=True)
                else:
                    print(f"[wp_mark] Mission inactive: armed={armed}, mode={mode}", flush=True)
            except Exception as e:
                print(f"[wp_mark] Heartbeat parse error: {e}", flush=True)
            continue
        
        elif msg.get_type() == "MISSION_ITEM_REACHED":
            # Handle waypoint reached
            try:
                seq = int(getattr(msg, "seq", -1))
            except Exception:
                seq = -1
            
            if seq < 0 or seq == last_seq:
                # Ignore invalid/duplicate
                continue
            last_seq = seq
            
            params = get_wp_mark_params()
            print(
                f"[wp_mark] Msg Arrived: WP seq={seq} | delays=({params['delay_before_on']}, {params['spray_duration']}, {params['delay_after_off']}) | servo={params['servo_number']} pwm_on={params['pwm_on']} pwm_off={params['pwm_off']}",
                flush=True,
            )
            
            # INDUSTRIAL: Verify all safety conditions before spraying
            can_spray, reason = should_spray()
            
            if not can_spray:
                print(f"[wp_mark] SPRAY BLOCKED @ WP{seq} | {reason}", flush=True)
                continue
            
            print(f"[wp_mark] SPRAY APPROVED @ WP{seq} | {reason}", flush=True)
            
            # 2) delay before ON
            t = params["delay_before_on"]
            if t > 0:
                time.sleep(t)
            
            # 3) ON
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                183,  # MAV_CMD_DO_SET_SERVO
                0,
                float(params["servo_number"]),
                float(params["pwm_on"]),
                0, 0, 0, 0, 0,
            )
            print(f"[wp_mark] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_on']}", flush=True)
            
            # 4) spray duration
            t = params["spray_duration"]
            if t > 0:
                time.sleep(t)
            
            # 5) OFF
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                183,  # MAV_CMD_DO_SET_SERVO
                0,
                float(params["servo_number"]),
                float(params["pwm_off"]),
                0, 0, 0, 0, 0,
            )
            print(f"[wp_mark] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_off']}", flush=True)
            
            # 6) delay after OFF
            t = params["delay_after_off"]
            if t > 0:
                time.sleep(t)
            
            # INDUSTRIAL: Log spray event with GPS metrics
            if spray_logger and current_gps_data:
                lat = current_gps_data.get('lat', 0)
                lon = current_gps_data.get('lon', 0)
                velocity = current_gps_data.get('velocity', 0)
                fix_type = current_gps_data.get('fix_type', 0)
                eph = current_gps_data.get('eph', 999) / 100.0
                
                spray_logger.log_spray_event(seq, (lat, lon), params["spray_duration"], 
                                           velocity, fix_type, eph)
                
                print(f"[wp_mark] LOGGED: WP{seq} @ {lat:.7f},{lon:.7f} | Duration: {params['spray_duration']}s | GPS: fix_type={fix_type}, acc={eph:.3f}m", flush=True)
    
    # Generate final report
    if spray_logger:
        report = spray_logger.generate_report()
        print(f"[wp_mark] FINAL REPORT: {report['total_events']} spray events, "
              f"{report['rtk_fixed_percentage']}% RTK fixed, "
              f"avg accuracy {report['avg_accuracy_m']:.3f}m", flush=True)
    
    print("[wp_mark] Exiting", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[wp_mark] Interrupted", flush=True)
