#!/usr/bin/env python3
"""
interval_spray.py - INDUSTRIAL-GRADE VERSION
Connects to ROS 2/MAVROS and toggles a servo ON/OFF at either time-based or
distance-based intervals while the mission runs. Includes RTK verification,
velocity monitoring, and mission state checks for agricultural precision.

INDUSTRIAL FEATURES:
- RTK Quality Verification (fix_type >= 5, eph <= 0.15m)
- Velocity Monitoring (prevents spraying when stopped)
- Mission State Monitoring (only sprays in AUTO mode)
- Emergency Stop System
- Spray Audit Logging with GPS quality metrics

Configuration (Backend/servo_manager/config.json → interval_spray):
  - servo_number (int, default 10)
  - pwm_on (int, default 650)
  - pwm_off (int, default 1000)
  - toggle_mode (str: 'timer' | 'distance', default 'timer')
  - on_time_s (float, for timer mode; default 1.0)
  - off_time_s (float, for timer mode; default 1.0)
  - distance_interval_m (float, for distance mode; default 1.0)
  - start_wp (int, optional; default -1 → start immediately)
  - end_wp (int, optional; default -1 → run until stopped)

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

running = True
active = False  # spraying window active (between start/end)
servo_state_on = False
last_toggle_time = time.monotonic()
acc_dist = 0.0
last_pos: Optional[Tuple[float, float]] = None
last_seq = -1

# Industrial-grade state variables
emergency_stop = False
mission_active = False
current_gps_data = {}  # RTK status, velocity, accuracy
spray_logger = None


class SprayLogger:
    """Industrial spray audit logger with GPS quality metrics."""

    def __init__(self, log_file: str = None):
        self.log_file = log_file or os.path.join(BASE, "logs", f"spray_audit_{int(time.time())}.csv")
        self.events: List[Dict[str, Any]] = []
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)

        # Initialize CSV header
        with open(self.log_file, 'w') as f:
            f.write("timestamp,lat,lon,spray_duration_s,velocity_ms,fix_type,accuracy_m,coverage_estimate_m\n")

    def log_spray_event(self, position: Tuple[float, float], spray_time: float,
                       velocity: float, fix_type: int, eph: float):
        """Log each spray event with GPS quality metrics."""
        event = {
            'timestamp': time.time(),
            'lat': position[0],
            'lon': position[1],
            'spray_duration_s': spray_time,
            'velocity_ms': velocity,
            'fix_type': fix_type,
            'accuracy_m': eph,
            'coverage_estimate_m': spray_time * velocity
        }
        self.events.append(event)

        # Write to CSV
        with open(self.log_file, 'a') as f:
            f.write(f"{event['timestamp']:.2f},{event['lat']:.7f},{event['lon']:.7f},"
                   f"{event['spray_duration_s']:.2f},{event['velocity_ms']:.2f},"
                   f"{event['fix_type']},{event['accuracy_m']:.3f},{event['coverage_estimate_m']:.3f}\n")

    def generate_report(self) -> Dict[str, Any]:
        """Generate spray coverage report."""
        if not self.events:
            return {"total_events": 0, "total_coverage_m": 0, "rtk_fixed_percentage": 0, "avg_accuracy_m": 0}

        total_coverage = sum(e['coverage_estimate_m'] for e in self.events)
        rtk_fixed_count = sum(1 for e in self.events if e['fix_type'] == 6)
        avg_accuracy = sum(e['accuracy_m'] for e in self.events) / len(self.events)

        return {
            'total_events': len(self.events),
            'total_coverage_m': round(total_coverage, 2),
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
    global emergency_stop, servo_state_on
    emergency_stop = True

    if servo_state_on:
        # Immediately turn off servo
        params = get_params()
        controller = getattr(emergency_stop_trigger, 'controller', None)
        if controller:
            controller.set_servo(params["servo_number"], params["pwm_off"])
            servo_state_on = False

    print("[EMERGENCY] Spray system stopped", flush=True)


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


def get_params() -> Dict[str, Any]:
    cfg = load_cfg().get("interval_spray", {})
    toggle_mode = str(cfg.get("toggle_mode", "timer")).strip().lower() or "timer"
    return {
        "servo_number": int(cfg.get("servo_number", 10) or 10),
        "pwm_on": int(cfg.get("pwm_on", 650) or 650),
        "pwm_off": int(cfg.get("pwm_off", 1000) or 1000),
        "toggle_mode": toggle_mode,
        "on_time_s": float(cfg.get("on_time_s", cfg.get("spray_on_time", 1.0)) or 1.0),
        "off_time_s": float(cfg.get("off_time_s", cfg.get("interval_time", 1.0)) or 1.0),
        "distance_interval_m": float(cfg.get("distance_interval_m", 1.0) or 1.0),
        "start_wp": int(cfg.get("start_wp", -1) or -1),
        "end_wp": int(cfg.get("end_wp", -1) or -1),
    }


def _matches_seq(reached_seq: int, wp_config: int) -> bool:
    return reached_seq == wp_config or reached_seq == wp_config - 1


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def waypoint_callback(msg):
    """Callback for waypoint reached messages."""
    global active, servo_state_on, last_seq, last_toggle_time, acc_dist, last_pos, running
    
    if not running:
        return
    
    try:
        seq = int(msg.wp_seq)
    except Exception:
        seq = -1
    
    if seq >= 0 and seq != last_seq:
        last_seq = seq
        params = get_params()
        controller = getattr(waypoint_callback, 'controller', None)
        
        if not active and params["start_wp"] >= 0 and _matches_seq(seq, params["start_wp"]):
            print(f"[interval_spray] Start window @seq={seq}", flush=True)
            active = True
            # reset timers/counters at window start
            last_toggle_time = time.monotonic()
            acc_dist = 0.0
            last_pos = None
            servo_state_on = False
        
        if active and params["end_wp"] >= 0 and _matches_seq(seq, params["end_wp"]):
            print(f"[interval_spray] End window @seq={seq}", flush=True)
            if servo_state_on and controller:
                controller.set_servo(params["servo_number"], params["pwm_off"])
                servo_state_on = False
            active = False


def position_callback(msg):
    """Callback for position updates (for distance mode) - INDUSTRIAL VERSION."""
    global active, servo_state_on, acc_dist, last_pos, running, spray_logger

    if not running or not active:
        return

    params = get_params()
    if params["toggle_mode"] != "distance":
        return

    controller = getattr(position_callback, 'controller', None)
    if not controller:
        return

    try:
        lat = msg.latitude
        lon = msg.longitude

        if lat == 0.0 and lon == 0.0:
            return

        if last_pos is not None:
            acc_dist += _haversine_m(last_pos[0], last_pos[1], lat, lon)
        last_pos = (lat, lon)

        if acc_dist >= params["distance_interval_m"]:
            # INDUSTRIAL: Verify all conditions before spraying
            can_spray, reason = should_spray()

            if can_spray:
                # Calculate spray time based on velocity
                velocity = current_gps_data.get('velocity', 0)
                spray_time = params["distance_interval_m"] / velocity if velocity > 0 else 0

                if servo_state_on:
                    controller.set_servo(params["servo_number"], params["pwm_off"])
                    servo_state_on = False
                    print(f"[SPRAY] OFF @ {lat:.7f},{lon:.7f} | {reason}", flush=True)
                else:
                    controller.set_servo(params["servo_number"], params["pwm_on"])
                    servo_state_on = True
                    print(f"[SPRAY] ON @ {lat:.7f},{lon:.7f} | Duration: {spray_time:.2f}s | {reason}", flush=True)

                    # Log spray event
                    if spray_logger and last_pos:
                        spray_logger.log_spray_event(
                            last_pos, spray_time, velocity,
                            current_gps_data.get('fix_type', 0),
                            current_gps_data.get('eph', 0) / 100.0
                        )
            else:
                # Cannot spray - log reason and ensure OFF
                print(f"[SPRAY] BLOCKED @ {lat:.7f},{lon:.7f} | {reason}", flush=True)
                if servo_state_on:
                    controller.set_servo(params["servo_number"], params["pwm_off"])
                    servo_state_on = False

            acc_dist = 0.0

    except Exception as e:
        print(f"[interval_spray] Position callback error: {e}", flush=True)


def timer_check(controller, params):
    """Check and toggle servo based on time (for timer mode) - INDUSTRIAL VERSION."""
    global active, servo_state_on, last_toggle_time, spray_logger

    if not active or params["toggle_mode"] != "timer":
        return

    now = time.monotonic()

    if servo_state_on and now - last_toggle_time >= params["on_time_s"]:
        # Turn OFF
        controller.set_servo(params["servo_number"], params["pwm_off"])
        servo_state_on = False
        last_toggle_time = now
        print(f"[SPRAY] TIMER OFF | Duration: {params['on_time_s']}s", flush=True)

    elif not servo_state_on and now - last_toggle_time >= params["off_time_s"]:
        # INDUSTRIAL: Verify all conditions before turning ON
        can_spray, reason = should_spray()

        if can_spray:
            controller.set_servo(params["servo_number"], params["pwm_on"])
            servo_state_on = True
            last_toggle_time = now
            print(f"[SPRAY] TIMER ON | Duration: {params['off_time_s']}s | {reason}", flush=True)

            # Log spray event
            if spray_logger and last_pos:
                velocity = current_gps_data.get('velocity', 0)
                spray_logger.log_spray_event(
                    last_pos, params['off_time_s'], velocity,
                    current_gps_data.get('fix_type', 0),
                    current_gps_data.get('eph', 0) / 100.0
                )
        else:
            # Cannot spray - log reason and stay OFF
            print(f"[SPRAY] TIMER BLOCKED | {reason}", flush=True)
            last_toggle_time = now  # Reset timer to try again later


def main():
    global running, active, servo_state_on, spray_logger, emergency_stop

    params = get_params()
    print(f"[interval_spray] Starting INDUSTRIAL-GRADE ROS mode with params: {params}", flush=True)

    # Initialize industrial features
    spray_logger = SprayLogger()
    emergency_stop = False  # Reset emergency stop

    print(f"[SPRAY] Industrial features initialized:", flush=True)
    print(f"  - RTK verification: fix_type >= 5, eph <= 15cm", flush=True)
    print(f"  - Velocity monitoring: >= 0.3 m/s", flush=True)
    print(f"  - Mission state monitoring: AUTO mode only", flush=True)
    print(f"  - Audit logging: {spray_logger.log_file}", flush=True)

    if params["start_wp"] < 0:
        active = True

    try:
        import rclpy
        from mavros_msgs.msg import WaypointReached, State
        from sensor_msgs.msg import NavSatFix

        if not rclpy.ok():
            rclpy.init()

        # Create servo controller
        controller = ServoController(use_ros=True)

        # Store controller in callback functions and emergency stop
        waypoint_callback.controller = controller
        position_callback.controller = controller
        emergency_stop_trigger.controller = controller

        # Subscribe to waypoint reached topic
        print("[interval_spray] Subscribing to /mavros/mission/reached...", flush=True)
        wp_subscription = controller.node.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            waypoint_callback,
            10
        )

        # INDUSTRIAL: Subscribe to GPS raw data for RTK monitoring
        from mavros_msgs.msg import GPSRAW
        def gps_callback(msg):
            global current_gps_data
            current_gps_data = {
                'fix_type': msg.fix_type,
                'eph': msg.eph,  # cm
                'epv': msg.epv,  # cm
                'velocity': msg.vel / 100.0,  # cm/s to m/s
                'satellites_visible': msg.satellites_visible
            }

        print("[interval_spray] Subscribing to /mavros/gpsstatus/gps1/raw for RTK monitoring...", flush=True)
        gps_subscription = controller.node.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            gps_callback,
            10
        )

        # INDUSTRIAL: Subscribe to MAVROS state for mission monitoring
        def state_callback(msg):
            global mission_active
            mission_active = (msg.mode == 'AUTO' and msg.armed)
            if not mission_active and active:
                print(f"[MISSION] State changed: mode={msg.mode}, armed={msg.armed} - pausing spray", flush=True)

        print("[interval_spray] Subscribing to /mavros/state for mission monitoring...", flush=True)
        state_subscription = controller.node.create_subscription(
            State,
            '/mavros/state',
            state_callback,
            10
        )

        print(f"[interval_spray] Listening for events (mode={params['toggle_mode']})...", flush=True)

        # Spin in loop
        while running and rclpy.ok():
            rclpy.spin_once(controller.node, timeout_sec=0.1)
            # Check timer mode toggling
            timer_check(controller, params)

            # Check if we're done (both start and end reached)
            if not active and params["start_wp"] >= 0 and params["end_wp"] >= 0 and last_seq >= 0:
                break

        # Generate final report
        if spray_logger:
            report = spray_logger.generate_report()
            print(f"[SPRAY] Final Report: {report}", flush=True)

        # Ensure OFF on exit
        if servo_state_on:
            controller.set_servo(params["servo_number"], params["pwm_off"])

        print("[interval_spray] Shutting down...", flush=True)
        controller.shutdown()

    except ImportError as e:
        print(f"[interval_spray] ROS 2 not available: {e}", flush=True)
        print("[interval_spray] Falling back to MAVLink mode...", flush=True)
        main_mavlink()
    except Exception as e:
        print(f"[interval_spray] ERROR: {e}", flush=True)
        raise


def main_mavlink():
    """Fallback to MAVLink if ROS is not available."""
    global running, active, servo_state_on, last_seq, last_toggle_time, acc_dist, last_pos
    
    from pymavlink import mavutil
    
    CONNECTION_STRING = os.environ.get("INTERVAL_CONNECTION", "udp:127.0.0.1:14550")
    
    def _extract_latlon(msg) -> Optional[Tuple[float, float]]:
        try:
            if msg.get_type() == "GLOBAL_POSITION_INT":
                lat = getattr(msg, "lat", None)
                lon = getattr(msg, "lon", None)
                if lat is not None and lon is not None:
                    return float(lat) / 1e7, float(lon) / 1e7
            if msg.get_type() == "GPS_RAW_INT":
                lat = getattr(msg, "lat", None)
                lon = getattr(msg, "lon", None)
                if lat is not None and lon is not None and lat != 0 and lon != 0:
                    return float(lat) / 1e7, float(lon) / 1e7
        except Exception:
            pass
        return None
    
    params = get_params()
    print(f"[interval_spray] Starting MAVLink mode with params: {params}", flush=True)

    master = None
    while running and master is None:
        try:
            master = mavutil.mavlink_connection(CONNECTION_STRING)
            master.wait_heartbeat(timeout=10)
            print(
                f"[interval_spray] Heartbeat received from system {master.target_system} component {master.target_component}",
                flush=True,
            )
        except Exception as e:
            print(f"[interval_spray] Connection failed: {e} (retrying in 2s)", flush=True)
            master = None
            time.sleep(2)

    if not running or master is None:
        print("[interval_spray] Exiting (not running or no connection)", flush=True)
        return

    active_local = False  # spraying window active (between start/end)
    if params["start_wp"] < 0:
        active_local = True

    last_seq_local = -1
    servo_state_on_local = False
    last_toggle_time_local = time.monotonic()
    acc_dist_local = 0.0
    last_pos_local = None  # (lat, lon)

    print("[interval_spray] Listening for position and mission events...", flush=True)

    while running:
        try:
            msg = master.recv_match(type=[
                "MISSION_ITEM_REACHED", "GLOBAL_POSITION_INT", "GPS_RAW_INT"
            ], blocking=True, timeout=0.5)
        except Exception:
            msg = None

        if msg is None:
            # Still process time-based toggling even if no messages arrive
            if active_local and params["toggle_mode"] == "timer":
                now = time.monotonic()
                if servo_state_on_local and now - last_toggle_time_local >= params["on_time_s"]:
                    master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        183,  # MAV_CMD_DO_SET_SERVO
                        0,
                        float(params["servo_number"]),
                        float(params["pwm_off"]),
                        0, 0, 0, 0, 0,
                    )
                    print(f"[interval_spray] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_off']}", flush=True)
                    servo_state_on_local = False
                    last_toggle_time_local = now
                elif not servo_state_on_local and now - last_toggle_time_local >= params["off_time_s"]:
                    master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        183,  # MAV_CMD_DO_SET_SERVO
                        0,
                        float(params["servo_number"]),
                        float(params["pwm_on"]),
                        0, 0, 0, 0, 0,
                    )
                    print(f"[interval_spray] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_on']}", flush=True)
                    servo_state_on_local = True
                    last_toggle_time_local = now
            continue

        mtype = msg.get_type()

        # Start/end window control
        if mtype == "MISSION_ITEM_REACHED":
            try:
                seq = int(getattr(msg, "seq", -1))
            except Exception:
                seq = -1
            if seq >= 0 and seq != last_seq_local:
                last_seq_local = seq
                if not active_local and params["start_wp"] >= 0 and _matches_seq(seq, params["start_wp"]):
                    print(f"[interval_spray] Start window @seq={seq}", flush=True)
                    active_local = True
                    # reset timers/counters at window start
                    last_toggle_time_local = time.monotonic()
                    acc_dist_local = 0.0
                    last_pos_local = None
                    servo_state_on_local = False
                if active_local and params["end_wp"] >= 0 and _matches_seq(seq, params["end_wp"]):
                    print(f"[interval_spray] End window @seq={seq}", flush=True)
                    if servo_state_on_local:
                        master.mav.command_long_send(
                            master.target_system,
                            master.target_component,
                            183,  # MAV_CMD_DO_SET_SERVO
                            0,
                            float(params["servo_number"]),
                            float(params["pwm_off"]),
                            0, 0, 0, 0, 0,
                        )
                        print(f"[interval_spray] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_off']}", flush=True)
                        servo_state_on_local = False
                    active_local = False
                    # If both start and end are defined and reached, we can exit
                    if params["start_wp"] >= 0:
                        break
            continue

        # Position updates for distance toggling
        if active_local and params["toggle_mode"] == "distance":
            latlon = _extract_latlon(msg)
            if latlon is not None:
                if last_pos_local is not None:
                    acc_dist_local += _haversine_m(last_pos_local[0], last_pos_local[1], latlon[0], latlon[1])
                last_pos_local = latlon
                if acc_dist_local >= params["distance_interval_m"]:
                    # toggle
                    if servo_state_on_local:
                        master.mav.command_long_send(
                            master.target_system,
                            master.target_component,
                            183,  # MAV_CMD_DO_SET_SERVO
                            0,
                            float(params["servo_number"]),
                            float(params["pwm_off"]),
                            0, 0, 0, 0, 0,
                        )
                        print(f"[interval_spray] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_off']}", flush=True)
                        servo_state_on_local = False
                    else:
                        master.mav.command_long_send(
                            master.target_system,
                            master.target_component,
                            183,  # MAV_CMD_DO_SET_SERVO
                            0,
                            float(params["servo_number"]),
                            float(params["pwm_on"]),
                            0, 0, 0, 0, 0,
                        )
                        print(f"[interval_spray] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_on']}", flush=True)
                        servo_state_on_local = True
                    acc_dist_local = 0.0

        # Timer toggling handled also when messages are present
        if active_local and params["toggle_mode"] == "timer":
            now = time.monotonic()
            if servo_state_on_local and now - last_toggle_time_local >= params["on_time_s"]:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    183,  # MAV_CMD_DO_SET_SERVO
                    0,
                    float(params["servo_number"]),
                    float(params["pwm_off"]),
                    0, 0, 0, 0, 0,
                )
                print(f"[interval_spray] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_off']}", flush=True)
                servo_state_on_local = False
                last_toggle_time_local = now
            elif not servo_state_on_local and now - last_toggle_time_local >= params["off_time_s"]:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    183,  # MAV_CMD_DO_SET_SERVO
                    0,
                    float(params["servo_number"]),
                    float(params["pwm_on"]),
                    0, 0, 0, 0, 0,
                )
                print(f"[interval_spray] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_on']}", flush=True)
                servo_state_on_local = True
                last_toggle_time_local = now

    # Ensure OFF on exit
    if master and servo_state_on_local:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            183,  # MAV_CMD_DO_SET_SERVO
            0,
            float(params["servo_number"]),
            float(params["pwm_off"]),
            0, 0, 0, 0, 0,
        )
        print(f"[interval_spray] DO_SET_SERVO ch={params['servo_number']} pwm={params['pwm_off']}", flush=True)
    print("[interval_spray] Exiting", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[interval_spray] Interrupted", flush=True)
