#!/usr/bin/env python3
"""
wp_mark.py
Connects to the rover via ROS 2/MAVROS and, on each waypoint reached event,
performs the sequence:
  1) Log "Msg Arrived" (with seq)
  2) delay = wp_mark.delay_before_on seconds
  3) DO_SET_SERVO (servo_number, pwm_on)
  4) delay = wp_mark.spray_duration seconds
  5) DO_SET_SERVO (servo_number, pwm_off)
  6) delay = wp_mark.delay_after_off seconds
  7) Wait for next waypoint (if exists)

Delays and PWM values are read from Backend/servo_manager/config.json under
the "wp_mark" section. Supported keys:
  - delay_before_on (float seconds)
  - spray_duration (float seconds)
  - delay_after_off (float seconds)
  - servo_number (int, optional; default: 10)
  - pwm_on (int, optional; default: 650)
  - pwm_off (int, optional; default: 1000)

This script is launched by the backend when the user presses Start in the
Servo Control tab. Output is captured to a log file by the backend.
"""

import json
import os
import signal
import sys
import time
from typing import Any, Dict

# Import ROS servo controller
from ros_servo import ServoController

BASE = os.path.dirname(__file__)
CFG = os.path.join(BASE, "config.json")

running = True
last_seq = -1


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
        "delay_before_on": float(cfg.get("delay_before_on", 0.5) or 0.5),
        "spray_duration": float(cfg.get("spray_duration", 1.5) or 1.5),
        "delay_after_off": float(cfg.get("delay_after_off", 0.5) or 0.5),
        "servo_number": int(cfg.get("servo_number", 10) or 10),
        "pwm_on": int(cfg.get("pwm_on", 650) or 650),
        "pwm_off": int(cfg.get("pwm_off", 1000) or 1000),
    }


def waypoint_callback(msg):
    """Callback for waypoint reached messages."""
    global last_seq, running
    
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


def main():
    global running
    
    print("[wp_mark] Starting ROS 2 servo controller...", flush=True)
    
    try:
        import rclpy
        from mavros_msgs.msg import WaypointReached
        
        if not rclpy.ok():
            rclpy.init()
        
        # Create servo controller
        controller = ServoController(use_ros=True)
        
        # Store controller in callback function
        waypoint_callback.controller = controller
        
        # Subscribe to waypoint reached topic
        print("[wp_mark] Subscribing to /mavros/mission/reached...", flush=True)
        subscription = controller.node.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            waypoint_callback,
            10
        )
        
        print("[wp_mark] Listening for waypoint reached events...", flush=True)
        
        # Spin in loop
        while running and rclpy.ok():
            rclpy.spin_once(controller.node, timeout_sec=0.5)
        
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
    """Fallback to MAVLink if ROS is not available."""
    global running, last_seq
    
    from pymavlink import mavutil
    
    CONNECTION_STRING = os.environ.get("WP_MARK_CONNECTION", "udp:127.0.0.1:14550")
    
    print("[wp_mark] Starting MAVLink mode...", flush=True)
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
    
    print("[wp_mark] Listening for MISSION_ITEM_REACHED messages...", flush=True)
    
    while running:
        try:
            # Use a short timeout so we can notice SIGTERM quickly
            msg = master.recv_match(type="MISSION_ITEM_REACHED", blocking=True, timeout=1.0)
        except Exception:
            msg = None
        
        if msg is None:
            continue
        
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
    
    print("[wp_mark] Exiting", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[wp_mark] Interrupted", flush=True)
