#!/usr/bin/env python3
"""
ros_servo.py
Helper module for sending servo commands via ROS 2 instead of direct MAVLink.
This integrates with the MAVROS command service.
"""

import os
import sys
import time
from typing import Optional

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

try:
    import rclpy
    from rclpy.node import Node
    from mavros_msgs.srv import CommandLong
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[ros_servo] WARNING: ROS 2 not available, falling back to MAVLink", flush=True)


class ServoController:
    """
    ROS 2-based servo controller using MAVROS CommandLong service.
    Falls back to pymavlink if ROS is not available.
    """
    
    MAV_CMD_DO_SET_SERVO = 183
    
    def __init__(self, use_ros: bool = True):
        """
        Initialize servo controller.
        
        Args:
            use_ros: If True, use ROS 2 (default). If False, use pymavlink.
        """
        self.use_ros = use_ros and ROS_AVAILABLE
        self.node: Optional[Node] = None
        self.client = None
        self.mavlink_master = None
        
        if self.use_ros:
            self._init_ros()
        else:
            self._init_mavlink()
    
    def _init_ros(self):
        """Initialize ROS 2 node and service client."""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = Node('servo_controller')
            self.client = self.node.create_client(
                CommandLong,
                '/mavros/cmd/command'
            )
            
            # Wait for service to be available
            print("[ros_servo] Waiting for /mavros/cmd/command service...", flush=True)
            timeout_sec = 10.0
            if not self.client.wait_for_service(timeout_sec=timeout_sec):
                print(f"[ros_servo] WARNING: Service not available after {timeout_sec}s", flush=True)
            else:
                print("[ros_servo] CommandLong service ready", flush=True)
                
        except Exception as e:
            print(f"[ros_servo] ERROR initializing ROS: {e}", flush=True)
            self.use_ros = False
            self._init_mavlink()
    
    def _init_mavlink(self):
        """Initialize pymavlink connection."""
        try:
            from pymavlink import mavutil
            connection_string = os.environ.get("SERVO_CONNECTION", "udp:127.0.0.1:14550")
            print(f"[ros_servo] Connecting via MAVLink to {connection_string}...", flush=True)
            self.mavlink_master = mavutil.mavlink_connection(connection_string)
            self.mavlink_master.wait_heartbeat(timeout=10)
            print(f"[ros_servo] MAVLink heartbeat from system {self.mavlink_master.target_system}", flush=True)
        except Exception as e:
            print(f"[ros_servo] ERROR initializing MAVLink: {e}", flush=True)
            raise
    
    def set_servo(self, servo_number: int, pwm_value: int, timeout_sec: float = 5.0) -> bool:
        """
        Set servo to specified PWM value.
        
        Args:
            servo_number: Servo channel (1-16)
            pwm_value: PWM in microseconds (typically 1000-2000)
            timeout_sec: Service call timeout
            
        Returns:
            True if command succeeded, False otherwise
        """
        if self.use_ros:
            return self._set_servo_ros(servo_number, pwm_value, timeout_sec)
        else:
            return self._set_servo_mavlink(servo_number, pwm_value)
    
    def _set_servo_ros(self, servo_number: int, pwm_value: int, timeout_sec: float) -> bool:
        """Send servo command via ROS service."""
        if not self.node or not self.client:
            print("[ros_servo] ERROR: ROS not initialized", flush=True)
            return False
        
        try:
            request = CommandLong.Request()
            request.broadcast = False
            request.command = self.MAV_CMD_DO_SET_SERVO
            request.confirmation = 0
            request.param1 = float(servo_number)
            request.param2 = float(pwm_value)
            request.param3 = 0.0
            request.param4 = 0.0
            request.param5 = 0.0
            request.param6 = 0.0
            request.param7 = 0.0
            
            future = self.client.call_async(request)
            
            # Spin until complete or timeout
            start_time = time.time()
            while not future.done():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if time.time() - start_time > timeout_sec:
                    print(f"[ros_servo] Service call timeout after {timeout_sec}s", flush=True)
                    return False
            
            response = future.result()
            if response.success:
                print(f"[ros_servo] DO_SET_SERVO ch={servo_number} pwm={pwm_value} ✓", flush=True)
                return True
            else:
                print(f"[ros_servo] DO_SET_SERVO failed: result={response.result}", flush=True)
                return False
                
        except Exception as e:
            print(f"[ros_servo] ERROR calling service: {e}", flush=True)
            return False
    
    def _set_servo_mavlink(self, servo_number: int, pwm_value: int) -> bool:
        """Send servo command via pymavlink."""
        if not self.mavlink_master:
            print("[ros_servo] ERROR: MAVLink not initialized", flush=True)
            return False
        
        try:
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                self.MAV_CMD_DO_SET_SERVO,
                0,
                float(servo_number),
                float(pwm_value),
                0, 0, 0, 0, 0
            )
            print(f"[ros_servo] DO_SET_SERVO ch={servo_number} pwm={pwm_value}", flush=True)
            return True
        except Exception as e:
            print(f"[ros_servo] ERROR sending MAVLink command: {e}", flush=True)
            return False
    
    def spin_once(self):
        """Process ROS callbacks (call this in your main loop if using ROS)."""
        if self.use_ros and self.node:
            rclpy.spin_once(self.node, timeout_sec=0.01)
    
    def shutdown(self):
        """Clean up resources."""
        if self.use_ros and self.node:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        
        if self.use_ros:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


def main():
    """Test the servo controller."""
    import argparse
    parser = argparse.ArgumentParser(description='Test servo control via ROS')
    parser.add_argument('--servo', type=int, default=10, help='Servo number')
    parser.add_argument('--pwm', type=int, default=1500, help='PWM value')
    parser.add_argument('--mavlink', action='store_true', help='Use MAVLink instead of ROS')
    args = parser.parse_args()
    
    controller = ServoController(use_ros=not args.mavlink)
    
    try:
        print(f"Setting servo {args.servo} to PWM {args.pwm}...")
        success = controller.set_servo(args.servo, args.pwm)
        print(f"Result: {'SUCCESS' if success else 'FAILED'}")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
