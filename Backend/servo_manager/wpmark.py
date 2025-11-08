#!/usr/bin/env python3
"""
wpmark.py - Waypoint Marking Script

=== HOW IT WORKS (3 STEPS) ===

1. UPLOAD MISSION → Load waypoints into ArduPilot mission planner

2. SELECT WPMARK MODE → Choose "Waypoint Mark" from UI servo control tab

3. CLICK RUN → Script automatically:
   - Navigates to each waypoint in GUIDED mode
   - Stops at waypoint in HOLD mode
   - Activates servo (spray) for configured duration
   - Moves to next waypoint
   - Repeats until all waypoints are marked

=== CONFIGURATION ===
Edit Backend/servo_manager/config.json → wpmark section:
  - servo_number: 10 (servo channel)
  - pwm_on: 1500 (spray ON)
  - pwm_off: 1000 (spray OFF)
  - delay_before_spray: 2.0s (wait after arrival)
  - spray_duration: 5.0s (how long to spray)
  - delay_after_spray: 1.0s (wait before next WP)
  - waypoint_threshold: 2.0m (arrival distance)
  - navigation_timeout: 120.0s (max time per WP)
"""

import json
import os
import signal
import sys
import time
import threading
from typing import Any, Dict, List, Optional
from enum import Enum
from dataclasses import dataclass
import math

# Import parent directory for mavros_bridge
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

try:
    from mavros_bridge import MavrosBridge, ServiceError
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False
    print("[wpmark] ERROR: mavros_bridge not available", flush=True)
    sys.exit(1)


BASE = os.path.dirname(__file__)
CFG = os.path.join(BASE, "config.json")
LOG_DIR = os.path.join(BASE, "logs")
os.makedirs(LOG_DIR, exist_ok=True)


class MissionState(Enum):
    """Mission execution states"""
    IDLE = "idle"
    LOADING_WAYPOINTS = "loading_waypoints"
    NAVIGATING = "navigating"
    HOLDING = "holding"
    SPRAYING = "spraying"
    COMPLETED = "completed"
    ERROR = "error"
    STOPPED = "stopped"


@dataclass
class WPMarkConfig:
    """Configuration for waypoint marking"""
    servo_number: int = 10
    pwm_on: int = 1500
    pwm_off: int = 1000
    delay_before_spray: float = 2.0
    spray_duration: float = 5.0
    delay_after_spray: float = 1.0
    waypoint_threshold: float = 2.0
    navigation_timeout: float = 120.0


class WaypointMarker:
    """Manages waypoint marking mission execution"""
    
    MAV_CMD_DO_SET_SERVO = 183
    
    def __init__(self, config: WPMarkConfig):
        self.config = config
        self.bridge: Optional[MavrosBridge] = None
        self.state = MissionState.IDLE
        self.running = True
        self.waypoints: List[Dict[str, Any]] = []
        self.current_wp_index = 0
        self.total_waypoints = 0
        
        # Thread safety
        self.lock = threading.Lock()
        self.position_lock = threading.Lock()
        
        # Current position tracking
        self.current_lat: Optional[float] = None
        self.current_lon: Optional[float] = None
        self.current_alt: Optional[float] = None
        
        # Mission tracking
        self.start_time: Optional[float] = None
        self.waypoint_start_time: Optional[float] = None
        self.completed_waypoints = 0
        self.failed_waypoints = 0
        
        # Logging
        self.log_file = os.path.join(LOG_DIR, f"wpmark_{int(time.time())}.log")
        
        signal.signal(signal.SIGTERM, self._sigterm_handler)
        signal.signal(signal.SIGINT, self._sigterm_handler)
    
    def _sigterm_handler(self, _sig, _frm):
        """Handle graceful shutdown"""
        self.log("Received shutdown signal")
        self.running = False
    
    def log(self, message: str, level: str = "INFO"):
        """Log message to file and console with millisecond precision"""
        now = time.time()
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
        milliseconds = int((now % 1) * 1000)
        full_timestamp = f"{timestamp}.{milliseconds:03d}"
        log_line = f"[{full_timestamp}] [{level}] {message}"
        print(f"[wpmark] {log_line}", flush=True)
        
        try:
            with open(self.log_file, "a") as f:
                f.write(log_line + "\n")
        except Exception:
            pass
    
    def calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS coordinates using Haversine formula"""
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c
    
    def position_callback(self, data: Dict[str, Any]):
        """Update current position from telemetry"""
        try:
            with self.position_lock:
                self.current_lat = data.get("latitude")
                self.current_lon = data.get("longitude")
                self.current_alt = data.get("altitude")
        except Exception as e:
            self.log(f"Error updating position: {e}", "WARNING")
    
    def connect_mavros(self) -> bool:
        """Initialize MAVROS bridge connection"""
        try:
            self.log("Connecting to MAVROS...")
            self.bridge = MavrosBridge()
            self.bridge.connect(timeout=10.0)
            
            # Wait for connection to be fully established
            max_wait = 5.0
            start = time.time()
            while not self.bridge.is_connected and (time.time() - start) < max_wait:
                time.sleep(0.1)
            
            if not self.bridge.is_connected:
                self.log("Failed to connect to MAVROS", "ERROR")
                return False
            
            # Subscribe to position updates
            self.bridge.subscribe_telemetry(lambda data: self.position_callback(data))
            
            self.log("MAVROS connected successfully")
            return True
            
        except Exception as e:
            self.log(f"MAVROS connection error: {e}", "ERROR")
            return False
    
    def load_waypoints(self) -> bool:
        """Load waypoints from MAVROS mission"""
        try:
            self.state = MissionState.LOADING_WAYPOINTS
            self.log("Pulling waypoints from mission...")
            
            result = self.bridge.pull_waypoints(timeout=15.0)
            
            if not result:
                self.log("No waypoints received from mission", "ERROR")
                return False
            
            # Extract waypoints list
            waypoints = result.get("waypoints", [])
            if not waypoints:
                self.log("Mission contains no waypoints", "ERROR")
                return False
            
            # Filter only WAYPOINT type (command 16)
            self.waypoints = [wp for wp in waypoints if wp.get("command") == 16]
            self.total_waypoints = len(self.waypoints)
            
            if self.total_waypoints == 0:
                self.log("No valid navigation waypoints in mission", "ERROR")
                return False
            
            self.log(f"Loaded {self.total_waypoints} waypoints from mission")
            for i, wp in enumerate(self.waypoints):
                self.log(f"  WP{i}: lat={wp.get('x_lat'):.6f}, lon={wp.get('y_long'):.6f}, alt={wp.get('z_alt'):.1f}")
            
            return True
            
        except Exception as e:
            self.log(f"Error loading waypoints: {e}", "ERROR")
            return False
    
    def set_mode(self, mode: str) -> bool:
        """Change flight mode"""
        try:
            self.log(f"Setting mode to {mode}...")
            self.bridge.set_mode(mode=mode, timeout=5.0)
            time.sleep(0.5)  # Allow mode change to settle
            self.log(f"Mode changed to {mode}")
            return True
        except Exception as e:
            self.log(f"Failed to set mode {mode}: {e}", "ERROR")
            return False
    
    def navigate_to_waypoint(self, wp_index: int) -> bool:
        """Navigate to specific waypoint"""
        try:
            if wp_index >= len(self.waypoints):
                self.log(f"Invalid waypoint index {wp_index}", "ERROR")
                return False
            
            waypoint = self.waypoints[wp_index]
            target_lat = waypoint.get("x_lat")
            target_lon = waypoint.get("y_long")
            
            self.log(f"Navigating to WP{wp_index} (lat={target_lat:.6f}, lon={target_lon:.6f})...")
            
            # Set current waypoint in mission
            self.bridge.set_current_waypoint(wp_seq=wp_index, timeout=5.0)
            
            self.state = MissionState.NAVIGATING
            self.waypoint_start_time = time.time()
            
            return True
            
        except Exception as e:
            self.log(f"Navigation error: {e}", "ERROR")
            return False
    
    def wait_for_waypoint_arrival(self, wp_index: int) -> bool:
        """Wait until rover reaches waypoint within threshold"""
        waypoint = self.waypoints[wp_index]
        target_lat = waypoint.get("x_lat")
        target_lon = waypoint.get("y_long")
        
        self.log(f"Waiting for arrival at WP{wp_index} (threshold={self.config.waypoint_threshold}m)...")
        
        last_distance = None
        check_count = 0
        
        while self.running:
            # Check timeout
            elapsed = time.time() - self.waypoint_start_time
            if elapsed > self.config.navigation_timeout:
                self.log(f"Navigation timeout after {elapsed:.1f}s", "ERROR")
                return False
            
            # Get current position
            with self.position_lock:
                curr_lat = self.current_lat
                curr_lon = self.current_lon
            
            if curr_lat is None or curr_lon is None:
                time.sleep(0.5)
                continue
            
            # Calculate distance to waypoint
            distance = self.calculate_distance(curr_lat, curr_lon, target_lat, target_lon)
            
            # Log progress every 10 checks
            check_count += 1
            if check_count % 10 == 0 or (last_distance and abs(distance - last_distance) > 1.0):
                self.log(f"  Distance to WP{wp_index}: {distance:.1f}m")
            last_distance = distance
            
            # Check if arrived
            if distance <= self.config.waypoint_threshold:
                self.log(f"Arrived at WP{wp_index} (distance={distance:.1f}m)")
                return True
            
            time.sleep(0.5)
        
        return False
    
    def set_servo(self, pwm: int) -> bool:
        """Send servo command"""
        try:
            self.log(f"Setting servo {self.config.servo_number} to PWM {pwm}...")
            
            result = self.bridge.send_command_long(
                command=self.MAV_CMD_DO_SET_SERVO,
                param1=float(self.config.servo_number),
                param2=float(pwm),
                timeout=5.0
            )
            
            success = result.get("success", False)
            if success:
                self.log(f"Servo command successful (PWM={pwm})")
            else:
                self.log(f"Servo command failed: {result}", "ERROR")
            
            return success
            
        except Exception as e:
            self.log(f"Servo command error: {e}", "ERROR")
            return False
    
    def process_waypoint(self, wp_index: int) -> bool:
        """Process single waypoint: navigate, hold, spray, release"""
        try:
            self.log(f"=== Processing Waypoint {wp_index + 1}/{self.total_waypoints} ===")
            
            # Step 1: Switch to GUIDED mode
            if not self.set_mode("GUIDED"):
                return False
            
            # Step 2: Navigate to waypoint
            if not self.navigate_to_waypoint(wp_index):
                return False
            
            # Step 3: Wait for arrival
            if not self.wait_for_waypoint_arrival(wp_index):
                return False
            
            # Step 4: Switch to HOLD mode
            self.state = MissionState.HOLDING
            if not self.set_mode("HOLD"):
                self.log("Failed to switch to HOLD, continuing anyway...", "WARNING")
            
            # Step 5: Wait before spraying
            self.log(f"Waiting {self.config.delay_before_spray}s before spray...")
            time.sleep(self.config.delay_before_spray)
            
            # Step 6: Activate servo (spray ON)
            self.state = MissionState.SPRAYING
            if not self.set_servo(self.config.pwm_on):
                self.log("Failed to activate servo", "WARNING")
            
            # Step 7: Spray duration
            self.log(f"Spraying for {self.config.spray_duration}s...")
            time.sleep(self.config.spray_duration)
            
            # Step 8: Deactivate servo (spray OFF)
            if not self.set_servo(self.config.pwm_off):
                self.log("Failed to deactivate servo", "WARNING")
            
            # Step 9: Wait after spraying
            self.log(f"Waiting {self.config.delay_after_spray}s after spray...")
            time.sleep(self.config.delay_after_spray)
            
            self.completed_waypoints += 1
            self.log(f"=== Waypoint {wp_index + 1} completed ({self.completed_waypoints}/{self.total_waypoints}) ===")
            
            return True
            
        except Exception as e:
            self.log(f"Error processing waypoint {wp_index}: {e}", "ERROR")
            self.failed_waypoints += 1
            return False
    
    def run_mission(self) -> bool:
        """Execute complete waypoint marking mission"""
        try:
            self.start_time = time.time()
            self.log("=== Starting Waypoint Marking Mission ===")
            
            # Connect to MAVROS
            if not self.connect_mavros():
                self.state = MissionState.ERROR
                return False
            
            # Load waypoints
            if not self.load_waypoints():
                self.state = MissionState.ERROR
                return False
            
            # Process each waypoint
            for wp_index in range(self.total_waypoints):
                if not self.running:
                    self.log("Mission stopped by user")
                    self.state = MissionState.STOPPED
                    break
                
                success = self.process_waypoint(wp_index)
                if not success:
                    self.log(f"Waypoint {wp_index + 1} failed, continuing to next...", "WARNING")
            
            # Mission complete
            if self.running:
                self.state = MissionState.COMPLETED
                duration = time.time() - self.start_time
                self.log("=== Mission Completed ===")
                self.log(f"Total waypoints: {self.total_waypoints}")
                self.log(f"Completed: {self.completed_waypoints}")
                self.log(f"Failed: {self.failed_waypoints}")
                self.log(f"Duration: {duration:.1f}s")
            
            return True
            
        except Exception as e:
            self.log(f"Mission error: {e}", "ERROR")
            self.state = MissionState.ERROR
            return False
        
        finally:
            # Cleanup: ensure servo is OFF and switch to HOLD
            try:
                self.log("Cleanup: Turning off servo...")
                self.set_servo(self.config.pwm_off)
                self.log("Cleanup: Switching to HOLD mode...")
                self.set_mode("HOLD")
            except Exception as e:
                self.log(f"Cleanup error: {e}", "WARNING")
            
            # Close MAVROS connection
            if self.bridge:
                try:
                    self.bridge.close()
                except Exception:
                    pass


def load_config() -> WPMarkConfig:
    """Load configuration from JSON file"""
    try:
        with open(CFG, "r") as f:
            cfg = json.load(f)
    except Exception:
        cfg = {}
    
    wpmark_cfg = cfg.get("wpmark", {})
    
    return WPMarkConfig(
        servo_number=int(wpmark_cfg.get("servo_number", 10)),
        pwm_on=int(wpmark_cfg.get("pwm_on", 1500)),
        pwm_off=int(wpmark_cfg.get("pwm_off", 1000)),
        delay_before_spray=float(wpmark_cfg.get("delay_before_spray", 2.0)),
        spray_duration=float(wpmark_cfg.get("spray_duration", 5.0)),
        delay_after_spray=float(wpmark_cfg.get("delay_after_spray", 1.0)),
        waypoint_threshold=float(wpmark_cfg.get("waypoint_threshold", 2.0)),
        navigation_timeout=float(wpmark_cfg.get("navigation_timeout", 120.0)),
    )


def main():
    """Main entry point"""
    config = load_config()
    
    print(f"[wpmark] Waypoint Marking Configuration:", flush=True)
    print(f"  Servo: {config.servo_number}", flush=True)
    print(f"  PWM ON: {config.pwm_on}", flush=True)
    print(f"  PWM OFF: {config.pwm_off}", flush=True)
    print(f"  Delay before spray: {config.delay_before_spray}s", flush=True)
    print(f"  Spray duration: {config.spray_duration}s", flush=True)
    print(f"  Delay after spray: {config.delay_after_spray}s", flush=True)
    print(f"  Waypoint threshold: {config.waypoint_threshold}m", flush=True)
    print(f"  Navigation timeout: {config.navigation_timeout}s", flush=True)
    
    marker = WaypointMarker(config)
    success = marker.run_mission()
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
