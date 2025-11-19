#!/usr/bin/env python3
"""
Integrated Mission Controller - Runs within Flask Application

This replaces the separate ROS2 node approach with a threaded mission controller
that runs directly in your Flask server.py process.
"""

import threading
import time
import json
from enum import Enum
from typing import Dict, List, Optional, Any, Callable
from datetime import datetime
import math


class MissionState(Enum):
    IDLE = "idle"
    LOADING = "loading" 
    READY = "ready"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    ERROR = "error"


class MissionMode(Enum):
    MANUAL = "manual"
    AUTO = "auto"


class IntegratedMissionController:
    """
    Mission Controller that runs as part of the Flask application.
    Handles waypoint-by-waypoint execution with status callbacks.
    """
    
    def __init__(self, mavros_bridge, status_callback: Optional[Callable] = None, logger=None):
        self.bridge = mavros_bridge
        self.status_callback = status_callback
        self.logger = logger
        
        # Mission state
        self.mission_state = MissionState.IDLE
        self.mission_mode = MissionMode.AUTO
        self.waypoints: List[Dict[str, Any]] = []
        self.current_waypoint_index = 0
        self.mission_start_time: Optional[float] = None
        self.current_position: Optional[Dict[str, float]] = None
        self.pixhawk_state: Optional[Dict[str, Any]] = None
        
        # Configuration
        self.waypoint_reached_threshold = 2.0  # meters
        self.hold_duration = 5.0  # seconds
        self.mission_timeout = 300.0  # 5 minutes per waypoint
        
        # Threading
        self.lock = threading.Lock()
        self.hold_timer: Optional[threading.Timer] = None
        self.mission_timer: Optional[threading.Timer] = None
        self.position_check_timer: Optional[threading.Timer] = None
        self.running = True
        
        # Mission execution state
        self.waiting_for_waypoint_reach = False
        self.waypoint_upload_time: Optional[float] = None
        
        # Start telemetry subscription
        self.start_telemetry_subscription()
        
        self.log("Mission controller initialized")
        self.emit_status("Mission controller initialized", "info")
    
    def log(self, message: str, level: str = "info"):
        """Log message with optional Flask logger"""
        if self.logger:
            getattr(self.logger, level, self.logger.info)(f"[MISSION_CONTROLLER] {message}")
        else:
            print(f"[MISSION_CONTROLLER] {message}")
    
    def emit_status(self, message: str, level: str, extra_data: Optional[Dict[str, Any]] = None):
        """Emit status update via callback"""
        status_data = {
            'timestamp': datetime.now().isoformat(),
            'message': message,
            'level': level,
            'mission_state': self.mission_state.value,
            'mission_mode': self.mission_mode.value,
            'current_waypoint': self.current_waypoint_index + 1 if self.waypoints else 0,
            'total_waypoints': len(self.waypoints),
            'current_position': self.current_position,
            'pixhawk_state': self.pixhawk_state
        }
        
        if extra_data:
            status_data.update(extra_data)
        
        if self.status_callback:
            try:
                self.status_callback(status_data)
            except Exception as e:
                self.log(f"Status callback error: {e}", "error")
    
    def start_telemetry_subscription(self):
        """Subscribe to telemetry updates from MAVROS bridge"""
        try:
            self.bridge.subscribe_telemetry(self.handle_telemetry_update)
            self.log("Subscribed to telemetry updates")
        except Exception as e:
            self.log(f"Failed to subscribe to telemetry: {e}", "error")
    
    def handle_telemetry_update(self, telemetry_data: Dict[str, Any]):
        """Handle telemetry updates from MAVROS bridge"""
        try:
            # Support multiple telemetry formats coming from MavrosBridge or
            # an existing normalized server-level telemetry payload.

            # Case A: telemetry_data uses flat keys from MavrosBridge
            # (e.g. 'latitude', 'longitude', 'altitude', 'mode', 'armed')
            if 'latitude' in telemetry_data or 'longitude' in telemetry_data:
                lat = telemetry_data.get('latitude') or telemetry_data.get('lat')
                lng = telemetry_data.get('longitude') or telemetry_data.get('lng')
                alt = telemetry_data.get('altitude') or telemetry_data.get('alt')
                try:
                    self.current_position = {
                        'lat': float(lat) if lat is not None else 0.0,
                        'lng': float(lng) if lng is not None else 0.0,
                        'alt': float(alt) if alt is not None else 0.0
                    }
                except Exception:
                    # ignore bad numeric values
                    pass

            # Case B: telemetry nested under 'global' (older formats)
            elif 'global' in telemetry_data:
                global_data = telemetry_data['global']
                self.current_position = {
                    'lat': global_data.get('latitude', 0.0),
                    'lng': global_data.get('longitude', 0.0),
                    'alt': global_data.get('altitude', 0.0)
                }

            # Update Pixhawk state from flat fields if present
            if any(k in telemetry_data for k in ('mode', 'armed', 'connected')):
                self.pixhawk_state = {
                    'mode': telemetry_data.get('mode', ''),
                    'armed': bool(telemetry_data.get('armed', False)),
                    'connected': bool(telemetry_data.get('connected', False)),
                    'system_status': telemetry_data.get('system_status', '')
                }

            # Or from nested 'state' field
            if 'state' in telemetry_data:
                state_data = telemetry_data['state']
                self.pixhawk_state = {
                    'mode': state_data.get('mode', ''),
                    'armed': state_data.get('armed', False),
                    'connected': state_data.get('connected', False),
                    'system_status': state_data.get('system_status', '')
                }

            # If mission progress info present, optionally emit as extra
            # (e.g. waypoint reached messages from bridge)
            extra = None
            if 'mission_progress' in telemetry_data:
                extra = {'mission_progress': telemetry_data.get('mission_progress')}

            # Check if we reached waypoint during mission
            if (self.mission_state == MissionState.RUNNING and
                self.waiting_for_waypoint_reach and
                self.current_position):
                self.check_waypoint_reached()

            # Emit a status update on any telemetry change so frontend can stay synced
            if self.current_position or self.pixhawk_state:
                self.emit_status("Telemetry update", "info", extra_data=extra)

        except Exception as e:
            self.log(f"Telemetry handling error: {e}", "error")
    
    def process_command(self, command_data: Dict[str, Any]) -> Dict[str, Any]:
        """Process mission command and return response"""
        try:
            command = command_data.get('command', '')
            self.log(f"Processing command: {command}")
            
            if command == 'load_mission':
                return self.load_mission(command_data)
            elif command == 'start':
                return self.start_mission()
            elif command == 'stop':
                return self.stop_mission()
            elif command == 'pause':
                return self.pause_mission()
            elif command == 'resume':
                return self.resume_mission()
            elif command == 'restart':
                return self.restart_mission()
            elif command == 'next':
                return self.next_waypoint()
            elif command == 'set_mode':
                mode = command_data.get('mode', 'auto')
                return self.set_mode(mode)
            else:
                return {'success': False, 'error': f'Unknown command: {command}'}
                
        except Exception as e:
            error_msg = f"Command processing error: {str(e)}"
            self.log(error_msg, "error")
            return {'success': False, 'error': error_msg}
    
    def load_mission(self, command_data: Dict[str, Any]) -> Dict[str, Any]:
        """Load waypoints and mission configuration"""
        with self.lock:
            try:
                waypoints = command_data.get('waypoints', [])
                config = command_data.get('config', {})
                
                if not waypoints:
                    return {'success': False, 'error': 'No waypoints provided'}
                
                # Validate waypoints
                for i, wp in enumerate(waypoints):
                    if 'lat' not in wp or 'lng' not in wp:
                        return {'success': False, 'error': f'Waypoint {i} missing lat/lng'}
                
                self.waypoints = waypoints
                self.current_waypoint_index = 0
                self.mission_state = MissionState.READY
                
                # Update configuration
                if 'waypoint_threshold' in config:
                    self.waypoint_reached_threshold = float(config['waypoint_threshold'])
                if 'hold_duration' in config:
                    self.hold_duration = float(config['hold_duration'])
                if 'auto_mode' in config:
                    self.mission_mode = MissionMode.AUTO if config['auto_mode'] else MissionMode.MANUAL
                
                self.log(f'Mission loaded: {len(waypoints)} waypoints, mode: {self.mission_mode.value}')
                self.emit_status(
                    f"Mission loaded with {len(waypoints)} waypoints",
                    "success",
                    extra_data={
                        "waypoints_count": len(waypoints),
                        "mode": self.mission_mode.value,
                        "waypoints": waypoints
                    }
                )
                
                return {
                    'success': True,
                    'message': f'Mission loaded with {len(waypoints)} waypoints',
                    'waypoints_count': len(waypoints)
                }
                
            except Exception as e:
                self.mission_state = MissionState.ERROR
                error_msg = f"Failed to load mission: {str(e)}"
                self.log(error_msg, "error")
                self.emit_status(error_msg, "error")
                return {'success': False, 'error': error_msg}
    
    def start_mission(self) -> Dict[str, Any]:
        """Start mission execution"""
        with self.lock:
            if self.mission_state not in [MissionState.READY, MissionState.PAUSED]:
                return {'success': False, 'error': 'Cannot start mission - invalid state'}
            
            if not self.waypoints:
                return {'success': False, 'error': 'No waypoints loaded'}
            
            self.mission_state = MissionState.RUNNING
            self.mission_start_time = time.time()
            
            if self.mission_state != MissionState.PAUSED:
                self.current_waypoint_index = 0
            
            self.log(f'Starting mission from waypoint {self.current_waypoint_index + 1}')
            self.emit_status("Mission started", "success")
            
            # Execute first/current waypoint
            self.execute_current_waypoint()
            
            return {
                'success': True,
                'message': 'Mission started',
                'current_waypoint': self.current_waypoint_index + 1
            }
    
    def stop_mission(self) -> Dict[str, Any]:
        """Stop mission execution"""
        with self.lock:
            if self.mission_state not in [MissionState.RUNNING, MissionState.PAUSED]:
                return {'success': False, 'error': 'No mission running'}
            
            self.mission_state = MissionState.IDLE
            self.cancel_timers()
            self.waiting_for_waypoint_reach = False
            
            # Set HOLD mode
            self.set_pixhawk_mode("HOLD")
            
            self.log('Mission stopped')
            self.emit_status("Mission stopped", "info")
            
            return {'success': True, 'message': 'Mission stopped'}
    
    def pause_mission(self) -> Dict[str, Any]:
        """Pause mission execution"""
        with self.lock:
            if self.mission_state != MissionState.RUNNING:
                return {'success': False, 'error': 'No mission running'}
            
            self.mission_state = MissionState.PAUSED
            self.cancel_timers()
            self.waiting_for_waypoint_reach = False
            
            # Set HOLD mode
            self.set_pixhawk_mode("HOLD")
            
            self.log('Mission paused')
            self.emit_status("Mission paused", "info")
            
            return {'success': True, 'message': 'Mission paused'}
    
    def resume_mission(self) -> Dict[str, Any]:
        """Resume paused mission"""
        with self.lock:
            if self.mission_state != MissionState.PAUSED:
                return {'success': False, 'error': 'No paused mission'}
            
            self.mission_state = MissionState.RUNNING
            
            self.log('Mission resumed')
            self.emit_status("Mission resumed", "success")
            
            # Continue with current waypoint
            self.execute_current_waypoint()
            
            return {'success': True, 'message': 'Mission resumed'}
    
    def restart_mission(self) -> Dict[str, Any]:
        """Restart mission from beginning"""
        with self.lock:
            if not self.waypoints:
                return {'success': False, 'error': 'No waypoints loaded'}
            
            self.current_waypoint_index = 0
            self.cancel_timers()
            self.waiting_for_waypoint_reach = False
            
            self.mission_state = MissionState.RUNNING
            self.mission_start_time = time.time()
            
            self.log('Mission restarted')
            self.emit_status("Mission restarted", "success")
            
            self.execute_current_waypoint()
            
            return {'success': True, 'message': 'Mission restarted'}
    
    def next_waypoint(self) -> Dict[str, Any]:
        """Proceed to next waypoint (manual mode)"""
        with self.lock:
            if self.mission_state != MissionState.RUNNING:
                return {'success': False, 'error': 'Mission not running'}
            
            if self.mission_mode == MissionMode.AUTO:
                return {'success': False, 'error': 'Mission in auto mode - cannot manually proceed'}
            
            self.proceed_to_next_waypoint()
            return {'success': True, 'message': 'Proceeding to next waypoint'}
    
    def set_mode(self, mode: str) -> Dict[str, Any]:
        """Set mission mode (auto/manual)"""
        with self.lock:
            try:
                new_mode = MissionMode.AUTO if mode.lower() == 'auto' else MissionMode.MANUAL
                self.mission_mode = new_mode
                
                self.log(f'Mission mode set to: {new_mode.value}')
                self.emit_status(f"Mission mode set to {new_mode.value}", "success")
                
                return {
                    'success': True,
                    'message': f'Mode set to {new_mode.value}',
                    'mode': new_mode.value
                }
                
            except Exception as e:
                error_msg = f"Failed to set mode: {str(e)}"
                self.log(error_msg, "error")
                return {'success': False, 'error': error_msg}
    
    def execute_current_waypoint(self):
        """Execute the current waypoint"""
        if self.current_waypoint_index >= len(self.waypoints):
            self.complete_mission()
            return
        
        waypoint = self.waypoints[self.current_waypoint_index]
        
        try:
            self.log(f'Executing waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: {waypoint}')
            
            # Create MAVROS waypoint
            mavros_waypoints = self.create_mavros_waypoint(waypoint)
            
            # Upload mission to Pixhawk
            if self.upload_mission_to_pixhawk(mavros_waypoints):
                # Set AUTO mode
                self.set_pixhawk_mode("AUTO.MISSION")
                
                # Start waypoint reached checking
                self.waiting_for_waypoint_reach = True
                self.waypoint_upload_time = time.time()
                
                self.emit_status(
                    f"Executing waypoint {self.current_waypoint_index + 1}",
                    "info",
                    extra_data={
                        "current_waypoint": self.current_waypoint_index + 1,
                        "total_waypoints": len(self.waypoints),
                        "waypoint": waypoint,
                        "mode": self.mission_mode.value
                    }
                )
                
                # Set timeout for waypoint execution
                self.mission_timer = threading.Timer(self.mission_timeout, self.waypoint_timeout)
                self.mission_timer.start()
                
            else:
                raise Exception("Failed to upload waypoint to Pixhawk")
                
        except Exception as e:
            error_msg = f"Failed to execute waypoint {self.current_waypoint_index + 1}: {str(e)}"
            self.log(error_msg, "error")
            self.mission_state = MissionState.ERROR
            self.emit_status(error_msg, "error")
    
    def check_waypoint_reached(self):
        """Check if current waypoint has been reached"""
        if (not self.waiting_for_waypoint_reach or 
            not self.current_position or 
            self.current_waypoint_index >= len(self.waypoints)):
            return
        
        waypoint = self.waypoints[self.current_waypoint_index]
        distance = self.calculate_distance(
            self.current_position['lat'], self.current_position['lng'],
            waypoint['lat'], waypoint['lng']
        )
        
        # If distance calculation failed, emit an error event for frontend
        if distance == float('inf'):
            self.log(f"Distance calculation failed for waypoint {self.current_waypoint_index + 1}", "warning")
            self.emit_status(
                f"Distance check failed for waypoint {self.current_waypoint_index + 1}",
                "error",
                extra_data={
                    "event_type": "waypoint_error",
                    "waypoint_id": self.current_waypoint_index + 1,
                    "current_waypoint": self.current_waypoint_index + 1,
                    "message": "Distance calculation failed"
                }
            )
            return

        if distance <= self.waypoint_reached_threshold:
            self.waypoint_reached()
    
    def waypoint_reached(self):
        """Handle waypoint reached"""
        with self.lock:
            if not self.waiting_for_waypoint_reach or self.mission_state != MissionState.RUNNING:
                return
            
            self.waiting_for_waypoint_reach = False
            
            # Cancel mission timeout
            if self.mission_timer:
                self.mission_timer.cancel()
                self.mission_timer = None
            
            # Set HOLD mode
            self.set_pixhawk_mode("HOLD")
            
            # Record waypoint completion
            current_time = datetime.now().isoformat() + "Z"
            waypoint = self.waypoints[self.current_waypoint_index]
            
            self.log(f'Waypoint {self.current_waypoint_index + 1} reached')
            # Emit waypoint_reached event for frontend table
            self.emit_status(
                f"Waypoint {self.current_waypoint_index + 1} reached",
                "success",
                extra_data={
                    "event_type": "waypoint_reached",
                    "waypoint_id": self.current_waypoint_index + 1,
                    "current_waypoint": self.current_waypoint_index + 1,
                    "timestamp": current_time,
                    "position": self.current_position.copy() if self.current_position else None,
                    "waypoint_target": waypoint,
                    "message": f"Waypoint {self.current_waypoint_index + 1} reached successfully"
                }
            )
            
            # Start hold period
            self.log(f'Starting {self.hold_duration}s hold period')
            self.hold_timer = threading.Timer(self.hold_duration, self.hold_period_complete)
            self.hold_timer.start()
    
    def hold_period_complete(self):
        """Called when hold period is complete"""
        with self.lock:
            if self.mission_state != MissionState.RUNNING:
                return
            
            self.log(f'Hold period complete for waypoint {self.current_waypoint_index + 1}')
            
            # Emit waypoint_marked event indicating marking completed
            mark_time = datetime.now().isoformat() + "Z"
            self.emit_status(
                f"Waypoint {self.current_waypoint_index + 1} marking completed",
                "success",
                extra_data={
                    "event_type": "waypoint_marked",
                    "waypoint_id": self.current_waypoint_index + 1,
                    "current_waypoint": self.current_waypoint_index + 1,
                    "timestamp": mark_time,
                    "marking_status": "completed",
                    "spray_duration": None,
                    "message": f"Waypoint {self.current_waypoint_index + 1} marking completed"
                }
            )

            # Check mission mode
            if self.mission_mode == MissionMode.AUTO:
                self.proceed_to_next_waypoint()
            else:
                # Manual mode - wait for next command
                self.emit_status(
                    f"Waypoint {self.current_waypoint_index + 1} completed - waiting for manual next",
                    "info",
                    extra_data={
                        "waiting_for_manual": True,
                        "current_waypoint": self.current_waypoint_index + 1,
                        "next_waypoint": self.current_waypoint_index + 2 if self.current_waypoint_index + 1 < len(self.waypoints) else None
                    }
                )
    
    def proceed_to_next_waypoint(self):
        """Proceed to the next waypoint"""
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.complete_mission()
        else:
            self.execute_current_waypoint()
    
    def complete_mission(self):
        """Complete the mission"""
        with self.lock:
            self.mission_state = MissionState.COMPLETED
            self.cancel_timers()
            self.waiting_for_waypoint_reach = False
            
            # Set HOLD mode
            self.set_pixhawk_mode("HOLD")
            
            mission_duration = time.time() - (self.mission_start_time or time.time())
            
            self.log(f'Mission completed in {mission_duration:.1f} seconds')
            self.emit_status(
                "Mission completed successfully",
                "success",
                extra_data={
                    "mission_duration": mission_duration,
                    "waypoints_completed": len(self.waypoints),
                    "completion_time": datetime.now().isoformat()
                }
            )
    
    def waypoint_timeout(self):
        """Handle waypoint execution timeout"""
        with self.lock:
            if self.mission_state == MissionState.RUNNING and self.waiting_for_waypoint_reach:
                error_msg = f"Waypoint {self.current_waypoint_index + 1} execution timeout"
                self.log(error_msg, "error")
                self.mission_state = MissionState.ERROR
                self.waiting_for_waypoint_reach = False
                
                # Set HOLD mode
                self.set_pixhawk_mode("HOLD")
                
                # Emit a failed marking event for the waypoint
                fail_time = datetime.now().isoformat() + "Z"
                self.emit_status(
                    error_msg,
                    "error",
                    extra_data={
                        "event_type": "waypoint_marked",
                        "waypoint_id": self.current_waypoint_index + 1,
                        "current_waypoint": self.current_waypoint_index + 1,
                        "timestamp": fail_time,
                        "marking_status": "failed",
                        "message": error_msg
                    }
                )
    
    def create_mavros_waypoint(self, waypoint: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Create MAVROS waypoint format"""
        return [{
            'frame': 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            'command': 16,  # MAV_CMD_NAV_WAYPOINT
            'is_current': True,
            'autocontinue': True,
            'param1': 0,  # Hold time
            'param2': 0,  # Accept radius
            'param3': 0,  # Pass radius
            'param4': 0,  # Yaw
            'x_lat': float(waypoint['lat']),
            'y_long': float(waypoint['lng']),
            'z_alt': float(waypoint.get('alt', 10.0))
        }]
    
    def upload_mission_to_pixhawk(self, waypoints: List[Dict[str, Any]]) -> bool:
        """Upload mission to Pixhawk via MAVROS bridge"""
        try:
            # Clear existing mission first
            clear_response = self.bridge.clear_waypoints()
            if not clear_response.get('success', False):
                self.log("Failed to clear existing waypoints", "warning")
            
            # Upload new waypoints
            response = self.bridge.push_waypoints(waypoints)
            
            if response.get('success', False):
                self.log(f"Successfully uploaded {len(waypoints)} waypoint(s)")
                return True
            else:
                self.log(f"Failed to upload waypoints: {response}", "error")
                return False
                
        except Exception as e:
            self.log(f"Mission upload error: {e}", "error")
            return False
    
    def set_pixhawk_mode(self, mode: str):
        """Set Pixhawk flight mode"""
        try:
            response = self.bridge.set_mode(mode=mode)
            if response.get('mode_sent', False):
                self.log(f"Set Pixhawk mode to {mode}")
            else:
                self.log(f"Failed to set mode to {mode}", "warning")
        except Exception as e:
            self.log(f"Mode setting error: {e}", "error")
    
    def calculate_distance(self, lat1: float, lng1: float, lat2: float, lng2: float) -> float:
        """Calculate distance between two GPS coordinates in meters"""
        try:
            # Haversine formula
            R = 6371000  # Earth radius in meters
            
            lat1_rad = math.radians(lat1)
            lat2_rad = math.radians(lat2)
            delta_lat = math.radians(lat2 - lat1)
            delta_lng = math.radians(lng2 - lng1)
            
            a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) +
                 math.cos(lat1_rad) * math.cos(lat2_rad) *
                 math.sin(delta_lng/2) * math.sin(delta_lng/2))
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            
            return R * c
            
        except Exception:
            return float('inf')  # Return large distance on error
    
    def cancel_timers(self):
        """Cancel all active timers"""
        if self.hold_timer:
            self.hold_timer.cancel()
            self.hold_timer = None
        
        if self.mission_timer:
            self.mission_timer.cancel()
            self.mission_timer = None
        
        if self.position_check_timer:
            self.position_check_timer.cancel()
            self.position_check_timer = None
    
    def get_status(self) -> Dict[str, Any]:
        """Get current mission status"""
        with self.lock:
            return {
                'mission_state': self.mission_state.value,
                'mission_mode': self.mission_mode.value,
                'current_waypoint': self.current_waypoint_index + 1 if self.waypoints else 0,
                'total_waypoints': len(self.waypoints),
                'current_position': self.current_position,
                'pixhawk_state': self.pixhawk_state,
                'waiting_for_waypoint_reach': self.waiting_for_waypoint_reach,
                'waypoints': self.waypoints
            }
    
    def shutdown(self):
        """Shutdown mission controller"""
        with self.lock:
            self.running = False
            self.cancel_timers()
            self.waiting_for_waypoint_reach = False
            
            if self.mission_state == MissionState.RUNNING:
                self.set_pixhawk_mode("HOLD")
            
            self.log("Mission controller shutdown")
