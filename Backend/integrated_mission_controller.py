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
        self.hold_duration = 2.0  # seconds
        self.mission_timeout = 300.0  # 5 minutes per waypoint

        # Servo configuration (loaded from config file, can be overridden)
        self.servo_enabled = True  # Enable servo control after hold
        self.servo_channel = 9
        self.servo_pwm_on = 2300
        self.servo_pwm_off = 1750
        self.servo_delay_before = 0.0  # Delay BEFORE turning servo ON
        self.servo_spray_duration = 0.5  # Time between ON and OFF
        self.servo_delay_after = 2.0  # Delay after OFF before continuing
        
        # Threading
        self.lock = threading.RLock()  # Use RLock to allow reentrant locking
        self.hold_timer: Optional[threading.Timer] = None
        self.mission_timer: Optional[threading.Timer] = None
        self.position_check_timer: Optional[threading.Timer] = None
        self.running = True
        
        # Mission execution state
        self.waiting_for_waypoint_reach = False
        self.waypoint_upload_time: Optional[float] = None
        self.home_set = False  # Track if HOME position has been set

        # REFACTOR: Use Pixhawk waypoint reached topic instead of distance calculation
        self.use_pixhawk_waypoint_reached = True  # Primary: use /mavros/mission/reached topic
        self.fallback_to_distance_check = True  # Fallback: use distance if topic fails
        self.last_telemetry_emission_time: float = 0  # Phase 2: Throttle telemetry emissions
        self.last_distance_check_time: float = 0  # For fallback distance monitoring

        # Start telemetry subscription
        self.start_telemetry_subscription()
        
        self.log("Mission controller initialized")
        self.emit_status("Mission controller initialized", "info")
    
    def log(self, message: str, level: str = "info"):
        """Log message with optional Flask logger"""
        import sys
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        log_msg = f"[MISSION_CONTROLLER] [{timestamp}] {message}"
        
        # Always print to stdout for journalctl (unbuffered)
        print(log_msg, flush=True)
        
        # Also log through Flask logger if available
        if self.logger:
            try:
                log_func = getattr(self.logger, level, self.logger.info)
                log_func(log_msg)
            except Exception as e:
                print(f"[MISSION_CONTROLLER] Logger error: {e}", flush=True)
    
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

            # REFACTOR: Check for Pixhawk waypoint reached message (primary method)
            if 'mission_progress' in telemetry_data:
                mission_progress = telemetry_data.get('mission_progress', {})
                message_type = telemetry_data.get('message_type', '')

                # Check if this is a waypoint reached message from Pixhawk
                if (message_type == 'mission_reached' and
                    self.use_pixhawk_waypoint_reached and
                    self.mission_state == MissionState.RUNNING and
                    self.waiting_for_waypoint_reach):

                    # Extract waypoint sequence from mission progress
                    wp_seq = mission_progress.get('wp_seq')
                    if wp_seq is not None:
                        self.log(f'📡 RECEIVED /mavros/mission/reached: wp_seq={wp_seq}', 'info')
                        self.handle_pixhawk_waypoint_reached(wp_seq)

            # Fallback: Check distance if enabled and topic hasn't triggered
            if (self.fallback_to_distance_check and
                self.mission_state == MissionState.RUNNING and
                self.waiting_for_waypoint_reach and
                self.current_position):

                # Only check distance every 0.5 seconds (not every telemetry update)
                current_time = time.time()
                if current_time - self.last_distance_check_time >= 0.5:
                    self.last_distance_check_time = current_time
                    self.check_waypoint_reached_distance_fallback()

            # PHASE 2 FIX: Throttle telemetry status emissions to reduce WebSocket flood
            # Only emit telemetry updates every 2 seconds instead of on every telemetry packet
            current_time = time.time()
            if self.current_position or self.pixhawk_state:
                time_since_last_emission = current_time - self.last_telemetry_emission_time
                if time_since_last_emission >= 2.0:
                    self.emit_status("Telemetry update", "info")
                    self.last_telemetry_emission_time = current_time

        except Exception as e:
            self.log(f"Telemetry handling error: {e}", "error")
    
    def process_command(self, command_data: Dict[str, Any]) -> Dict[str, Any]:
        """Process mission command and return response"""
        try:
            command = command_data.get('command', '')
            self.log(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self.log(f"📥 COMMAND RECEIVED: {command.upper()}")
            self.log(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            
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
            elif command == 'skip':
                return self.skip_waypoint()
            elif command == 'set_mode':
                mode = command_data.get('mode', 'auto')
                return self.set_mode(mode)
            else:
                return {'success': False, 'error': f'Unknown command: {command}'}
                
        except Exception as e:
            error_msg = f"Command processing error: {str(e)}"
            self.log(error_msg, "error")
            return {'success': False, 'error': error_msg}
    
    def update_servo_config(self, servo_config: Dict[str, Any]) -> Dict[str, Any]:
        """Update servo configuration dynamically"""
        try:
            updated_params = []
            
            if 'servo_channel' in servo_config:
                self.servo_channel = int(servo_config['servo_channel'])
                updated_params.append(f"servo_channel={self.servo_channel}")
            
            if 'servo_pwm_on' in servo_config:
                self.servo_pwm_on = int(servo_config['servo_pwm_on'])
                updated_params.append(f"servo_pwm_on={self.servo_pwm_on}")
            
            if 'servo_pwm_off' in servo_config:
                self.servo_pwm_off = int(servo_config['servo_pwm_off'])
                updated_params.append(f"servo_pwm_off={self.servo_pwm_off}")
            
            if 'servo_delay_before' in servo_config:
                self.servo_delay_before = float(servo_config['servo_delay_before'])
                updated_params.append(f"servo_delay_before={self.servo_delay_before}s")
            
            if 'servo_spray_duration' in servo_config:
                self.servo_spray_duration = float(servo_config['servo_spray_duration'])
                updated_params.append(f"servo_spray_duration={self.servo_spray_duration}s")
            
            if 'servo_delay_after' in servo_config:
                self.servo_delay_after = float(servo_config['servo_delay_after'])
                updated_params.append(f"servo_delay_after={self.servo_delay_after}s")
            
            if 'servo_enabled' in servo_config:
                self.servo_enabled = bool(servo_config['servo_enabled'])
                updated_params.append(f"servo_enabled={self.servo_enabled}")
            
            self.log(f"✓ Servo config updated: {', '.join(updated_params)}")
            
            return {
                'success': True,
                'message': 'Servo configuration updated',
                'updated_params': updated_params
            }
        except Exception as e:
            error_msg = f"Failed to update servo config: {str(e)}"
            self.log(error_msg, "error")
            return {'success': False, 'error': error_msg}
    
    def load_mission(self, command_data: Dict[str, Any]) -> Dict[str, Any]:
        """Load waypoints and mission configuration"""
        with self.lock:
            try:
                waypoints = command_data.get('waypoints', [])
                config = command_data.get('config', {})
                
                self.log(f"📦 Received {len(waypoints)} waypoint(s) in load_mission command")
                
                if not waypoints:
                    self.log(f"❌ Error: No waypoints provided", "error")
                    return {'success': False, 'error': 'No waypoints provided'}
                
                # Validate waypoints
                self.log(f"✓ Validating waypoints...")
                for i, wp in enumerate(waypoints):
                    if 'lat' not in wp or 'lng' not in wp:
                        self.log(f"❌ Waypoint {i} validation failed: missing lat/lng", "error")
                        return {'success': False, 'error': f'Waypoint {i} missing lat/lng'}
                self.log(f"✓ All waypoints validated successfully")
                
                self.waypoints = waypoints
                self.current_waypoint_index = 0
                self.log(f"📝 Setting mission state: IDLE → READY")
                self.mission_state = MissionState.READY
                
                # Update configuration
                if 'waypoint_threshold' in config:
                    self.waypoint_reached_threshold = float(config['waypoint_threshold'])
                if 'hold_duration' in config:
                    self.hold_duration = float(config['hold_duration'])
                if 'auto_mode' in config:
                    self.mission_mode = MissionMode.AUTO if config['auto_mode'] else MissionMode.MANUAL
                
                # Update servo configuration if provided
                if any(key.startswith('servo_') for key in config.keys()):
                    servo_config = {k: v for k, v in config.items() if k.startswith('servo_')}
                    self.update_servo_config(servo_config)
                    self.log(f"✓ Applied servo configuration from load_mission")
                
                # Log mission load details
                self.log(f'═══════════════════════════════════════════════════')
                self.log(f'MISSION LOADED: {len(waypoints)} waypoints')
                self.log(f'Mission Mode: {self.mission_mode.value}')
                self.log(f'Waypoint Threshold: {self.waypoint_reached_threshold}m')
                self.log(f'Hold Duration: {self.hold_duration}s')
                self.log(f'Servo Config: channel={self.servo_channel}, pwm_on={self.servo_pwm_on}, pwm_off={self.servo_pwm_off}')
                self.log(f'Servo Timing: before={self.servo_delay_before}s, spray={self.servo_spray_duration}s, after={self.servo_delay_after}s, enabled={self.servo_enabled}')
                for i, wp in enumerate(waypoints, 1):
                    self.log(f'  WP{i}: lat={wp["lat"]:.6f}, lng={wp["lng"]:.6f}, alt={wp.get("alt", 10.0)}m')
                self.log(f'═══════════════════════════════════════════════════')
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
            self.log(f"🎬 START command received")
            self.log(f"Current state: {self.mission_state.value}")
            self.log(f"Waypoints loaded: {len(self.waypoints)}")

            # PHASE 3 FIX: Allow starting from COMPLETED state (in case auto-transition fails)
            # Allow starting from READY, PAUSED, IDLE, or COMPLETED (if waypoints are loaded)
            if self.mission_state not in [MissionState.READY, MissionState.PAUSED, MissionState.IDLE, MissionState.COMPLETED]:
                self.log(f"❌ Cannot start - invalid state: {self.mission_state.value}", "error")
                return {'success': False, 'error': f'Cannot start mission - invalid state: {self.mission_state.value}'}

            if not self.waypoints:
                self.log(f"❌ Cannot start - no waypoints loaded", "error")
                return {'success': False, 'error': 'No waypoints loaded'}
            
            # Remember if we're resuming from pause
            was_paused = self.mission_state == MissionState.PAUSED
            
            self.mission_state = MissionState.RUNNING
            self.mission_start_time = time.time()
            
            # Only reset waypoint index if not resuming from pause
            if not was_paused:
                self.current_waypoint_index = 0
            
            self.log(f'═══════════════════════════════════════════════════')
            self.log(f'🚀 MISSION STARTED')
            self.log(f'Starting from waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}')
            self.log(f'Mission Mode: {self.mission_mode.value}')
            self.log(f'═══════════════════════════════════════════════════')
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

            # Set state to READY instead of IDLE so mission can be restarted without reload
            self.mission_state = MissionState.READY
            self.cancel_timers()
            self.waiting_for_waypoint_reach = False
            self.home_set = False  # Reset HOME flag so it gets set again on restart

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
    
    def skip_waypoint(self) -> Dict[str, Any]:
        """Skip current waypoint and proceed to next (works in both manual and auto mode)"""
        with self.lock:
            if self.mission_state != MissionState.RUNNING:
                return {'success': False, 'error': 'Mission not running'}
            
            current_wp = self.current_waypoint_index
            self.log(f"Skipping waypoint {current_wp}", "info")
            self.emit_status(f"Skipping waypoint {current_wp}", "warning")
            
            self.proceed_to_next_waypoint()
            return {'success': True, 'message': f'Skipped waypoint {current_wp}'}
    
    def set_mode(self, mode: str) -> Dict[str, Any]:
        """Set mission mode (auto/manual)"""
        with self.lock:
            try:
                old_mode = self.mission_mode
                new_mode = MissionMode.AUTO if mode.lower() == 'auto' else MissionMode.MANUAL
                self.mission_mode = new_mode
                
                # Log with SUCCESS level so it shows clearly in journalctl
                self.log(f'MISSION MODE CHANGED: {old_mode.value} → {new_mode.value}', 'success')
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
        """Execute the current waypoint - send one waypoint at a time"""
        if self.current_waypoint_index >= len(self.waypoints):
            self.complete_mission()
            return

        waypoint = self.waypoints[self.current_waypoint_index]

        try:
            self.log(f'───────────────────────────────────────────────────')
            self.log(f'📍 EXECUTING WAYPOINT {self.current_waypoint_index + 1}/{len(self.waypoints)}')
            self.log(f'Target: lat={waypoint["lat"]:.6f}, lng={waypoint["lng"]:.6f}, alt={waypoint.get("alt", 10.0)}m')

            # Step 1: Set HOME position (only first time)
            if not self.home_set:
                self.log(f'🏠 Setting HOME position (first time only)...')
                if self.set_home_position():
                    self.home_set = True
                    self.log(f'✅ HOME position set successfully')
                else:
                    self.log(f'⚠ Warning: Could not set HOME position', "warning")

            # Step 2: Upload waypoint (includes HOME position for ArduPilot compatibility)
            self.log(f'📤 Uploading waypoint {self.current_waypoint_index + 1}...')
            mavros_waypoint = self.create_single_waypoint(waypoint, seq=0)

            if self.upload_single_waypoint(mavros_waypoint):
                self.log(f'✅ Waypoint ready for navigation')

                # Step 3: ARM if not already armed (check before AUTO mode)
                if not self.ensure_pixhawk_armed():
                    self.log("⚠ Warning: Could not arm Pixhawk, attempting AUTO mode anyway", "warning")
                else:
                    self.log(f'✅ PIXHAWK ARMED')

                # Step 4: Set AUTO mode
                self.log(f'🔄 Setting AUTO mode...')
                self.set_pixhawk_mode("AUTO")
                self.log(f'✅ AUTO mode activated - rover should move to waypoint')

                # Step 5: Start monitoring for waypoint reached
                self.waiting_for_waypoint_reach = True
                self.waypoint_upload_time = time.time()
                self.start_periodic_status_logging()

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

    def handle_pixhawk_waypoint_reached(self, wp_seq: int):
        """
        Handle waypoint reached message from Pixhawk via /mavros/mission/reached topic.
        This is the PRIMARY method for waypoint detection (more robust than distance calculation).

        Args:
            wp_seq: Waypoint sequence number from Pixhawk (1-based indexing for mission waypoints)
        """
        try:
            # Pixhawk uses 1-based indexing for mission waypoints (seq=0 is HOME, seq=1 is first mission waypoint)
            # Our controller uses 0-based indexing for waypoints array
            # So Pixhawk seq=1 corresponds to our waypoint index 0
            expected_seq = self.current_waypoint_index + 1

            self.log(f'🎯 Pixhawk reports waypoint reached: seq={wp_seq}, expected={expected_seq}')

            # Verify this is the waypoint we're expecting
            if wp_seq != expected_seq:
                self.log(
                    f'⚠️ WARNING: Received wp_seq={wp_seq} but expected wp_seq={expected_seq}. '
                    f'Current waypoint index={self.current_waypoint_index}',
                    'warning'
                )
                # Don't trigger waypoint_reached for unexpected sequence
                return

            # Verify we're in the correct state
            if not self.waiting_for_waypoint_reach:
                self.log(f'⚠️ Received waypoint reached but not waiting for it (state inconsistency)', 'warning')
                return

            if self.mission_state != MissionState.RUNNING:
                self.log(f'⚠️ Received waypoint reached but mission not running (state={self.mission_state.value})', 'warning')
                return

            # All checks passed - waypoint truly reached by Pixhawk
            self.log(f'✅ PIXHAWK CONFIRMED: Waypoint {self.current_waypoint_index + 1} reached (seq={wp_seq})')

            # Trigger the waypoint reached handler (same as distance-based method)
            self.waypoint_reached()

        except Exception as e:
            self.log(f'❌ Error handling Pixhawk waypoint reached message: {e}', 'error')

    def check_waypoint_reached_distance_fallback(self):
        """
        FALLBACK: Simple distance-based waypoint detection (backup method).
        This is only used if Pixhawk topic messages are not received.
        Simpler than original - just monitors distance for logging and emergency backup.
        """
        if (not self.waiting_for_waypoint_reach or
            not self.current_position or
            self.current_waypoint_index >= len(self.waypoints)):
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        distance = self.calculate_distance(
            self.current_position['lat'], self.current_position['lng'],
            waypoint['lat'], waypoint['lng']
        )

        # If distance calculation failed, just log it (not critical since topic is primary)
        if distance == float('inf'):
            return

        # Simple logging every 5 seconds for monitoring
        if not hasattr(self, '_last_distance_log_time'):
            self._last_distance_log_time = 0

        current_time = time.time()
        if current_time - self._last_distance_log_time > 5:
            self.log(f'📊 Distance monitor: WP{self.current_waypoint_index + 1} is {distance:.2f}m away (waiting for Pixhawk topic)')
            self._last_distance_log_time = current_time

        # Emergency fallback: If within threshold for 10+ seconds and no topic message
        if distance <= self.waypoint_reached_threshold:
            if not hasattr(self, '_fallback_threshold_entry_time'):
                self._fallback_threshold_entry_time = current_time
                self.log(f'⚠️ FALLBACK: Entered threshold zone ({distance:.2f}m) - waiting 10s for Pixhawk topic before fallback trigger')
            else:
                time_in_zone = current_time - self._fallback_threshold_entry_time
                if time_in_zone >= 10.0:
                    # Topic didn't fire for 10 seconds - use fallback
                    self.log(f'🚨 FALLBACK TRIGGERED: No Pixhawk topic after 10s in zone - using distance detection', 'warning')
                    self.log(f'✓ Waypoint {self.current_waypoint_index + 1} reached via FALLBACK distance check ({distance:.2f}m)')
                    self.waypoint_reached()
                    del self._fallback_threshold_entry_time
        else:
            # Outside threshold - reset fallback timer
            if hasattr(self, '_fallback_threshold_entry_time'):
                del self._fallback_threshold_entry_time
    
    def waypoint_reached(self):
        """Handle waypoint reached (called by either Pixhawk topic or distance fallback)"""
        with self.lock:
            if not self.waiting_for_waypoint_reach or self.mission_state != MissionState.RUNNING:
                return

            self.waiting_for_waypoint_reach = False
            self.stop_periodic_status_logging()
            
            # Cancel mission timeout
            if self.mission_timer:
                self.mission_timer.cancel()
                self.mission_timer = None
            
            # Set HOLD mode
            self.log(f'🛑 Setting HOLD mode')
            self.set_pixhawk_mode("HOLD")
            
            # Record waypoint completion
            current_time = datetime.now().isoformat() + "Z"
            waypoint = self.waypoints[self.current_waypoint_index]
            
            self.log(f'✅ WAYPOINT {self.current_waypoint_index + 1} REACHED')
            self.log(f'Position: lat={self.current_position["lat"]:.6f}, lng={self.current_position["lng"]:.6f}' if self.current_position else 'Position: unknown')
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
            self.log(f'⏱ Starting {self.hold_duration}s hold period at waypoint {self.current_waypoint_index + 1}')
            self.hold_timer = threading.Timer(self.hold_duration, self.hold_period_complete)
            self.hold_timer.start()

    def execute_servo_sequence(self):
        """
        Execute servo ON/OFF sequence after hold period.
        Sequence: Before delay → Servo ON → Spray duration → Servo OFF → After delay
        """
        try:
            if not self.servo_enabled:
                self.log('⚠ Servo control disabled - skipping servo sequence')
                return

            self.log(f'═══════════════════════════════════════════════════')
            self.log(f'🎯 STARTING SERVO SEQUENCE')
            self.log(f'Servo Channel: {self.servo_channel}')
            self.log(f'═══════════════════════════════════════════════════')

            # Step 1: Wait before delay (delay BEFORE turning servo ON)
            if self.servo_delay_before > 0:
                self.log(f'⏱ Waiting {self.servo_delay_before}s (pre-spray delay)...')
                time.sleep(self.servo_delay_before)

            # Step 2: Turn servo ON
            self.log(f'📡 Setting servo {self.servo_channel} to {self.servo_pwm_on}µs (ON)')
            response_on = self.bridge.set_servo(self.servo_channel, self.servo_pwm_on)

            if response_on.get('success'):
                self.log(f'✅ Servo ON: {self.servo_pwm_on}µs')
            else:
                self.log(f'⚠ Servo ON command sent (response: {response_on})', 'warning')

            # Step 3: Wait spray duration (time between ON and OFF)
            self.log(f'⏱ Waiting {self.servo_spray_duration}s (spray duration)...')
            time.sleep(self.servo_spray_duration)

            # Step 4: Turn servo OFF
            self.log(f'📡 Setting servo {self.servo_channel} to {self.servo_pwm_off}µs (OFF)')
            response_off = self.bridge.set_servo(self.servo_channel, self.servo_pwm_off)

            if response_off.get('success'):
                self.log(f'✅ Servo OFF: {self.servo_pwm_off}µs')
            else:
                self.log(f'⚠ Servo OFF command sent (response: {response_off})', 'warning')

            # Step 5: Wait delay after spray (before continuing to next waypoint)
            self.log(f'⏱ Waiting {self.servo_delay_after}s (post-spray delay)...')
            time.sleep(self.servo_delay_after)

            self.log(f'✅ SERVO SEQUENCE COMPLETE')
            self.log(f'═══════════════════════════════════════════════════')

        except Exception as e:
            # Don't fail mission if servo fails - just log and continue
            self.log(f'❌ Servo sequence error: {e}', 'error')
            self.log(f'⚠ Continuing mission despite servo failure', 'warning')

    def hold_period_complete(self):
        """Called when hold period is complete"""
        with self.lock:
            if self.mission_state != MissionState.RUNNING:
                return

            self.log(f'✓ Hold period complete for waypoint {self.current_waypoint_index + 1}')

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

        # Execute servo sequence OUTSIDE the lock to prevent blocking
        # This allows telemetry updates to continue during servo operations
        self.execute_servo_sequence()

        # Re-acquire lock for final waypoint progression logic
        with self.lock:
            if self.mission_state != MissionState.RUNNING:
                return

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
        # REFACTOR: No longer need delay or debounce reset - Pixhawk handles waypoint transitions
        self.waiting_for_waypoint_reach = False

        self.current_waypoint_index += 1

        if self.current_waypoint_index >= len(self.waypoints):
            self.complete_mission()
        else:
            self.log(f'➡ Proceeding to next waypoint ({self.current_waypoint_index + 1}/{len(self.waypoints)})')
            self.execute_current_waypoint()
    
    def complete_mission(self):
        """Complete the mission with proper state cleanup"""
        # PHASE 3 FIX: Prepare data BEFORE acquiring lock to minimize lock time
        mission_duration = time.time() - (self.mission_start_time or time.time()) if self.mission_start_time else 0
        waypoints_completed = len(self.waypoints)
        completion_time = datetime.now().isoformat()

        with self.lock:
            self.mission_state = MissionState.COMPLETED
            self.cancel_timers()
            self.waiting_for_waypoint_reach = False

            # PHASE 3 FIX: Stop periodic status logging thread
            self.stop_periodic_status_logging()

            # PHASE 3 FIX: Complete state cleanup for restart capability
            self.current_waypoint_index = 0
            self.mission_start_time = None
            self.home_set = False
            self.waypoint_upload_time = None

            # REFACTOR: Clean up fallback tracking (no longer using Phase 1 variables)
            if hasattr(self, '_fallback_threshold_entry_time'):
                del self._fallback_threshold_entry_time
            if hasattr(self, '_last_distance_log_time'):
                del self._last_distance_log_time

            self.log(f'═══════════════════════════════════════════════════')
            self.log(f'🎉 MISSION COMPLETED SUCCESSFULLY')
            self.log(f'Duration: {mission_duration:.1f} seconds ({mission_duration/60:.1f} minutes)')
            self.log(f'Waypoints completed: {waypoints_completed}')
            self.log(f'═══════════════════════════════════════════════════')

        # PHASE 3 FIX: Emit status OUTSIDE of lock to prevent deadlock
        try:
            self.emit_status(
                "Mission completed successfully",
                "success",
                extra_data={
                    "mission_duration": mission_duration,
                    "waypoints_completed": waypoints_completed,
                    "completion_time": completion_time
                }
            )
        except Exception as e:
            self.log(f"Error emitting completion status: {e}", "error")

        # PHASE 3 FIX: Set HOLD mode OUTSIDE of lock to prevent deadlock
        try:
            self.log(f'🛑 Setting final HOLD mode')
            self.set_pixhawk_mode("HOLD")
        except Exception as e:
            self.log(f"Failed to set HOLD mode on completion: {e}", "warning")

        # PHASE 3 FIX: Auto-transition to READY state for restart capability
        with self.lock:
            self.log(f'📝 Transitioning state: COMPLETED → READY (mission can be restarted)')
            self.mission_state = MissionState.READY
    
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
    
    def set_home_position(self) -> bool:
        """Set HOME position using current rover position (called only once at mission start)"""
        try:
            if not self.current_position:
                self.log("⚠ No current position available", "warning")
                self.log("✓ HOME will be auto-set by ArduPilot on ARM")
                return True

            home_lat = self.current_position['lat']
            home_lng = self.current_position['lng']
            home_alt = self.current_position.get('alt', 0.0)

            self.log(f"🏠 HOME will be: lat={home_lat:.6f}, lng={home_lng:.6f}, alt={home_alt:.1f}m")
            self.log(f"✓ ArduPilot will auto-set HOME on ARM at current position")

            # ArduPilot automatically sets HOME position when vehicle is armed
            # No explicit command needed - this is the standard behavior
            return True

        except Exception as e:
            self.log(f"⚠ Error checking HOME position: {e}", "warning")
            # Don't fail - ArduPilot will auto-set HOME on arm
            return True

    def create_single_waypoint(self, waypoint: Dict[str, Any], seq: int = 0) -> Dict[str, Any]:
        """Create a single MAVROS waypoint"""
        return {
            'frame': 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT (relative to HOME)
            'command': 16,  # MAV_CMD_NAV_WAYPOINT
            'is_current': True,  # This waypoint is the current target
            'autocontinue': True,
            'param1': 0,  # Hold time
            'param2': float(self.waypoint_reached_threshold),  # Accept radius
            'param3': 0,  # Pass through radius
            'param4': 0,  # Yaw (0 = maintain current heading)
            'x_lat': float(waypoint['lat']),
            'y_long': float(waypoint['lng']),
            'z_alt': float(waypoint.get('alt', 10.0))
        }
    
    def upload_single_waypoint(self, waypoint: Dict[str, Any]) -> bool:
        """Upload waypoint with HOME position - ArduPilot requires HOME + mission waypoint"""
        try:
            # CRITICAL: ArduPilot requires HOME waypoint (seq=0) + mission waypoint (seq=1)
            # Cannot upload single waypoint alone - AUTO mode will reject it

            if not self.current_position:
                self.log("❌ No current position available for HOME", "error")
                return False

            # Create HOME waypoint (seq=0) - REQUIRED by ArduPilot
            home_waypoint = {
                'frame': 0,  # MAV_FRAME_GLOBAL (absolute altitude)
                'command': 16,  # MAV_CMD_NAV_WAYPOINT
                'is_current': True,  # HOME is initially current
                'autocontinue': True,
                'param1': 0,  # Hold time
                'param2': 0,  # Accept radius
                'param3': 0,  # Pass through radius
                'param4': 0,  # Yaw
                'x_lat': float(self.current_position['lat']),
                'y_long': float(self.current_position['lng']),
                'z_alt': float(self.current_position.get('alt', 0.0))
            }

            # Create mission waypoint (seq=1) - This is the actual target
            mission_waypoint = {
                'frame': 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT (relative to HOME)
                'command': 16,  # MAV_CMD_NAV_WAYPOINT
                'is_current': False,  # Will become current after upload
                'autocontinue': True,
                'param1': 0,  # Hold time
                'param2': float(self.waypoint_reached_threshold),  # Accept radius
                'param3': 0,  # Pass through radius
                'param4': 0,  # Yaw
                'x_lat': float(waypoint['x_lat']),
                'y_long': float(waypoint['y_long']),
                'z_alt': float(waypoint['z_alt'])
            }

            # Clear existing waypoints
            self.log(f"🗑️ Clearing existing waypoints...")
            clear_response = self.bridge.clear_waypoints()
            if clear_response.get('success', False):
                self.log(f"✓ Cleared existing waypoints")

            # Upload HOME + mission waypoint (2 waypoints total)
            self.log(f"📤 Uploading HOME + waypoint to Pixhawk...")
            response = self.bridge.push_waypoints([home_waypoint, mission_waypoint])

            if response.get('success', False):
                self.log(f"✅ HOME + waypoint uploaded successfully")

                # Set current waypoint to 1 (mission waypoint, since 0 is HOME)
                self.log(f"🎯 Setting current waypoint to 1 (mission target)")
                self.bridge.set_current_waypoint(1)

                # Simple verification delay
                time.sleep(0.5)

                return True
            else:
                self.log(f"❌ Failed to upload waypoints: {response}", "error")
                return False

        except Exception as e:
            self.log(f"❌ Waypoint upload error: {e}", "error")
            return False

    def upload_mission_to_pixhawk(self, waypoints: List[Dict[str, Any]]) -> bool:
        """Upload mission to Pixhawk via MAVROS bridge"""
        try:
            # Clear existing mission first
            self.log(f"Clearing existing waypoints...")
            clear_response = self.bridge.clear_waypoints()
            if not clear_response.get('success', False):
                self.log("⚠ Failed to clear existing waypoints", "warning")
            else:
                self.log(f"✓ Cleared existing waypoints")

            # Upload new waypoints (HOME + mission waypoint = 2 total)
            expected_count = len(waypoints)
            self.log(f"Uploading {expected_count} waypoint(s) to Pixhawk...")
            self.log(f"  • Waypoint 0: HOME position")
            self.log(f"  • Waypoint 1: Mission target")

            response = self.bridge.push_waypoints(waypoints)

            if response.get('success', False):
                self.log(f"✅ Successfully uploaded {expected_count} waypoint(s)")

                # CRITICAL: Verify mission was properly stored on Pixhawk
                self.log(f"🔍 Verifying mission on Pixhawk...")

                # Retry verification up to 3 times with increasing delays
                max_retries = 3
                for retry in range(max_retries):
                    delay = 1.0 + (retry * 0.5)  # 1.0s, 1.5s, 2.0s
                    self.log(f"⏱ Waiting {delay}s for Pixhawk to commit mission (attempt {retry + 1}/{max_retries})...")
                    time.sleep(delay)

                    try:
                        verify_response = self.bridge.pull_waypoints()
                        stored_waypoints = verify_response.get('waypoints', [])

                        self.log(f"📊 Verification attempt {retry + 1}: Found {len(stored_waypoints)} waypoints on Pixhawk")

                        if len(stored_waypoints) != expected_count:
                            if retry < max_retries - 1:
                                self.log(f"⚠ Verification attempt {retry + 1} incomplete - retrying...", "warning")
                                continue
                            else:
                                self.log(f"❌ Mission verification FAILED after {max_retries} attempts:", "error")
                                self.log(f"   Expected: {expected_count} waypoints (HOME + mission)", "error")
                                self.log(f"   Actual: {len(stored_waypoints)} waypoints", "error")
                                self.log(f"   Mission not properly stored - AUTO mode will fail", "error")
                                return False

                        # Verify sequence numbers and commands
                        home_found = False
                        mission_found = False
                        for wp in stored_waypoints:
                            seq = wp.get('seq', -1)
                            cmd = wp.get('command', -1)
                            if seq == 0:
                                home_found = True
                                self.log(f"✓ HOME waypoint found (seq=0, cmd={cmd})")
                            elif seq == 1:
                                mission_found = True
                                self.log(f"✓ Mission waypoint found (seq=1, cmd={cmd})")

                        if not home_found or not mission_found:
                            if retry < max_retries - 1:
                                self.log(f"⚠ Mission structure incomplete on attempt {retry + 1} - retrying...", "warning")
                                continue
                            else:
                                self.log(f"❌ Mission structure invalid after {max_retries} attempts:", "error")
                                self.log(f"   HOME found: {home_found}, Mission found: {mission_found}", "error")
                                return False

                        self.log(f"✅ Mission verified on attempt {retry + 1}: Complete HOME + mission waypoint structure")
                        self.log(f"✅ Mission is VALID and ready for AUTO mode")

                        # Debug: Show detailed mission structure using already-fetched waypoints
                        self.debug_mission_structure_with_data(stored_waypoints)

                        return True

                    except Exception as verify_error:
                        if retry < max_retries - 1:
                            self.log(f"⚠ Verification attempt {retry + 1} error: {verify_error} - retrying...", "warning")
                            continue
                        else:
                            self.log(f"⚠ Mission verification failed after {max_retries} attempts: {verify_error}", "warning")
                            self.log(f"⚠ Proceeding without verification - AUTO mode may fail", "warning")
                            return True  # Continue anyway on final attempt, but warn

                # Should not reach here
                return False

            else:
                self.log(f"❌ Failed to upload waypoints: {response}", "error")
                return False

        except Exception as e:
            self.log(f"Mission upload error: {e}", "error")
            return False
    
    def ensure_pixhawk_armed(self) -> bool:
        """Ensure Pixhawk is armed before setting AUTO mode. Returns True if armed, False if failed."""
        try:
            # Check current armed state from telemetry
            if self.pixhawk_state and self.pixhawk_state.get('armed', False):
                self.log("✓ Pixhawk already armed")
                return True
            
            # Try to arm the vehicle
            self.log("⚡ Attempting to arm Pixhawk...")
            arm_response = self.bridge.set_armed(True)
            
            if arm_response.get('success', False) or arm_response.get('armed', False):
                self.log("✅ PIXHAWK ARMED SUCCESSFULLY")
                return True
            else:
                self.log(f"✗ Failed to arm Pixhawk: {arm_response}", "warning")
                return False
                
        except Exception as e:
            self.log(f"Error during arm attempt: {e}", "error")
            return False
    
    def set_pixhawk_mode(self, mode: str):
        """Set Pixhawk flight mode"""
        try:
            current_mode = self.pixhawk_state.get('mode', 'UNKNOWN') if self.pixhawk_state else 'UNKNOWN'
            self.log(f"🔄 MODE CHANGE: {current_mode} → {mode}")
            response = self.bridge.set_mode(mode=mode)
            if response.get('mode_sent', False):
                self.log(f"✅ PIXHAWK MODE CHANGED TO: {mode}")
            else:
                self.log(f"❌ Failed to set mode to {mode}", "warning")
        except Exception as e:
            self.log(f"❌ Mode setting error: {e}", "error")
    
    def set_current_waypoint(self, wp_seq: int):
        """Set the current waypoint in the mission"""
        try:
            response = self.bridge.set_current_waypoint(wp_seq)
            if response.get('success', False):
                self.log(f"✅ Current waypoint set to {wp_seq}")
            else:
                self.log(f"⚠ Failed to set current waypoint: {response}", "warning")
        except Exception as e:
            self.log(f"❌ Error setting current waypoint: {e}", "error")
    
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
    
    def start_periodic_status_logging(self):
        """Start periodic status logging at 2Hz (every 0.5 seconds)"""
        if hasattr(self, '_status_logging_active') and self._status_logging_active:
            return
        
        self._status_logging_active = True
        
        def log_status_periodically():
            while self._status_logging_active and self.mission_state == MissionState.RUNNING:
                with self.lock:
                    if self.waiting_for_waypoint_reach and self.current_position:
                        waypoint = self.waypoints[self.current_waypoint_index]
                        distance = self.calculate_distance(
                            self.current_position['lat'], self.current_position['lng'],
                            waypoint['lat'], waypoint['lng']
                        )
                        
                        mode = self.pixhawk_state.get('mode', 'UNKNOWN') if self.pixhawk_state else 'UNKNOWN'
                        armed = self.pixhawk_state.get('armed', False) if self.pixhawk_state else False
                        
                        self.log(
                            f"📍 Status: WP{self.current_waypoint_index + 1}/{len(self.waypoints)} | "
                            f"Distance: {distance:.2f}m | "
                            f"Mode: {mode} | "
                            f"Armed: {'YES' if armed else 'NO'} | "
                            f"Pos: ({self.current_position['lat']:.6f}, {self.current_position['lng']:.6f})"
                        )
                
                time.sleep(0.5)  # 2Hz = 0.5 seconds
        
        # Start logging thread
        self._status_logging_thread = threading.Thread(target=log_status_periodically, daemon=True)
        self._status_logging_thread.start()
        self.log(f"📊 Started periodic status logging at 2Hz")
    
    def stop_periodic_status_logging(self):
        """Stop periodic status logging"""
        if hasattr(self, '_status_logging_active'):
            self._status_logging_active = False
            self.log(f"⏹ Stopped periodic status logging")
    
    def debug_mission_structure(self):
        """Debug helper to verify mission structure on Pixhawk (pulls waypoints)"""
        try:
            verify_response = self.bridge.pull_waypoints()
            waypoints = verify_response.get('waypoints', [])
            self.debug_mission_structure_with_data(waypoints)
        except Exception as e:
            self.log(f"❌ Debug mission structure failed: {e}", "error")

    def debug_mission_structure_with_data(self, waypoints: List[Dict[str, Any]]):
        """Debug helper to display mission structure using already-fetched waypoints"""
        try:
            self.log(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self.log(f"📋 MISSION STRUCTURE DEBUG")
            self.log(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self.log(f"Total waypoints on Pixhawk: {len(waypoints)}")
            self.log(f"UI mission waypoints: {len(self.waypoints)}")
            self.log(f"Current UI waypoint index: {self.current_waypoint_index}")
            self.log(f"")

            for wp in waypoints:
                seq = wp.get('seq', 'unknown')
                cmd = wp.get('command', 'unknown')
                frame = wp.get('frame', 'unknown')
                lat = wp.get('x_lat', 0)
                lng = wp.get('y_long', 0)
                alt = wp.get('z_alt', 0)
                current = wp.get('is_current', False)

                wp_type = "HOME" if seq == 0 else f"MISSION-{seq}"
                current_marker = "← CURRENT" if current else ""

                self.log(
                    f"  Pixhawk WP{seq}: {wp_type:12} | "
                    f"CMD:{cmd:3} | FRAME:{frame} | "
                    f"({lat:.6f}, {lng:.6f}, {alt:.1f}m) | "
                    f"{current_marker}"
                )

                # Show which UI waypoint this corresponds to
                if seq > 0 and seq - 1 < len(self.waypoints):
                    ui_wp = self.waypoints[seq - 1]
                    self.log(
                        f"            → UI WP{seq}: "
                        f"({ui_wp['lat']:.6f}, {ui_wp['lng']:.6f}, {ui_wp.get('alt', 10.0):.1f}m)"
                    )

            self.log(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

        except Exception as e:
            self.log(f"❌ Debug mission structure display failed: {e}", "error")

    def shutdown(self):
        """Shutdown mission controller"""
        with self.lock:
            self.running = False
            self.stop_periodic_status_logging()
            self.cancel_timers()
            self.waiting_for_waypoint_reach = False

            if self.mission_state == MissionState.RUNNING:
                self.set_pixhawk_mode("HOLD")

            self.log("Mission controller shutdown")
