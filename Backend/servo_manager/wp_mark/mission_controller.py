"""
WP_MARK Mission Controller - ROS2 Node
Autonomous waypoint navigation with configurable servo actuation
Enhanced with robust state management, retry mechanisms, and monitoring
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import asyncio
import time
import threading
from enum import Enum
from typing import Optional, List, Dict, Any

# ROS2 message types
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, Waypoint, WaypointList
from mavros_msgs.srv import WaypointPull, WaypointSetCurrent, CommandLong, SetMode

# Local imports
from .utils import calculate_gps_distance, setup_logging, ConfigManager, format_gps_coordinates
from .validators import WPMarkConfig
from .state_machine import StateMachine, MissionPhase, StateTransitionError, StateValidationError
from .retry_mechanism import get_retry_mechanism, RetryError
from .monitoring import performance_monitor, metrics_collector


class WPMarkError(Exception):
    """Base exception for WP_MARK mission errors"""
    pass


class GPSFixLostError(WPMarkError):
    """Raised when GPS fix is lost during mission"""
    pass


class WaypointTimeoutError(WPMarkError):
    """Raised when waypoint arrival times out"""
    pass


class ServoCommandFailedError(WPMarkError):
    """Raised when servo command fails"""
    pass


class WPMarkMissionNode(Node):
    """
    ROS2 Node for WP_MARK mission execution
    Enhanced with robust state management and monitoring
    """
    
    def __init__(self, config: WPMarkConfig):
        """
        Initialize WP_MARK mission node with robust state management
        
        Args:
            config: Validated mission configuration
        """
        super().__init__('wp_mark_mission_node')
        
        # Configuration
        self.config = config.to_dict()
        self.config_manager = ConfigManager()
        self.settings = self.config_manager.get_settings()
        
        # Setup logging
        self.logger = setup_logging()
        self.logger.info("=== WP_MARK Mission Node Initializing ===")
        
        # Initialize robust state management
        self.state_machine = StateMachine(self.logger)
        self.state_machine.add_transition_listener(self._on_state_transition)
        
        # Initialize retry mechanisms
        self.servo_retry = get_retry_mechanism('servo_command', self.logger)
        self.waypoint_retry = get_retry_mechanism('waypoint_pull', self.logger)
        self.gps_retry = get_retry_mechanism('gps_operation', self.logger)
        
        # Mission state (legacy compatibility)
        self.mission_active = False
        self.current_waypoint_index = 0
        self.waypoints: List[Waypoint] = []
        self.mission_start_time = None
        self.last_action = "Initialized"
        
        # ROS2 data
        self.current_gps: Optional[NavSatFix] = None
        self.current_state: Optional[State] = None
        self.gps_lock = threading.Lock()
        self.state_lock = threading.Lock()
        
        # Mission execution
        self.mission_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        
        # Initialize ROS2 interfaces
        self._setup_subscribers()
        self._setup_service_clients()
        
        # Update state machine with initial context
        self.state_machine.update_context(
            total_waypoints=0,
            mission_active=False,
            gps_valid=False,
            services_available=False
        )
        
        self.logger.info("WP_MARK Mission Node initialized with robust state management")
    
    def _on_state_transition(self, old_phase: MissionPhase, new_phase: MissionPhase, action: str):
        """
        Handle state machine transitions
        
        Args:
            old_phase: Previous phase
            new_phase: New phase
            action: Action that caused transition
        """
        self.logger.info(f"State transition: {old_phase.value} -> {new_phase.value} ({action})")
        
        # Update legacy compatibility fields
        self.mission_active = new_phase not in [MissionPhase.IDLE, MissionPhase.ERROR, MissionPhase.COMPLETED]
        self.last_action = action
        
        # Record metrics
        performance_monitor.record_mission_event('state_transition', {
            'from_phase': old_phase.value,
            'to_phase': new_phase.value,
            'action': action
        })
    
    def _gps_callback(self, msg: NavSatFix):
        """GPS position callback with enhanced state tracking"""
        with self.gps_lock:
            self.current_gps = msg
        
        # Update state machine with GPS validity
        gps_valid = msg.status.status >= self.settings['gps_fix_required']
        self.state_machine.update_context(gps_valid=gps_valid)
        
        # Record GPS metrics
        metrics_collector.record_histogram('gps.hdop', msg.position_covariance[0])
        metrics_collector.set_gauge('gps.satellites_visible', msg.status.service if hasattr(msg.status, 'service') else 0)
    
    def _state_callback(self, msg: State):
        """Flight controller state callback with enhanced monitoring"""
        with self.state_lock:
            self.current_state = msg
        
        # Update state machine with service availability
        services_available = msg.armed  # Basic check - could be enhanced
        self.state_machine.update_context(services_available=services_available)
        
        # Record flight controller metrics
        metrics_collector.set_gauge('fc.armed', 1 if msg.armed else 0)
        metrics_collector.set_gauge('fc.mode', msg.mode)
    
    def _waypoint_callback(self, msg: WaypointList):
        """Waypoint list callback with state updates"""
        self.waypoints = msg.waypoints
        
        # Update state machine with waypoint count
        self.state_machine.update_context(total_waypoints=len(self.waypoints))
        
        self.logger.debug(f"Received waypoint list: {len(self.waypoints)} waypoints")
        metrics_collector.set_gauge('mission.waypoints_loaded', len(self.waypoints))
    
    def _setup_subscribers(self):
        """Setup ROS2 topic subscriptions"""
        self.logger.info("Setting up ROS2 subscriptions...")
        
        # GPS Position
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._gps_callback,
            10
        )
        
        # Flight Controller State
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            10
        )
        
        # Waypoint List
        self.waypoint_sub = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self._waypoint_callback,
            10
        )
        
        self.logger.info("ROS2 subscriptions configured")
    
    def _setup_service_clients(self):
        """Setup ROS2 service clients"""
        self.logger.info("Setting up ROS2 service clients...")
        
        # Pull waypoints from flight controller
        self.waypoint_pull_client = self.create_client(
            WaypointPull,
            '/mavros/mission/pull'
        )
        
        # Set current waypoint
        self.waypoint_set_current_client = self.create_client(
            WaypointSetCurrent,
            '/mavros/mission/set_current'
        )
        
        # Command servo
        self.command_long_client = self.create_client(
            CommandLong,
            '/mavros/cmd/command'
        )
        
        # Set flight mode
        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )
        
        self.logger.info("ROS2 service clients configured")
    
    def get_current_gps(self) -> Optional[NavSatFix]:
        """Thread-safe GPS getter"""
        with self.gps_lock:
            return self.current_gps
    
    def get_current_state(self) -> Optional[State]:
        """Thread-safe state getter"""
        with self.state_lock:
            return self.current_state
    
    def set_flight_mode(self, mode: str) -> bool:
        """
        Switch flight mode (e.g., AUTO, HOLD)
        
        Args:
            mode: Flight mode string (AUTO, HOLD, RTL, etc.)
        
        Returns:
            True if mode change successful, False otherwise
        """
        self.logger.info(f"Requesting flight mode change to {mode}")
        
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("SetMode service not available")
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        try:
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            
            if future.result() is not None:
                response = future.result()
                if response.mode_sent:
                    self.logger.info(f"✓ Flight mode changed to {mode}")
                    metrics_collector.record_counter(f'flight_mode.change.{mode.lower()}')
                    return True
                else:
                    self.logger.error(f"✗ Flight mode change to {mode} rejected by FC")
                    return False
            else:
                self.logger.error(f"SetMode service call failed for {mode}")
                return False
        except Exception as e:
            self.logger.error(f"Exception during mode change to {mode}: {e}")
            return False
    
    def check_mission_safety(self) -> tuple[bool, str]:
        """
        Perform pre-mission safety checks with enhanced monitoring
        
        Returns:
            (is_safe, error_message)
        """
        performance_monitor.start_operation('safety_check')
        
        try:
            gps = self.get_current_gps()
            state = self.get_current_state()
            
            # Check GPS fix with retry
            if not gps:
                self.logger.warning("No GPS data available, attempting to get GPS...")
                # Give GPS a moment to populate
                time.sleep(1.0)
                gps = self.get_current_gps()
            
            if not gps or gps.status.status < self.settings['gps_fix_required']:
                performance_monitor.end_operation('safety_check', success=False)
                return False, "Insufficient GPS fix. Require 3D fix or better."
            
            # Check armed state
            if not state or not state.armed:
                performance_monitor.end_operation('safety_check', success=False)
                return False, "Rover must be armed before starting mission."
            
            # Check mode
            if state.mode not in ['AUTO', 'GUIDED']:
                performance_monitor.end_operation('safety_check', success=False)
                return False, f"Invalid mode: {state.mode}. Require AUTO or GUIDED."
            
            # Check waypoint count
            if len(self.waypoints) == 0:
                performance_monitor.end_operation('safety_check', success=False)
                return False, "No waypoints loaded. Upload mission first."
            
            performance_monitor.end_operation('safety_check', success=True)
            return True, "All safety checks passed"
            
        except Exception as e:
            self.logger.error(f"Safety check error: {str(e)}")
            performance_monitor.end_operation('safety_check', success=False)
            return False, f"Safety check failed: {str(e)}"
    
    def pull_waypoints(self) -> bool:
        """
        Pull waypoints from flight controller with retry mechanism
        
        Returns:
            True if successful
        """
        def _pull_operation():
            performance_monitor.start_operation('waypoint_pull')
            
            if not self.waypoint_pull_client.wait_for_service(timeout_sec=5.0):
                raise Exception("Waypoint pull service not available")
            
            request = WaypointPull.Request()
            future = self.waypoint_pull_client.call_async(request)
            
            # Wait for response
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if not future.result() or not future.result().success:
                raise Exception("Waypoint pull service call failed")
            
            performance_monitor.end_operation('waypoint_pull', success=True)
            return future.result()
        
        try:
            result = self.state_machine.execute_with_circuit_breaker('waypoint', _pull_operation)
            self.logger.info(f"Successfully pulled {result.wp_received} waypoints")
            # Wait for waypoint list callback to populate
            time.sleep(1.0)
            return True
        except Exception as e:
            self.logger.error(f"Failed to pull waypoints: {str(e)}")
            performance_monitor.end_operation('waypoint_pull', success=False)
            return False
    
    def set_current_waypoint(self, wp_index: int) -> bool:
        """
        Set current active waypoint on flight controller with retry
        
        Args:
            wp_index: Waypoint index to navigate to
            
        Returns:
            True if successful
        """
        def _set_waypoint_operation():
            performance_monitor.start_operation('waypoint_set')
            
            if not self.waypoint_set_current_client.wait_for_service(timeout_sec=5.0):
                raise Exception("Waypoint set current service not available")
            
            request = WaypointSetCurrent.Request()
            request.wp_seq = wp_index
            
            future = self.waypoint_set_current_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.result() or not future.result().success:
                raise Exception(f"Waypoint set current service call failed for index {wp_index}")
            
            performance_monitor.end_operation('waypoint_set', success=True)
            return future.result()
        
        try:
            result = self.state_machine.execute_with_circuit_breaker('waypoint', _set_waypoint_operation)
            self.logger.info(f"Set current waypoint to {wp_index}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to set waypoint {wp_index}: {str(e)}")
            performance_monitor.end_operation('waypoint_set', success=False)
            return False
    
    def set_servo(self, pwm_value: int, servo_channel: Optional[int] = None) -> bool:
        """
        Set servo PWM value using MAV_CMD_DO_SET_SERVO with retry mechanism
        
        Args:
            pwm_value: PWM in microseconds (100-2000)
            servo_channel: Servo output channel (default from config)
            
        Returns:
            True if command successful
        """
        if servo_channel is None:
            servo_channel = self.config.get('servo_channel', self.settings['servo_channel'])
        
        def _servo_operation():
            performance_monitor.start_operation('servo_command')
            
            if not self.command_long_client.wait_for_service(timeout_sec=5.0):
                raise Exception("Command service not available")
            
            request = CommandLong.Request()
            request.command = 183  # MAV_CMD_DO_SET_SERVO
            request.param1 = float(servo_channel)
            request.param2 = float(pwm_value)
            
            future = self.command_long_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if not future.result() or not future.result().success:
                raise Exception(f"Servo command failed for channel {servo_channel}, PWM {pwm_value}")
            
            performance_monitor.end_operation('servo_command', success=True)
            return future.result()
        
        try:
            result = self.state_machine.execute_with_circuit_breaker('servo', _servo_operation)
            self.logger.info(f"Servo {servo_channel} set to {pwm_value} PWM")
            metrics_collector.set_gauge(f'servo.channel_{servo_channel}', pwm_value)
            return True
        except Exception as e:
            self.logger.error(f"Failed to set servo {servo_channel} to {pwm_value}: {str(e)}")
            performance_monitor.end_operation('servo_command', success=False)
            self.state_machine.increment_error_count()
            return False
    
    def has_reached_waypoint(self, waypoint: Waypoint) -> bool:
        """
        Check if rover has reached waypoint
        
        Args:
            waypoint: Target waypoint
            
        Returns:
            True if within arrival threshold
        """
        gps = self.get_current_gps()
        if not gps:
            return False
        
        distance = calculate_gps_distance(
            gps.latitude,
            gps.longitude,
            waypoint.x_lat,
            waypoint.y_long
        )
        
        threshold = self.settings['waypoint_arrival_threshold']
        return distance < threshold
    
    def wait_for_arrival(self, waypoint: Waypoint) -> bool:
        """
        Wait for rover to reach waypoint
        
        Args:
            waypoint: Target waypoint
            
        Returns:
            True if arrived within timeout
        """
        start_time = time.time()
        timeout = self.settings['waypoint_timeout']
        
        self.logger.info(f"Waiting for arrival at {format_gps_coordinates(waypoint.x_lat, waypoint.y_long)}")
        
        while time.time() - start_time < timeout:
            if self.stop_event.is_set():
                self.logger.warning("Mission stopped during waypoint arrival wait")
                return False
            
            if self.has_reached_waypoint(waypoint):
                return True
            
            # Check GPS status
            gps = self.get_current_gps()
            if not gps or gps.status.status < self.settings['gps_fix_required']:
                raise GPSFixLostError("GPS fix lost during navigation")
            
            # Log distance periodically
            distance = calculate_gps_distance(
                gps.latitude, gps.longitude,
                waypoint.x_lat, waypoint.y_long
            )
            self.logger.debug(f"Distance to waypoint: {distance:.2f}m")
            
            time.sleep(0.5)
        
        raise WaypointTimeoutError(f"Timeout waiting for waypoint arrival (>{timeout}s)")
    
    def execute_mission(self):
        """
        Main mission execution loop with robust state management
        """
        try:
            # Phase 1: Initialization
            self.state_machine.transition_to(MissionPhase.INITIALIZING, "Starting mission initialization")
            self.logger.info("\n" + "="*60)
            self.logger.info("=== WP_MARK Mission Starting ===")
            self.logger.info("="*60)
            
            # Safety checks
            is_safe, error_msg = self.check_mission_safety()
            if not is_safe:
                raise WPMarkError(f"Safety check failed: {error_msg}")
            
            self.logger.info("✅ Safety checks passed")
            
            # Pull waypoints
            self.logger.info("Pulling waypoints from flight controller...")
            if not self.pull_waypoints():
                raise WPMarkError("Failed to retrieve waypoints")
            
            self.logger.info(f"✅ Mission loaded: {len(self.waypoints)} waypoints")
            
            # Save configuration
            self.config_manager.save_config(self.config)
            
            # Phase 2: Execute waypoint sequence
            for wp_index in range(len(self.waypoints)):
                if self.stop_event.is_set():
                    self.logger.warning("Mission stopped by user")
                    break
                
                waypoint = self.waypoints[wp_index]
                self.current_waypoint_index = wp_index
                self.state_machine.update_context(waypoint_index=wp_index)
                
                self.logger.info("\n" + "="*60)
                self.logger.info(f"Waypoint {wp_index + 1}/{len(self.waypoints)}")
                self.logger.info("="*60)
                
                # Step 1: Navigate to waypoint
                self.state_machine.transition_to(MissionPhase.NAVIGATING, f"Navigating to WP {wp_index + 1}")
                self.logger.info(f"Setting waypoint {wp_index}...")
                
                if not self.set_current_waypoint(wp_index):
                    self.logger.error(f"Failed to set waypoint {wp_index}")
                    self.state_machine.increment_error_count()
                    continue
                
                # Step 2: Wait for arrival
                self.state_machine.transition_to(MissionPhase.WAITING_ARRIVAL, f"Waiting for arrival at WP {wp_index + 1}")
                self.logger.info("Waiting for arrival...")
                
                if not self.wait_for_arrival(waypoint):
                    self.logger.error(f"Timeout waiting for waypoint {wp_index}")
                    self.state_machine.increment_error_count()
                    continue
                
                self.logger.info("✅ Waypoint reached!")
                performance_monitor.record_mission_event('waypoint_reached', {'waypoint': wp_index})
                self.config_manager.log_mission_event(
                    'WAYPOINT_REACHED',
                    {'waypoint': wp_index},
                    {'lat': waypoint.x_lat, 'lon': waypoint.y_long},
                    wp_index
                )
                
                # Step 3: Delay before start
                if self.config['delay_before_start'] > 0:
                    self.state_machine.transition_to(MissionPhase.DELAY_BEFORE_START, f"Delay before spray at WP {wp_index + 1}")
                    self.logger.info(f"Delay before start: {self.config['delay_before_start']}s")
                    time.sleep(self.config['delay_before_start'])
                
                if self.stop_event.is_set():
                    break
                
                # Step 4: Change mode to HOLD before spraying
                self.logger.info("Switching to HOLD mode for spraying...")
                if not self.set_flight_mode('HOLD'):
                    self.logger.error("Failed to switch to HOLD mode")
                    self.state_machine.increment_error_count()
                    continue
                
                # Step 5: Activate servo (spray ON)
                self.state_machine.transition_to(MissionPhase.SPRAYING, f"Spraying at WP {wp_index + 1}")
                self.logger.info(f"💧 Activating servo (PWM: {self.config['pwm_start']})")
                
                if not self.set_servo(self.config['pwm_start']):
                    self.logger.error("Failed to activate servo")
                    self.state_machine.increment_error_count()
                else:
                    performance_monitor.record_mission_event('servo_activated', {
                        'waypoint': wp_index,
                        'pwm': self.config['pwm_start']
                    })
                    self.config_manager.log_mission_event(
                        'SERVO_ON',
                        {'pwm': self.config['pwm_start'], 'waypoint': wp_index}
                    )
                
                # Step 6: Spraying duration
                self.logger.info(f"Spraying for {self.config['delay_before_stop']}s...")
                time.sleep(self.config['delay_before_stop'])
                
                if self.stop_event.is_set():
                    # Emergency stop: turn off servo
                    self.set_servo(self.config['pwm_stop'])
                    break
                
                # Step 7: Deactivate servo (spray OFF)
                self.logger.info(f"🛑 Deactivating servo (PWM: {self.config['pwm_stop']})")
                
                if not self.set_servo(self.config['pwm_stop']):
                    self.logger.error("Failed to deactivate servo")
                    self.state_machine.increment_error_count()
                else:
                    performance_monitor.record_mission_event('servo_deactivated', {
                        'waypoint': wp_index,
                        'pwm': self.config['pwm_stop']
                    })
                    self.config_manager.log_mission_event(
                        'SERVO_OFF',
                        {'pwm': self.config['pwm_stop'], 'waypoint': wp_index}
                    )
                
                # Step 8: Delay after stop
                if self.config['delay_after_stop'] > 0:
                    self.state_machine.transition_to(MissionPhase.DELAY_AFTER_STOP, f"Delay after spray at WP {wp_index + 1}")
                    self.logger.info(f"Delay after stop: {self.config['delay_after_stop']}s")
                    time.sleep(self.config['delay_after_stop'])
                
                # Step 9: Resume AUTO mode to continue mission
                self.logger.info("Resuming AUTO mode...")
                if not self.set_flight_mode('AUTO'):
                    self.logger.error("Failed to resume AUTO mode")
                    self.state_machine.increment_error_count()
                    continue
                if self.config['delay_after_stop'] > 0:
                    self.state_machine.transition_to(MissionPhase.DELAY_AFTER_STOP, f"Delay after spray at WP {wp_index + 1}")
                    self.logger.info(f"Delay after stop: {self.config['delay_after_stop']}s")
                    time.sleep(self.config['delay_after_stop'])
            
            # Phase 3: Mission complete
            self.state_machine.transition_to(MissionPhase.COMPLETED, "Mission completed successfully")
            self.logger.info("\n✅ Mission Complete!")
            performance_monitor.record_mission_event('mission_completed', {
                'total_waypoints': len(self.waypoints),
                'completed_waypoints': self.current_waypoint_index + 1
            })
            self.config_manager.log_mission_event(
                'MISSION_COMPLETE',
                {
                    'total_waypoints': len(self.waypoints),
                    'completed_waypoints': self.current_waypoint_index + 1
                }
            )
            
        except GPSFixLostError as e:
            self.logger.error(f"GPS fix lost: {str(e)}")
            self.state_machine.transition_to(MissionPhase.ERROR, f"GPS fix lost: {str(e)}", force=True)
            performance_monitor.record_mission_event('mission_error_gps', {'error': str(e)})
            
        except WaypointTimeoutError as e:
            self.logger.error(f"Waypoint timeout: {str(e)}")
            self.state_machine.transition_to(MissionPhase.ERROR, f"Waypoint timeout: {str(e)}", force=True)
            performance_monitor.record_mission_event('mission_error_timeout', {'error': str(e)})
            
        except WPMarkError as e:
            self.logger.error(f"Mission error: {str(e)}")
            self.state_machine.transition_to(MissionPhase.ERROR, f"Mission error: {str(e)}", force=True)
            performance_monitor.record_mission_event('mission_error', {'error': str(e)})
            
        except Exception as e:
            self.logger.error(f"Unexpected mission error: {str(e)}")
            self.state_machine.transition_to(MissionPhase.ERROR, f"Unexpected error: {str(e)}", force=True)
            performance_monitor.record_mission_event('mission_error_unexpected', {'error': str(e)})
            
        finally:
            # Cleanup
            self.logger.info("Mission execution ended")
            self.mission_active = False
            self.stop_event.clear()
    
    def start_mission(self) -> Dict[str, Any]:
        """
        Start mission execution with robust state management
        
        Returns:
            Dictionary with mission info
        """
        try:
            if self.state_machine.context.mission_active:
                return {
                    'success': False,
                    'error': 'Mission already running'
                }
            
            # Reset error state
            self.state_machine.reset_error_state()
            
            self.mission_active = True
            self.mission_start_time = time.time()
            self.stop_event.clear()
            
            # Update state machine
            self.state_machine.update_context(
                mission_active=True,
                waypoint_index=0,
                total_waypoints=len(self.waypoints)
            )
            
            # Start mission thread
            self.mission_thread = threading.Thread(target=self.execute_mission, daemon=True)
            self.mission_thread.start()
            
            # Wait a moment for initialization
            time.sleep(0.5)
            
            performance_monitor.record_mission_event('mission_started', {
                'waypoints': len(self.waypoints)
            })
            
            return {
                'success': True,
                'message': 'WP_MARK mission started successfully',
                'config': self.config,
                'mission_info': {
                    'total_waypoints': len(self.waypoints),
                    'started_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
                }
            }
        except StateTransitionError as e:
            self.logger.error(f"Failed to start mission: {str(e)}")
            return {
                'success': False,
                'error': f'State transition error: {str(e)}'
            }
        except Exception as e:
            self.logger.error(f"Unexpected error starting mission: {str(e)}")
            return {
                'success': False,
                'error': f'Unexpected error: {str(e)}'
            }
    
    def stop_mission(self) -> Dict[str, Any]:
        """
        Stop currently running mission with graceful state transition
        
        Returns:
            Dictionary with mission statistics
        """
        try:
            if not self.state_machine.context.mission_active:
                return {
                    'success': False,
                    'error': 'No mission currently running'
                }
            
            self.logger.warning("Stopping mission...")
            self.stop_event.set()
            
            # Try to transition to idle state
            try:
                self.state_machine.transition_to(MissionPhase.IDLE, "Mission stopped by user")
            except StateTransitionError:
                # Force transition if normal transition fails
                self.state_machine.transition_to(MissionPhase.IDLE, "Mission stopped by user", force=True)
            
            # Wait for mission thread to finish (with timeout)
            if self.mission_thread:
                self.mission_thread.join(timeout=5.0)
            
            duration = time.time() - self.mission_start_time if self.mission_start_time else 0
            
            performance_monitor.record_mission_event('mission_stopped', {
                'duration': duration,
                'waypoints_completed': self.current_waypoint_index
            })
            
            return {
                'success': True,
                'message': 'WP_MARK mission stopped successfully',
                'stats': {
                    'waypoints_completed': self.current_waypoint_index,
                    'total_waypoints': len(self.waypoints),
                    'duration_seconds': round(duration, 1)
                }
            }
        except Exception as e:
            self.logger.error(f"Error stopping mission: {str(e)}")
            return {
                'success': False,
                'error': f'Error stopping mission: {str(e)}'
            }
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get current mission status with comprehensive state information
        
        Returns:
            Dictionary with current status including robust state data
        """
        uptime = time.time() - self.mission_start_time if self.mission_start_time else 0
        
        # Get state machine status
        state_status = self.state_machine.get_status()
        
        # Combine with legacy status for backward compatibility
        status = {
            'running': self.mission_active,
            'current_waypoint': self.current_waypoint_index + 1,
            'total_waypoints': len(self.waypoints),
            'current_phase': state_status['phase'],  # Use state machine phase
            'config': self.config,
            'uptime_seconds': round(uptime, 1),
            'last_action': self.last_action,
            
            # Enhanced state information
            'robust_state': state_status,
            'health_status': performance_monitor.get_performance_report()['overall_health'],
            'error_count': state_status['context']['error_count'],
            'retry_count': state_status['context']['retry_count']
        }
        
        return status


# Global mission node instance
_mission_node: Optional[WPMarkMissionNode] = None
_node_lock = threading.Lock()
_executor: Optional[MultiThreadedExecutor] = None
_spin_thread: Optional[threading.Thread] = None


def get_mission_node() -> Optional[WPMarkMissionNode]:
    """Get global mission node instance"""
    with _node_lock:
        return _mission_node


def initialize_mission_node(config: WPMarkConfig) -> WPMarkMissionNode:
    """
    Initialize or reinitialize mission node
    
    Args:
        config: Mission configuration
        
    Returns:
        Mission node instance
    """
    global _mission_node, _executor, _spin_thread
    
    with _node_lock:
        # Shutdown existing node if running
        if _mission_node is not None:
            shutdown_mission_node()
        
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        
        # Create new node
        _mission_node = WPMarkMissionNode(config)
        
        # Create executor and spin in separate thread
        _executor = MultiThreadedExecutor()
        _executor.add_node(_mission_node)
        
        _spin_thread = threading.Thread(target=_executor.spin, daemon=True)
        _spin_thread.start()
        
        return _mission_node


def shutdown_mission_node():
    """Shutdown mission node and cleanup resources"""
    global _mission_node, _executor, _spin_thread
    
    with _node_lock:
        if _mission_node is not None:
            # Stop mission if running
            if _mission_node.mission_active:
                _mission_node.stop_mission()
            
            # Shutdown executor
            if _executor is not None:
                _executor.shutdown()
                _executor = None
            
            # Destroy node
            _mission_node.destroy_node()
            _mission_node = None
            
            # Wait for spin thread
            if _spin_thread is not None:
                _spin_thread.join(timeout=2.0)
                _spin_thread = None
