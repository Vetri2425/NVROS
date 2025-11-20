from __future__ import annotations

import eventlet
import subprocess
eventlet.monkey_patch()  # <-- ADD THIS LINE
import threading
import time
import math
import json
import base64
import socket
import sys
import tempfile
import os
from datetime import datetime
from collections import deque
from itertools import count
from flask import Flask, request, jsonify, Response, render_template
from flask_socketio import SocketIO, emit


# Mission Controller Process Management - REMOVED
# The mission controller node has been removed due to causing rover rotation issues
# with problematic Pixhawk parameter interactions

class RosNodeManager:
    def __init__(self):
        pass  # Mission controller removed

ros_manager = RosNodeManager()
from flask_cors import CORS
from pymavlink.dialects.v20 import common as mavlink
from dataclasses import dataclass, asdict, field
from typing import Optional, List, TextIO
import os
import atexit
import signal

try:
    import fcntl  # type: ignore[attr-defined]
except ImportError:  # pragma: no cover - Windows lacks fcntl
    fcntl = None  # type: ignore

try:
    # When running as a package (e.g., python -m Backend.server)
    from .mavros_bridge import MavrosBridge  # type: ignore
except Exception:
    # When running as a script from the Backend directory
    from mavros_bridge import MavrosBridge  # type: ignore

try:
    # Import integrated mission controller
    from .integrated_mission_controller import IntegratedMissionController, MissionState, MissionMode  # type: ignore
except Exception:
    from integrated_mission_controller import IntegratedMissionController, MissionState, MissionMode  # type: ignore


# Import network monitor for WiFi/LoRa status
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.network_monitor import NetworkMonitor



# --- Flask & SocketIO Setup ---
# --- Flask & SocketIO Setup ---
app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

# --- WebSocket Mission Controller Integration ---

# Enhanced CORS handling for external browser access
@app.before_request
def handle_preflight():
    """Handle CORS preflight OPTIONS requests"""
    if request.method == "OPTIONS":
        response = app.make_default_options_response()
        headers = response.headers
        headers['Access-Control-Allow-Origin'] = '*'
        headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
        headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization, X-Requested-With'
        headers['Access-Control-Max-Age'] = '3600'
        return response


@app.before_request
def log_request():
    """Log incoming HTTP requests for debugging."""
    # Skip logging for health checks to reduce noise
    if request.path != '/api/health':
        app.logger.info(f"{request.method} {request.path} from {request.remote_addr}")


@app.after_request
def log_response(response):
    """Log HTTP responses for debugging."""
    # Skip logging for health checks to reduce noise
    if request.path != '/api/health':
        app.logger.info(f"Response: {response.status_code} for {request.method} {request.path}")
    
    # Ensure CORS headers are set on all responses
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response



socketio = SocketIO(
    app, 
    cors_allowed_origins="*", 
    async_mode='eventlet',
    ping_timeout=60, 
    ping_interval=25,
    engineio_logger=False,
    logger=False,
    # Enable more aggressive reconnection handling
    always_connect=True,
    # Increase buffer size for better reliability
    max_http_buffer_size=1e8
)

# --- Mission Controller WebSocket Handlers - REMOVED ---
# Mission controller node and related handlers have been removed
# Use direct MAVROS integration for mission control instead

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')
_ros_lib_dir = f"/opt/ros/{ROS_DISTRO}/lib"
if os.path.isdir(_ros_lib_dir):
    _existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    if _ros_lib_dir not in _existing_ld.split(':'):
        _patched_ld = ':'.join(filter(None, [_ros_lib_dir, _existing_ld]))
        os.environ['LD_LIBRARY_PATH'] = _patched_ld

ROS_AVAILABLE = False
ROS_IMPORT_ERROR: Exception | None = None
telemetry_bridge = None
command_bridge: Optional["CommandBridge"] = None
# Executor may be created only when ROS is available.
_ros_executor: Optional["MultiThreadedExecutor"] = None # type: ignore
ros_thread: Optional[threading.Thread] = None
_singleton_lock_handle: Optional[TextIO] = None

try:
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from rclpy.executors import MultiThreadedExecutor  # type: ignore
    from std_msgs.msg import String  # type: ignore

    try:
        from mavros_msgs.srv import CommandBool, SetMode  # type: ignore
    except ImportError:
        CommandBool = None  # type: ignore
        SetMode = None  # type: ignore

    ROS_AVAILABLE = True
except Exception as exc:
    rclpy = None  # type: ignore
    Node = None  # type: ignore
    MultiThreadedExecutor = None  # type: ignore
    String = None  # type: ignore
    CommandBool = None  # type: ignore
    SetMode = None  # type: ignore
    ROS_IMPORT_ERROR = exc
    print(f"[WARN] ROS2 support disabled: {exc}", flush=True)


def log_message(message, level='INFO', event_type: str = 'general', meta=None):
    """Simple logger that prints to stdout, stores activity, and emits to frontend."""
    try:
        record_activity(message, level=level, event_type=event_type, meta=meta)
        ts = time.strftime('%Y-%m-%d %H:%M:%S')
        text = f"[{ts}] [{level}] {message}"
        print(text, flush=True)
        try:
            socketio.emit('server_log', {'message': text, 'level': level})
        except Exception:
            pass
    except Exception:
        try:
            print(f"[LOG-ERR] {message}", flush=True)
        except Exception:
            pass


if not ROS_AVAILABLE and ROS_IMPORT_ERROR is not None:
    log_message(
        f"ROS2 features disabled: {ROS_IMPORT_ERROR}",
        level='WARNING',
        event_type='startup',
        meta={'ros_distro': ROS_DISTRO}
    )

if ROS_AVAILABLE:

    class TelemetryBridge(Node):  # type: ignore[misc]
        def __init__(self):
            super().__init__('telemetry_bridge')
            self.subscription = self.create_subscription(
                String,
                '/nrp/telemetry',
                self.telemetry_callback,
                10
            )

        def telemetry_callback(self, msg):
            global _last_telemetry_activity_log
            try:
                telemetry_data = json.loads(msg.data)
                # Removed direct emit of 'telemetry' event to avoid redundant/partial updates
                # socketio.emit('telemetry', telemetry_data)
                
                # Debug: check if function exists
                import sys
                if '_merge_ros2_telemetry' not in globals():
                    print(f"[ERROR] _merge_ros2_telemetry not in globals! Available: {list(globals().keys())[:20]}", flush=True)
                else:
                    print(f"[DEBUG] _merge_ros2_telemetry found in globals", flush=True)
                
                _merge_ros2_telemetry(telemetry_data)
                now = time.time()
                if now - _last_telemetry_activity_log >= TELEMETRY_ACTIVITY_INTERVAL:
                    record_activity(
                        "Forwarded telemetry update to frontend",
                        level='DEBUG',
                        event_type='ros_topic',
                        meta={
                            'topic': '/nrp/telemetry',
                            'field_count': len(telemetry_data) if isinstance(telemetry_data, dict) else None,
                            'fields': sorted(list(telemetry_data.keys()))[:10] if isinstance(telemetry_data, dict) else None,
                        }
                    )
                    _last_telemetry_activity_log = now
            except Exception as e:
                log_message(f"Error processing telemetry: {e}", "ERROR", event_type='ros_topic')

    class CommandBridge(Node):  # type: ignore[misc]
        def __init__(self):
            if CommandBool is None or SetMode is None:
                raise RuntimeError("mavros_msgs.srv not available; CommandBridge disabled")
            super().__init__('command_bridge')
            self._lock = threading.Lock()
            self._set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
            self._arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        def _call_service(self, client, request, timeout: float) -> object:
            if client is None:
                raise RuntimeError("ROS2 client is not initialized")

            service_name = getattr(client, 'srv_name', 'unknown service')

            with self._lock:
                if not client.wait_for_service(timeout_sec=timeout):
                    raise TimeoutError(f"Service {service_name} not available")

                future = client.call_async(request)

            deadline = time.time() + max(timeout, 0.1)
            while time.time() < deadline:
                if future.done():
                    result = future.result()
                    if result is not None:
                        return result
                    exc = future.exception()
                    if exc:
                        raise exc
                    raise RuntimeError("Service call returned no result")
                time.sleep(0.01)

            raise TimeoutError(f"Service {service_name} timed out")

        def set_mode(self, *, mode: str, base_mode: int = 0, timeout: float = 5.0) -> dict:
            request = SetMode.Request()
            request.base_mode = int(base_mode)
            request.custom_mode = str(mode)
            response = self._call_service(self._set_mode_client, request, timeout)
            return {
                'success': bool(getattr(response, 'success', False)),
                'mode_sent': bool(getattr(response, 'mode_sent', False)),
            }

        def arm(self, *, value: bool, timeout: float = 5.0) -> dict:
            request = CommandBool.Request()
            request.value = bool(value)
            response = self._call_service(self._arm_client, request, timeout)
            return {
                'success': bool(getattr(response, 'success', False)),
                'result': bool(getattr(response, 'result', False)),
            }

    # Initialize ROS2 bridge (guard against multiple init calls)
    try:
        # Force initialization with try/except for duplicate init
        rclpy.init()
        print("[INFO] ROS2 context initialized successfully", flush=True)
        print(f"[DEBUG] server.py: rclpy.ok() after init = {rclpy.ok()}", flush=True)
        print(f"[DEBUG] server.py: rclpy module id = {id(rclpy)}", flush=True)
    except RuntimeError as exc:
        # Context already exists - this is OK
        if "has already been initialized" in str(exc) or "already been called" in str(exc):
            print(f"[INFO] ROS2 context already initialized (OK)", flush=True)
            print(f"[DEBUG] server.py: rclpy.ok() = {rclpy.ok()}", flush=True)
            print(f"[DEBUG] server.py: rclpy module id = {id(rclpy)}", flush=True)
        else:
            print(f"[ERROR] Unexpected rclpy.init() error: {exc}", flush=True)
            raise
    telemetry_bridge = TelemetryBridge()
    try:
        command_bridge = CommandBridge()
    except Exception as exc:
        print(f"Failed to initialize CommandBridge: {exc}", flush=True)
        command_bridge = None

    _ros_executor = MultiThreadedExecutor()
    _ros_executor.add_node(telemetry_bridge)
    if command_bridge is not None:
        _ros_executor.add_node(command_bridge)

    def _ros_executor_spin() -> None:
        """Run the ROS executor until shutdown, ignoring Ctrl-C noise."""
        try:
            _ros_executor.spin()  # type: ignore[union-attr]
        except KeyboardInterrupt:
            # Suppress noisy stack traces when the process is interrupted.
            pass
        except Exception as exc:
            print(f"[ROS] executor thread error: {exc}", flush=True)

    ros_thread = threading.Thread(target=_ros_executor_spin)
    ros_thread.daemon = True
    ros_thread.start()
else:

    class CommandBridge:  # type: ignore[no-redef]
        """ROS2 disabled placeholder to keep type-checkers satisfied."""

        def __init__(self):
            raise RuntimeError("ROS2 stack not available")


_ros_shutdown_lock = threading.Lock()
_ros_shutdown_done = False


def _shutdown_ros_runtime() -> None:
    """Best-effort cleanup for ROS resources when the process exits."""
    global _ros_shutdown_done, mission_controller
    with _ros_shutdown_lock:
        if _ros_shutdown_done:
            return
        _ros_shutdown_done = True
    
    print("[INFO] _shutdown_ros_runtime() called - cleaning up ROS resources", flush=True)
    import traceback
    traceback.print_stack()  # Show who called shutdown

    # Cleanup mission controller first
    try:
        if mission_controller is not None:
            print("[INFO] Shutting down integrated mission controller", flush=True)
            mission_controller.shutdown()
            mission_controller = None
    except Exception as exc:
        print(f"[WARN] Mission controller cleanup failed: {exc}", flush=True)

    try:
        if _ros_executor is not None:
            _ros_executor.shutdown()
    except Exception:
        pass

    try:
        if ros_thread is not None and ros_thread.is_alive() and threading.current_thread() is not ros_thread:
            ros_thread.join(timeout=2.0)
    except Exception:
        pass

    try:
        if _ros_executor is not None and telemetry_bridge is not None:
            _ros_executor.remove_node(telemetry_bridge)
    except Exception:
        pass
    try:
        if telemetry_bridge is not None:
            telemetry_bridge.destroy_node()
    except Exception:
        pass

    if command_bridge is not None:
        try:
            if _ros_executor is not None:
                _ros_executor.remove_node(command_bridge)
        except Exception:
            pass
        try:
            command_bridge.destroy_node()
        except Exception:
            pass

    try:
        if mavros_bridge is not None:
            mavros_bridge.close()
    except Exception:
        pass

    try:
        if ROS_AVAILABLE and rclpy is not None:
            print("[INFO] Calling rclpy.shutdown()", flush=True)
            rclpy.shutdown()
    except Exception as exc:
        print(f"[WARN] rclpy.shutdown() failed: {exc}", flush=True)

    if fcntl is not None:
        global _singleton_lock_handle
        if _singleton_lock_handle is not None:
            try:
                fcntl.flock(_singleton_lock_handle, fcntl.LOCK_UN)
            except Exception:
                pass
            try:
                _singleton_lock_handle.close()
            except Exception:
                pass
            _singleton_lock_handle = None


os.environ["MAVROS_BRIDGE_HOST"] = "127.0.0.1"    # Local MAVROS instance
os.environ["MAVROS_BRIDGE_PORT"] = "9090"         # rosbridge_server WebSocket port
os.environ["MAVROS_FCU_URL"] = "/dev/ttyACM0:115200"  # Rover connection parameters

# --- MAVROS Connection ---
mavros_bridge: Optional[MavrosBridge] = None
is_vehicle_connected = False
current_mission = []  # Store current mission waypoints
mavros_bridge_lock = threading.Lock()
mavros_telem_lock = threading.Lock()
mavros_connection_last_attempt = 0.0

# --- Integrated Mission Controller ---
mission_controller: Optional[IntegratedMissionController] = None
latest_mission_status: dict = {}

# PHASE 2 FIX: Status history cache for frontend reconnection
# Store last 10 mission status messages
mission_status_history: deque = deque(maxlen=10)

 

@dataclass
class Position:
    lat: float
    lng: float


@dataclass
class CurrentState:
    # Will be set when GPS available
    position: Optional[Position] = None
    heading: float = 0.0
    
    battery: int = -1
    status: str = 'disarmed'
    mode: str = 'UNKNOWN'
    rtk_status: str = 'No GPS'
    signal_strength: str = 'No Link'
    current_waypoint_id: Optional[int] = None
    rc_connected: bool = False
    last_heartbeat: Optional[float] = None
    # UI expected fields
    hrms: str = '0.000'
    vrms: str = '0.000'
    imu_status: str = 'UNALIGNED'
    imu: Optional[dict] = None
    satellites_visible: int = 0
    activeWaypointIndex: Optional[int] = None
    completedWaypointIds: List[int] = field(default_factory=list)
    distanceToNext: float = 0.0
    # Servo output telemetry (PWM values from /mavros/rc/out)
    servo_output: Optional[dict] = None
    # Ground speed (m/s) as derived from MAVROS velocity or ROS2 telemetry
    groundspeed: float = 0.0
    # RTK detailed status (for frontend RTKPanel)
    rtk_fix_type: int = 0
    rtk_baseline_age: float = 0.0
    rtk_base_linked: bool = False
    # Raw RTK baseline payload (from /mavros/gps_rtk/rtk_baseline)
    rtk_baseline: Optional[dict] = None
    # Timestamp (seconds since epoch) when last baseline was received
    rtk_baseline_ts: Optional[float] = None
    # Timestamp pushed to frontend (seconds since epoch)
    last_update: Optional[float] = None

    def to_dict(self) -> dict:
        d = asdict(self)
        return d


# Initialize current vehicle state
current_state = CurrentState()

# Initialize network monitor for WiFi and LoRa status telemetry
# Cache duration of 3 seconds to avoid overhead in high-frequency telemetry loop
network_monitor = NetworkMonitor(interface="wlan0", cache_duration=3.0)

mission_upload_lock = threading.Lock()
mission_download_lock = threading.Lock()

# Real-time streaming throttle configuration
def _compute_emit_interval() -> float:
    try:
        min_ms_env = os.getenv('TELEMETRY_MIN_MS')
        if min_ms_env is not None:
            min_ms = max(1.0, float(min_ms_env))
            return max(0.001, min_ms / 1000.0)
        hz = float(os.getenv('TELEMETRY_HZ', '20'))
        return max(0.001, 1.0 / max(1.0, hz))
    except Exception:
        return 0.05  # ~20Hz


EMIT_MIN_INTERVAL = _compute_emit_interval()
last_emit_monotonic = 0.0
_emit_lock = threading.Lock()
_emit_timer: Optional[object] = None

MISSION_LOG_HISTORY_MAX = 1000
mission_log_history: deque[dict] = deque(maxlen=MISSION_LOG_HISTORY_MAX)
mission_log_state = {
    'last_active_seq': None,
    'last_reached_seq': None,
}

ACTIVITY_LOG_MAX = int(os.getenv('ACTIVITY_LOG_MAX', '2000'))
activity_log: deque[dict] = deque(maxlen=ACTIVITY_LOG_MAX)
_activity_log_lock = threading.Lock()
_activity_seq = count(1)
TELEMETRY_ACTIVITY_INTERVAL = float(os.getenv('TELEMETRY_ACTIVITY_INTERVAL', '5.0'))
_last_telemetry_activity_log: float = 0.0

# HTTP response tracking log
_response_log: list[dict] = []

def _make_json_safe(obj, depth: int = 2):
    """Return a JSON-serialisable version of obj, truncating deeply nested data."""
    try:
        if depth < 0:
            return str(obj)
        if obj is None or isinstance(obj, (str, int, float, bool)):
            return obj
        if isinstance(obj, dict):
            limited = {}
            for idx, (key, value) in enumerate(obj.items()):
                if idx >= 15:
                    limited["..."] = f"+{len(obj) - idx} more"
                    break
                limited[str(key)] = _make_json_safe(value, depth - 1)
            return limited
        if isinstance(obj, (list, tuple, set)):
            seq = list(obj)
            limited = [_make_json_safe(value, depth - 1) for value in seq[:15]]
            if len(seq) > 15:
                limited.append(f"... (+{len(seq) - 15} more)")
            return limited
        return json.loads(json.dumps(obj))
    except Exception:
        return str(obj)

def record_activity(message: str, *, level: str = 'INFO', event_type: str = 'general', meta: dict | list | str | int | float | None = None):
    """Store and broadcast backend activity events."""
    timestamp = time.time()
    entry = {
        'id': next(_activity_seq),
        'timestamp': timestamp,
        'isoTime': time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(timestamp)),
        'level': str(level),
        'event': str(event_type),
        'message': str(message),
        'meta': _make_json_safe(meta),
    }
    try:
        with _activity_log_lock:
            activity_log.append(entry)
    except Exception:
        pass
    try:
        socketio.emit('server_activity', entry)
    except Exception:
        pass
    return entry

COMMAND_ID_TO_NAME = {
    mavlink.MAV_CMD_COMPONENT_ARM_DISARM: 'ARM_DISARM',
    mavlink.MAV_CMD_DO_SET_MODE: 'SET_MODE',
}

COMMAND_NAME_TO_ID = {
    'WAYPOINT': mavlink.MAV_CMD_NAV_WAYPOINT,
    'NAV_WAYPOINT': mavlink.MAV_CMD_NAV_WAYPOINT,
    'TAKEOFF': mavlink.MAV_CMD_NAV_TAKEOFF,
    'NAV_TAKEOFF': mavlink.MAV_CMD_NAV_TAKEOFF,
    'LAND': mavlink.MAV_CMD_NAV_LAND,
    'NAV_LAND': mavlink.MAV_CMD_NAV_LAND,
    'SPLINE_WAYPOINT': mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
    'DO_SET_SERVO': mavlink.MAV_CMD_DO_SET_SERVO,
    'DO_CHANGE_SPEED': mavlink.MAV_CMD_DO_CHANGE_SPEED,
    'DO_SET_HOME': mavlink.MAV_CMD_DO_SET_HOME,
    # 🔵 ADDED: wait N seconds before continuing (used for spray duration)
    'CONDITION_DELAY': mavlink.MAV_CMD_CONDITION_DELAY,
}

# Mode mappings
COPTER_MODES = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
    5: 'LOITER', 6: 'RTL', 7: 'CIRCLE', 9: 'LAND', 11: 'DRIFT',
    13: 'SPORT', 14: 'FLIP', 15: 'AUTOTUNE', 16: 'POSHOLD', 17: 'BRAKE',
    18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL',
    22: 'FLOWHOLD', 23: 'FOLLOW', 24: 'ZIGZAG', 25: 'SYSTEMID', 26: 'AUTOROTATE'
}

ROVER_MODES = {
    0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD', 10: 'AUTO',
    11: 'RTL', 12: 'SMART_RTL', 15: 'GUIDED', 16: 'INITIALISING'
}


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================


@app.before_request
def _track_http_request():
    """Record incoming HTTP requests (excluding Socket.IO transport noise)."""
    try:
        if request.path.startswith('/socket.io'):
            return
        meta: dict[str, object] = {
            'method': request.method,
            'path': request.path,
            'args': request.args.to_dict(flat=True),
        }
        if request.method in ('POST', 'PUT', 'PATCH'):
            body = request.get_json(silent=True)
            if isinstance(body, dict):
                meta['json_keys'] = sorted(list(body.keys()))[:15]
                if 'waypoints' in body and isinstance(body['waypoints'], list):
                    meta['waypoint_count'] = len(body['waypoints'])
        event_type = 'ui_request' if request.path.startswith('/api/') else 'http_request'
        record_activity(
            f"HTTP {request.method} {request.path} request",
            level='DEBUG',
            event_type=event_type,
            meta=meta
        )
    except Exception:
        pass


@app.after_request
def unified_response_handler(response):
    """Unified handler: Add CORS headers + track HTTP responses"""
    # Add CORS headers
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization, X-Requested-With'
    
    # Track HTTP responses for observability
    try:
        if request.path.startswith('/socket.io'):
            return response
        method = request.method
        path = request.path
        status = response.status_code
        _response_log.append({
            'ts': time.time(),
            'method': method,
            'path': path,
            'status': status
        })
        # Keep last 500
        if len(_response_log) > 500:
            _response_log.pop(0)
    except Exception:
        pass
    
    level = 'INFO' if response.status_code < 400 else 'ERROR'
    record_activity(
        f"HTTP {request.method} {request.path} responded",
        level=level,
        event_type='server_response',
        meta={
            'status': response.status_code,
            'content_type': response.content_type,
            'length': response.content_length,
        }
    )
    
    return response


def _current_position() -> tuple[float | None, float | None]:
    try:
        if current_state and current_state.position:
            return current_state.position.lat, current_state.position.lng
    except Exception:
        pass
    return None, None


def _record_mission_event(
    message: str,
    *,
    status: str | None = None,
    waypoint_id: int | None = None,
    servo_action: str | None = None
) -> None:
    lat, lng = _current_position()
    entry = {
        'timestamp': time.time(),
        'message': message,
        'status': status,
        'waypointId': waypoint_id,
        'servoAction': servo_action,
        'lat': lat,
        'lng': lng,
    }
    mission_log_history.append(entry)
    try:
        socketio.emit('mission_event', entry)
    except Exception:
        pass


def safe_float(value, default=0.0) -> float:
    """Best-effort float conversion with fallback."""
    try:
        if value is None:
            return float(default)
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _merge_ros2_telemetry(payload: dict) -> None:
    """Merge telemetry published via the ROS2 bridge into the shared rover state."""
    if not payload or not isinstance(payload, dict):
        return
    global is_vehicle_connected
    changed = False
    now = time.time()
    try:
        state = payload.get('state')
        global_data = payload.get('global')
        position = payload.get('position')  # Legacy support
        rtk = payload.get('rtk')

        with mavros_telem_lock:
            if isinstance(state, dict):
                armed_flag = bool(state.get('armed'))
                status = 'armed' if armed_flag else 'disarmed'
                if status != current_state.status:
                    current_state.status = status
                    changed = True
                connected = state.get('connected')
                if isinstance(connected, bool) and connected != is_vehicle_connected:
                    is_vehicle_connected = connected
                    changed = True
                mode = state.get('mode')
                if isinstance(mode, str) and mode:
                    normalized_mode = mode.upper()
                    if normalized_mode != current_state.mode:
                        current_state.mode = normalized_mode
                        changed = True
                heartbeat_ms = state.get('heartbeat_ts')
                if isinstance(heartbeat_ms, (int, float)) and heartbeat_ms > 0:
                    current_state.last_heartbeat = float(heartbeat_ms) / 1000.0
                system_status = state.get('system_status')
                if isinstance(system_status, str) and system_status:
                    current_state.signal_strength = derive_signal_strength(
                        current_state.last_heartbeat,
                        current_state.rc_connected
                    )
                current_state.last_update = now

            # Process global position data (NEW FORMAT)
            if isinstance(global_data, dict):
                lat = global_data.get('latitude')
                lon = global_data.get('longitude')
                alt = global_data.get('altitude')
                vel = global_data.get('vel')
                satellites = global_data.get('satellites_visible')
                
                if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                    current_state.position = Position(lat=float(lat), lng=float(lon))
                    changed = True
                    current_state.last_update = now
                
                # Update satellites count
                if isinstance(satellites, (int, float)):
                    if int(satellites) != current_state.satellites_visible:
                        current_state.satellites_visible = int(satellites)
                        changed = True

                # Map velocity from ROS2 'global.vel' into shared groundspeed field
                try:
                    if isinstance(vel, (int, float)):
                        new_gs = float(vel)
                        if current_state.groundspeed != new_gs:
                            current_state.groundspeed = new_gs
                            changed = True
                except Exception:
                    pass

            # Legacy position format support
            elif isinstance(position, dict):
                lat = position.get('latitude')
                lon = position.get('longitude')
                if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                    current_state.position = Position(lat=float(lat), lng=float(lon))
                    changed = True
                    current_state.last_update = now

            # Process RTK status data (NEW)
            if isinstance(rtk, dict):
                fix_type = rtk.get('fix_type')
                baseline_age = rtk.get('baseline_age')
                base_linked = rtk.get('base_linked')
                
                if isinstance(fix_type, (int, float)):
                    fix_type_int = int(fix_type)
                    # Map fix type to RTK status string
                    rtk_status_map = {
                        0: 'No GPS',
                        1: 'GPS Fix',
                        2: 'DGPS',
                        3: 'RTK Float',
                        4: 'RTK Fixed'
                    }
                    new_rtk_status = rtk_status_map.get(fix_type_int, 'No GPS')
                    if new_rtk_status != current_state.rtk_status:
                        current_state.rtk_status = new_rtk_status
                        changed = True
                    
                    # Update detailed RTK fields for frontend
                    if current_state.rtk_fix_type != fix_type_int:
                        current_state.rtk_fix_type = fix_type_int
                        changed = True
                
                if isinstance(baseline_age, (int, float)):
                    baseline_age_float = float(baseline_age)
                    if current_state.rtk_baseline_age != baseline_age_float:
                        current_state.rtk_baseline_age = baseline_age_float
                        changed = True
                
                if isinstance(base_linked, bool):
                    if current_state.rtk_base_linked != base_linked:
                        current_state.rtk_base_linked = base_linked
                        changed = True

            # Process battery telemetry if provided via ROS2 bridge (/nrp/telemetry)
            battery = payload.get('battery')
            if isinstance(battery, dict):
                try:
                    pct = battery.get('percentage')
                    volt = battery.get('voltage')
                    curr = battery.get('current')
                    # Accept either fraction (0.0-1.0) or percent (0-100)
                    if pct is not None:
                        try:
                            pctf = float(pct)
                            # Check for NaN or invalid values
                            import math
                            if not math.isnan(pctf) and not math.isinf(pctf):
                                if pctf <= 1.0:
                                    pct_val = max(0, int(round(pctf * 100)))
                                else:
                                    pct_val = max(0, int(round(pctf)))
                                if current_state.battery != pct_val:
                                    current_state.battery = pct_val
                                    changed = True
                        except Exception:
                            pass
                    # Debug: log battery values received from ROS2 telemetry
                    try:
                        log_message(f"ROS2 telemetry battery: pct={pct}, volt={volt}, curr={curr}", 'DEBUG', event_type='ros_topic')
                    except Exception:
                        pass
                    if volt is not None:
                        try:
                            current_state.voltage = float(volt)  # may be used by future UI
                            changed = True
                        except Exception:
                            pass
                    if curr is not None:
                        try:
                            current_state.current = float(curr)  # may be used by future UI
                            changed = True
                        except Exception:
                            pass
                    if changed:
                        current_state.last_update = now
                except Exception:
                    pass
    except Exception as exc:
        log_message(f"Failed to merge ROS2 telemetry: {exc}", "WARNING", event_type='ros_topic')
        return

    if changed:
        schedule_fast_emit()


# ============================================================
# SERVO CONTROL MISSION BUILDER (3 modes)
# ============================================================

def _is_nav_waypoint(cmd_name_or_id) -> bool:
    cid = resolve_mav_command(cmd_name_or_id)
    return command_requires_nav_coordinates(cid)

def _mk_do_set_servo(channel: int, pwm: float) -> dict:
    # DO_SET_SERVO uses param1=servo no, param2=PWM. No coordinates required.
    return {
        'command': 'DO_SET_SERVO',
        'param1': float(channel),
        'param2': float(pwm),
        'param3': 0.0,
        'param4': 0.0,
        'lat': 0.0, 'lng': 0.0, 'alt': 0.0,
        'frame': mavlink.MAV_FRAME_MISSION
    }

def _mk_condition_delay(seconds: float) -> dict:
    # Wait N seconds before proceeding
    return {
        'command': 'CONDITION_DELAY',
        'param1': float(seconds),
        'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
        'lat': 0.0, 'lng': 0.0, 'alt': 0.0,
        'frame': mavlink.MAV_FRAME_MISSION
    }

def _mk_nav_wp(lat: float, lng: float, alt: float, frame: int = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT) -> dict:
    return {
        'command': 'WAYPOINT',
        'lat': float(lat), 'lng': float(lng), 'alt': float(alt),
        'frame': int(frame),
        'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0
    }

def apply_servo_modes(waypoints: list[dict], servo_config: dict) -> list[dict]:
    """
    Expand mission with DO_SET_SERVO items based on the popup config.
    Modes:
      1) MARK_AT_WAYPOINT: ON -> delay(s) -> OFF after each WP in [fromWp..toWp]
      2) CONTINUOUS_LINE: ON after startPoint, OFF after endPoint
      3) INTERVAL_SPRAY: insert mini-waypoints every intervalCm, toggling ON/OFF
    """
    if not servo_config:
        return waypoints

    mode = str(servo_config.get('mode', '')).upper()
    servo = int(safe_float(servo_config.get('servoNumber', 10), 10))
    pwm_on = float(safe_float(servo_config.get('pwmOn', 650), 650))
    pwm_off = float(safe_float(servo_config.get('pwmOff', 1000), 1000))

    # Helper: iterate original nav WPs with a 1-based index
    def _iter_nav_indexed():
        for idx, wp in enumerate(waypoints, start=1):
            yield (idx, wp)

    # -------------------------
    # Mode 1: MARK_AT_WAYPOINT
    # -------------------------
    if mode == 'MARK_AT_WAYPOINT':
        from_wp = int(safe_float(servo_config.get('fromWp', 1), 1))
        to_wp = int(safe_float(servo_config.get('toWp', len(waypoints)), len(waypoints)))
        dur_s = float(safe_float(servo_config.get('sprayDuration', 0.5), 0.5))

        new_list: list[dict] = []
        for idx, wp in _iter_nav_indexed():
            new_list.append(wp)
            if from_wp <= idx <= to_wp and _is_nav_waypoint(wp.get('command')):
                # ON right after reaching this WP, hold for dur_s, then OFF
                new_list.append(_mk_do_set_servo(servo, pwm_on))
                if dur_s > 0:
                    new_list.append(_mk_condition_delay(dur_s))
                new_list.append(_mk_do_set_servo(servo, pwm_off))
        return new_list

    # -------------------------
    # Mode 2: CONTINUOUS_LINE
    # -------------------------
    if mode == 'CONTINUOUS_LINE':
        start_pt = int(safe_float(servo_config.get('startPoint', 1), 1))
        end_pt = int(safe_float(servo_config.get('endPoint', len(waypoints)), len(waypoints)))
        if end_pt < start_pt:
            end_pt = start_pt

        new_list: list[dict] = []
        for idx, wp in _iter_nav_indexed():
            new_list.append(wp)
            if idx == start_pt and _is_nav_waypoint(wp.get('command')):
                # turn ON immediately after we "arrive" at start WP
                new_list.append(_mk_do_set_servo(servo, pwm_on))
            if idx == end_pt and _is_nav_waypoint(wp.get('command')):
                # turn OFF after end WP
                new_list.append(_mk_do_set_servo(servo, pwm_off))
        return new_list

    # -------------------------
    # Mode 3: INTERVAL_SPRAY
    # -------------------------
    if mode == 'INTERVAL_SPRAY':
        interval_cm = float(safe_float(servo_config.get('intervalCm', 30.0), 30.0))
        step_m = max(0.01, interval_cm / 100.0)  # meters

        # Prepare a new mission with sub-steps between each consecutive NAV WP
        import math
        def haversine_m(lat1, lon1, lat2, lon2):
            # rough distance in meters (good enough for path splitting)
            R = 6371000.0
            dlat = math.radians(lat2 - lat1)
            dlon = math.radians(lon2 - lon1)
            a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
            return 2 * R * math.asin(min(1, math.sqrt(a)))

        def lerp(a, b, t): return a + (b - a) * t

        # filter only the NAV waypoints (we preserve order and any pre-existing DO_* too)
        # but we build intervals only between NAV->NAV legs
        new_list: list[dict] = []
        toggle_on = True  # alternate ON/OFF per sub-step

        # build a list of indices of NAV waypoints in original list
        nav_indices = [i for i, w in enumerate(waypoints) if _is_nav_waypoint(w.get('command'))]

        for i, wp in enumerate(waypoints):
            new_list.append(wp)

            # if this is a NAV wp and the next NAV wp exists, insert sub-steps after this one
            if i in nav_indices:
                cur_idx_in_nav = nav_indices.index(i)
                if cur_idx_in_nav < len(nav_indices) - 1:
                    a = waypoints[nav_indices[cur_idx_in_nav]]
                    b = waypoints[nav_indices[cur_idx_in_nav + 1]]
                    lat1, lon1, alt1 = float(a.get('lat', 0)), float(a.get('lng', 0)), float(a.get('alt', 0))
                    lat2, lon2, alt2 = float(b.get('lat', 0)), float(b.get('lng', 0)), float(b.get('alt', 0))
                    dist = haversine_m(lat1, lon1, lat2, lon2)
                    steps = int(max(0, math.floor(dist / step_m)))

                    # Insert at each sub-distance: a small NAV WP + a DO_SET_SERVO toggle
                    for s in range(1, steps + 1):
                        t = s / (steps + 1)
                        lat = lerp(lat1, lat2, t)
                        lon = lerp(lon1, lon2, t)
                        alt = lerp(alt1, alt2, t)

                        # go to subpoint
                        new_list.append(_mk_nav_wp(lat, lon, alt))
                        # toggle servo value at this subpoint
                        new_list.append(_mk_do_set_servo(servo, pwm_on if toggle_on else pwm_off))
                        toggle_on = not toggle_on

        return new_list

    # Fallback: unchanged
    return waypoints

def resolve_mav_command(command_value) -> int:
    """Return MAV_CMD integer for mission item."""
    if isinstance(command_value, (int, float)):
        return int(command_value)
    if isinstance(command_value, str):
        normalized = command_value.strip().upper()
        resolved = COMMAND_NAME_TO_ID.get(normalized)
        if resolved is not None:
            return resolved
        log_message(f"Unknown mission command '{command_value}', defaulting to NAV_WAYPOINT", "WARNING")
    return mavlink.MAV_CMD_NAV_WAYPOINT


def command_requires_nav_coordinates(command_id: int) -> bool:
    """True when mission item expects lat/lon/alt."""
    return command_id in {
        mavlink.MAV_CMD_NAV_WAYPOINT,
        mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
        mavlink.MAV_CMD_NAV_TAKEOFF,
        mavlink.MAV_CMD_NAV_LAND,
        mavlink.MAV_CMD_NAV_LOITER_TIME,
        mavlink.MAV_CMD_NAV_LOITER_TURNS,
        mavlink.MAV_CMD_NAV_LOITER_UNLIM,
    }


def get_mode_name(mode_num, vehicle_type='ROVER'):
    """Convert mode number to mode name based on vehicle type."""
    if vehicle_type == 'ROVER':
        return ROVER_MODES.get(mode_num, f'UNKNOWN({mode_num})')
    else:
        return COPTER_MODES.get(mode_num, f'UNKNOWN({mode_num})')


def get_mode_number(mode_name, vehicle_type='ROVER'):
    """Convert mode name to mode number based on vehicle type."""
    mode_map = ROVER_MODES if vehicle_type == 'ROVER' else COPTER_MODES
    for num, name in mode_map.items():
        if name == mode_name.upper():
            return num
    return None


def _map_navsat_status(status_code: Optional[int]) -> str:
    """Map sensor_msgs/NavSatStatus codes to human readable text."""
    mapping = {
        None: 'Unknown',
        -1: 'No Fix',
        0: 'GPS Fix',
        1: 'DGPS',
        2: 'RTK Fixed',
        3: 'RTK Float',
    }
    return mapping.get(status_code, f"Status {status_code}")


def _convert_mavros_waypoints_to_ui(waypoints: List[dict]) -> List[dict]:
    """Convert MAVROS waypoint dictionaries into the UI mission format."""
    converted: List[dict] = []
    for idx, wp in enumerate(waypoints or []):
        try:
            cmd = int(wp.get("command", mavlink.MAV_CMD_NAV_WAYPOINT))
        except Exception:
            cmd = mavlink.MAV_CMD_NAV_WAYPOINT
        converted.append({
            "id": idx + 1,
            "command": 'WAYPOINT' if cmd == mavlink.MAV_CMD_NAV_WAYPOINT else COMMAND_ID_TO_NAME.get(cmd, f'COMMAND_{cmd}'),
            "lat": safe_float(wp.get("x_lat", 0.0)),
            "lng": safe_float(wp.get("y_long", 0.0)),
            "alt": safe_float(wp.get("z_alt", 0.0)),
            "frame": int(safe_float(wp.get("frame", mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
                                   mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)),
            "param1": safe_float(wp.get("param1", 0.0)),
            "param2": safe_float(wp.get("param2", 0.0)),
            "param3": safe_float(wp.get("param3", 0.0)),
            "param4": safe_float(wp.get("param4", 0.0)),
            "autocontinue": int(safe_float(wp.get("autocontinue", 1), 1)),
        })
    return converted


def _build_mavros_waypoints(waypoints: List[dict]) -> List[dict]:
    """Translate UI waypoint dictionaries into MAVROS Waypoint structures."""
    mavros_waypoints: List[dict] = []
    for idx, wp in enumerate(waypoints):
        command_id = resolve_mav_command(wp.get("command"))
        frame = int(safe_float(wp.get("frame", mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
                               mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))
        autocontinue = bool(safe_float(wp.get("autocontinue", 1), 1))
        param1 = safe_float(wp.get("param1", 0.0))
        param2 = safe_float(wp.get("param2", 0.0))
        param3 = safe_float(wp.get("param3", 0.0))
        param4 = safe_float(wp.get("param4", 0.0))

        # Unified coordinate extraction - always prioritize lat/lng field names for consistency
        # This ensures coordinates are never swapped regardless of command type
        lat = safe_float(wp.get("lat", wp.get("x_lat", 0.0)))
        lon = safe_float(wp.get("lng", wp.get("y_long", 0.0)))
        alt = safe_float(wp.get("alt", wp.get("z_alt", 0.0)))

        mavros_waypoints.append({
            "frame": frame,
            "command": command_id,
            "is_current": idx == 0,
            "autocontinue": autocontinue,
            "param1": param1,
            "param2": param2,
            "param3": param3,
            "param4": param4,
            "x_lat": lat,
            "y_long": lon,
            "z_alt": alt,
        })
    return mavros_waypoints


def derive_signal_strength(last_heartbeat: float | None, rc_connected: bool) -> str:
    """Return a human readable link quality label for the UI."""
    if last_heartbeat is None:
        return 'No Link'
    age = time.time() - last_heartbeat
    if age <= 2:
        return 'Excellent' if rc_connected else 'Good'
    if age <= 5:
        return 'Fair'
    if age <= 10:
        return 'Weak'
    return 'Lost'


def _get_vehicle_bridge(*, require_vehicle: bool = True, timeout: float = 5.0) -> MavrosBridge:
    """
    Return a MAVROS bridge, optionally ensuring the rover reports as connected.

    When attempting to arm/disarm the vehicle we may not yet have seen a telemetry
    heartbeat, but the command can still succeed; allowing callers to bypass the
    vehicle connectivity assertion prevents false negatives.
    """
    bridge = _init_mavros_bridge()
    if not bridge.is_connected:
        bridge.connect(timeout=timeout)
    if require_vehicle and not is_vehicle_connected:
        raise RuntimeError("Vehicle not connected")
    return bridge


def handle_mission_status(status_data: dict):
    """Handle mission status updates from integrated controller and emit to frontend"""
    global latest_mission_status, mission_status_history
    try:
        # Store latest status
        latest_mission_status = status_data

        # Add server timestamp
        status_data['server_timestamp'] = time.time()

        # PHASE 2 FIX: Cache status in history for frontend reconnection
        mission_status_history.append(status_data.copy())

        # Log important updates
        level = status_data.get('level', 'info')
        message = status_data.get('message', '')

        if level == 'error':
            log_message(f"Mission: {message}", "ERROR", event_type='mission')
        elif level == 'warning':
            log_message(f"Mission: {message}", "WARNING", event_type='mission')
        elif level == 'success':
            log_message(f"Mission: {message}", "INFO", event_type='mission')

        # PHASE 2 FIX: Add emission verification logging
        # Log critical mission events to track if they reach frontend
        event_type = status_data.get('extra_data', {}).get('event_type') if 'extra_data' in status_data else None
        if event_type in ['waypoint_reached', 'waypoint_marked']:
            log_message(f"📡 EMITTING {event_type}: WP{status_data.get('current_waypoint')} - {message}", "INFO", event_type='mission')

        # Emit to frontend via WebSocket
        socketio.emit('mission_status', status_data, namespace='/')

        # PHASE 2 FIX: Log emission success for critical events
        if event_type in ['waypoint_reached', 'waypoint_marked']:
            log_message(f"✓ Emission sent to frontend: {event_type}", "DEBUG", event_type='mission')

    except Exception as e:
        log_message(f"Mission status handler error: {e}", "ERROR", event_type='mission')


def initialize_mission_controller():
    """Initialize the integrated mission controller after MAVROS bridge connects"""
    global mission_controller
    try:
        bridge = _init_mavros_bridge()
        mission_controller = IntegratedMissionController(
            mavros_bridge=bridge,
            status_callback=handle_mission_status,
            logger=app.logger
        )
        log_message("Integrated mission controller initialized successfully", "SUCCESS", event_type='mission')
        return True
    except Exception as e:
        log_message(f"Failed to initialize mission controller: {e}", "ERROR", event_type='mission')
        return False


def _require_vehicle_bridge() -> MavrosBridge:
    """Backward-compatible helper enforcing a connected vehicle."""
    return _get_vehicle_bridge(require_vehicle=True)


# ============================================================================
# TELEMETRY EMISSION
# ============================================================================

def emit_rover_data_now(reason: str | None = None) -> None:
    """Emit current_state to all clients with throttling bookkeeping."""
    global last_emit_monotonic
    data = get_rover_data()
    
    # DEBUG: Log position data being sent to frontend
    if data.get('position'):
        pos = data['position']
        print(f"[EMIT] Sending rover_data ({reason}): position={pos}", flush=True)
    else:
        print(f"[EMIT] Sending rover_data ({reason}): NO POSITION DATA", flush=True)
    
    try:
        socketio.emit('rover_data', data)
        if reason:
            log_message(f"rover_data emitted ({reason}) @ {time.strftime('%H:%M:%S')}", 'INFO')
    finally:
        last_emit_monotonic = time.monotonic()


def _emit_timer_cb():
    global _emit_timer
    with _emit_lock:
        _emit_timer = None
    emit_rover_data_now(reason='throttled')


def schedule_fast_emit():
    """Emit at most EMIT_MIN_INTERVAL; schedule deferred emit if needed."""
    global _emit_timer
    now = time.monotonic()
    elapsed = now - last_emit_monotonic
    if elapsed >= EMIT_MIN_INTERVAL:
        emit_rover_data_now(reason='realtime')
        return
    delay = EMIT_MIN_INTERVAL - elapsed
    with _emit_lock:
        if _emit_timer is None:
            try:
                _emit_timer = eventlet.spawn_after(delay, _emit_timer_cb)
            except Exception:
                emit_rover_data_now(reason='realtime')


def get_rover_data():
    """Return complete rover data structure with all required fields."""
    try:
        current_state.signal_strength = derive_signal_strength(
            current_state.last_heartbeat, current_state.rc_connected
        )
    except Exception:
        pass
    
    data = current_state.to_dict()
    
    # Add network telemetry (WiFi signal strength and LoRa status)
    try:
        network_data = network_monitor.get_network_data()
        data['network'] = network_data
    except Exception as e:
        # Provide safe defaults on network monitoring failure
        print(f"[WARN] Network monitoring error: {e}", flush=True)
        data['network'] = {
            'connection_type': 'none',
            'wifi_signal_strength': 0,
            'wifi_rssi': -100,
            'interface': 'unknown',
            'wifi_connected': False,
            'lora_connected': False
        }
    
    # Compute baseline age dynamically from last baseline timestamp if available
    try:
        if getattr(current_state, 'rtk_baseline_ts', None) is not None:
            age = time.time() - float(current_state.rtk_baseline_ts)
            # Update the dataclass value so emitted dict contains current age
            current_state.rtk_baseline_age = float(age)
            # Ensure the emitted dict reflects updated value
            data = current_state.to_dict()
    except Exception:
        pass

    # Debug: Log RTK fix type being sent
    print(f"[DEBUG] get_rover_data() sending rtk_fix_type={current_state.rtk_fix_type}, rtk_base_linked={current_state.rtk_base_linked}, rtk_baseline_age={current_state.rtk_baseline_age}", flush=True)
    
    return data


# ============================================================================
# MAVLINK CONNECTION & MESSAGE PROCESSING
# ============================================================================

def _init_mavros_bridge() -> MavrosBridge:
    """Ensure a single MavrosBridge instance and attach telemetry subscriptions."""
    global mavros_bridge
    with mavros_bridge_lock:
        if mavros_bridge is None:
            log_message("Creating MAVROS bridge instance")
            mavros_bridge = MavrosBridge()
            mavros_bridge.subscribe_telemetry(_handle_mavros_telemetry)
    return mavros_bridge


def maintain_mavros_connection():
    """Establish and monitor the MAVROS bridge connection."""
    global is_vehicle_connected, mavros_connection_last_attempt, mission_controller
    while True:
        try:
            bridge = _init_mavros_bridge()
            if bridge.is_connected:
                # Initialize mission controller if not already done and bridge is connected
                if mission_controller is None:
                    initialize_mission_controller()
                socketio.sleep(2.0)
                continue

            now = time.time()
            if now - mavros_connection_last_attempt < 1.0:
                socketio.sleep(1.0)
                continue

            mavros_connection_last_attempt = now
            log_message(f"Attempting MAVROS connection to {bridge.host}:{bridge.port}")
            bridge.connect(timeout=float(os.getenv("MAVROS_CONNECT_TIMEOUT", "10")))
            log_message("MAVROS bridge connected to rosbridge", "SUCCESS")
            
            # Initialize mission controller after successful connection
            if mission_controller is None:
                initialize_mission_controller()
                
        except Exception as exc:
            if is_vehicle_connected:
                is_vehicle_connected = False
                log_message("Vehicle connection lost via MAVROS", "ERROR")
                socketio.emit('connection_status', {
                    'status': 'WAITING_FOR_ROVER',
                    'message': str(exc)
                })
            log_message(f"MAVROS connection error: {exc}", "ERROR")
            socketio.sleep(2.0)
        socketio.sleep(2.0)


def _handle_mavros_telemetry(message: dict) -> None:
    """Translate MAVROS telemetry payloads into the shared rover state."""
    global is_vehicle_connected, current_mission
    msg_type = str(message.get("type", "")).lower()

    try:
        if msg_type == "state":
            connected = bool(message.get("connected", False))
            armed = bool(message.get("armed", False))
            mode = str(message.get("mode", "UNKNOWN")).upper()
            previous = is_vehicle_connected
            is_vehicle_connected = connected

            with mavros_telem_lock:
                current_state.status = "armed" if armed else "disarmed"
                current_state.mode = mode
                current_state.last_heartbeat = time.time() if connected else None
                current_state.signal_strength = derive_signal_strength(
                    current_state.last_heartbeat,
                    current_state.rc_connected
                )
                current_state.last_update = time.time()

            if connected and not previous:
                log_message("Vehicle connected through MAVROS", "SUCCESS")
                socketio.emit('connection_status', {'status': 'CONNECTED_TO_ROVER'})
            elif not connected and previous:
                log_message("Vehicle disconnected from MAVROS", "WARNING")
                socketio.emit('connection_status', {'status': 'WAITING_FOR_ROVER'})

            schedule_fast_emit()

        elif msg_type == "navsat":
            lat = message.get("latitude")
            lon = message.get("longitude")
            alt = message.get("altitude", 0.0)
            
            # DEBUG: Log navsat message reception
            print(f"[SERVER] Received navsat: lat={lat}, lon={lon}, alt={alt}", flush=True)
            
            if lat is not None and lon is not None:
                with mavros_telem_lock:
                    current_state.position = Position(lat=float(lat), lng=float(lon))
                    current_state.last_update = time.time()
                    current_state.distanceToNext = float(alt or 0.0)
                    print(f"[SERVER] Updated current_state.position: lat={current_state.position.lat:.7f}, lng={current_state.position.lng:.7f}", flush=True)
                schedule_fast_emit()

        elif msg_type == "gps_fix":
            # gps_fix messages may come from the bridge with either derived hrms/vrms
            # fields (preferred) or as a standard NavSatFix-like payload with
            # position_covariance. Prefer explicit hrms/vrms when provided.
            status = message.get("status")
            fix_type = message.get("fix_type")
            satellites_visible = message.get("satellites_visible")
            covariance = message.get("position_covariance", [])
            hrms_msg = message.get("hrms")
            vrms_msg = message.get("vrms")

            with mavros_telem_lock:
                current_state.rtk_status = _map_navsat_status(status)

                # RTK fix type added - Update detailed RTK fields for frontend RTKPanel
                if fix_type is not None:
                    fix_type_int = int(fix_type)
                    current_state.rtk_fix_type = fix_type_int
                    # Respect recent RTK baseline information if present; only infer from fix_type
                    # when no recent baseline has been received (fallback).
                    try:
                        baseline_ts = getattr(current_state, 'rtk_baseline_ts', None)
                        if baseline_ts is None or (time.time() - float(baseline_ts)) > 5.0:
                            current_state.rtk_base_linked = (fix_type_int >= 5)
                    except Exception:
                        current_state.rtk_base_linked = (fix_type_int >= 5)

                # Update satellites count
                if satellites_visible is not None:
                    current_state.satellites_visible = int(satellites_visible)

                # Extract baseline age if available
                baseline_age = message.get("baseline_age")
                if baseline_age is not None:
                    current_state.rtk_baseline_age = float(baseline_age)

                # Prefer explicit hrms/vrms published by the bridge
                try:
                    if hrms_msg is not None:
                        current_state.hrms = f"{float(hrms_msg):.3f}"
                    if vrms_msg is not None:
                        current_state.vrms = f"{float(vrms_msg):.3f}"
                except Exception:
                    # ignore malformed values and fall back to covariance
                    pass

                # Fallback: compute from NavSat covariance if explicit metrics not present
                if ((hrms_msg is None) or (vrms_msg is None)) and isinstance(covariance, list) and len(covariance) >= 3:
                    try:
                        hrms = math.sqrt(abs(float(covariance[0])))
                        vrms = math.sqrt(abs(float(covariance[2])))
                        # Only overwrite if not already set by explicit fields
                        if hrms_msg is None:
                            current_state.hrms = f"{hrms:.3f}"
                        if vrms_msg is None:
                            current_state.vrms = f"{vrms:.3f}"
                    except Exception:
                        current_state.hrms = current_state.hrms or '0.000'
                        current_state.vrms = current_state.vrms or '0.000'

                current_state.last_update = time.time()
            schedule_fast_emit()

        elif msg_type == "estimator_status":
            # Bridge publishes a normalized 'imu_aligned' boolean when available
            imu_aligned = message.get('imu_aligned')
            if isinstance(imu_aligned, bool):
                with mavros_telem_lock:
                    new_status = 'ALIGNED' if imu_aligned else 'UNALIGNED'
                    if current_state.imu_status != new_status:
                        current_state.imu_status = new_status
                        current_state.last_update = time.time()
                schedule_fast_emit()

        elif msg_type == 'imu':
            # Store last IMU sample (orientation / angular velocity / linear acc)
            imu_payload = message.get('imu')
            if isinstance(imu_payload, dict):
                with mavros_telem_lock:
                    current_state.imu = imu_payload
                    current_state.last_update = time.time()
                schedule_fast_emit()

        elif msg_type == "heading":
            heading = message.get("heading")
            if heading is not None:
                with mavros_telem_lock:
                    current_state.heading = float(heading)
                    current_state.last_update = time.time()
            schedule_fast_emit()

        elif msg_type == "velocity":
            # Velocity messages published by MAVROS bridge may include a 'groundspeed' key.
            gs = message.get('groundspeed')
            if gs is not None:
                try:
                    with mavros_telem_lock:
                        current_state.groundspeed = float(gs)
                        current_state.last_update = time.time()
                except Exception:
                    pass
                schedule_fast_emit()

        elif msg_type == "rtk_baseline":
            # Baseline vectors from RTK base (broadcast by MAVROS bridge)
            baseline = message.get('rtk_baseline') or message
            print(f"[DEBUG] rtk_baseline handler: message keys={list(message.keys())}, baseline type={type(baseline)}", flush=True)
            try:
                with mavros_telem_lock:
                    current_state.rtk_baseline = dict(baseline) if isinstance(baseline, dict) else None
                    current_state.rtk_baseline_ts = time.time()
                    # Reset/initialize baseline_age to zero on receipt
                    current_state.rtk_baseline_age = 0.0
                    print(f"[DEBUG] Stored rtk_baseline: {current_state.rtk_baseline is not None}, ts={current_state.rtk_baseline_ts}", flush=True)

                    # Heuristic for base_linked: explicit field, or use iar_num_hypotheses==1 or small accuracy
                    base_linked = None
                    if isinstance(baseline, dict):
                        if 'base_linked' in baseline and isinstance(baseline.get('base_linked'), bool):
                            base_linked = bool(baseline.get('base_linked'))
                        else:
                            iar = baseline.get('iar_num_hypotheses')
                            acc = baseline.get('accuracy')
                            try:
                                if iar is not None and int(iar) == 1:
                                    base_linked = True
                                elif acc is not None:
                                    # accuracy in meters: consider very small accuracy as linked
                                    if float(acc) <= 0.05:
                                        base_linked = True
                                    else:
                                        base_linked = False
                            except Exception:
                                base_linked = None

                    if isinstance(base_linked, bool):
                        current_state.rtk_base_linked = base_linked
                        try:
                            log_message(f"[RTK] rtk_baseline received, setting rtk_base_linked={base_linked}", 'DEBUG')
                        except Exception:
                            pass
                    else:
                        # Fallback: infer from rtk_fix_type if available
                        try:
                            current_state.rtk_base_linked = (current_state.rtk_fix_type >= 5)
                            try:
                                log_message(f"[RTK] rtk_baseline received, inferred rtk_base_linked from fix_type: {current_state.rtk_base_linked}", 'DEBUG')
                            except Exception:
                                pass
                        except Exception:
                            pass

                    current_state.last_update = time.time()
            except Exception:
                pass
            schedule_fast_emit()

        elif msg_type == "battery":
            # Accept either 'percentage' (fraction 0.0-1.0 or 0-100) or 'battery' (already percent)
            perc_value = message.get("percentage")
            if perc_value is None:
                perc_value = message.get("battery")
            volt_value = message.get("voltage")
            curr_value = message.get("current")

            try:
                log_message(f"Battery telem received: perc={perc_value}, volt={volt_value}, curr={curr_value}, keys={list(message.keys())}", 'DEBUG', event_type='ros_topic')
            except Exception:
                pass

            updated = False
            if perc_value is not None:
                with mavros_telem_lock:
                    try:
                        pctf = float(perc_value)
                        # Check for NaN or invalid values
                        import math
                        if not math.isnan(pctf) and not math.isinf(pctf):
                            pct_val = max(0, int(round(pctf * 100))) if pctf <= 1.0 else max(0, int(round(pctf)))
                            if current_state.battery != pct_val:
                                current_state.battery = pct_val
                                updated = True
                    except Exception:
                        pass

            # Optionally store voltage/current for future UI use
            with mavros_telem_lock:
                try:
                    if volt_value is not None:
                        v = float(volt_value)
                        if not math.isnan(v) and not math.isinf(v):
                            current_state.voltage = v  # type: ignore[attr-defined]
                            updated = True
                except Exception:
                    pass
                try:
                    if curr_value is not None:
                        c = float(curr_value)
                        if not math.isnan(c) and not math.isinf(c):
                            current_state.current = c  # type: ignore[attr-defined]
                            updated = True
                except Exception:
                    pass
                if updated:
                    current_state.last_update = time.time()
            schedule_fast_emit()

        elif msg_type == "mission_list":
            waypoints = message.get("waypoints", [])
            current_seq = message.get("current_seq")
            converted = _convert_mavros_waypoints_to_ui(waypoints)
            with mavros_telem_lock:
                current_mission = converted
                current_state.activeWaypointIndex = int(current_seq) if current_seq is not None else current_state.activeWaypointIndex
                current_state.current_waypoint_id = current_state.activeWaypointIndex
                current_state.last_update = time.time()
            schedule_fast_emit()

        elif msg_type == "mission_reached":
            seq = message.get("wp_seq")
            if seq is not None:
                waypoint_id = int(seq) + 1
                with mavros_telem_lock:
                    if waypoint_id not in current_state.completedWaypointIds:
                        current_state.completedWaypointIds.append(waypoint_id)
                    current_state.current_waypoint_id = int(seq)
                    current_state.activeWaypointIndex = int(seq)
                    current_state.last_update = time.time()
                _record_mission_event(
                    f"Waypoint {waypoint_id} reached",
                    status='REACHED',
                    waypoint_id=waypoint_id
                )
                schedule_fast_emit()

        elif msg_type == "servo_output":
            servo_state = message.get("servo_output")
            if servo_state and isinstance(servo_state, dict):
                with mavros_telem_lock:
                    current_state.servo_output = servo_state
                    current_state.last_update = time.time()
                schedule_fast_emit()

        # Fallback: process battery fields even if message type isn't explicitly 'battery'
        try:
            perc_value = message.get("percentage")
            if perc_value is None:
                perc_value = message.get("battery")
            volt_value = message.get("voltage")
            curr_value = message.get("current")
            updated = False
            if perc_value is not None:
                with mavros_telem_lock:
                    try:
                        pctf = float(perc_value)
                        import math
                        if not math.isnan(pctf) and not math.isinf(pctf):
                            pct_val = max(0, int(round(pctf * 100))) if pctf <= 1.0 else max(0, int(round(pctf)))
                            if current_state.battery != pct_val:
                                current_state.battery = pct_val
                                updated = True
                    except Exception:
                        pass
                
            with mavros_telem_lock:
                try:
                    if volt_value is not None:
                        v = float(volt_value)
                        if not math.isnan(v) and not math.isinf(v):
                            current_state.voltage = v  # type: ignore[attr-defined]
                            updated = True
                except Exception:
                    pass
                try:
                    if curr_value is not None:
                        c = float(curr_value)
                        if not math.isnan(c) and not math.isinf(c):
                            current_state.current = c  # type: ignore[attr-defined]
                            updated = True
                except Exception:
                    pass
                if updated:
                    current_state.last_update = time.time()
                    schedule_fast_emit()
        except Exception:
            pass

    except Exception as exc:
        log_message(f"MAVROS telemetry handler error: {exc}", "ERROR")



last_sent_telemetry = {}


def telemetry_loop():
    """Continuously sends telemetry data to the frontend."""
    global last_sent_telemetry, last_emit_monotonic
    fallback_interval = float(os.getenv('FALLBACK_EMIT_SEC', '60.0'))
    while True:
        if is_vehicle_connected:
            now = time.monotonic()
            if now - last_emit_monotonic >= fallback_interval:
                emit_rover_data_now(reason='fallback')
                last_sent_telemetry = get_rover_data()
        else:
            last_sent_telemetry = {}
        socketio.sleep(0.5)


def connection_health_monitor():
    """Monitor client connections and emit health status periodically."""
    while True:
        try:
            current_time = time.time()
            
            # Check for stale client connections
            stale_clients = []
            for client_id, session in list(_client_sessions.items()):
                time_since_ping = current_time - session.get('last_ping', current_time)
                
                # Mark clients as stale if no ping for 30 seconds
                if time_since_ping > 30:
                    stale_clients.append(client_id)
                    session['missed_pings'] = session.get('missed_pings', 0) + 1
                    
                    # Emit warning to specific client
                    socketio.emit('connection_warning', {
                        'message': 'Connection may be unstable',
                        'time_since_ping': time_since_ping,
                        'timestamp': current_time
                    }, room=client_id)
            
            # Clean up very stale sessions (no activity for 5 minutes)
            for client_id in list(_client_sessions.keys()):
                session = _client_sessions[client_id]
                if current_time - session.get('last_ping', current_time) > 300:
                    _client_sessions.pop(client_id, None)
                    log_message(f'Cleaned up stale session: {client_id}', 'INFO')
            
            # Emit overall health status
            active_clients = len(_client_sessions) - len(stale_clients)
            socketio.emit('server_health', {
                'timestamp': current_time,
                'active_clients': active_clients,
                'total_clients': len(_client_sessions),
                'vehicle_connected': is_vehicle_connected,
                'ros_available': ROS_AVAILABLE,
                'uptime': current_time
            })
            
        except Exception as exc:
            log_message(f"Connection health monitor error: {exc}", "ERROR")
        
        socketio.sleep(10)  # Check every 10 seconds


# ============================================================================
# COMMAND HANDLERS
# ============================================================================

def _handle_arm_disarm(data):
    """Handle arm/disarm command."""
    arm = bool(data.get('arm', False))
    global is_vehicle_connected
    log_message(f"{'Arming' if arm else 'Disarming'} vehicle...")

    response_payload: dict[str, object] = {}
    errors: list[str] = []
    ros2_sent = False
    mavros_sent = False

    if command_bridge is not None:
        try:
            response = command_bridge.arm(value=arm)
            ros2_sent = bool(response.get('success', False))
            if not ros2_sent and response.get('result') is not None:
                ros2_sent = bool(response['result'])
            log_message(f"ROS2 arm service {'accepted' if ros2_sent else 'rejected'} ({response})")
            response_payload['ros2'] = response
        except Exception as exc:
            log_message(f"ROS2 arm service failed: {exc}", "WARNING")
            errors.append(f"ROS2: {exc}")

    if not ros2_sent:
        try:
            bridge = _get_vehicle_bridge(require_vehicle=False)
            mavros_response = bridge.arm(value=arm)
            response_payload['mavros'] = mavros_response
            log_message(f"MAVROS arm service acknowledged ({mavros_response})", 'INFO', event_type='ui_request')
            mavros_sent = True
            if not is_vehicle_connected:
                # Assume vehicle is reachable if the command succeeded.
                is_vehicle_connected = True
        except Exception as exc:
            errors.append(f"MAVROS: {exc}")
            log_message(f"MAVROS arm/disarm failed: {exc}", "ERROR", event_type='ui_request')

    if not ros2_sent and not mavros_sent:
        error_text = "; ".join(errors) if errors else "unknown error"
        raise RuntimeError(f"Arm/disarm command failed ({error_text})")

    with mavros_telem_lock:
        current_state.status = 'armed' if arm else 'disarmed'
        current_state.last_update = time.time()
        current_state.last_heartbeat = time.time()
    emit_rover_data_now(reason='arm_disarm')

    record_activity(
        f"Vehicle {'armed' if arm else 'disarmed'}",
        event_type='server_response',
        meta={
            'via_ros2': ros2_sent,
            'via_mavros': mavros_sent,
            'response': _make_json_safe(response_payload),
            'errors': errors,
        }
    )

    return {
        'status': 'success',
        'via_ros2': ros2_sent,
        'via_mavros': mavros_sent,
        'message': f"Vehicle {'armed' if arm else 'disarmed'}"
    }


def _handle_set_mode(data):
    """Handle mode change command."""
    new_mode = data.get('mode', 'MANUAL')
    mode_num = get_mode_number(new_mode)
    if mode_num is None:
        raise Exception(f"Unknown mode: {new_mode}")
    log_message(f"Setting vehicle mode to: {new_mode} (mode #{mode_num})")

    target_mode = str(new_mode).upper()
    ros2_sent = False
    if command_bridge is not None:
        try:
            response = command_bridge.set_mode(mode=target_mode)
            ros2_sent = bool(response.get('success', False) and response.get('mode_sent', False))
            log_message(f"ROS2 set_mode response: {response}")
        except Exception as exc:
            log_message(f"ROS2 set_mode failed: {exc}", "WARNING")

    if not ros2_sent:
        bridge = _require_vehicle_bridge()
        bridge.set_mode(mode=target_mode, custom_mode=target_mode)

    with mavros_telem_lock:
        current_state.mode = target_mode
        current_state.last_update = time.time()
    emit_rover_data_now(reason='set_mode')
    return {
        'status': 'success',
        'via_ros2': ros2_sent,
        'message': f"Mode change requested: {new_mode}"
    }


def _handle_goto(data):
    """Handle goto command."""
    bridge = _require_vehicle_bridge()
    lat = data['lat']
    lon = data['lon']
    alt = data.get('alt', 0)
    log_message(f"Sending vehicle to: {lat}, {lon}, alt: {alt}m")
    bridge.send_global_position_target(
        latitude=float(lat),
        longitude=float(lon),
        altitude=float(alt),
        coordinate_frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask=0b0000111111111000
    )
    return {
        'status': 'success',
        'message': f"GOTO command sent to {lat}, {lon}"
    }


def _handle_upload_mission(data):
    """
    Upload a mission to the rover via MAVROS.
    
    This is the centralized mission upload handler used by both:
    - REST API /api/mission/upload
    - Socket.IO mission_upload (deprecated)
    
    Args:
        data: Dict with 'waypoints' (required) and optional 'servoConfig'
        
    Returns:
        Dict with status and message
    """
    global current_mission
    bridge = _require_vehicle_bridge()
    
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("Mission upload requires waypoints")
    
    # Validate coordinates before upload to catch any lat/lng swap issues
    for wp in waypoints:
        lat_val = wp.get('lat', 0)
        lng_val = wp.get('lng', 0)
        if abs(lat_val) > 90:
            raise ValueError(f"Invalid latitude {lat_val} in waypoint {wp.get('id', '?')} - must be between -90 and +90")
        if abs(lng_val) > 180:
            raise ValueError(f"Invalid longitude {lng_val} in waypoint {wp.get('id', '?')} - must be between -180 and +180")
    
    # Apply servo configuration if provided
    servo_config = data.get("servoConfig")
    if servo_config:
        log_message(f"[mission_upload] Applying servo mode: {servo_config.get('mode')}", "INFO")
        waypoints = apply_servo_modes(waypoints, servo_config)
    
    mission_count = len(waypoints)
    log_message(f"[mission_upload] Uploading {mission_count} waypoint(s) via MAVROS", "INFO")
    
    # Convert to MAVROS format and push to vehicle
    mavros_waypoints = _build_mavros_waypoints(waypoints)
    response = bridge.push_waypoints(mavros_waypoints)
    
    if not response.get("success", False):
        raise RuntimeError(f"MAVROS mission push failed: {response}")
    
    # Update mission state
    current_mission = list(waypoints)
    with mavros_telem_lock:
        current_state.completedWaypointIds = []
        current_state.activeWaypointIndex = None
        current_state.current_waypoint_id = None
        mission_log_state['last_active_seq'] = None
        mission_log_state['last_reached_seq'] = None
    
    # Send load_mission command to mission controller node
    try:
        load_mission_data = {
            'command': 'load_mission',
            'waypoints': waypoints,
            'config': servo_config or {}
        }
        if hasattr(bridge, 'publish_mission_command'):
            bridge.publish_mission_command(load_mission_data)
            log_message(f"Sent load_mission command to mission controller ({mission_count} waypoints)", "INFO")
        else:
            log_message("Warning: MAVROS bridge not available for load_mission command", "WARNING")
    except Exception as e:
        log_message(f"Failed to send load_mission to mission controller: {e}", "ERROR")
    
    # Record activity
    _record_mission_event(
        f"Mission uploaded ({mission_count} items)",
        status='MISSION_UPLOADED'
    )
    
    log_message(f"Mission uploaded successfully via MAVROS ({mission_count} waypoints)", "SUCCESS")
    
    return {
        "status": "success",
        "message": f"Mission uploaded ({mission_count} waypoints)"
    }


def _handle_get_mission(data):
    """
    Handle mission download from vehicle.
    
    Returns waypoints from vehicle if available, otherwise returns cached mission.
    Provides clear error messages for different failure scenarios.
    """
    global current_mission
    bridge = _require_vehicle_bridge()
    
    try:
        log_message("[MISSION] Downloading mission from vehicle...", "INFO")
        mission_waypoints = download_mission_from_vehicle()
        
        if not mission_waypoints or len(mission_waypoints) == 0:
            log_message("[MISSION] Downloaded empty mission from vehicle", "INFO")
            return {
                'status': 'success',
                'message': 'Rover mission is empty (0 waypoints)',
                'waypoints': []
            }
        
        # Validate coordinates to catch any lat/lng swap issues
        for wp in mission_waypoints:
            lat_val = wp.get('lat', 0)
            lng_val = wp.get('lng', 0)
            if abs(lat_val) > 90:
                log_message(f"WARNING: Invalid latitude {lat_val} in waypoint {wp.get('id')} - possible lat/lng swap!", "ERROR")
            if abs(lng_val) > 180:
                log_message(f"WARNING: Invalid longitude {lng_val} in waypoint {wp.get('id')} - possible lat/lng swap!", "ERROR")
        
        log_message(f"[MISSION] Successfully downloaded {len(mission_waypoints)} waypoints", "SUCCESS")
        return {
            'status': 'success',
            'message': f'Downloaded {len(mission_waypoints)} waypoints from rover',
            'waypoints': mission_waypoints
        }
        
    except TimeoutError as e:
        log_message(f"[MISSION] Download timeout: {e}", "ERROR")
        if current_mission and len(current_mission) > 0:
            return {
                'status': 'success',
                'message': f'Download timed out. Using cached mission ({len(current_mission)} waypoints)',
                'waypoints': current_mission,
                'warning': 'timeout'
            }
        raise TimeoutError("Mission download timed out. Please check MAVROS connection and try again.")
        
    except ConnectionError as e:
        log_message(f"[MISSION] Connection error during download: {e}", "ERROR")
        raise ConnectionError("Unable to connect to rover. Please check MAVROS bridge is running.")
        
    except Exception as e:
        log_message(f"[MISSION] Download failed: {e}", "ERROR")
        
        # Try to provide cached mission as fallback
        if current_mission and len(current_mission) > 0:
            return {
                'status': 'success',
                'message': f'Download failed. Using cached mission ({len(current_mission)} waypoints)',
                'waypoints': current_mission,
                'warning': 'fallback'
            }
        
        # No cached mission available
        error_msg = str(e)
        if "not available" in error_msg.lower():
            raise RuntimeError("MAVROS service not available. Please check rosbridge connection.")
        elif "timeout" in error_msg.lower():
            raise TimeoutError("Mission download timed out. Vehicle may not be responding.")
        else:
            raise RuntimeError(f"Mission download failed: {error_msg}")


def _handle_clear_mission(_data=None):
    """Clear all mission waypoints from the vehicle."""
    global current_mission
    bridge = _require_vehicle_bridge()
    log_message("Clearing mission from vehicle...")
    response = bridge.clear_waypoints()
    if not response.get('success', False):
        raise RuntimeError(f"MAVROS mission clear failed: {response}")
    with mavros_telem_lock:
        current_mission = []
        current_state.completedWaypointIds = []
        current_state.activeWaypointIndex = None
        current_state.current_waypoint_id = None
        current_state.last_update = time.time()
    _record_mission_event('Mission cleared from vehicle', status='MISSION_CLEARED')
    emit_rover_data_now(reason='mission_clear')
    log_message("Mission cleared successfully via MAVROS", "SUCCESS")
    return {'status': 'success', 'message': 'Mission cleared'}


COMMAND_HANDLERS = {
    'ARM_DISARM': _handle_arm_disarm,
    'SET_MODE': _handle_set_mode,
    'GOTO': _handle_goto,
    'UPLOAD_MISSION': _handle_upload_mission,
    'GET_MISSION': _handle_get_mission,
    'CLEAR_MISSION': _handle_clear_mission,
}


@socketio.on('request_mission_logs')
def handle_request_mission_logs():
    try:
        snapshot = list(mission_log_history)
        emit('mission_logs_snapshot', snapshot, room=request.sid)
    except Exception as exc:
        log_message(f"Failed to serve mission logs snapshot: {exc}", 'ERROR')


# ============================================================================
# MISSION UPLOAD/DOWNLOAD
# ============================================================================
# NOTE: Socket.IO mission_upload handler is DEPRECATED.
# All mission uploads should use REST API /api/mission/upload
# which properly integrates with MAVROS through _handle_upload_mission().
#
# This Socket.IO handler is kept only for backward compatibility but
# now redirects to the same MAVROS implementation.

@socketio.on("mission_upload")
def on_mission_upload(data):
    """
    DEPRECATED: Mission upload via Socket.IO.
    Use REST API /api/mission/upload instead.
    
    This handler redirects to _handle_upload_mission() for MAVROS integration.
    """
    global current_mission
    
    log_message("[mission_upload] Socket.IO upload called (DEPRECATED - use REST API)", "WARNING")

    try:
        # Use the centralized handler that REST API uses
        result = _handle_upload_mission(data)
        
        # Emit progress events for backward compatibility
        socketio.emit('mission_upload_progress', {'progress': 100})
        
        # Emit completion event with success response
        emit("mission_uploaded", {
            "ok": True,
            "count": len(data.get("waypoints", [])),
            "message": result.get("message", "Mission uploaded via MAVROS")
        })
        
    except Exception as e:
        log_message(f"[mission_upload] Error: {e}", "ERROR")
        emit("mission_uploaded", {"ok": False, "error": str(e)})


def download_mission_from_vehicle():
    """Download mission from vehicle."""
    global current_mission
    bridge = _require_vehicle_bridge()
    if not mission_download_lock.acquire(blocking=False):
        raise Exception("Another download in progress")

    try:
        socketio.emit('mission_download_progress', {'progress': 5})
        response = bridge.pull_waypoints()
        waypoint_list = response.get("waypoints", [])
        converted = _convert_mavros_waypoints_to_ui(waypoint_list)
        current_mission = converted.copy()
        socketio.emit('mission_download_progress', {'progress': 100})
        log_message(f"[mission_download] Completed via MAVROS: {len(converted)} waypoints", "SUCCESS")
        return converted
    except Exception as e:
        log_message(f"Error downloading mission: {e}", "ERROR")
        raise
    finally:
        try:
            mission_download_lock.release()
        except Exception:
            pass


# ============================================================================
# RTK INJECTION SYSTEM (Enhanced with Debug Headers)
# ============================================================================

rtk_thread: Optional[threading.Thread] = None
rtk_running = False
rtk_bytes_total = 0
rtk_bytes_lock = threading.Lock()
rtk_current_caster = None


# Replace the existing _ntrip_request() function with:
def _ntrip_request(host: str, mount: str, user: str, pwd: str) -> bytes:
    """Builds an exact NTRIP 2.0 HTTP/1.0 request identical to manual telnet test."""
    auth = base64.b64encode(f"{user}:{pwd}".encode()).decode()
    req = (
        f"GET /{mount} HTTP/1.0\r\n"
        f"User-Agent: NTRIP_RoverPlan/1.0\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\n"
        f"Authorization: Basic {auth}\r\n"
        f"\r\n"
    )
    return req.encode("ascii", errors="ignore")


def _inject_rtcm_to_mav(rtcm_bytes: bytes) -> bool:
    """Send RTCM correction data to rover's GPS module."""
    try:
        bridge = _require_vehicle_bridge()
    except Exception as exc:
        log_message(f"[RTK] ⚠️ Cannot send - {exc}", "WARNING")
        return False

    MAX_CHUNK = 180
    try:
        total_sent = 0
        for i in range(0, len(rtcm_bytes), MAX_CHUNK):
            chunk = rtcm_bytes[i:i + MAX_CHUNK]
            if chunk:
                bridge.send_rtcm(bytes(chunk))
                total_sent += len(chunk)

        if total_sent > 0:
            with rtk_bytes_lock:
                global rtk_bytes_total
                rtk_bytes_total += total_sent
                current_total = rtk_bytes_total
            # Tell frontend how many bytes forwarded
            socketio.emit('rtk_forwarded', {
                'added': total_sent,
                'total': current_total,
            })
        return True
    except Exception as e:
        log_message(f"[RTK] ❌ Failed to send data: {e}", "ERROR")
        socketio.emit('rtk_log', {'message': f'❌ Send error: {e}'})
        return False


@socketio.on('connect_caster')
def handle_connect_caster(caster_details):
    """Start RTK corrections stream from caster network."""
    global rtk_thread, rtk_running, rtk_current_caster

    try:
        sender_id = request.sid
    except:
        sender_id = None

    # --- IMPROVEMENT: Automatically stop the existing stream ---
    if rtk_running:
        log_message("[RTK] 🔄 Restart requested. Stopping existing stream first...", "INFO")
        socketio.emit('rtk_log', {'message': '🔄 Restarting stream...'})
        rtk_running = False
        # Give the old thread a moment to see the flag and exit
        socketio.sleep(0.2)
    # --- END IMPROVEMENT ---

    try:
        host = str(caster_details.get('host', '')).strip()
        port = int(caster_details.get('port', 2101))
        mount = str(caster_details.get('mountpoint', '')).strip()
        user = str(caster_details.get('user') or caster_details.get('username', '')).strip()
        pwd = str(caster_details.get('password', '')).strip()

        if not host or not mount:
            raise ValueError("Missing host or mountpoint")
        # Enforce credentials presence to prevent unauthenticated streams
        if not user or not pwd:
            raise ValueError("Missing username or password")

    except Exception as e:
        msg = f'❌ Invalid settings: {str(e)}'
        socketio.emit('rtk_log', {'message': msg})
        socketio.emit('caster_status', {
            'status': 'error',
            'message': msg
        }, to=sender_id)
        return

    # Ensure the bridge and vehicle are connected before starting RTK
    try:
        _require_vehicle_bridge()
    except Exception as exc:
        msg = f'❌ {exc}'
        log_message(f"[RTK] Rejecting stream start: {msg}", "WARNING")
        socketio.emit('rtk_log', {'message': msg})
        socketio.emit('caster_status', {
            'status': 'error',
            'message': msg
        }, to=sender_id)
        return

    rtk_current_caster = caster_details
    log_message(f"[RTK] 🚀 Starting stream from {host}:{port}/{mount}", "INFO")
    socketio.emit('rtk_log', {'message': f'🔄 Connecting to {host}:{port}...'})
    socketio.emit('caster_status', {
        'status': 'connecting',
        'message': f'Connecting to {mount}...'
    })

    rtk_running = True

    def rtk_worker():
        """Background thread for continuous RTK data streaming."""
        global rtk_running, rtk_bytes_total
        sock = None

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)

            log_message(f"[RTK] Attempting connection to {host}:{port}...")
            socketio.emit('rtk_log', {'message': f'📡 Connecting to {host}:{port}...'})
            sock.connect((host, port))
            log_message(f"[RTK] Connection to {host}:{port} successful!", "SUCCESS")
            socketio.emit('rtk_log', {'message': f'📡 Connected to {host}:{port}'})

            request_data = _ntrip_request(host, mount, user, pwd)
            sock.sendall(request_data)
            socketio.emit('rtk_log', {
                'message': f'📤 Sent NTRIP request for mountpoint {mount}'
            })

            # --- Wait for optional HTTP header or direct binary stream ---
            buffer = b''
            header_timeout = time.time() + 12.0
            header_found = False

            while True:
                chunk = sock.recv(1024)
                if not chunk:
                    raise RuntimeError('Caster closed connection during handshake')
                buffer += chunk

                if b'\r\n\r\n' in buffer:
                    headers, leftover_data = buffer.split(b'\r\n\r\n', 1)
                    header_text = headers.decode('ascii', errors='ignore')
                    first_line = header_text.split('\n')[0]
                    # Emit the full caster response header for visibility
                    socketio.emit('rtk_log', {'message': f'🧾 Caster Response:\n{header_text}'})
                    upper_hdr = header_text.upper()
                    # Explicitly detect auth failures and sourcetable responses
                    if ('SOURCETABLE' in upper_hdr or
                        '401' in upper_hdr or 'UNAUTHORIZED' in upper_hdr or
                        '403' in upper_hdr or 'FORBIDDEN' in upper_hdr or
                        'NTRIP-ERROR' in upper_hdr):
                        raise RuntimeError('❌ Unauthorized or bad username/password!')
                    # Accept either ICY 200 or standard 200 OK
                    if b'ICY 200' not in headers and b'200 OK' not in headers:
                        raise RuntimeError(f'❌ Unexpected response:\n{header_text}')
                    socketio.emit('rtk_log', {'message': '✅ Connection established! Streaming RTCM data...'})
                    leftover = leftover_data
                    header_found = True
                    break

                # direct binary
                if any(b < 32 and b not in (b'\r'[0], b'\n'[0], b'\t'[0]) for b in chunk):
                    socketio.emit('rtk_log', {'message': '✅ No header — direct RTCM stream detected'})
                    leftover = buffer
                    break

                if time.time() > header_timeout:
                    raise TimeoutError('Caster took too long to respond')

            if header_found:
                socketio.emit('caster_status', {
                    'status': 'Connected',
                    'message': 'ICY 200 OK – RTCM stream active'
                })
            else:
                socketio.emit('caster_status', {
                    'status': 'Connected',
                    'message': 'RTCM stream active (no header)'
                })

            with rtk_bytes_lock:
                rtk_bytes_total = 0

            # Inject leftover RTCM if present
            if 'leftover' in locals() and leftover:
                if _inject_rtcm_to_mav(leftover):
                    socketio.emit('rtk_log', {
                        'message': f'📥 Sent initial {len(leftover)} bytes'
                    })
                    # Debug: print RTCM header bytes to browser console
                    if leftover.startswith(b'\xd3'):
                        msg_len = int.from_bytes(leftover[1:3], "big") & 0x03FF
                        socketio.emit('rtk_log', {
                            'message': f'🔍 RTCM Detected: Header 0xD3, length {msg_len} bytes'
                        })

            sock.settimeout(3.0)
            last_data_time = time.time()

            while rtk_running:
                try:
                    data = sock.recv(4096)

                    if not data:
                        if time.time() - last_data_time > 10:
                            raise RuntimeError('No data received for 10 seconds')
                        time.sleep(0.1)
                        continue

                    last_data_time = time.time()

                    # Debug: parse RTCM header
                    if data.startswith(b'\xd3') and len(data) >= 3:
                        msg_len = int.from_bytes(data[1:3], "big") & 0x03FF
                        socketio.emit('rtk_log', {
                            'message': f'🔍 RTCM Packet: 0xD3 len={msg_len}'
                        })

                    if _inject_rtcm_to_mav(data):
                        socketio.emit('rtcm_data', data)

                except socket.timeout:
                    continue

                except Exception as stream_error:
                    log_message(f"[RTK] Stream error: {stream_error}", "ERROR")
                    socketio.emit('rtk_log', {
                        'message': f'⚠️ Stream error: {stream_error}'
                    })
                    break

        except Exception as e:
            error_msg = str(e)
            log_message(f"[RTK] ❌ Connection failed: {error_msg}", "ERROR")
            socketio.emit('rtk_log', {'message': f'❌ Error: {error_msg}'})
            socketio.emit('caster_status', {
                'status': 'error',
                'message': error_msg
            })

        finally:
            rtk_running = False
            if sock:
                try:
                    sock.close()
                except:
                    pass

            socketio.emit('rtk_log', {'message': '🛑 RTK stream stopped'})
            socketio.emit('caster_status', {
                'status': 'Disconnected',
                'message': 'Stream ended'
            })
            log_message("[RTK] Stream thread ended", "INFO")

    rtk_thread = threading.Thread(target=rtk_worker, daemon=True, name="RTK-Worker")
    rtk_thread.start()

    socketio.emit('rtk_log', {'message': '🎬 RTK thread started'})


@socketio.on('disconnect_caster')
def handle_disconnect_caster():
    """Stop the RTK stream."""
    global rtk_running

    try:
        sender_id = request.sid
    except:
        sender_id = None

    if rtk_running:
        log_message("[RTK] 🛑 Stop requested by user", "INFO")
        rtk_running = False
        socketio.emit('rtk_log', {'message': '🛑 Stopping RTK stream...'})
        socketio.emit('caster_status', {
            'status': 'Disconnected',
            'message': 'Stop requested'
        }, to=sender_id)
    else:
        socketio.emit('rtk_log', {'message': 'ℹ️ Stream was not running'})
        socketio.emit('caster_status', {
            'status': 'Disconnected',
            'message': 'Already stopped'
        }, to=sender_id)



# ============================================================================
# SOCKET.IO EVENT HANDLERS
# ============================================================================

@socketio.on('connect')
def handle_connect():
    """Handles a new client connection."""
    try:
        client_id = request.sid
        client_addr = request.remote_addr
        log_message(f'✅ Client connected: {client_id} from {client_addr}', 'INFO')
        
        # Send connection confirmation with session ID
        emit('connection_response', {
            'status': 'connected',
            'sid': client_id,
            'timestamp': time.time()
        })
        
        if is_vehicle_connected:
            log_message("Emitting 'CONNECTED_TO_ROVER' to NEW client...")
            emit('connection_status', {'status': 'CONNECTED_TO_ROVER'})
            rover_data = get_rover_data()
            if rover_data:
                log_message("Sending current rover data to new client")
                emit('rover_data', rover_data)
        else:
            log_message("Emitting 'WAITING_FOR_ROVER' to NEW client...")
            emit('connection_status', {'status': 'WAITING_FOR_ROVER'})
    except Exception as e:
        # Do not reject the connection if our handler has a transient error
        log_message(f"Error in connect handler: {e}", 'ERROR')
        try:
            emit('connection_response', {
                'status': 'connected',
                'sid': request.sid if hasattr(request, 'sid') else None,
                'timestamp': time.time()
            })
        except Exception:
            pass

    # Also inform new clients of current RTK caster status for seamless UX
    try:
        if rtk_running:
            caster = rtk_current_caster or {}
            host = str(caster.get('host', ''))
            port = str(caster.get('port', ''))
            mount = str(caster.get('mountpoint', ''))
            emit('caster_status', {
                'status': 'Connected',
                'message': f'Active stream: {host}:{port}/{mount}'
            })
        else:
            emit('caster_status', {
                'status': 'Disconnected',
                'message': 'No active RTK stream'
            })
    except Exception as e:
        log_message(f"Failed to emit caster status to new client: {e}", 'WARNING')


@socketio.on('disconnect')
def handle_disconnect():
    client_id = request.sid
    log_message(f'❌ Client disconnected: {client_id}', 'INFO')
    # Clear client-specific state if needed
    _client_sessions.pop(client_id, None)


# Track client sessions for connection quality monitoring
_client_sessions = {}

@socketio.on('ping')
def handle_ping(data=None):
    """Handle ping from client for connection testing with latency tracking."""
    client_id = request.sid
    current_time = time.time()
    
    # Initialize client session if not exists
    if client_id not in _client_sessions:
        _client_sessions[client_id] = {
            'connected_at': current_time,
            'last_ping': current_time,
            'ping_count': 0,
            'missed_pings': 0
        }
    
    session = _client_sessions[client_id]
    session['last_ping'] = current_time
    session['ping_count'] += 1
    
    # Calculate connection quality metrics
    ping_interval = current_time - session.get('last_ping_response', current_time)
    
    # Respond with comprehensive connection info
    response_data = {
        'timestamp': current_time,
        'server_time': current_time,
        'session_id': client_id,
        'ping_count': session['ping_count'],
        'connection_quality': 'good' if ping_interval < 2.0 else 'degraded' if ping_interval < 5.0 else 'poor'
    }
    
    # Include client-sent timestamp if available for RTT calculation
    if data and isinstance(data, dict) and 'client_timestamp' in data:
        response_data['client_timestamp'] = data['client_timestamp']
    
    emit('pong', response_data)
    session['last_ping_response'] = current_time


@socketio.on('request_rover_reconnect')
def handle_rover_reconnect():
    """Force the backend to drop and re-establish the vehicle link."""
    global is_vehicle_connected

    log_message('Frontend requested rover reconnect', 'INFO')
    emit('rover_reconnect_ack', {'status': 'success', 'message': 'Reconnect initiated'})

    try:
        bridge = _init_mavros_bridge()
        bridge.close()
    except Exception as exc:
        log_message(f'Error closing MAVROS bridge: {exc}', 'WARNING')

    is_vehicle_connected = False
    current_state.last_heartbeat = None
    current_state.signal_strength = 'No Link'
    log_message('Flagged vehicle connection as lost; will retry', 'INFO')
    socketio.emit('connection_status', {
        'status': 'WAITING_FOR_ROVER',
        'message': 'Reconnect requested by operator'
    })


@socketio.on('subscribe_mission_status')
def handle_subscribe_mission_status():
    """Handle frontend subscription to mission status updates"""
    try:
        global latest_mission_status, mission_status_history

        # PHASE 2 FIX: Send status history on subscription for frontend sync
        if mission_status_history:
            emit('mission_status_history', {
                'success': True,
                'history': list(mission_status_history)
            })
            log_message(f"Sent {len(mission_status_history)} cached status messages to client", event_type='mission')

        # Send current status immediately if available
        if latest_mission_status:
            emit('mission_status', latest_mission_status)

        # Send confirmation
        emit('mission_status_subscribed', {
            'success': True,
            'message': 'Subscribed to mission status updates'
        })

        log_message("Client subscribed to mission status", event_type='mission')

    except Exception as e:
        log_message(f"Mission status subscription error: {e}", "ERROR", event_type='mission')
        emit('mission_status_subscribed', {
            'success': False,
            'error': str(e)
        })


@socketio.on('get_mission_status')
def handle_get_mission_status():
    """Handle real-time mission status request via WebSocket"""
    try:
        global mission_controller, latest_mission_status
        if mission_controller:
            status = mission_controller.get_status()
            emit('mission_status_response', {
                'success': True,
                'status': status,
                'latest_update': latest_mission_status
            })
        else:
            emit('mission_status_response', {
                'success': False,
                'error': 'Mission controller not available'
            })
            
    except Exception as e:
        log_message(f"Mission status request error: {e}", "ERROR", event_type='mission')
        emit('mission_status_response', {
            'success': False,
            'error': str(e)
        })


@socketio.on('send_command')
def handle_command(data, ack_callback=None):
    """Handle incoming commands from the frontend with acknowledgment support."""
    try:
        _require_vehicle_bridge()
    except Exception as exc:
        log_message("Command rejected - vehicle not connected", "ERROR", event_type='ui_request')
        payload = {'status': 'error', 'message': str(exc), 'acked': True}
        
        # Send acknowledgment if callback provided
        if ack_callback:
            ack_callback(payload)
        else:
            emit('command_response', payload)
        
        record_activity(
            "Socket command rejected - vehicle not connected",
            level='ERROR',
            event_type='server_response',
            meta={'reason': str(exc)}
        )
        return

    command_type = data.get('command')
    log_message(f"Received command: {command_type}", event_type='ui_request')
    record_activity(
       
        event_type='ui_request',
        meta={
            'command': command_type,
            'payload_keys': sorted(list(data.keys()))[:10] if isinstance(data, dict) else None
        }
    )

    handler = COMMAND_HANDLERS.get(command_type)
# ---------------------------------------------------------------------------
    if not handler:
        log_message(f"Unknown command: {command_type}", "ERROR", event_type='ui_request')
        payload = {
            'status': 'error',
            'message': f'Unknown command: {command_type}',
            'acked': True
        }
        
        if ack_callback:
            ack_callback(payload)
        else:
            emit('command_response', payload)
        
        record_activity(
            f"Unknown command received: {command_type}",
            level='ERROR',
            event_type='server_response',
            meta={'command': command_type}
        )
        return

    try:
        result = handler(data)
        response_payload = None
        
        if isinstance(result, dict):
            response_payload = {**result, 'acked': True}
            record_activity(
                f"Socket command '{command_type}' processed",
                event_type='server_response',
                meta={'command': command_type, 'response': _make_json_safe(result)}
            )
        elif isinstance(result, str):
            response_payload = {'status': 'success', 'message': result, 'acked': True}
            record_activity(
                f"Socket command '{command_type}' processed",
                event_type='server_response',
                meta={'command': command_type, 'message': result}
            )
        else:
            response_payload = {'status': 'success', 'message': f"{command_type} dispatched", 'acked': True}
            record_activity(
                f"Socket command '{command_type}' processed",
                event_type='server_response',
                meta={'command': command_type, 'message': response_payload['message']}
            )
        
        # Send acknowledgment
        if ack_callback:
            ack_callback(response_payload)
        else:
            emit('command_response', response_payload)
            
    except Exception as e:
        log_message(f"Command error ({command_type}): {e}", "ERROR", event_type='server_response')
        payload = {
            'status': 'error',
            'message': str(e),
            'command': command_type,
            'acked': True
        }
        
        if ack_callback:
            ack_callback(payload)
        else:
            emit('command_response', payload)
        
        record_activity(
            f"Socket command '{command_type}' failed",
            level='ERROR',
            event_type='server_response',
            meta=_make_json_safe(payload)
        )

# ---------------------------------------------------------------------------
# Testing / dev helpers
# ---------------------------------------------------------------------------
@socketio.on('inject_mavros_telemetry')
def handle_inject_mavros_telemetry(data):
    """Inject a MAVROS-like telemetry payload into the server for testing.

    Use with care: this is intended for local testing only (development).
    Emits a short ack and triggers the usual telemetry merge/emit path.
    """
    try:
        # Accept either full envelope or a partial payload
        _handle_mavros_telemetry(data)
        emit('inject_ack', {'status': 'ok'})
    except Exception as exc:
        log_message(f"inject_mavros_telemetry failed: {exc}", 'ERROR')
        try:
            emit('inject_ack', {'status': 'error', 'error': str(exc)})
        except Exception:
            pass


# ============================================================================
# ROOT & HEALTH ENDPOINTS
# ============================================================================

@app.route('/')
def root():
    """Root endpoint - returns server status and available endpoints."""
    return jsonify({
        'status': 'online',
        'message': 'NRP Rover Backend Server',
        'version': '1.0',
        'ros_available': ROS_AVAILABLE,
        'endpoints': {
            'health': '/api/health',
            'rover_data': '/api/rover-data'
        }
    }), 200

@app.get("/monitor")
def monitor_dashboard():
    """Serve the activity monitor page from a template file.

    The large HTML previously embedded in `MONITOR_PAGE_HTML` has been moved to
    `Backend/templates/monitor.html` and is rendered here via Flask's
    `render_template`.
    """
    return render_template('monitor.html')


@app.get("/node/<node_name>")
def node_details(node_name):
    """Serve the node details page."""
    return render_template('node_details.html', node_name=node_name)


# ============================================================================
# ERROR HANDLERS
# ============================================================================

def _http_success(message: str | None = None, status: int = 200, **extra):
    payload = {'success': True}
    if message:
        payload['message'] = message
    payload.update(extra)
    return jsonify(payload), status


def _http_error(message: str, status: int = 400, **extra):
    payload = {'success': False, 'message': message}
    payload.update(extra)
    return jsonify(payload), status


@app.get("/api/activity")
def api_activity_feed():
    """Return recent backend activity entries with optional filtering."""
    try:
        limit = int(request.args.get('limit', 500))
    except Exception:
        limit = 500
    limit = max(1, min(limit, ACTIVITY_LOG_MAX))
    event_filter = request.args.get('event')

    with _activity_log_lock:
        entries = list(activity_log)

    if event_filter:
        entries = [entry for entry in entries if entry.get('event') == event_filter]

    if len(entries) > limit:
        entries = entries[-limit:]

    return jsonify({
        'entries': entries,
        'returned': len(entries),
        'total': len(activity_log),
    })


@app.get("/api/activity/types")
def api_activity_types():
    """Expose the set of known activity event types."""
    with _activity_log_lock:
        events = sorted({entry.get('event', 'general') for entry in activity_log})
    return jsonify({'events': events, 'count': len(events)})


@app.get("/api/activity/download")
def api_activity_download():
    """Provide a text download of the current activity log."""
    with _activity_log_lock:
        entries = list(activity_log)

    lines: list[str] = []
    for entry in entries:
        iso = entry.get('isoTime') or time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(entry.get('timestamp', time.time())))
        level = entry.get('level', 'INFO')
        event = entry.get('event', 'general')
        message = entry.get('message', '')
        meta = entry.get('meta')
        meta_str = ''
        if meta not in (None, '', {}):
            try:
                meta_str = json.dumps(meta, ensure_ascii=True)
            except Exception:
                meta_str = str(meta)
        line = f"[{iso}] [{level}] ({event}) {message}"
        if meta_str:
            line = f"{line} | meta={meta_str}"
        lines.append(line)

    payload = "\n".join(lines)
    headers = {
        'Content-Disposition': 'attachment; filename=\"nrp-activity.log\"'
    }
    return Response(payload, mimetype='text/plain', headers=headers)


@app.post("/api/arm")
def api_arm_vehicle():
    body = request.get_json(silent=True) or {}
    arm_value = bool(body.get('value', body.get('arm', True)))
    try:
        result = _handle_arm_disarm({'arm': arm_value})
        message = result.get('message') if isinstance(result, dict) else None
        return _http_success(message or "Arm command dispatched")
    except Exception as exc:
        log_message(f"/api/arm error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/set_mode")
def api_set_mode():
    body = request.get_json(silent=True) or {}
    mode = str(body.get('mode') or '').strip()
    if not mode:
        return _http_error("Missing mode", 400)
    try:
        result = _handle_set_mode({'mode': mode})
        message = result.get('message') if isinstance(result, dict) else None
        return _http_success(message or f"Mode change requested: {mode}")
    except Exception as exc:
        log_message(f"/api/set_mode error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/upload")
def api_mission_upload():
    body = request.get_json(silent=True) or {}
    waypoints = body.get('waypoints')
    if not isinstance(waypoints, list) or not waypoints:
        return _http_error("No waypoints provided", 400)
    payload = {
        'waypoints': waypoints,
        'servoConfig': body.get('servoConfig'),
    }
    try:
        result = _handle_upload_mission(payload)
        message = result.get('message') if isinstance(result, dict) else None
        count = len(waypoints)
        return _http_success(message or f"Mission upload queued ({count} waypoints)", count=count)
    except Exception as exc:
        log_message(f"/api/mission/upload error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.get("/api/mission/download")
def api_mission_download():
    try:
        result = _handle_get_mission({})
        waypoints = result.get('waypoints', []) if isinstance(result, dict) else []
        message = result.get('message') if isinstance(result, dict) else None
        return _http_success(message or f"Mission retrieved ({len(waypoints)} waypoints)", waypoints=waypoints)
    except Exception as exc:
        log_message(f"/api/mission/download error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/clear")
def api_mission_clear():
    try:
        result = _handle_clear_mission({})
        message = result.get('message') if isinstance(result, dict) else None
        return _http_success(message or "Mission cleared")
    except Exception as exc:
        log_message(f"/api/mission/clear error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/set_current")
def api_mission_set_current():
    """
    Set the current mission waypoint (skip to specific waypoint).
    
    Expected JSON body:
    {
        "wp_seq": 5  // 0-based waypoint sequence number
    }
    """
    body = request.get_json(silent=True) or {}
    wp_seq = body.get('wp_seq')
    
    if wp_seq is None:
        return _http_error("Missing wp_seq parameter", 400)
    
    try:
        wp_seq_int = int(wp_seq)
        if wp_seq_int < 0:
            return _http_error("wp_seq must be >= 0", 400)
        
        bridge = _require_vehicle_bridge()
        response = bridge.set_current_waypoint(wp_seq_int)
        
        log_message(f"[MISSION] Set current waypoint to {wp_seq_int}", "INFO")
        
        return _http_success(
            f"Current waypoint set to {wp_seq_int}",
            wp_seq=wp_seq_int
        )
    except Exception as exc:
        log_message(f"/api/mission/set_current error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/resume_deprecated")
def api_mission_resume():
    """
    Resume mission execution by switching to AUTO mode (DEPRECATED - use controller).
    """
    try:
        bridge = _require_vehicle_bridge()
        
        # Switch to AUTO mode to resume mission (DEPRECATED - use controller)
        response = bridge.set_mode(mode="AUTO")
        
        log_message("[MISSION] Mission resumed (switched to AUTO mode)", "INFO")
        _record_mission_event("Mission resumed", status='MISSION_RESUMED')
        
        return _http_success("Mission resumed (AUTO mode)")
    except Exception as exc:
        log_message(f"/api/mission/resume error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/load_controller")
def api_mission_load_controller():
    """
    Load mission waypoints to the Jetson mission controller node.
    
    This sends a load_mission command to the ROS mission controller
    without uploading to the Pixhawk. Used for spraying operations.
    
    Expected JSON body:
    {
        "waypoints": [{"lat": 13.071922, "lng": 80.2619957, ...}],
        "servoConfig": {"mode": "interval", ...}  // optional
    }
    """
    body = request.get_json(silent=True) or {}
    waypoints = body.get('waypoints')
    
    if not isinstance(waypoints, list) or not waypoints:
        return _http_error("No waypoints provided", 400)
    
    # Validate waypoints have required fields
    for wp in waypoints:
        if not all(k in wp for k in ['lat', 'lng']):
            return _http_error(f"Waypoint missing required fields: {wp}", 400)
    
    try:
        bridge = _require_vehicle_bridge()
        
        # Apply servo configuration if provided
        servo_config = body.get('servoConfig')
        if servo_config:
            log_message(f"[mission_load_controller] Applying servo mode: {servo_config.get('mode')}", "INFO")
            waypoints = apply_servo_modes(waypoints, servo_config)
        
        mission_count = len(waypoints)
        
        # Send load_mission command to mission controller node
        load_mission_data = {
            'command': 'load_mission',
            'waypoints': waypoints,
            'config': servo_config or {}
        }
        
        if hasattr(bridge, 'publish_mission_command'):
            bridge.publish_mission_command(load_mission_data)
            log_message(f"Sent load_mission command to mission controller ({mission_count} waypoints)", "INFO")
            
            # Update local mission state for consistency
            global current_mission
            current_mission = list(waypoints)
            
            return _http_success(f"Mission loaded to controller ({mission_count} waypoints)")
        else:
            return _http_error("MAVROS bridge not available for mission controller commands", 500)
            
    except Exception as exc:
        log_message(f"/api/mission/load_controller error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/start_controller")
def api_mission_start_controller():
    """
    Start mission execution on the Jetson mission controller node.
    
    This sends a 'start' command to the ROS mission controller
    to begin autonomous waypoint execution with spraying.
    
    The mission controller must have waypoints loaded first
    via /api/mission/load_controller.
    """
    try:
        bridge = _require_vehicle_bridge()
        
        # Send start command to mission controller node
        start_command = {'command': 'start'}
        
        if hasattr(bridge, 'publish_mission_command'):
            bridge.publish_mission_command(start_command)
            log_message("Sent start command to mission controller", "INFO")
            
            return _http_success("Mission controller started")
        else:
            return _http_error("MAVROS bridge not available for mission controller commands", 500)
            
    except Exception as exc:
        log_message(f"/api/mission/start_controller error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.get("/api/mission/config")
def api_mission_config():
    """
    Return full mission controller configuration (mission_controller section) from
    the mission_controller_config.json file.
    """
    try:
        config_file = os.path.join(os.path.dirname(__file__), 'config', 'mission_controller_config.json')
        if not os.path.exists(config_file):
            return _http_error("Configuration file not found", 404)
        with open(config_file, 'r') as f:
            config = json.load(f)
        mc = config.get('mission_controller', {})
        return _http_success(mc)
    except Exception as exc:
        log_message(f"/api/mission/config error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/config")
def api_mission_config_update():
    """
    Update mission controller servo configuration and timing parameters.
    
    RESPONSIBILITY: Servo & Timer Configuration ONLY
    - Updates servo channel, PWM values, spray timing, GPS timeout, mode
    - Applies to currently loaded mission (if any)
    - Does NOT handle waypoint loading
    
    Expected JSON body:
    {
        "servo_channel": 10,
        "servo_pwm_start": 1500,
        "servo_pwm_stop": 1100,
        "spray_duration": 5.0,
        "delay_before_spray": 1.0,
        "delay_after_spray": 1.0,
        "gps_timeout": 30.0,
        "auto_mode": true
    }
    """
    global current_mission
    try:
        body = request.get_json(silent=True) or {}
        
        # Validate required servo/timer parameters
        required_params = ['servo_channel', 'servo_pwm_start', 'servo_pwm_stop', 
                          'spray_duration', 'delay_before_spray', 'delay_after_spray', 
                          'gps_timeout', 'auto_mode']
        
        for param in required_params:
            if param not in body:
                return _http_error(f"Missing required parameter: {param}", 400)
        
        # Reject waypoint data if included (wrong endpoint)
        if 'waypoints' in body:
            return _http_error("Waypoints should not be included in /api/mission/config. Use /api/mission/load instead.", 400)
        
        # Load existing config
        config_file = os.path.join(os.path.dirname(__file__), 'config', 'mission_controller_config.json')
        
        if os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = json.load(f)
        else:
            config = {
                "mission_controller": {
                    "sprayer_parameters": {},
                    "mission_parameters": {
                        "waypoint_reach_threshold": 2.0,
                        "max_retry_attempts": 3,
                        "retry_delay": 5.0,
                        "status_update_interval": 1.0
                    },
                    "safety_parameters": {
                        "max_speed": 5.0,
                        "emergency_stop_enabled": True,
                        "low_battery_threshold": 20.0,
                        "connection_timeout": 10.0
                    }
                },
                "metadata": {
                    "version": "1.0",
                    "last_updated": "2025-11-10",
                    "description": "Mission controller configuration for NRP ROS spraying system"
                }
            }
        
        # Update ONLY sprayer parameters (servo & timing)
        config['mission_controller']['sprayer_parameters'] = body
        config['metadata']['last_updated'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Save config to file
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2)
        
        log_message("Servo config saved to file", "INFO")
        
        # Send load_mission command to mission controller with updated config
        try:
            bridge = _require_vehicle_bridge()
            
            # Send command only if there are waypoints loaded
            if current_mission and len(current_mission) > 0:
                load_mission_data = {
                    'command': 'load_mission',
                    'waypoints': current_mission,
                    'config': body
                }
                
                if hasattr(bridge, 'publish_mission_command'):
                    bridge.publish_mission_command(load_mission_data)
                    log_message(f"Sent load_mission command with updated servo config to mission controller", "INFO")
                else:
                    log_message("Warning: publish_mission_command not available, config saved but not applied", "WARNING")
            else:
                log_message("No mission waypoints loaded, config saved but not sent to mission controller", "INFO")
                
        except Exception as e:
            # Config was saved successfully, log warning but don't fail the request
            log_message(f"Warning: Config saved but failed to send command to mission controller: {e}", "WARNING")
        
        return _http_success("Servo config updated (applies to current mission)")
        
    except Exception as exc:
        log_message(f"/api/mission/config POST error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/load")
def api_mission_load():
    """
    Load waypoints to the mission controller.
    
    RESPONSIBILITY: Waypoint Loading ONLY
    - Accepts waypoint list with lat/lng coordinates
    - Updates current_mission global variable
    - Sends load_mission command with waypoints to mission controller
    - Does NOT handle servo configuration (use /api/mission/config for that)
    - Can optionally apply servo modes based on waypoint attributes
    
    Expected JSON body:
    {
      "waypoints": [
        {"lat": 13.071922, "lng": 80.2619957},
        {"lat": 13.072000, "lng": 80.2620000}
      ]
    }
    
    Note: servoConfig field (if included) is for backward compatibility only.
          New implementations should use /api/mission/config endpoint.
    """
    body = request.get_json(silent=True) or {}
    waypoints = body.get('waypoints')
    if not isinstance(waypoints, list) or not waypoints:
        return _http_error("No waypoints provided", 400)

    # Validate required fields for waypoints
    for wp in waypoints:
        if not all(k in wp for k in ['lat', 'lng']):
            return _http_error(f"Waypoint missing required fields (lat, lng): {wp}", 400)

    try:
        bridge = _require_vehicle_bridge()

        # Optional: Apply servo modes based on waypoint data (backward compatibility)
        servo_config = body.get('servoConfig')
        if servo_config:
            log_message(f"[mission_load] Applying servo mode: {servo_config.get('mode')}", "INFO")
            waypoints = apply_servo_modes(waypoints, servo_config)

        mission_count = len(waypoints)
        
        # Prepare load_mission command with waypoints
        # Config will be applied separately via /api/mission/config
        load_mission_data = {
            'command': 'load_mission',
            'waypoints': waypoints,
            'config': servo_config or {}
        }

        # Use integrated mission controller
        global mission_controller, current_mission
        
        if not mission_controller:
            return _http_error("Mission controller not initialized", 503)
        
        result = mission_controller.process_command(load_mission_data)
        
        if result.get('success', False):
            # Update local mission state for consistency
            current_mission = list(waypoints)
            _record_mission_event(f"Mission loaded to controller ({mission_count} waypoints)", status='MISSION_LOADED')
            log_message(f"Mission loaded to controller ({mission_count} waypoints)", "INFO")
            return _http_success(f"Mission loaded to controller ({mission_count} waypoints)")
        else:
            error_msg = result.get('error', 'Unknown error')
            return _http_error(error_msg, 400)
            
    except Exception as exc:
        log_message(f"/api/mission/load error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


def _publish_controller_cmd_or_error(cmd: dict):
    """Helper to process mission command via integrated controller or return an error tuple."""
    global mission_controller
    
    try:
        if not mission_controller:
            return None, _http_error("Mission controller not initialized", 503)
        
        # Process command through integrated controller
        result = mission_controller.process_command(cmd)
        
        if result.get('success', False):
            log_message(f"Mission command '{cmd.get('command')}' executed successfully", "INFO", event_type='mission')
            return True, None
        else:
            error_msg = result.get('error', 'Unknown error')
            log_message(f"Mission command '{cmd.get('command')}' failed: {error_msg}", "WARNING", event_type='mission')
            return None, _http_error(error_msg, 400)
        
    except Exception as exc:
        error_msg = f"Mission command error: {str(exc)}"
        log_message(error_msg, "ERROR", event_type='mission')
        return None, _http_error(error_msg, 500)


@app.post("/api/mission/start")
def api_mission_start():
    try:
        ok, err = _publish_controller_cmd_or_error({'command': 'start'})
        if err:
            return err
        _record_mission_event('Mission controller started', status='MISSION_STARTED')
        return _http_success('Mission controller started')
    except Exception as exc:
        log_message(f"/api/mission/start error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.get("/api/mission/start")
def api_mission_start_get():
    """Inform clients that POST must be used to start missions."""
    return _http_error("Method not allowed: use POST /api/mission/start to start missions", 405)


@app.post("/api/mission/stop")
def api_mission_stop():
    try:
        ok, err = _publish_controller_cmd_or_error({'command': 'stop'})
        if err:
            return err
        _record_mission_event('Mission controller stopped', status='MISSION_STOPPED')
        return _http_success('Mission controller stopped')
    except Exception as exc:
        log_message(f"/api/mission/stop error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/pause")
def api_mission_pause_frontend():
    try:
        ok, err = _publish_controller_cmd_or_error({'command': 'pause'})
        if err:
            return err
        _record_mission_event('Mission paused (controller)', status='MISSION_PAUSED')
        return _http_success('Mission paused')
    except Exception as exc:
        log_message(f"/api/mission/pause error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/resume")
def api_mission_resume_frontend():
    try:
        ok, err = _publish_controller_cmd_or_error({'command': 'resume'})
        if err:
            return err
        _record_mission_event('Mission resumed (controller)', status='MISSION_RESUMED')
        return _http_success('Mission resumed')
    except Exception as exc:
        log_message(f"/api/mission/resume error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/restart")
def api_mission_restart():
    """
    Restart the mission controller. Prefer explicit 'restart' command; if not
    supported, perform stop then start.
    """
    try:
        # Try restart first
        ok, err = _publish_controller_cmd_or_error({'command': 'restart'})
        if err is None and ok:
            _record_mission_event('Mission controller restarted', status='MISSION_RESTARTED')
            return _http_success('Mission controller restarted')

        # Fallback: stop then start
        _publish_controller_cmd_or_error({'command': 'stop'})
        socketio.sleep(0.1)
        _publish_controller_cmd_or_error({'command': 'start'})
        _record_mission_event('Mission controller restarted (stop/start)', status='MISSION_RESTARTED')
        return _http_success('Mission controller restarted (stop/start)')
    except Exception as exc:
        log_message(f"/api/mission/restart error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/next")
def api_mission_next():
    """Advance mission controller to the next waypoint/step."""
    try:
        ok, err = _publish_controller_cmd_or_error({'command': 'next'})
        if err:
            return err
        _record_mission_event('Mission controller advanced to next', status='MISSION_NEXT')
        return _http_success('Advanced to next mission step')
    except Exception as exc:
        log_message(f"/api/mission/next error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/mission/skip")
def api_mission_skip():
    """Skip current waypoint and advance to the next waypoint/step."""
    try:
        ok, err = _publish_controller_cmd_or_error({'command': 'skip'})
        if err:
            return err
        _record_mission_event('Mission controller skipped waypoint', status='MISSION_SKIP')
        return _http_success('Skipped to next mission step')
    except Exception as exc:
        log_message(f"/api/mission/skip error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.get("/api/mission/status")
def api_mission_status():
    """Get current mission controller status"""
    try:
        global mission_controller, latest_mission_status
        if mission_controller:
            status = mission_controller.get_status()
            return jsonify({
                'success': True,
                'status': status,
                'latest_update': latest_mission_status
            })
        else:
            return jsonify({
                'success': False,
                'error': 'Mission controller not available'
            }), 503
            
    except Exception as exc:
        log_message(f"Failed to get mission status: {exc}", "ERROR", event_type='mission')
        return jsonify({
            'success': False,
            'error': str(exc)
        }), 500


@app.get("/api/mission/mode")
def api_mission_mode_get():
    """Get current mission mode (auto/manual)"""
    try:
        global mission_controller
        if mission_controller:
            status = mission_controller.get_status()
            return jsonify({
                'success': True,
                'mode': status.get('mission_mode', 'unknown')
            })
        else:
            return jsonify({
                'success': False,
                'error': 'Mission controller not available'
            }), 503
            
    except Exception as exc:
        log_message(f"Failed to get mission mode: {exc}", "ERROR", event_type='mission')
        return jsonify({
            'success': False,
            'error': str(exc)
        }), 500


@app.post("/api/mission/mode")
def api_mission_mode_set():
    """Set mission mode (auto/manual)"""
    try:
        global mission_controller
        data = request.get_json() or {}
        mode = data.get('mode', 'auto').lower()
        
        if mode not in ['auto', 'manual']:
            log_message(f"Invalid mission mode requested: {mode}", "WARNING", event_type='mission')
            return jsonify({
                'success': False,
                'error': 'Mode must be "auto" or "manual"'
            }), 400
        
        if not mission_controller:
            log_message("Mission controller not available for mode change", "ERROR", event_type='mission')
            return jsonify({
                'success': False,
                'error': 'Mission controller not available'
            }), 503
        
        result = mission_controller.set_mode(mode)
        
        # Log the result of mode change
        if result.get('success', False):
            log_message(f"Mission mode changed to: {mode.upper()}", "SUCCESS", event_type='mission')
        else:
            log_message(f"Failed to change mission mode to {mode}: {result.get('error')}", "ERROR", event_type='mission')
        
        return jsonify(result)
        
    except Exception as exc:
        log_message(f"Failed to set mission mode: {exc}", "ERROR", event_type='mission')
        return jsonify({
            'success': False,
            'error': str(exc)
        }), 500


@app.post("/api/mission/stop_controller")
def api_mission_stop_controller():
    """
    Stop mission execution on the Jetson mission controller node.
    
    This sends a 'stop' command to the ROS mission controller
    to halt autonomous waypoint execution and spraying operations.
    
    The mission controller will return to idle state.
    """
    try:
        bridge = _require_vehicle_bridge()
        
        # Send stop command to mission controller node
        stop_command = {'command': 'stop'}
        
        if hasattr(bridge, 'publish_mission_command'):
            bridge.publish_mission_command(stop_command)
            log_message("Sent stop command to mission controller", "INFO")
            
            return _http_success("Mission controller stopped")
        else:
            return _http_error("MAVROS bridge not available for mission controller commands", 500)
            
    except Exception as exc:
        log_message(f"/api/mission/stop_controller error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.get("/api/config/sprayer")
def api_get_sprayer_config():
    """
    Get current sprayer configuration parameters for the mission controller.
    
    Returns the current sprayer parameters from the config file.
    """
    try:
        config_file = os.path.join(os.path.dirname(__file__), 'config', 'mission_controller_config.json')
        
        if not os.path.exists(config_file):
            return _http_error("Configuration file not found", 404)
        
        with open(config_file, 'r') as f:
            config = json.load(f)
        
        sprayer_params = config.get('mission_controller', {}).get('sprayer_parameters', {})
        return _http_success(sprayer_params)
        
    except Exception as exc:
        log_message(f"/api/config/sprayer GET error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/config/sprayer")
def api_save_sprayer_config():
    """
    Save sprayer configuration parameters for the mission controller.
    
    Expected JSON body:
    {
        "servo_channel": 10,
        "servo_pwm_start": 1500,
        "servo_pwm_stop": 1100,
        "spray_duration": 5.0,
        "delay_before_spray": 1.0,
        "delay_after_spray": 1.0,
        "gps_timeout": 30.0,
        "auto_mode": true
    }
    """
    global current_mission
    try:
        body = request.get_json(silent=True) or {}
        
        # Validate required parameters
        required_params = ['servo_channel', 'servo_pwm_start', 'servo_pwm_stop', 
                          'spray_duration', 'delay_before_spray', 'delay_after_spray', 
                          'gps_timeout', 'auto_mode']
        
        for param in required_params:
            if param not in body:
                return _http_error(f"Missing required parameter: {param}", 400)
        
        # Load existing config
        config_file = os.path.join(os.path.dirname(__file__), 'config', 'mission_controller_config.json')
        
        if os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = json.load(f)
        else:
            # Create default config structure if file doesn't exist
            config = {
                "mission_controller": {
                    "sprayer_parameters": {},
                    "mission_parameters": {
                        "waypoint_reach_threshold": 2.0,
                        "max_retry_attempts": 3,
                        "retry_delay": 5.0,
                        "status_update_interval": 1.0
                    },
                    "safety_parameters": {
                        "max_speed": 5.0,
                        "emergency_stop_enabled": True,
                        "low_battery_threshold": 20.0,
                        "connection_timeout": 10.0
                    }
                },
                "metadata": {
                    "version": "1.0",
                    "last_updated": "2025-11-10",
                    "description": "Mission controller configuration for NRP ROS spraying system"
                }
            }
        
        # Update sprayer parameters
        config['mission_controller']['sprayer_parameters'] = body
        config['metadata']['last_updated'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Save config
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2)
        
        log_message("Sprayer configuration saved", "INFO")
        
        # Send load_mission command to mission controller with updated servo config
        try:
            bridge = _require_vehicle_bridge()
            
            # Send command only if there are waypoints loaded
            if current_mission and len(current_mission) > 0:
                load_mission_data = {
                    'command': 'load_mission',
                    'waypoints': current_mission,
                    'config': body
                }
                
                if hasattr(bridge, 'publish_mission_command'):
                    bridge.publish_mission_command(load_mission_data)
                    log_message(f"Sent load_mission command to mission controller with updated servo config", "INFO")
                else:
                    log_message("Warning: publish_mission_command not available, config saved but not applied", "WARNING")
            else:
                log_message("No mission waypoints loaded, config saved but not applied to mission controller", "INFO")
                
        except Exception as e:
            # Config was saved successfully, log warning but don't fail the request
            log_message(f"Warning: Config saved but failed to send command to mission controller: {e}", "WARNING")
        
        return _http_success("Sprayer configuration saved and applied to mission controller")
        
    except Exception as exc:
        log_message(f"/api/config/sprayer POST error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/rtk/inject")
def api_rtk_inject():
    """
    Inject RTK corrections from an NTRIP caster.
    
    Expected JSON body:
    {
        "ntrip_url": "rtcm://user:pass@host:port/mountpoint"
        OR
        "host": "...", "port": 2101, "mountpoint": "...", "user": "...", "password": "..."
    }
    """
    body = request.get_json(silent=True) or {}
    ntrip_url = str(body.get('ntrip_url') or body.get('url') or '').strip()
    
    # Parse NTRIP URL if provided (format: rtcm://user:pass@host:port/mountpoint)
    if ntrip_url:
        try:
            from urllib.parse import urlparse
            
            # Handle both rtcm:// and http:// schemes
            if ntrip_url.startswith('rtcm://'):
                url = urlparse(ntrip_url.replace('rtcm://', 'http://'))
            else:
                url = urlparse(ntrip_url)
            
            caster_details = {
                'host': url.hostname or '',
                'port': url.port or 2101,
                'mountpoint': url.path.lstrip('/') or '',
                'user': url.username or '',
                'password': url.password or ''
            }
        except Exception as e:
            log_message(f"[RTK] Failed to parse NTRIP URL: {e}", "ERROR")
            return _http_error(f"Invalid NTRIP URL format: {e}", 400)
    else:
        # Use individual parameters
        caster_details = {
            'host': str(body.get('host', '')).strip(),
            'port': int(body.get('port', 2101)),
            'mountpoint': str(body.get('mountpoint', '')).strip(),
            'user': str(body.get('user') or body.get('username', '')).strip(),
            'password': str(body.get('password', '')).strip()
        }
    
    # Validate required fields
    if not caster_details.get('host') or not caster_details.get('mountpoint'):
        return _http_error("Missing host or mountpoint", 400)
    if not caster_details.get('user') or not caster_details.get('password'):
        return _http_error("Missing user or password", 400)
    
    # Use the existing Socket.IO handler
    try:
        with app.test_request_context():
            handle_connect_caster(caster_details)
        
        log_message(f"[RTK] REST API triggered RTK injection to {caster_details['host']}", "INFO")
        return jsonify({
            'success': True,
            'message': f"RTK stream started from {caster_details['host']}:{caster_details['port']}/{caster_details['mountpoint']}"
        })
    except Exception as exc:
        log_message(f"/api/rtk/inject error: {exc}", "ERROR")
        return _http_error(str(exc), 500)


@app.post("/api/rtk/stop")
def api_rtk_stop():
    """Stop the RTK corrections stream."""
    global rtk_running
    
    if rtk_running:
        log_message("[RTK] 🛑 REST API stop requested", "INFO")
        rtk_running = False
        return jsonify({
            'success': True,
            'message': 'RTK stream stopped successfully'
        })
    else:
        return jsonify({
            'success': True,
            'message': 'RTK stream was not running'
        })


@app.get("/api/rtk/status")
def api_rtk_status():
    """Get the current RTK stream status."""
    global rtk_running, rtk_current_caster, rtk_bytes_total
    
    with rtk_bytes_lock:
        total_bytes = rtk_bytes_total
    
    return jsonify({
        'success': True,
        'running': rtk_running,
        'caster': rtk_current_caster if rtk_running else None,
        'total_bytes': total_bytes
    })


@app.post("/api/servo/control")
def api_servo_control():
    """
    Control a servo directly via MAVROS.
    
    Expected JSON body:
    {
        "servo_id": 10,  // Servo channel number (1-16)
        "angle": 90      // Angle in degrees (0-180) OR
        "pwm": 1500      // Direct PWM value (1000-2000)
    }
    """
    body = request.get_json(silent=True) or {}
    servo_id = body.get('servo_id')
    
    if servo_id is None:
        return _http_error("Missing servo_id", 400)
    
    # Accept either angle (0-180) or direct PWM (1000-2000)
    angle = body.get('angle')
    pwm = body.get('pwm')
    
    if angle is None and pwm is None:
        return _http_error("Missing angle or pwm parameter", 400)
    
    # Convert angle to PWM if needed
    if pwm is None:
        # Map angle (0-180) to PWM (1000-2000)
        # Standard servo: 0° = 1000µs, 90° = 1500µs, 180° = 2000µs
        angle_val = float(angle)
        if angle_val < 0 or angle_val > 180:
            return _http_error("Angle must be between 0 and 180", 400)
        pwm_value = int(1000 + (angle_val / 180.0) * 1000)
    else:
        pwm_value = int(pwm)
        if pwm_value < 1000 or pwm_value > 2000:
            return _http_error("PWM must be between 1000 and 2000", 400)
    
    try:
        bridge = _require_vehicle_bridge()
        result = bridge.set_servo(int(servo_id), pwm_value)
        
        if result.get('success'):
            log_message(f"[SERVO] Set servo {servo_id} to PWM {pwm_value}", "INFO")
            return jsonify({
                'success': True,
                'message': f'Servo {servo_id} set to PWM {pwm_value}',
                'servo_id': servo_id,
                'pwm': pwm_value,
                'angle': angle if angle is not None else None
            })
        else:
            return _http_error(f"Servo command failed: result={result.get('result')}", 500)
    except Exception as exc:
        log_message(f"/api/servo/control error: {exc}", "ERROR")
        return _http_error(str(exc), 500)

@app.get("/api/nodes")
def api_nodes():
    """Return list of active ROS 2 nodes."""
    if not ROS_AVAILABLE:
        return _http_error("ROS 2 not available", 503)
    try:
        import subprocess
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            nodes = [node.strip() for node in nodes if node.strip()]
            return jsonify({'nodes': nodes, 'count': len(nodes)})
        else:
            return _http_error(f"Failed to list nodes: {result.stderr}", 500)
    except Exception as exc:
        log_message(f"/api/nodes error: {exc}", "ERROR")
        return _http_error(str(exc), 500)

@app.get("/api/node/<node_name>")
def api_node_info(node_name):
    """Return detailed information about a specific ROS 2 node."""
    if not ROS_AVAILABLE:
        return _http_error("ROS 2 not available", 503)
    try:
        import subprocess
        # Use the same command that works in the terminal
        result = subprocess.run(['ros2', 'node', 'info', node_name], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            # Parse the output
            lines = result.stdout.strip().split('\n')
            info = {
                'node_name': node_name,
                'subscribers': [],
                'publishers': [],
                'service_servers': [],
                'service_clients': [],
                'action_servers': [],
                'action_clients': []
            }
            
            current_section = None
            for line in lines[1:]:  # Skip the node name line
                line = line.strip()
                if not line:
                    continue
                
                if line.startswith('Subscribers:'):
                    current_section = 'subscribers'
                elif line.startswith('Publishers:'):
                    current_section = 'publishers'
                elif line.startswith('Service Servers:'):
                    current_section = 'service_servers'
                elif line.startswith('Service Clients:'):
                    current_section = 'service_clients'
                elif line.startswith('Action Servers:'):
                    current_section = 'action_servers'
                elif line.startswith('Action Clients:'):
                    current_section = 'action_clients'
                elif current_section and line.startswith('/'):
                    # Extract topic/service name and type
                    if ':' in line:
                        parts = line.split(':', 1)
                        name = parts[0].strip()
                        type_info = parts[1].strip()
                        info[current_section].append({'name': name, 'type': type_info})
                    else:
                        info[current_section].append({'name': line, 'type': 'unknown'})
            
            return jsonify(info)
        else:
            return _http_error(f"Failed to get node info: {result.stderr}", 404)
    except Exception as exc:
        log_message(f"/api/node/{node_name} error: {exc}", "ERROR")
        return _http_error(str(exc), 500)

@socketio.on_error_default
def default_error_handler(e):
    log_message(f"SocketIO error: {e}", "ERROR")


@app.errorhandler(404)
def handle_404(e):
    """Handle 404 Not Found errors with proper JSON response."""
    app.logger.warning(f"404 Not Found: {request.path}")
    return jsonify({
        'success': False,
        'error': '404 Not Found',
        'message': f'The requested endpoint {request.path} does not exist',
        'path': request.path
    }), 404


@app.errorhandler(Exception)
def handle_exception(e):
    """Handle all unhandled exceptions with proper JSON response."""
    app.logger.error(f"Flask error: {str(e)}", exc_info=True)
    
    # Don't leak internal error details in production
    error_message = str(e)
    error_type = type(e).__name__
    
    return jsonify({
        'success': False,
        'error': error_message,
        'type': error_type
    }), 500


# ============================================================================
# MAIN EXECUTION
# ============================================================================

# ============================================================================
# SERVO CONTROL API (integrated for concurrent backend)
# ============================================================================
# Simple process manager to launch/stop servo scripts living under Backend/servo_manager
import subprocess as _subprocess

SERVO_BASE = os.path.join(os.path.dirname(__file__), "servo_manager")
CONFIG_PATH = os.path.join(SERVO_BASE, "config.json")
LOG_DIR = os.path.join(SERVO_BASE, "logs")
os.makedirs(LOG_DIR, exist_ok=True)

_running: dict[str, dict] = {}

def _is_pid_alive(pid: int | None) -> bool:
    if not pid:
        return False
    try:
        return os.system(f"ps -p {int(pid)} > /dev/null 2>&1") == 0
    except Exception:
        return False

def _cleanup_running() -> None:
    for m in list(_running.keys()):
        pid = _running[m].get("pid")
        if not _is_pid_alive(pid):
            try:
                del _running[m]
            except Exception:
                pass

def _kill_mode(mode: str, timeout_sec: float = 3.0) -> None:
    info = _running.get(mode)
    if not info:
        return
    pid = info.get("pid")
    if pid:
        try:
            os.kill(pid, signal.SIGTERM)
        except Exception:
            pass
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            if not _is_pid_alive(pid):
                break
            time.sleep(0.1)
        if _is_pid_alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
    try:
        del _running[mode]
    except Exception:
        pass

def _find_running_mode(except_mode: str | None = None) -> str | None:
    _cleanup_running()
    for m, info in _running.items():
        if except_mode and m == except_mode:
            if _is_pid_alive(info.get("pid")):
                return m
            continue
        if _is_pid_alive(info.get("pid")):
            return m
    return None


@app.get("/servo/run")
def servo_run():
    mode = request.args.get("mode")
    if not mode:
        return jsonify({"error": "missing mode"}), 400

    scripts = {
        "continuous": "continuous_line.py",
        "interval": "interval_spray.py",
        "wpmark": "wpmark.py",
    }
    if mode not in scripts:
        return jsonify({"error": "invalid mode"}), 400

    # Enforce only one script running at a time
    replace = str(request.args.get("replace", "0")).lower() in ("1", "true", "yes")
    current = _find_running_mode()
    if current and current != mode and not replace:
        return jsonify({
            "error": "another mode is running",
            "current_mode": current,
            "requested_mode": mode,
            "action": "pass replace=1 to switch"
        }), 409
    if current and current != mode and replace:
        _kill_mode(current)

    # If same mode already running, return its info
    info = _running.get(mode)
    if info and _is_pid_alive(info.get("pid")):
        return jsonify({
            "status": f"{mode} already running",
            "pid": info.get("pid"),
            "log": info.get("log"),
            "start": info.get("start")
        })

    log_path = os.path.join(LOG_DIR, f"{mode}_{int(time.time())}.log")
    log_file = open(log_path, "a")

    p = _subprocess.Popen(
        ["python3", os.path.join(SERVO_BASE, scripts[mode])],
        stdout=log_file, stderr=_subprocess.STDOUT,
        cwd=SERVO_BASE
    )
    _running[mode] = {"pid": p.pid, "log": log_path, "start": time.time()}
    return jsonify({"status": f"{mode} started", "pid": p.pid, "log": log_path})


@app.get("/servo/stop")
def servo_stop():
    mode = request.args.get("mode")
    if not mode or mode not in _running:
        return jsonify({"error": "not running"}), 400
    try:
        _kill_mode(mode)
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    return jsonify({"status": f"{mode} stopped"})


@app.post("/servo/emergency_stop")
def servo_emergency_stop():
    """
    Emergency stop all running servo scripts immediately.
    
    Returns:
    {
        "status": "emergency stop initiated",
        "stopped_modes": ["mode1", "mode2", ...]
    }
    """
    stopped_modes = []
    for mode in list(_running.keys()):
        try:
            _kill_mode(mode)
            stopped_modes.append(mode)
        except Exception as e:
            # Log error but continue stopping other modes
            print(f"Error stopping {mode}: {e}")
    
    return jsonify({
        "status": "emergency stop initiated",
        "stopped_modes": stopped_modes
    })


@app.get("/servo/status")
def servo_status():
    """
    Return servo status including running servo scripts and current servo output telemetry.
    
    Returns:
    {
        "success": true,
        "active": bool,           // Any servo script is running
        "running_modes": {...},   // Active servo scripts (wpmark, continuous, etc.)
        "servo_output": {...}     // Current PWM values from MAVROS telemetry
    }
    """
    _cleanup_running()
    for m, info in list(_running.items()):
        alive = _is_pid_alive(info.get("pid"))
        info["running"] = bool(alive)
    
    # Get current servo telemetry from MAVROS
    servo_telemetry = {}
    if current_state.servo_output:
        servo_telemetry = current_state.servo_output
    
    return jsonify({
        "success": True,
        "active": len(_running) > 0,
        "running_modes": _running,
        "servo_output": servo_telemetry,
        "timestamp": time.time()
    })


@app.get("/servo/edit")
def servo_edit():
    data = dict(request.args)
    try:
        with open(CONFIG_PATH, "r") as f:
            cfg = json.load(f)
    except Exception:
        cfg = {}
    for k, v in data.items():
        keys = k.split(".")
        ref = cfg
        for part in keys[:-1]:
            ref = ref.setdefault(part, {})
        try:
            val = float(v) if "." in v else int(v)
        except Exception:
            val = v
        ref[keys[-1]] = val
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    return jsonify({"status": "updated", "config": cfg})


@app.post("/servo/edit")
def servo_edit_post():
    """Accept JSON body to update servo config (more robust than querystring)."""
    body = request.get_json(silent=True) or {}
    try:
        with open(CONFIG_PATH, "r") as f:
            cfg = json.load(f)
    except Exception:
        cfg = {}

    def _coerce(s):
        try:
            if isinstance(s, (int, float)):
                return s
            if isinstance(s, str):
                if s.strip() == "":
                    return s
                if "." in s:
                    return float(s)
                return int(s)
        except Exception:
            return s
        return s

    for k, v in body.items():
        keys = str(k).split(".")
        ref = cfg
        for part in keys[:-1]:
            ref = ref.setdefault(part, {})
        ref[keys[-1]] = _coerce(v)

    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    return jsonify({"status": "updated", "config": cfg})


@app.get("/servo/log")
def servo_log():
    path = request.args.get("path")
    if not path:
        return jsonify({"error": "invalid log path"}), 400
    try:
        # Prevent directory traversal: require logs under LOG_DIR
        abs_path = os.path.abspath(path)
        if not abs_path.startswith(os.path.abspath(LOG_DIR)):
            return jsonify({"error": "forbidden path"}), 403
        if not os.path.isfile(abs_path):
            return jsonify({"error": "invalid log path"}), 400
        with open(abs_path, "r") as f:
            lines = f.readlines()[-200:]
        return jsonify({"log": "".join(lines)})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# --- Mission Controller WebSocket Event Handlers - REMOVED ---
# Mission controller node has been deleted
# Use direct MAVROS services for mission operations


try:
    log_message("Starting Flask-SocketIO server...")

    # Start cooperative background tasks under Socket.IO's greenlet scheduler
    socketio.start_background_task(maintain_mavros_connection)
    log_message("MAVROS connection maintenance task started")

    socketio.start_background_task(telemetry_loop)
    log_message("Telemetry task started")

    socketio.start_background_task(connection_health_monitor)
    log_message("Connection health monitor started")

    log_message("All background tasks started successfully", "SUCCESS")
    log_message("Starting Flask-SocketIO server on http://0.0.0.0:5001", "SUCCESS")

    # This check prevents the dev server from running on module import,
    # but allows direct execution (`python server.py`) to work.
    if __name__ in ('__main__', 'Backend.server'):
      socketio.run(app, host='0.0.0.0', port=5001, debug=False)

except Exception as e:
    log_message(f"Failed to start server: {e}", "ERROR")
    # Ensure ROS cleanup happens even on startup failure
    _shutdown_ros_runtime()
    raise e
