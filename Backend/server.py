from __future__ import annotations

import eventlet
eventlet.monkey_patch()  # <-- ADD THIS LINE
import threading
import time
import math
import json
import base64
import socket
import sys
import tempfile
from collections import deque
from itertools import count
from flask import Flask, request, jsonify, Response
from flask_socketio import SocketIO, emit
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

# --- Flask & SocketIO Setup ---
app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet',
                    ping_timeout=60, ping_interval=25)

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

    # Initialize ROS2 bridge
    rclpy.init()
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
    global _ros_shutdown_done
    with _ros_shutdown_lock:
        if _ros_shutdown_done:
            return
        _ros_shutdown_done = True

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
            rclpy.shutdown()
    except Exception:
        pass

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


atexit.register(_shutdown_ros_runtime)


def _acquire_singleton_lock() -> None:
    """Prevent multiple backend instances from sharing the same TCP port."""
    global _singleton_lock_handle

    if fcntl is None:
        return  # Platform without fcntl cannot rely on this lock; fall back to best effort.

    lock_path = os.environ.get('NRP_BACKEND_LOCK_FILE')
    if not lock_path:
        lock_path = os.path.join(tempfile.gettempdir(), 'nrp_backend.lock')

    try:
        handle = open(lock_path, 'w')
    except Exception as exc:
        print(f"[WARN] Unable to open backend lock file {lock_path}: {exc}", flush=True)
        return

    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(f"[WARN] Another NRP backend instance is already running (lock {lock_path}). Exiting duplicate.", flush=True)
        try:
            handle.close()
        except Exception:
            pass
        sys.exit(0)
    except Exception as exc:
        print(f"[WARN] Unable to lock backend file {lock_path}: {exc}", flush=True)
        try:
            handle.close()
        except Exception:
            pass
        return

    try:
        handle.truncate(0)
        handle.write(str(os.getpid()))
        handle.flush()
    except Exception:
        pass

    _singleton_lock_handle = handle


# --- MAVROS Connection Configuration ---
_acquire_singleton_lock()

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
    activeWaypointIndex: Optional[int] = None
    completedWaypointIds: List[int] = field(default_factory=list)
    distanceToNext: float = 0.0
    # Timestamp pushed to frontend (seconds since epoch)
    last_update: Optional[float] = None

    def to_dict(self) -> dict:
        d = asdict(self)
        return d


# Initialize current vehicle state
current_state = CurrentState()

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
def _track_http_response(response):
    """Record outgoing HTTP responses for observability."""
    try:
        if request.path.startswith('/socket.io'):
            return response
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
    except Exception:
        pass
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
        position = payload.get('position')

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

            if isinstance(position, dict):
                lat = position.get('latitude')
                lon = position.get('longitude')
                if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                    current_state.position = Position(lat=float(lat), lng=float(lon))
                    changed = True
                    current_state.last_update = now
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
        autocontinue = int(bool(safe_float(wp.get("autocontinue", 1), 1)))
        param1 = safe_float(wp.get("param1", 0.0))
        param2 = safe_float(wp.get("param2", 0.0))
        param3 = safe_float(wp.get("param3", 0.0))
        param4 = safe_float(wp.get("param4", 0.0))

        if command_requires_nav_coordinates(command_id):
            lat = safe_float(wp.get("lat", wp.get("x", 0.0)))
            lon = safe_float(wp.get("lng", wp.get("y", 0.0)))
            alt = safe_float(wp.get("alt", wp.get("z", 0.0)))
        else:
            lat = safe_float(wp.get("x", 0.0))
            lon = safe_float(wp.get("y", 0.0))
            alt = safe_float(wp.get("alt", wp.get("z", 0.0)))

        mavros_waypoints.append({
            "frame": frame,
            "command": command_id,
            "is_current": 1 if idx == 0 else 0,
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
    return current_state.to_dict()


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
    global is_vehicle_connected, mavros_connection_last_attempt
    while True:
        try:
            bridge = _init_mavros_bridge()
            if bridge.is_connected:
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
            if lat is not None and lon is not None:
                with mavros_telem_lock:
                    current_state.position = Position(lat=float(lat), lng=float(lon))
                    current_state.last_update = time.time()
                    current_state.distanceToNext = float(alt or 0.0)
                schedule_fast_emit()

        elif msg_type == "gps_fix":
            status = message.get("status")
            covariance = message.get("position_covariance", [])
            with mavros_telem_lock:
                current_state.rtk_status = _map_navsat_status(status)
                if isinstance(covariance, list) and len(covariance) >= 3:
                    try:
                        hrms = math.sqrt(abs(float(covariance[0])))
                        vrms = math.sqrt(abs(float(covariance[2])))
                        current_state.hrms = f"{hrms:.3f}"
                        current_state.vrms = f"{vrms:.3f}"
                    except Exception:
                        current_state.hrms = current_state.hrms or '0.000'
                        current_state.vrms = current_state.vrms or '0.000'
                current_state.last_update = time.time()
            schedule_fast_emit()

        elif msg_type == "heading":
            heading = message.get("heading")
            if heading is not None:
                with mavros_telem_lock:
                    current_state.heading = float(heading)
                    current_state.last_update = time.time()
            schedule_fast_emit()

        elif msg_type == "battery":
            percentage = message.get("percentage")
            if percentage is not None:
                with mavros_telem_lock:
                    current_state.battery = max(0, int(round(float(percentage) * 100))) if percentage <= 1.0 else int(round(float(percentage)))
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
    """Upload a mission via Socket.IO to the Pixhawk."""
    global current_mission
    bridge = _require_vehicle_bridge()
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("Mission upload requires waypoints")
    mission_count = len(waypoints)
    log_message(f"Mission upload initiated for {mission_count} waypoint(s)")
    mavros_waypoints = _build_mavros_waypoints(waypoints)
    response = bridge.push_waypoints(mavros_waypoints)
    if not response.get("success", False):
        raise RuntimeError(f"MAVROS mission push failed: {response}")
    current_mission = list(waypoints)
    with mavros_telem_lock:
        current_state.completedWaypointIds = []
        current_state.activeWaypointIndex = None
        current_state.current_waypoint_id = None
        mission_log_state['last_active_seq'] = None
        mission_log_state['last_reached_seq'] = None
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
    """Handle GET_MISSION command."""
    global current_mission
    _require_vehicle_bridge()
    try:
        mission_waypoints = download_mission_from_vehicle()
        return {
            'status': 'success',
            'message': f'Downloaded {len(mission_waypoints)} waypoints',
            'waypoints': mission_waypoints
        }
    except Exception as e:
        log_message(f"Mission download failed: {e}", "WARNING")
        return {
            'status': 'success',
            'message': f'Using cached mission: {len(current_mission)} waypoints',
            'waypoints': current_mission
        }


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

@socketio.on("mission_upload")
def on_mission_upload(data):
    """Mission upload via Socket.IO with progress updates."""
    global current_mission

    try:
        bridge = _require_vehicle_bridge()
    except Exception as exc:
        emit("mission_uploaded", {"ok": False, "error": str(exc)})
        return

    waypoints = data.get("waypoints", [])
    servo_config = data.get("servoConfig")

    if not waypoints:
        emit("mission_uploaded", {"ok": False, "error": "No waypoints provided"})
        return

    if servo_config:
        log_message(f"[mission_upload] Applying servo mode: {servo_config.get('mode')}")
        waypoints = apply_servo_modes(waypoints, servo_config)

    mission_count = len(waypoints)
    log_message(f"[mission_upload] Uploading {mission_count} waypoints via MAVROS")

    if not mission_upload_lock.acquire(blocking=False):
        emit("mission_uploaded", {"ok": False, "error": "Upload in progress"})
        return

    try:
        socketio.emit('mission_upload_progress', {'progress': 5})
        mavros_waypoints = _build_mavros_waypoints(waypoints)
        response = bridge.push_waypoints(mavros_waypoints)
        if not response.get("success", False):
            raise RuntimeError(f"MAVROS mission push failed: {response}")

        current_mission = list(waypoints)
        with mavros_telem_lock:
            current_state.completedWaypointIds = []
            current_state.activeWaypointIndex = None
            current_state.current_waypoint_id = None
            mission_log_state['last_active_seq'] = None
            mission_log_state['last_reached_seq'] = None
        _record_mission_event(
            f"Mission uploaded ({mission_count} items)",
            status='MISSION_UPLOADED'
        )
        socketio.emit('mission_upload_progress', {'progress': 100})
        emit("mission_uploaded", {"ok": True, "count": mission_count})
    except Exception as e:
        log_message(f"[mission_upload] Error: {e}", "ERROR")
        emit("mission_uploaded", {"ok": False, "error": str(e)})
    finally:
        try:
            mission_upload_lock.release()
        except Exception:
            pass


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
    log_message('Client connected')
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
    log_message('Client disconnected')


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


@socketio.on('send_command')
def handle_command(data):
    """Handle incoming commands from the frontend."""
    try:
        _require_vehicle_bridge()
    except Exception as exc:
        log_message("Command rejected - vehicle not connected", "ERROR", event_type='ui_request')
        payload = {'status': 'error', 'message': str(exc)}
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
        f"Socket command received: {command_type}",
        event_type='ui_request',
        meta={
            'command': command_type,
            'payload_keys': sorted(list(data.keys()))[:10] if isinstance(data, dict) else None
        }
    )

    handler = COMMAND_HANDLERS.get(command_type)
    if not handler:
        log_message(f"Unknown command: {command_type}", "ERROR", event_type='ui_request')
        payload = {
            'status': 'error',
            'message': f'Unknown command: {command_type}'
        }
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
        if isinstance(result, dict):
            emit('command_response', result)
            record_activity(
                f"Socket command '{command_type}' processed",
                event_type='server_response',
                meta={'command': command_type, 'response': _make_json_safe(result)}
            )
        elif isinstance(result, str):
            emit('command_response', {'status': 'success', 'message': result})
            record_activity(
                f"Socket command '{command_type}' processed",
                event_type='server_response',
                meta={'command': command_type, 'message': result}
            )
        else:
            payload = {'status': 'success', 'message': f"{command_type} dispatched"}
            emit('command_response', payload)
            record_activity(
                f"Socket command '{command_type}' processed",
                event_type='server_response',
                meta={'command': command_type, 'message': payload['message']}
            )
    except Exception as e:
        log_message(f"Command error ({command_type}): {e}", "ERROR", event_type='server_response')
        payload = {
            'status': 'error',
            'message': str(e),
            'command': command_type
        }
        emit('command_response', payload)
        record_activity(
            f"Socket command '{command_type}' failed",
            level='ERROR',
            event_type='server_response',
            meta=_make_json_safe(payload)
        )


# ============================================================================
# ACTIVITY MONITOR PAGE
# ============================================================================

MONITOR_PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>NRP Backend Activity Monitor</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body {
    font-family: Arial, Helvetica, sans-serif;
    margin: 0;
    background-color: #0f172a;
    color: #e2e8f0;
}
header {
    background: #111827;
    padding: 1rem 1.5rem;
    font-size: 1.4rem;
    font-weight: bold;
    border-bottom: 1px solid #1f2937;
}
.page {
    padding: 1.5rem;
    display: flex;
    flex-direction: column;
    gap: 1rem;
}
.toolbar {
    display: flex;
    flex-wrap: wrap;
    gap: 1rem;
    align-items: center;
    background: #111827;
    padding: 1rem;
    border: 1px solid #1f2937;
    border-radius: 0.5rem;
}
.toolbar label {
    display: flex;
    flex-direction: column;
    font-size: 0.85rem;
    color: #94a3b8;
    gap: 0.3rem;
}
.toolbar select,
.toolbar input {
    background: #0f172a;
    border: 1px solid #1f2937;
    color: #e2e8f0;
    padding: 0.4rem 0.6rem;
    border-radius: 0.35rem;
}
.toolbar button {
    background: #2563eb;
    color: #e2e8f0;
    border: none;
    border-radius: 0.35rem;
    padding: 0.45rem 0.8rem;
    cursor: pointer;
    font-weight: 600;
}
.toolbar button:hover {
    background: #1d4ed8;
}
.status-strip {
    display: flex;
    gap: 1.5rem;
    flex-wrap: wrap;
    font-size: 0.9rem;
    color: #94a3b8;
}
.status-strip span {
    display: flex;
    align-items: center;
    gap: 0.35rem;
}
.status-pill {
    padding: 0.15rem 0.5rem;
    border-radius: 999px;
    font-size: 0.8rem;
    text-transform: uppercase;
    letter-spacing: 0.05em;
}
.status-pill.ok {
    background: #047857;
    color: #d1fae5;
}
.status-pill.warn {
    background: #b91c1c;
    color: #fee2e2;
}
.log-container {
    background: #111827;
    border: 1px solid #1f2937;
    border-radius: 0.5rem;
    overflow: hidden;
    display: flex;
    flex-direction: column;
}
.log-scroll {
    overflow-y: auto;
    max-height: 65vh;
}
table {
    width: 100%;
    border-collapse: collapse;
    font-size: 0.85rem;
}
thead {
    background: #1f2937;
    position: sticky;
    top: 0;
    z-index: 1;
}
th, td {
    padding: 0.55rem 0.75rem;
    border-bottom: 1px solid #1f2937;
    text-align: left;
    vertical-align: top;
}
tbody tr:nth-child(even) {
    background: rgba(15, 23, 42, 0.6);
}
tr.level-error {
    background: rgba(220, 38, 38, 0.15);
}
tr.level-warning {
    background: rgba(234, 179, 8, 0.18);
}
code {
    font-family: Consolas, Menlo, Monaco, monospace;
    font-size: 0.8rem;
}
.message {
    white-space: pre-wrap;
}
@media (max-width: 720px) {
    header {
        font-size: 1.1rem;
    }
    th, td {
        font-size: 0.75rem;
    }
    .toolbar {
        flex-direction: column;
        align-items: flex-start;
    }
    .toolbar label {
        width: 100%;
    }
    .status-strip {
        flex-direction: column;
    }
}
</style>
</head>
<body>
<header>NRP Backend Activity Monitor</header>
<div class="page">
    <div class="toolbar">
        <label>
            Event type
            <select id="event-filter">
                <option value="">All events</option>
            </select>
        </label>
        <label>
            Text filter
            <input type="text" id="text-filter" placeholder="Search message or meta">
        </label>
        <label>
            <span>Display options</span>
            <span>
                <input type="checkbox" id="auto-scroll" checked> Auto-scroll
            </span>
        </label>
        <button id="refresh-button" type="button">Refresh</button>
        <button id="download-button" type="button">Download</button>
    </div>
    <div class="status-strip">
        <span>Socket:
            <span id="socket-status" class="status-pill warn">Connecting</span>
        </span>
        <span>Showing <span id="result-count">0</span> entries</span>
        <span>Last update <span id="last-updated">-</span></span>
    </div>
    <div class="log-container">
        <div class="log-scroll" id="log-scroll">
            <table id="activity-table">
                <thead>
                    <tr>
                        <th>Time (UTC)</th>
                        <th>Level</th>
                        <th>Event</th>
                        <th>Message</th>
                        <th>Meta</th>
                    </tr>
                </thead>
                <tbody id="activity-body"></tbody>
            </table>
        </div>
    </div>
</div>
<script src="https://cdn.socket.io/4.7.5/socket.io.min.js" crossorigin="anonymous"></script>
<script>
(function () {
    const API_LIMIT = 1000;
    const entries = [];
    const tableBody = document.getElementById('activity-body');
    const filterSelect = document.getElementById('event-filter');
    const searchInput = document.getElementById('text-filter');
    const autoScroll = document.getElementById('auto-scroll');
    const statusEl = document.getElementById('socket-status');
    const countEl = document.getElementById('result-count');
    const lastUpdatedEl = document.getElementById('last-updated');
    const logScroll = document.getElementById('log-scroll');
    const refreshButton = document.getElementById('refresh-button');
    const downloadButton = document.getElementById('download-button');
    const knownEvents = new Set();
    let renderQueued = false;

    function classForLevel(level) {
        if (!level) {
            return '';
        }
        const lower = String(level).toLowerCase();
        if (lower === 'error') {
            return 'level-error';
        }
        if (lower === 'warning' || lower === 'warn') {
            return 'level-warning';
        }
        return '';
    }

    function formatMeta(meta) {
        if (meta === null || meta === undefined) {
            return '';
        }
        if (typeof meta === 'string' || typeof meta === 'number' || typeof meta === 'boolean') {
            return String(meta);
        }
        try {
            return JSON.stringify(meta);
        } catch (err) {
            return String(meta);
        }
    }

    function matchesFilter(entry) {
        const selected = filterSelect.value;
        if (selected && entry.event !== selected) {
            return false;
        }
        const search = searchInput.value.trim().toLowerCase();
        if (!search) {
            return true;
        }
        const text = [
            entry.message || '',
            entry.level || '',
            entry.event || '',
            formatMeta(entry.meta)
        ].join(' ').toLowerCase();
        return text.indexOf(search) !== -1;
    }

    function updateFilterOptions() {
        let changed = false;
        entries.forEach((entry) => {
            if (entry.event && !knownEvents.has(entry.event)) {
                knownEvents.add(entry.event);
                changed = true;
            }
        });
        if (!changed) {
            return;
        }
        const current = filterSelect.value;
        const options = [''];
        knownEvents.forEach((event) => options.push(event));
        options.sort((a, b) => a.localeCompare(b));
        filterSelect.innerHTML = '';
        options.forEach((value) => {
            const option = document.createElement('option');
            option.value = value;
            option.textContent = value ? value : 'All events';
            filterSelect.appendChild(option);
        });
        filterSelect.value = current;
    }

    function render() {
        const filtered = entries.filter(matchesFilter);
        tableBody.innerHTML = '';
        filtered.forEach((entry) => {
            const tr = document.createElement('tr');
            tr.className = classForLevel(entry.level);

            const tsCell = document.createElement('td');
            tsCell.textContent = entry.isoTime || '';
            tr.appendChild(tsCell);

            const levelCell = document.createElement('td');
            levelCell.textContent = entry.level || '';
            tr.appendChild(levelCell);

            const eventCell = document.createElement('td');
            eventCell.textContent = entry.event || '';
            tr.appendChild(eventCell);

            const messageCell = document.createElement('td');
            messageCell.className = 'message';
            messageCell.textContent = entry.message || '';
            tr.appendChild(messageCell);

            const metaCell = document.createElement('td');
            metaCell.innerHTML = '<code></code>';
            const code = metaCell.querySelector('code');
            code.textContent = formatMeta(entry.meta);
            tr.appendChild(metaCell);

            tableBody.appendChild(tr);
        });
        countEl.textContent = filtered.length;
        if (filtered.length > 0) {
            lastUpdatedEl.textContent = filtered[filtered.length - 1].isoTime || '-';
        }
        if (autoScroll.checked) {
            logScroll.scrollTop = logScroll.scrollHeight;
        }
    }

    function scheduleRender() {
        if (renderQueued) {
            return;
        }
        renderQueued = true;
        window.requestAnimationFrame(() => {
            renderQueued = false;
            render();
        });
    }

    async function loadInitial() {
        try {
            const response = await fetch('/api/activity?limit=' + API_LIMIT);
            const data = await response.json();
            entries.length = 0;
            if (Array.isArray(data.entries)) {
                data.entries.forEach((entry) => entries.push(entry));
            }
            updateFilterOptions();
            scheduleRender();
        } catch (err) {
            console.error('Failed to fetch activity', err);
        }
    }

    async function refreshTypes() {
        try {
            const response = await fetch('/api/activity/types');
            const data = await response.json();
            if (Array.isArray(data.events)) {
                data.events.forEach((event) => knownEvents.add(event));
                updateFilterOptions();
            }
        } catch (err) {
            console.error('Failed to fetch activity types', err);
        }
    }

    filterSelect.addEventListener('change', scheduleRender);
    searchInput.addEventListener('input', scheduleRender);
    autoScroll.addEventListener('change', scheduleRender);

    refreshButton.addEventListener('click', (event) => {
        event.preventDefault();
        loadInitial();
    });

    downloadButton.addEventListener('click', (event) => {
        event.preventDefault();
        window.open('/api/activity/download', '_blank');
    });

    const socket = io();
    socket.on('connect', () => {
        statusEl.textContent = 'Connected';
        statusEl.className = 'status-pill ok';
    });
    socket.on('disconnect', () => {
        statusEl.textContent = 'Disconnected';
        statusEl.className = 'status-pill warn';
    });
    socket.on('server_activity', (entry) => {
        entries.push(entry);
        if (entries.length > API_LIMIT * 5) {
            entries.splice(0, entries.length - API_LIMIT * 5);
        }
        if (entry && entry.event) {
            knownEvents.add(entry.event);
            updateFilterOptions();
        }
        scheduleRender();
    });

    loadInitial();
    refreshTypes();
})();</script>
</body>
</html>
"""


@app.get("/monitor")
def monitor_dashboard():
    return Response(MONITOR_PAGE_HTML, mimetype='text/html')


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


@app.post("/api/rtk/inject")
def api_rtk_inject():
    body = request.get_json(silent=True) or {}
    ntrip_url = str(body.get('ntrip_url') or body.get('url') or '').strip()
    if not ntrip_url:
        return _http_error("Missing ntrip_url", 400)
    log_message("REST RTK inject endpoint not yet implemented", "WARNING")
    socketio.emit('rtk_log', {'message': 'REST RTK inject endpoint invoked but not implemented'})
    return _http_error("RTK injection via REST is not supported yet. Use the Socket.IO control panel.", 501)


@app.post("/api/servo/control")
def api_servo_control():
    body = request.get_json(silent=True) or {}
    servo_id = body.get('servo_id')
    angle = body.get('angle')
    if servo_id is None or angle is None:
        return _http_error("Missing servo_id or angle", 400)
    log_message("REST servo control endpoint not yet implemented", "WARNING")
    return _http_error("Direct servo control via REST is not available.", 501)

@socketio.on_error_default
def default_error_handler(e):
    log_message(f"SocketIO error: {e}", "ERROR")


@app.errorhandler(Exception)
def handle_exception(e):
    log_message(f"Flask error: {e}", "ERROR")
    return {'error': str(e)}, 500


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
        "wpmark": "wp_mark.py",
        "continuous": "continuous_line.py",
        "interval": "interval_spray.py",
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
        stdout=log_file, stderr=_subprocess.STDOUT
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


@app.get("/servo/status")
def servo_status():
    _cleanup_running()
    for m, info in list(_running.items()):
        alive = _is_pid_alive(info.get("pid"))
        info["running"] = bool(alive)
    return jsonify(_running)


@app.get("/servo/edit")
def servo_edit():
    data = dict(request.args)
    try:
        with open(CONFIG_PATH, "r") as f:
            cfg = json.load(f)
    except Exception:
        cfg = {}
    for k, v in data.items():
        # e.g. "wp_mark.delay_before_on"=2.0
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


try:
    log_message("Starting Flask-SocketIO server...")

    # Start cooperative background tasks under Socket.IO's greenlet scheduler
    socketio.start_background_task(maintain_mavros_connection)
    log_message("MAVROS connection maintenance task started")

    socketio.start_background_task(telemetry_loop)
    log_message("Telemetry task started")

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
