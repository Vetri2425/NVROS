"""
Infrastructure layer translating backend requests into MAVROS ROS topics and services.

The rest of the Flask application should remain agnostic of whether the vehicle
is accessed through PyMAVLink or MAVROS. This module provides the minimum set of
operations required by the server today (connection state, telemetry fan-out,
mission management, RTK forwarding, and common command helpers).
"""

from __future__ import annotations

import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional

import roslibpy

TelemetryCallback = Callable[[Dict[str, Any]], None]


def _ros_time() -> Dict[str, int]:
    """Return a ROS-style time dictionary based on wall-clock time."""
    now = time.time()
    secs = int(now)
    nsecs = int((now - secs) * 1_000_000_000)
    return {"secs": secs, "nsecs": nsecs}


class ServiceError(RuntimeError):
    """Raised when a ROS service call fails."""


class MavrosBridge:
    """
    Thin wrapper around MAVROS topics/services via rosbridge.

    All ROS interactions are executed with threading-friendly primitives so that
    the Flask eventlet server can keep running in parallel.
    """

    def __init__(
        self,
        *,
        host: Optional[str] = None,
        port: Optional[int] = None,
    ) -> None:
        self.host = host or os.getenv("MAVROS_BRIDGE_HOST", "127.0.0.1")
        self.port = int(port if port is not None else os.getenv("MAVROS_BRIDGE_PORT", "9090"))  # rosbridge_server WebSocket port
        self.fcu_url = os.getenv("MAVROS_FCU_URL", "/dev/ttyACM0:115200")

        self._ros = roslibpy.Ros(host=self.host, port=self.port)
        self._lock = threading.Lock()
        self._connected = False
        self._subscriptions_ready = False
        self._telemetry_state: Dict[str, Any] = {}

        self._telemetry_callbacks: List[TelemetryCallback] = []
        self._waypoint_event = threading.Event()
        self._latest_waypoints: Optional[Dict[str, Any]] = None

        self._state_topic: Optional[roslibpy.Topic] = None
        self._navsat_topic: Optional[roslibpy.Topic] = None
        self._gps_fix_topic: Optional[roslibpy.Topic] = None
        self._heading_topic: Optional[roslibpy.Topic] = None
        self._battery_topic: Optional[roslibpy.Topic] = None
        self._mission_topic: Optional[roslibpy.Topic] = None
        self._mission_reached_topic: Optional[roslibpy.Topic] = None

        self._rtcm_topic: Optional[roslibpy.Topic] = None
        self._setpoint_topic: Optional[roslibpy.Topic] = None

        self._ros.on_ready(self._on_ready, run_in_thread=True)
        self._ros.on("close", lambda _: self._on_close())

    # ------------------------------------------------------------------ public

    def connect(self, timeout: float = 10.0) -> None:
        """Open the rosbridge WebSocket and await the initial connection."""
        try:
            if not self._ros.is_connected:
                self._ros.run()

            start = time.time()
            while not self._ros.is_connected:
                if timeout and (time.time() - start) >= timeout:
                    raise TimeoutError(f"Failed to connect to MAVROS bridge at {self.host}:{self.port}")
                time.sleep(0.1)

            with self._lock:
                if not self._subscriptions_ready:
                    self._setup_subscriptions()
                    self._subscriptions_ready = True
        except Exception as e:
            raise ConnectionError(f"Failed to connect to MAVROS: {str(e)}")

    def close(self) -> None:
        """Close the rosbridge connection."""
        try:
            if hasattr(self._ros, 'terminate'):
                self._ros.terminate()
            else:
                self._ros.close()
        except Exception:
            pass
        finally:
            with self._lock:
                self._connected = False

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected and self._ros.is_connected

    def subscribe_telemetry(self, callback: TelemetryCallback) -> None:
        """Register a callback invoked with telemetry updates."""
        with self._lock:
            self._telemetry_callbacks.append(callback)

    def arm(self, *, value: bool, timeout: float = 5.0) -> Dict[str, Any]:
        service = roslibpy.Service(self._ros, "/mavros/cmd/arming", "mavros_msgs/CommandBool")
        request = roslibpy.ServiceRequest({"value": bool(value)})
        response = self._call_service(service, request, timeout)
        # CommandBool returns both success and result fields; treat either as acknowledgement.
        success = bool(response.get("success")) or bool(response.get("result"))
        if not success:
            raise ServiceError(f"Vehicle rejected arm value {value}: {response}")
        return response

    def set_mode(
        self,
        *,
        mode: str,
        base_mode: int = 0,
        custom_mode: Optional[str] = None,
        timeout: float = 5.0
    ) -> Dict[str, Any]:
        service = roslibpy.Service(self._ros, "/mavros/set_mode", "mavros_msgs/SetMode")
        request = roslibpy.ServiceRequest({
            "base_mode": int(base_mode),
            "custom_mode": str(custom_mode if custom_mode is not None else mode),
        })
        response = self._call_service(service, request, timeout)
        if not bool(response.get("mode_sent", False)):
            raise ServiceError(f"Vehicle rejected mode '{mode}': {response}")
        return response

    def send_global_position_target(
        self,
        *,
        latitude: float,
        longitude: float,
        altitude: float,
        type_mask: int,
        coordinate_frame: int,
    ) -> None:
        if self._setpoint_topic is None:
            self._setpoint_topic = roslibpy.Topic(self._ros, "/mavros/setpoint_raw/global", "mavros_msgs/GlobalPositionTarget")
            self._setpoint_topic.advertise()

        message = roslibpy.Message({
            "header": {"stamp": _ros_time(), "frame_id": "map"},
            "coordinate_frame": int(coordinate_frame),
            "type_mask": int(type_mask),
            "latitude": float(latitude),
            "longitude": float(longitude),
            "altitude": float(altitude),
            "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "acceleration_or_force": {"x": 0.0, "y": 0.0, "z": 0.0},
            "yaw": 0.0,
            "yaw_rate": 0.0,
        })
        self._setpoint_topic.publish(message)

    def push_waypoints(self, waypoints: List[Dict[str, Any]], timeout: float = 15.0) -> Dict[str, Any]:
        service = roslibpy.Service(self._ros, "/mavros/mission/push", "mavros_msgs/WaypointPush")
        request = roslibpy.ServiceRequest({
            "start_index": 0,
            "waypoints": waypoints,
        })
        return self._call_service(service, request, timeout)

    def pull_waypoints(self, timeout: float = 15.0) -> Dict[str, Any]:
        with self._lock:
            self._waypoint_event.clear()
            self._latest_waypoints = None

        service = roslibpy.Service(self._ros, "/mavros/mission/pull", "mavros_msgs/WaypointPull")
        response = self._call_service(service, roslibpy.ServiceRequest({}), timeout)
        if not response.get("success", False):
            raise ServiceError(f"Waypoint pull rejected: {response}")

        if not self._waypoint_event.wait(timeout):
            raise TimeoutError("Timed out waiting for waypoint list after pull")

        with self._lock:
            if self._latest_waypoints is None:
                raise ServiceError("No waypoint list received from MAVROS")
            return self._latest_waypoints

    def clear_waypoints(self, timeout: float = 5.0) -> Dict[str, Any]:
        service = roslibpy.Service(self._ros, "/mavros/mission/clear", "mavros_msgs/WaypointClear")
        return self._call_service(service, roslibpy.ServiceRequest({}), timeout)

    def send_rtcm(self, payload: bytes) -> None:
        if not payload:
            return
        if self._rtcm_topic is None:
            self._rtcm_topic = roslibpy.Topic(self._ros, "/mavros/gps_rtk/send_rtcm", "mavros_msgs/RTCM")
            self._rtcm_topic.advertise()

        message = roslibpy.Message({
            "header": {"stamp": _ros_time(), "frame_id": "rtcm"},
            "data": list(payload),
        })
        self._rtcm_topic.publish(message)

    def latest_waypoints(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._latest_waypoints

    # ---------------------------------------------------------------- internal

    def _call_service(self, service: roslibpy.Service, request: roslibpy.ServiceRequest, timeout: float) -> Dict[str, Any]:
        result_holder: Dict[str, Any] = {}
        event = threading.Event()

        def _success(result: Dict[str, Any]) -> None:
            result_holder["result"] = result
            event.set()

        def _error(error: Any) -> None:
            result_holder["error"] = error
            event.set()

        service.call(request, callback=_success, errback=_error)

        if not event.wait(timeout):
            raise TimeoutError(f"Service call timeout for {service.name}")

        if "error" in result_holder:
            raise ServiceError(f"Service {service.name} failed: {result_holder['error']}")

        return result_holder.get("result", {})

    def _broadcast_telem(self, payload: Dict[str, Any], message_type: Optional[str] = None) -> None:
        # Update internal state with new data
        with self._lock:
            self._telemetry_state.update(payload)
            # Create complete telemetry update with all current state
            complete_update = {
                "lastUpdate": int(time.time() * 1000),  # milliseconds
                **self._telemetry_state
            }
            if message_type is not None:
                complete_update["type"] = message_type
            callbacks = list(self._telemetry_callbacks)

        for callback in callbacks:
            try:
                callback(complete_update)
            except Exception:
                # Telemetry callbacks are best-effort; ignore downstream failures.
                pass

    def _on_ready(self, *_: Any) -> None:
        with self._lock:
            self._connected = True

    def _on_close(self) -> None:
        with self._lock:
            self._connected = False

    def _setup_subscriptions(self) -> None:
        """Set up topic subscriptions for telemetry data."""
        # Vehicle state
        self._state_topic = roslibpy.Topic(self._ros, "/mavros/state", "mavros_msgs/State")
        
        # Position and navigation
        self._navsat_topic = roslibpy.Topic(self._ros, "/mavros/global_position/global", "sensor_msgs/NavSatFix")
        self._gps_fix_topic = roslibpy.Topic(self._ros, "/mavros/global_position/raw/fix", "sensor_msgs/NavSatFix")
        self._heading_topic = roslibpy.Topic(self._ros, "/mavros/global_position/compass_hdg", "std_msgs/Float64")
        self._velocity_topic = roslibpy.Topic(self._ros, "/mavros/global_position/raw/gps_vel", "geometry_msgs/TwistStamped")
        
        # System status
        self._battery_topic = roslibpy.Topic(self._ros, "/mavros/battery", "sensor_msgs/BatteryState")
        self._sys_status_topic = roslibpy.Topic(self._ros, "/mavros/sys_status", "mavros_msgs/SysStatus")
        
        # RC and signal
        self._rc_topic = roslibpy.Topic(self._ros, "/mavros/rc/in", "mavros_msgs/RCIn")
        self._signal_topic = roslibpy.Topic(self._ros, "/mavros/radio_status", "mavros_msgs/RadioStatus")
        
        # Mission tracking
        self._mission_topic = roslibpy.Topic(self._ros, "/mavros/mission/waypoints", "mavros_msgs/WaypointList")
        self._mission_reached_topic = roslibpy.Topic(self._ros, "/mavros/mission/reached", "mavros_msgs/WaypointReached")

        # Subscribe to all topics
        self._state_topic.subscribe(self._handle_state)
        self._navsat_topic.subscribe(self._handle_navsat)
        self._gps_fix_topic.subscribe(self._handle_gps_fix)
        self._heading_topic.subscribe(self._handle_heading)
        self._velocity_topic.subscribe(self._handle_velocity)
        self._battery_topic.subscribe(self._handle_battery)
        self._sys_status_topic.subscribe(self._handle_sys_status)
        self._rc_topic.subscribe(self._handle_rc)
        self._signal_topic.subscribe(self._handle_signal)
        self._mission_topic.subscribe(self._handle_waypoint_list)
        self._mission_reached_topic.subscribe(self._handle_waypoint_reached)

    # ------------------------------- topic handlers

    def _handle_state(self, message: Dict[str, Any]) -> None:
        """Handle vehicle state updates (mode, armed status)."""
        connected = bool(message.get("connected", False))
        with self._lock:
            self._connected = connected and self._ros.is_connected

        self._broadcast_telem({
            "mode": str(message.get("mode", "UNKNOWN")),
            "status": "armed" if bool(message.get("armed", False)) else "disarmed",
            "connected": connected
        }, message_type="state")

    def _handle_navsat(self, message: Dict[str, Any]) -> None:
        """Handle global position updates."""
        self._broadcast_telem({
            "latitude": float(message.get("latitude", 0.0)),
            "longitude": float(message.get("longitude", 0.0)),
            "altitude": float(message.get("altitude", 0.0)),
            "relative_altitude": float(message.get("relative_altitude", 0.0))
        }, message_type="navsat")

    def _handle_gps_fix(self, message: Dict[str, Any]) -> None:
        """Handle GPS fix quality and RTK status."""
        status = message.get("status", {})
        fix_type = status.get("status", 0)
        
        # Map fix type to RTK status string
        rtk_status = "No Fix"
        if fix_type == 4:
            rtk_status = "RTK Fixed"
        elif fix_type == 5:
            rtk_status = "RTK Float"
        elif fix_type == 2:
            rtk_status = "DGPS"
        elif fix_type == 1:
            rtk_status = "GPS"
            
        covariance = message.get("position_covariance", [])
        hrms = (covariance[0] + covariance[4]) ** 0.5 if len(covariance) >= 5 else None
        
        self._broadcast_telem({
            "rtk_status": rtk_status,
            "fix_type": fix_type,
            "satellites_visible": status.get("satellites_visible", 0),
            "hrms": hrms
        }, message_type="gps_fix")

    def _handle_heading(self, message: Dict[str, Any]) -> None:
        """Handle compass heading updates."""
        self._broadcast_telem({
            "heading": float(message.get("data", 0.0))
        }, message_type="heading")

    def _handle_battery(self, message: Dict[str, Any]) -> None:
        """Handle battery status updates."""
        percentage = float(message.get("percentage", -1.0)) * 100  # Convert to percentage
        self._broadcast_telem({
            "battery": percentage,
            "voltage": float(message.get("voltage", 0.0)),
            "current": float(message.get("current", 0.0))
        }, message_type="battery")

    def _handle_sys_status(self, message: Dict[str, Any]) -> None:
        """Handle system status updates."""
        self._broadcast_telem({
            "cpu_load": float(message.get("load", 0.0)),
            "drop_rate": float(message.get("drop_rate_comm", 0.0))
        }, message_type="sys_status")

    def _handle_velocity(self, message: Dict[str, Any]) -> None:
        """Handle velocity updates."""
        twist = message.get("twist", {})
        linear = twist.get("linear", {})
        self._broadcast_telem({
            "groundspeed": ((linear.get("x", 0.0) ** 2 + linear.get("y", 0.0) ** 2) ** 0.5)
        }, message_type="velocity")

    def _handle_rc(self, message: Dict[str, Any]) -> None:
        """Handle RC input status."""
        channels = message.get("channels", [])
        self._broadcast_telem({
            "rc_connected": len(channels) > 0
        }, message_type="rc")

    def _handle_signal(self, message: Dict[str, Any]) -> None:
        """Handle radio signal strength."""
        rssi = message.get("rssi", 0)
        # Convert RSSI to descriptive signal strength
        if rssi > -50:
            strength = "Excellent"
        elif rssi > -60:
            strength = "Good"
        elif rssi > -70:
            strength = "Fair"
        else:
            strength = "Poor"
            
        self._broadcast_telem({
            "signal_strength": strength
        }, message_type="signal")

    def _handle_waypoint_list(self, message: Dict[str, Any]) -> None:
        """Handle mission waypoint updates."""
        with self._lock:
            self._latest_waypoints = message
            self._waypoint_event.set()

        waypoints = message.get("waypoints", [])
        current_seq = message.get("current_seq")
        payload: Dict[str, Any] = {
            "waypoints": waypoints,
            "current_seq": current_seq
        }
        if waypoints:
            payload["mission_progress"] = {
                "total": len(waypoints),
                "current": 0  # Will be updated by reached messages
            }
        self._broadcast_telem(payload, message_type="mission_list")

    def _handle_waypoint_reached(self, message: Dict[str, Any]) -> None:
        """Handle waypoint reached updates."""
        wp_seq = message.get("wp_seq", 0)
        self._broadcast_telem({
            "mission_progress": {
                "current": wp_seq + 1  # Convert to 1-based for display
            },
            "wp_seq": wp_seq
        }, message_type="mission_reached")
