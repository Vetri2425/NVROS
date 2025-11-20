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
import json
from std_msgs.msg import String
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

        # ...existing code...
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
        self._estimator_topic: Optional[roslibpy.Topic] = None
        self._imu_topic: Optional[roslibpy.Topic] = None
        self._rtk_baseline_topic: Optional[roslibpy.Topic] = None
        self._heading_topic: Optional[roslibpy.Topic] = None
        self._battery_topic: Optional[roslibpy.Topic] = None
        self._mission_topic: Optional[roslibpy.Topic] = None
        self._mission_reached_topic: Optional[roslibpy.Topic] = None
        self._rc_out_topic: Optional[roslibpy.Topic] = None

        self._rtcm_topic: Optional[roslibpy.Topic] = None
        self._setpoint_topic: Optional[roslibpy.Topic] = None

        # Mission controller integration - REMOVED
        # Mission controller topics and handlers have been removed
        # Use direct MAVROS services for mission operations

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

    def set_armed(self, value: bool, timeout: float = 5.0) -> Dict[str, Any]:
        """Convenience method for mission controller compatibility. Arms or disarms the vehicle."""
        try:
            response = self.arm(value=value, timeout=timeout)
            return {
                'success': True,
                'armed': value,
                'response': response
            }
        except Exception as e:
            return {
                'success': False,
                'armed': False,
                'error': str(e)
            }

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

    def set_current_waypoint(self, wp_seq: int, timeout: float = 5.0) -> Dict[str, Any]:
        """
        Set the current mission waypoint (skip to specific waypoint).
        
        Args:
            wp_seq: Waypoint sequence number to jump to (0-based)
            timeout: Service call timeout
            
        Returns:
            Dict with 'success' (bool) field
        """
        service = roslibpy.Service(self._ros, "/mavros/mission/set_current", "mavros_msgs/WaypointSetCurrent")
        request = roslibpy.ServiceRequest({"wp_seq": int(wp_seq)})
        response = self._call_service(service, request, timeout)
        if not response.get("success", False):
            raise ServiceError(f"Failed to set current waypoint to {wp_seq}: {response}")
        return response

    def send_command_long(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        timeout: float = 5.0
    ) -> Dict[str, Any]:
        """
        Send a MAVLink COMMAND_LONG via MAVROS.
        
        Args:
            command: MAV_CMD command ID (e.g., 183 for DO_SET_SERVO)
            param1-7: Command parameters
            timeout: Service call timeout
            
        Returns:
            Dict with 'success' (bool) and 'result' (int) fields
        """
        service = roslibpy.Service(self._ros, "/mavros/cmd/command", "mavros_msgs/CommandLong")
        request = roslibpy.ServiceRequest({
            "broadcast": False,
            "command": int(command),
            "confirmation": 0,
            "param1": float(param1),
            "param2": float(param2),
            "param3": float(param3),
            "param4": float(param4),
            "param5": float(param5),
            "param6": float(param6),
            "param7": float(param7)
        })
        return self._call_service(service, request, timeout)

    def set_servo(self, servo_number: int, pwm_value: int, timeout: float = 5.0) -> Dict[str, Any]:
        """
        Set a servo to a specific PWM value using MAV_CMD_DO_SET_SERVO (183).
        
        Args:
            servo_number: Servo/channel number (typically 1-16)
            pwm_value: PWM value in microseconds (typically 1000-2000)
            timeout: Service call timeout
            
        Returns:
            Dict with 'success' (bool) and 'result' (int) fields
        """
        MAV_CMD_DO_SET_SERVO = 183
        return self.send_command_long(
            command=MAV_CMD_DO_SET_SERVO,
            param1=float(servo_number),
            param2=float(pwm_value),
            timeout=timeout
        )

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
        # HYBRID APPROACH:
        # 1) Position (lat/lon/alt): From /mavros/global_position/global_corrected (corrected, lower Hz)
        # 2) RTK quality: From /mavros/gpsstatus/gps1/raw (fix_type, eph, epv, satellites - 5Hz)
        self._navsat_topic = roslibpy.Topic(self._ros, "/mavros/global_position/global_corrected", "sensor_msgs/NavSatFix")
        self._gps_raw_topic = roslibpy.Topic(self._ros, "/mavros/gpsstatus/gps1/raw", "mavros_msgs/GPSRAW")
        
        self._rtk_baseline_topic = roslibpy.Topic(self._ros, "/mavros/gps_rtk/rtk_baseline", "mavros_msgs/RTKBaseline")
        self._heading_topic = roslibpy.Topic(self._ros, "/mavros/global_position/compass_hdg", "std_msgs/Float64")
        # Estimator / IMU topics (alignment & raw IMU data)
        self._estimator_topic = roslibpy.Topic(self._ros, "/mavros/estimator_status", "mavros_msgs/EstimatorStatus")
        self._imu_topic = roslibpy.Topic(self._ros, "/mavros/imu/data", "sensor_msgs/Imu")
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
        
        # Servo output (PWM values)
        self._rc_out_topic = roslibpy.Topic(self._ros, "/mavros/rc/out", "mavros_msgs/RCOut")

        # Subscribe to all topics
        self._state_topic.subscribe(self._handle_state)
        self._navsat_topic.subscribe(self._handle_navsat)  # Position from global_corrected (corrected altitude)
        self._gps_raw_topic.subscribe(self._handle_gps_raw)  # RTK quality, eph, epv, satellites (5Hz)
        self._rtk_baseline_topic.subscribe(self._handle_rtk_baseline)
        self._heading_topic.subscribe(self._handle_heading)
        # Subscribe to estimator and IMU topics to expose alignment and raw IMU to the backend
        try:
            self._estimator_topic.subscribe(self._handle_estimator_status)
        except Exception:
            # Best-effort: some systems may not publish estimator_status
            pass
        try:
            self._imu_topic.subscribe(self._handle_imu_data)
        except Exception:
            pass
        self._velocity_topic.subscribe(self._handle_velocity)
        self._battery_topic.subscribe(self._handle_battery)
        self._sys_status_topic.subscribe(self._handle_sys_status)
        self._rc_topic.subscribe(self._handle_rc)
        self._signal_topic.subscribe(self._handle_signal)
        self._mission_topic.subscribe(self._handle_waypoint_list)
        self._mission_reached_topic.subscribe(self._handle_waypoint_reached)
        self._rc_out_topic.subscribe(self._handle_servo_output)

    # ------------------------------- topic handlers

    def _handle_state(self, message: Dict[str, Any]) -> None:
        """Handle vehicle state updates (mode, armed status)."""
        connected = bool(message.get("connected", False))
        with self._lock:
            self._connected = connected and self._ros.is_connected

        self._broadcast_telem({
            "mode": str(message.get("mode", "UNKNOWN")),
            "armed": bool(message.get("armed", False)),
            "status": "armed" if bool(message.get("armed", False)) else "disarmed",
            "connected": connected
        }, message_type="state")

    def _handle_gps_raw(self, message: Dict[str, Any]) -> None:
        """Handle raw GPS data from /mavros/gpsstatus/gps1/raw.
        
        THIS HANDLER PROCESSES RTK QUALITY ONLY (NOT POSITION):
        - fix_type: 0-6 where 6=RTK Fixed, 5=RTK Float, 4=DGPS, etc.
        - eph/epv: horizontal/vertical accuracy in centimeters
        - satellites_visible: actual satellite count
        
        Position (lat/lon/alt) comes from /mavros/global_position/global_corrected
        via _handle_navsat() for better altitude accuracy.
        
        Example raw message:
        {
            "fix_type": 6,
            "lat": 130720581,
            "lon": 802619324,
            "alt": 16610,
            "eph": 70,
            "epv": 120,
            "satellites_visible": 29,
            "vel": 0,
            "cog": 18000
        }
        """
        # Extract accuracy data (in centimeters)
        eph_cm = int(message.get("eph", 0))
        epv_cm = int(message.get("epv", 0))
        eph = eph_cm / 100.0  # meters (horizontal accuracy)
        epv = epv_cm / 100.0  # meters (vertical accuracy)
        
        # Extract fix quality
        fix_type = int(message.get("fix_type", 0))
        satellites_visible = int(message.get("satellites_visible", 0))
        
        # Extract velocity and course
        vel_cm_s = int(message.get("vel", 0))
        vel = vel_cm_s / 100.0  # m/s
        cog_cdeg = int(message.get("cog", 0))
        cog = cog_cdeg / 100.0  # degrees
        
        # Map fix type to RTK status string
        # Based on GPS fix type definitions:
        # 0: No GPS, 1: No Fix, 2: DGPS, 3: 3D Fix, 4: 3D DGPS, 5: RTK Float, 6: RTK Fixed
        rtk_status_map = {
            0: "No GPS",
            1: "No Fix",
            2: "DGPS",
            3: "3D Fix",
            4: "3D DGPS",
            5: "RTK Float",
            6: "RTK Fixed"
        }
        rtk_status = rtk_status_map.get(fix_type, "Unknown")
        
        # DEBUG: Log GPS quality updates
        print(f"[MAVROS_BRIDGE] GPS_RAW Quality: fix={fix_type} ({rtk_status}), sats={satellites_visible}, "
              f"eph={eph:.2f}m, epv={epv:.2f}m, vel={vel:.2f}m/s, cog={cog:.1f}°", flush=True)
        
        # Broadcast GPS fix quality (RTK status, accuracy metrics, satellites)
        # NOTE: Position is now handled separately by _handle_navsat() from global_corrected
        self._broadcast_telem({
            "rtk_status": rtk_status,
            "fix_type": fix_type,
            "satellites_visible": satellites_visible,
            "hrms": eph,  # horizontal RMS accuracy
            "vrms": epv,  # vertical RMS accuracy
            "velocity": vel,
            "course": cog
        }, message_type="gps_fix")

    def _handle_navsat(self, message: Dict[str, Any]) -> None:
        """Handle global position updates from /mavros/global_position/global_corrected.
        
        Position data source: /mavros/global_position/global_corrected
        - Provides corrected altitude (+92.2m fix applied by gps_altitude_corrector.py ROS node)
        - Lower update rate (~1 Hz) but more accurate position
        - This is the PRIMARY source for rover position coordinates
        
        RTK quality metrics (fix_type, eph, epv, satellites) still come from
        /mavros/gpsstatus/gps1/raw via _handle_gps_raw() at higher frequency (5 Hz).
        """
        lat = float(message.get("latitude", 0.0))
        lon = float(message.get("longitude", 0.0))
        alt = float(message.get("altitude", 0.0))  # Already corrected by ROS node (+92.2m)
        
        # DEBUG: Log position updates from global_corrected
        print(f"[MAVROS_BRIDGE] NavSat Position: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}m (corrected)", flush=True)
        
        # Broadcast position data (ONLY position, not RTK quality)
        self._broadcast_telem({
            "latitude": lat,
            "longitude": lon,
            "altitude": alt,
            "relative_altitude": 0.0  # Not available in NavSatFix
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

    def _handle_rtk_baseline(self, message: Dict[str, Any]) -> None:
        """Handle RTK baseline updates (distance from base station)."""
        # Debug log raw baseline message for diagnostics
        try:
            print(f"[MAVROS_BRIDGE] Received rtk_baseline message: {message}", flush=True)
        except Exception:
            pass
        # Extract baseline vectors (in meters from base station)
        baseline_a = message.get("baseline_a_mm", 0) / 1000.0  # Convert mm to meters
        baseline_b = message.get("baseline_b_mm", 0) / 1000.0
        baseline_c = message.get("baseline_c_mm", 0) / 1000.0
        
        # Calculate total baseline distance
        baseline_distance = (baseline_a**2 + baseline_b**2 + baseline_c**2) ** 0.5
        
        # Extract accuracy information
        accuracy = message.get("accuracy", 0)
        iar_num_hypotheses = message.get("iar_num_hypotheses", 0)
        
        # Extract time of week
        tow = message.get("tow", 0)
        rtk_receiver_id = message.get("rtk_receiver_id", 0)
        rtk_health = message.get("rtk_health", 0)
        rtk_rate = message.get("rtk_rate", 0)
        nsats = message.get("nsats", 0)
        
        self._broadcast_telem({
            "rtk_baseline": {
                "baseline_a_mm": baseline_a * 1000,  # Keep original mm values
                "baseline_b_mm": baseline_b * 1000,
                "baseline_c_mm": baseline_c * 1000,
                "baseline_distance": baseline_distance,  # Total distance in meters
                "accuracy": accuracy,
                "iar_num_hypotheses": iar_num_hypotheses,
                "tow": tow,
                "rtk_receiver_id": rtk_receiver_id,
                "rtk_health": rtk_health,
                "rtk_rate": rtk_rate,
                "nsats": nsats
            }
        }, message_type="rtk_baseline")

    def _handle_heading(self, message: Dict[str, Any]) -> None:
        """Handle compass heading updates."""
        self._broadcast_telem({
            "heading": float(message.get("data", 0.0))
        }, message_type="heading")

    def _handle_battery(self, message: Dict[str, Any]) -> None:
        """Handle battery status updates."""
        try:
            raw_pct = message.get("percentage", None)
            raw_volt = message.get("voltage", None)
            raw_curr = message.get("current", None)
            print(f"[MAVROS_BRIDGE] Battery msg: percentage={raw_pct}, voltage={raw_volt}, current={raw_curr}", flush=True)

            pct = float(raw_pct) if raw_pct is not None else -1.0
            import math
            if math.isnan(pct) or math.isinf(pct):
                # Skip invalid metrics
                return
            percentage = pct * 100.0  # Convert to percentage scale

            voltage = float(raw_volt) if raw_volt is not None else 0.0
            current = float(raw_curr) if raw_curr is not None else 0.0
            self._broadcast_telem({
                "battery": percentage,
                "voltage": voltage,
                "current": current
            }, message_type="battery")
            print(f"[MAVROS_BRIDGE] Battery broadcast: battery={percentage:.2f}%, voltage={voltage}, current={current}", flush=True)
        except Exception as e:
            # Never crash on telemetry
            print(f"[MAVROS_BRIDGE] Battery handler error: {e}", flush=True)

    def _handle_estimator_status(self, message: Dict[str, Any]) -> None:
        """Handle estimator status updates and expose IMU alignment state.

        The exact field naming can vary by autopilot/driver. Try a few common
        keys and fall back to a conservative default (unaligned).
        """
        aligned = None
        try:
            # Common field names from various stacks
            if 'aligned' in message:
                aligned = bool(message.get('aligned'))
            elif 'attitude_aligned' in message:
                aligned = bool(message.get('attitude_aligned'))
            elif isinstance(message.get('status'), dict) and 'attitude_aligned' in message.get('status'):
                aligned = bool(message.get('status', {}).get('attitude_aligned'))
            # Some messages provide numeric flags; try a simple bit-check fallback
            elif 'flags' in message:
                try:
                    flags = int(message.get('flags', 0))
                    aligned = bool(flags & 0x01)
                except Exception:
                    aligned = None
        except Exception:
            aligned = None

        if aligned is None:
            aligned = False

        # Broadcast a small payload for server to merge
        self._broadcast_telem({
            'imu_aligned': bool(aligned)
        }, message_type='estimator_status')

    def _handle_imu_data(self, message: Dict[str, Any]) -> None:
        """Expose raw IMU fields (orientation/ang vel/linear acc) to the backend.

        Avoid publishing large nested structures frequently; only include the
        main numeric groups so the frontend can display an 'IMU' value if
        desired.
        """
        try:
            orientation = message.get('orientation')
            angular_velocity = message.get('angular_velocity') or message.get('angular_velocity', {})
            linear_acceleration = message.get('linear_acceleration') or message.get('linear_acceleration', {})

            payload = {
                'imu': {
                    'orientation': orientation,
                    'angular_velocity': angular_velocity,
                    'linear_acceleration': linear_acceleration
                }
            }
            self._broadcast_telem(payload, message_type='imu')
        except Exception:
            # best-effort
            pass

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

    def _handle_servo_output(self, message: Dict[str, Any]) -> None:
        """Handle servo output (PWM) updates from /mavros/rc/out."""
        channels = message.get("channels", [])
        
        # Build servo state with PWM values for each channel
        # Channels are 0-indexed, so servo 1 is channels[0], etc.
        servo_state = {
            "channels": channels,
            "count": len(channels)
        }
        
        # Add individual servo PWM values (up to 16 servos)
        for i, pwm in enumerate(channels[:16], start=1):
            servo_state[f"servo{i}_pwm"] = int(pwm)
        
        self._broadcast_telem({
            "servo_output": servo_state
        }, message_type="servo_output")

