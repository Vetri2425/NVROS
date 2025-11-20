# Latitude/Longitude Data Handling from MAVROS - Code Analysis

## Overview
This document provides a comprehensive analysis of how latitude and longitude GPS data is retrieved from MAVROS, processed, and transmitted through the system.

---

## 1. Data Flow Architecture

```
MAVROS Topics
    ↓
MavrosBridge (mavros_bridge.py)
    ↓
_handle_gps_raw() / _handle_navsat()
    ↓
_broadcast_telem() with "navsat" message type
    ↓
server.py: _handle_mavros_telemetry()
    ↓
current_state.position (Position dataclass)
    ↓
get_rover_data() → emit to frontend
    ↓
UI receives rover_data via WebSocket
```

---

## 2. MAVROS Data Sources

### 2.1 GPS Raw Topic (`/mavros/gpsstatus/gps1/raw`)
**File:** `Backend/mavros_bridge.py:430-505`

#### Raw Data Format (from MAVROS):
```python
{
    "fix_type": 6,           # 0-6: 0/1=No Fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed
    "lat": 130720581,        # Raw integer in 1e7 format (degrees * 10,000,000)
    "lon": 802619324,        # Raw integer in 1e7 format (degrees * 10,000,000)
    "alt": 16610,            # Millimeters (AMSL altitude)
    "eph": 70,               # Horizontal accuracy in centimeters
    "epv": 120,              # Vertical accuracy in centimeters
    "satellites_visible": 29,
    "vel": 0,                # Velocity in cm/s
    "cog": 18000             # Course over ground in centidegrees
}
```

#### Conversion Logic (Lines 464-482):
```python
# Extract and convert position data
lat_raw = int(message.get("lat", 0))
lon_raw = int(message.get("lon", 0))
alt_mm = int(message.get("alt", 0))

# Convert to standard units
lat = lat_raw / 1e7  # Convert to degrees
lon = lon_raw / 1e7  # Convert to degrees
alt = alt_mm / 1000.0  # Convert to meters (AMSL)

# Extract accuracy data (in centimeters)
eph_cm = int(message.get("eph", 0))
epv_cm = int(message.get("epv", 0))
eph = eph_cm / 100.0  # Convert to meters (horizontal accuracy)
epv = epv_cm / 100.0  # Convert to meters (vertical accuracy)
```

**Key Points:**
- ✅ **Correct**: Divides by 1e7 to convert lat/lon from integer format to decimal degrees
- ✅ **Explicit fix_type mapping** ensures correct RTK status display
- ✅ Handles accuracy metrics (HRMS/VRMS) properly

---

### 2.2 NavSat Topic (`/mavros/global_position/global_corrected`)
**File:** `Backend/mavros_bridge.py:507-525`

#### Data Format:
```python
{
    "latitude": 13.0720581,      # Already in decimal degrees
    "longitude": 80.2619324,     # Already in decimal degrees
    "altitude": 16.610           # Altitude in meters (already corrected +92.2m by ROS node)
}
```

#### Handling Logic (Lines 514-525):
```python
lat = float(message.get("latitude", 0.0))
lon = float(message.get("longitude", 0.0))
alt = float(message.get("altitude", 0.0))  # Already corrected by ROS node

self._broadcast_telem({
    "latitude": lat,
    "longitude": lon,
    "altitude": alt,
    "relative_altitude": 0.0
}, message_type="navsat")
```

**Key Points:**
- ✅ Simple float conversion (already in decimal degrees)
- ✅ Altitude includes +92.2m correction applied by `gps_altitude_corrector.py` ROS node

---

## 3. Telemetry Broadcasting

### 3.1 `_broadcast_telem()` Method
**File:** `Backend/mavros_bridge.py` (inherited pattern)

The bridge publishes telemetry messages with standardized structure:
```python
self._broadcast_telem({
    "latitude": float,      # Decimal degrees
    "longitude": float,     # Decimal degrees
    "altitude": float,      # Meters
    "relative_altitude": float  # Meters
}, message_type="navsat")

self._broadcast_telem({
    "rtk_status": str,
    "fix_type": int,
    "satellites_visible": int,
    "hrms": float,          # Horizontal RMS accuracy in meters
    "vrms": float           # Vertical RMS accuracy in meters
}, message_type="gps_fix")
```

---

## 4. Server-Side Processing

### 4.1 Telemetry Handler
**File:** `Backend/server.py:1361-1450`

#### NavSat Message Handling (Lines 1398-1410):
```python
elif msg_type == "navsat":
    lat = message.get("latitude")
    lon = message.get("longitude")
    alt = message.get("altitude", 0.0)
    
    # DEBUG: Log navsat message reception
    print(f"[SERVER] Received navsat: lat={lat}, lon={lon}, alt={alt}", flush=True)
    
    if lat is not None and lon is not None:
        with mavros_telem_lock:
            # Store in Position dataclass
            current_state.position = Position(lat=float(lat), lng=float(lon))
            current_state.last_update = time.time()
            current_state.distanceToNext = float(alt or 0.0)
            print(f"[SERVER] Updated current_state.position: lat={current_state.position.lat:.7f}, lng={current_state.position.lng:.7f}", flush=True)
        schedule_fast_emit()
```

**Key Points:**
- ✅ Thread-safe update using `mavros_telem_lock`
- ✅ Stores lat/lon in `Position` dataclass
- ✅ Includes validation (`is not None`)
- ✅ High precision logging (7 decimal places = ~0.011m accuracy)

#### GPS Fix Quality Handling (Lines 1411-1476):
```python
elif msg_type == "gps_fix":
    status = message.get("status")
    fix_type = message.get("fix_type")
    satellites_visible = message.get("satellites_visible")
    covariance = message.get("position_covariance", [])
    hrms_msg = message.get("hrms")
    vrms_msg = message.get("vrms")

    with mavros_telem_lock:
        current_state.rtk_status = _map_navsat_status(status)
        
        if fix_type is not None:
            fix_type_int = int(fix_type)
            current_state.rtk_fix_type = fix_type_int
            # Infer RTK base link from fix_type (5-6 = fixed/float)
            current_state.rtk_base_linked = (fix_type_int >= 5)
        
        # Update satellites count
        if satellites_visible is not None:
            current_state.satellites_visible = int(satellites_visible)
        
        # Store accuracy metrics
        if hrms_msg is not None:
            current_state.hrms = f"{float(hrms_msg):.3f}"
        if vrms_msg is not None:
            current_state.vrms = f"{float(vrms_msg):.3f}"
```

**Key Points:**
- ✅ Extracts both fix type and accuracy
- ✅ Maps fix_type to RTK status
- ✅ Stores HRMS/VRMS with 3 decimal places precision

---

## 5. Data Emission to Frontend

### 5.1 `get_rover_data()` Function
**File:** `Backend/server.py:1268-1309`

```python
def get_rover_data():
    """Return complete rover data structure with all required fields."""
    try:
        current_state.signal_strength = derive_signal_strength(
            current_state.last_heartbeat, current_state.rc_connected
        )
    except Exception:
        pass
    
    data = current_state.to_dict()  # Serialize Position dataclass to dict
    
    # Add network telemetry
    try:
        network_data = network_monitor.get_network_data()
        data['network'] = network_data
    except Exception as e:
        print(f"[WARN] Network monitoring error: {e}", flush=True)
        data['network'] = {...}
    
    return data
```

### 5.2 Position Dataclass Structure
The `Position` class stores:
```python
@dataclass
class Position:
    lat: float      # Latitude in decimal degrees
    lng: float      # Longitude in decimal degrees
```

When converted to dict via `to_dict()`:
```python
{
    "position": {
        "lat": 13.0720581,   # Decimal degrees
        "lng": 80.2619324    # Decimal degrees
    },
    "heading": 45.0,
    "altitude": 16.61,
    "groundspeed": 2.5,
    "rtk_status": "RTK Fixed",
    "rtk_fix_type": 6,
    "satellites_visible": 29,
    "hrms": "0.070",
    "vrms": "0.120"
    ...
}
```

---

## 6. Waypoint/Mission Coordinate Handling

### 6.1 UI to MAVROS Conversion
**File:** `Backend/server.py:1145-1183`

#### Input Format (from UI):
```python
{
    "lat": 13.0720581,      # Decimal degrees
    "lng": 80.2619324,      # Decimal degrees
    "alt": 100.0,           # Meters
    "command": "WAYPOINT"
}
```

#### Conversion to MAVROS Format:
```python
def _build_mavros_waypoints(waypoints: List[dict]) -> List[dict]:
    """Translate UI waypoint dictionaries into MAVROS Waypoint structures."""
    mavros_waypoints: List[dict] = []
    for idx, wp in enumerate(waypoints):
        command_id = resolve_mav_command(wp.get("command"))
        
        # Unified coordinate extraction - always prioritize lat/lng field names
        lat = safe_float(wp.get("lat", wp.get("x_lat", 0.0)))
        lon = safe_float(wp.get("lng", wp.get("y_long", 0.0)))
        alt = safe_float(wp.get("alt", wp.get("z_alt", 0.0)))

        mavros_waypoints.append({
            "frame": frame,
            "command": command_id,
            "x_lat": lat,      # MAVROS field name: x_lat holds LATITUDE
            "y_long": lon,     # MAVROS field name: y_long holds LONGITUDE
            "z_alt": alt,
            ...
        })
    return mavros_waypoints
```

**Key Points:**
- ✅ Prioritizes UI field names (`lat`/`lng`) first
- ✅ Falls back to MAVROS field names (`x_lat`/`y_long`) if needed
- ✅ Prevents coordinate swapping with unified extraction logic

### 6.2 MAVROS to UI Conversion
**File:** `Backend/server.py:1127-1142`

#### Input (from MAVROS):
```python
{
    "x_lat": 13.0720581,     # MAVROS: x_lat = latitude
    "y_long": 80.2619324,    # MAVROS: y_long = longitude
    "z_alt": 100.0,
    "command": 16             # MAV_CMD_NAV_WAYPOINT
}
```

#### Conversion to UI Format:
```python
def _convert_mavros_waypoints_to_ui(waypoints: List[dict]) -> List[dict]:
    """Convert MAVROS waypoint dictionaries into the UI mission format."""
    converted: List[dict] = []
    for idx, wp in enumerate(waypoints or []):
        converted.append({
            "id": idx + 1,
            "command": 'WAYPOINT' if cmd == mavlink.MAV_CMD_NAV_WAYPOINT else ...,
            "lat": safe_float(wp.get("x_lat", 0.0)),      # Extract x_lat as lat
            "lng": safe_float(wp.get("y_long", 0.0)),     # Extract y_long as lng
            "alt": safe_float(wp.get("z_alt", 0.0)),
            ...
        })
    return converted
```

**Key Points:**
- ✅ Correctly maps `x_lat` → `lat` and `y_long` → `lng`
- ✅ Maintains data integrity through bidirectional conversion

---

## 7. Global Position Target Command

### 7.1 Send Global Position Target
**File:** `Backend/mavros_bridge.py:158-179`

Used for GOTO commands:
```python
def send_global_position_target(
    self,
    *,
    latitude: float,      # Decimal degrees
    longitude: float,     # Decimal degrees
    altitude: float,
    type_mask: int,
    coordinate_frame: int,
) -> None:
    if self._setpoint_topic is None:
        self._setpoint_topic = roslibpy.Topic(
            self._ros, 
            "/mavros/setpoint_raw/global", 
            "mavros_msgs/GlobalPositionTarget"
        )
        self._setpoint_topic.advertise()

    message = roslibpy.Message({
        "header": {"stamp": _ros_time(), "frame_id": "map"},
        "coordinate_frame": int(coordinate_frame),
        "type_mask": int(type_mask),
        "latitude": float(latitude),           # Decimal degrees
        "longitude": float(longitude),         # Decimal degrees
        "altitude": float(altitude),
        "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "acceleration_or_force": {"x": 0.0, "y": 0.0, "z": 0.0},
        "yaw": 0.0,
        "yaw_rate": 0.0,
    })
    self._setpoint_topic.publish(message)
```

**Usage in Server:**
```python
def _handle_goto(data):
    """Handle goto command."""
    bridge = _require_vehicle_bridge()
    lat = data['lat']
    lon = data['lon']
    alt = data.get('alt', 0)
    
    bridge.send_global_position_target(
        latitude=float(lat),
        longitude=float(lon),
        altitude=float(alt),
        coordinate_frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask=0b0000111111111000
    )
```

**Key Points:**
- ✅ Direct pass-through of decimal degrees
- ✅ Uses `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` for relative altitude missions
- ✅ Type mask controls which fields are used

---

## 8. Coordinate Precision & Standards

### 8.1 Decimal Degrees Precision
| Decimal Places | Precision | Use Case |
|---|---|---|
| 1 | ~11.1 km | Country/region level |
| 2 | ~1.1 km | City level |
| 3 | ~111 m | Large building |
| 4 | ~11.1 m | Small building |
| 5 | ~1.1 m | Tree level |
| 6 | ~0.11 m | Survey accuracy ✅ Used |
| 7 | ~0.011 m (1.1 cm) | Centimeter precision ✅ Used for logging |
| 8 | ~1.1 mm | Sub-centimeter |

**Current Implementation Uses:**
- **Position storage:** 6-7 decimal places (0.011m - 0.11m accuracy)
- **Logging precision:** 7 decimal places for debugging
- **Display format:** Typically 6-8 decimals in frontend

### 8.2 Reference Standards
```python
# Valid latitude range: -90 to +90 degrees
# Valid longitude range: -180 to +180 degrees

# Integer format (1e7):
# lat_degrees = lat_raw / 10000000
# lon_degrees = lon_raw / 10000000
# 
# Example: 130720581 / 1e7 = 13.0720581°N
```

---

## 9. Error Handling & Validation

### 9.1 Null Check Pattern
```python
if lat is not None and lon is not None:
    current_state.position = Position(lat=float(lat), lng=float(lon))
    schedule_fast_emit()
```

### 9.2 Safe Float Conversion
```python
def safe_float(value, default=0.0):
    """Safely convert to float, handling None and invalid values."""
    try:
        return float(value) if value is not None else default
    except (ValueError, TypeError):
        return default
```

### 9.3 Covariance Fallback
```python
# Prefer explicit HRMS/VRMS if available
if hrms_msg is not None:
    current_state.hrms = f"{float(hrms_msg):.3f}"

# Fallback: compute from covariance matrix
if isinstance(covariance, list) and len(covariance) >= 3:
    try:
        hrms = math.sqrt(abs(float(covariance[0])))
        vrms = math.sqrt(abs(float(covariance[2])))
        current_state.hrms = f"{hrms:.3f}"
        current_state.vrms = f"{vrms:.3f}"
    except Exception:
        current_state.hrms = current_state.hrms or '0.000'
```

---

## 10. Threading & Synchronization

### 10.1 Thread-Safe Updates
```python
with mavros_telem_lock:
    current_state.position = Position(lat=float(lat), lng=float(lon))
    current_state.last_update = time.time()

schedule_fast_emit()
```

**Lock Protects:**
- Position updates
- RTK status updates
- Accuracy metrics
- Timestamp records

### 10.2 Throttled Emission
```python
EMIT_MIN_INTERVAL = 0.05  # 50ms minimum between emissions

def schedule_fast_emit():
    """Emit at most EMIT_MIN_INTERVAL; schedule deferred emit if needed."""
    now = time.monotonic()
    elapsed = now - last_emit_monotonic
    if elapsed >= EMIT_MIN_INTERVAL:
        emit_rover_data_now(reason='realtime')
        return
    delay = EMIT_MIN_INTERVAL - elapsed
    # Schedule deferred emit to avoid flooding
```

---

## 11. Debugging Features

### 11.1 Logging Points
```python
# GPS Raw Handler (mavros_bridge.py:492)
print(f"[MAVROS_BRIDGE] GPS RAW: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}m, "
      f"fix={fix_type} ({rtk_status}), sats={satellites_visible}, eph={eph:.2f}m", flush=True)

# Server Telemetry Handler (server.py:1401)
print(f"[SERVER] Received navsat: lat={lat}, lon={lon}, alt={alt}", flush=True)

# Position Update (server.py:1407)
print(f"[SERVER] Updated current_state.position: lat={current_state.position.lat:.7f}, lng={current_state.position.lng:.7f}", flush=True)

# Emission Debug (server.py:1210)
print(f"[EMIT] Sending rover_data ({reason}): position={pos}", flush=True)
```

---

## 12. Summary & Key Points

### ✅ Strengths
1. **Correct Unit Conversion:** Properly converts 1e7 integer format to decimal degrees
2. **Bidirectional Accuracy:** Maintains coordinate integrity in both directions (MAVROS ↔ UI)
3. **Thread Safety:** Protected by locks during updates
4. **Comprehensive Logging:** 7 decimal place precision for debugging
5. **Multiple Data Sources:** Handles both GPS_RAW_INT and NavSatFix topics
6. **RTK Support:** Correct mapping of fix types and base linking detection
7. **Accuracy Metrics:** Tracks HRMS/VRMS with fallback from covariance
8. **Throttled Updates:** Prevents flooding with 50ms minimum emission interval

### ⚠️ Considerations
1. **MAVROS Field Naming:** Uses non-intuitive `x_lat`/`y_long` naming (standard MAVLink)
2. **Altitude Correction:** Assumes +92.2m correction applied at ROS level
3. **Precision Limits:** Decimal degrees have inherent floating-point limitations
4. **Covariance Fallback:** Secondary fallback may provide less accurate metrics

### 🔄 Data Flow Summary
```
Raw GPS (1e7 format)
    ↓ (÷ 1e7)
Decimal Degrees
    ↓
Position Dataclass
    ↓
get_rover_data() serialization
    ↓
WebSocket emit
    ↓
Frontend UI display/use
```

---

## 13. Integration Points

### Incoming Data Sources:
- `/mavros/gpsstatus/gps1/raw` - Primary GPS data
- `/mavros/global_position/global_corrected` - Altitude-corrected position

### Outgoing Data Channels:
- `rover_data` WebSocket emission
- `setpoint_raw/global` ROS topic (for GOTO commands)
- `/mavros/mission/push` (for mission upload with waypoints)

### Related Components:
- `gps_altitude_corrector.py` - Applies altitude correction
- `network_monitor.py` - Provides network telemetry alongside position
- Frontend UI - Displays position, heading, and RTK status

---

**Document Generated:** 2025-11-07  
**Last Updated:** Analysis of current production code
