# Mission Status Report Format

## ROS Topic: `/mission/status`

**Message Type**: `std_msgs/String` (JSON payload)

---

## JSON Payload Format

```json
{
  "timestamp": "2025-11-11T14:30:45.123456Z",
  "event_type": "navigating",
  "message": "Distance to waypoint: 45.32 m",
  "current_waypoint": 2,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {
    "lat": 37.4419,
    "lon": -122.1430
  },
  "distance_m": 45.32
}
```

---

## Field Descriptions

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| `timestamp` | string (ISO 8601) | UTC timestamp of status event | `2025-11-11T14:30:45.123456Z` |
| `event_type` | string | Event classification | `"navigating"`, `"spray_start"`, etc. |
| `message` | string | Human-readable description | `"Distance to waypoint: 45.32 m"` |
| `current_waypoint` | integer | Index of current waypoint (0-based) | `2` (3rd waypoint) |
| `total_waypoints` | integer | Total waypoints in mission | `5` |
| `mission_active` | boolean | Mission execution active | `true` or `false` |
| `mission_paused` | boolean | Mission paused state | `true` or `false` |
| `auto_mode` | boolean | Auto (true) or manual (false) waypoint advance | `true` |
| `gps` | object | Current GPS position | `{"lat": 37.4419, "lon": -122.1430}` |
| `gps.lat` | float | Current latitude (degrees) | `37.4419` |
| `gps.lon` | float | Current longitude (degrees) | `-122.1430` |
| **extra_data** | object | Optional additional fields (dynamic) | See event types below |

---

## Event Types and Payload Examples

### 1. **mission_loaded**
When mission is loaded with waypoints.

```json
{
  "timestamp": "2025-11-11T14:25:10.000000Z",
  "event_type": "mission_loaded",
  "message": "Mission loaded with 5 waypoints",
  "current_waypoint": 0,
  "total_waypoints": 5,
  "mission_active": false,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": null, "lon": null}
}
```

---

### 2. **mission_started**
When mission start command is received.

```json
{
  "timestamp": "2025-11-11T14:26:00.000000Z",
  "event_type": "mission_started",
  "message": "Mission started",
  "current_waypoint": 0,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4419, "lon": -122.1430}
}
```

---

### 3. **navigating** ⭐ (Periodic, every 0.5s)
During waypoint navigation.

```json
{
  "timestamp": "2025-11-11T14:26:15.250000Z",
  "event_type": "navigating",
  "message": "Distance to waypoint: 45.32 m",
  "current_waypoint": 0,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4420, "lon": -122.1431},
  "distance_m": 45.32
}
```

---

### 4. **waypoint_reached**
When distance to waypoint ≤ 2m (acceptance radius).

```json
{
  "timestamp": "2025-11-11T14:26:45.000000Z",
  "event_type": "waypoint_reached",
  "message": "Waypoint reached",
  "current_waypoint": 0,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

### 5. **spray_start**
When servo activated for spraying.

```json
{
  "timestamp": "2025-11-11T14:26:46.000000Z",
  "event_type": "spray_start",
  "message": "Spray started",
  "current_waypoint": 0,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

### 6. **spray_stop**
When servo deactivated after spraying.

```json
{
  "timestamp": "2025-11-11T14:26:51.000000Z",
  "event_type": "spray_stop",
  "message": "Spray stopped",
  "current_waypoint": 0,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

### 7. **waypoint_completed**
After full waypoint execution sequence.

```json
{
  "timestamp": "2025-11-11T14:26:55.000000Z",
  "event_type": "waypoint_completed",
  "message": "Waypoint 1 completed",
  "current_waypoint": 1,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

### 8. **mission_paused**
When mission paused due to failure or manual pause.

```json
{
  "timestamp": "2025-11-11T14:27:00.000000Z",
  "event_type": "mission_paused",
  "message": "Mission paused",
  "current_waypoint": 1,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": true,
  "auto_mode": true,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

### 9. **mission_paused** (Manual mode waiting)
After waypoint when using manual mode.

```json
{
  "timestamp": "2025-11-11T14:26:56.000000Z",
  "event_type": "mission_paused",
  "message": "Waiting for manual next_sequence",
  "current_waypoint": 1,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": true,
  "auto_mode": false,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

### 10. **mission_resumed**
When mission resumed from pause.

```json
{
  "timestamp": "2025-11-11T14:28:00.000000Z",
  "event_type": "mission_resumed",
  "message": "Mission resumed",
  "current_waypoint": 1,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

### 11. **mission_stopped**
When mission stopped via stop command.

```json
{
  "timestamp": "2025-11-11T14:29:00.000000Z",
  "event_type": "mission_stopped",
  "message": "Mission stopped and HOLD set",
  "current_waypoint": 0,
  "total_waypoints": 5,
  "mission_active": false,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

### 12. **mission_restarted**
When mission restarted to first waypoint.

```json
{
  "timestamp": "2025-11-11T14:29:10.000000Z",
  "event_type": "mission_restarted",
  "message": "Restarted to first waypoint",
  "current_waypoint": 0,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4419, "lon": -122.1430}
}
```

---

### 13. **mission_completed**
When all waypoints processed.

```json
{
  "timestamp": "2025-11-11T14:35:00.000000Z",
  "event_type": "mission_completed",
  "message": "All waypoints completed",
  "current_waypoint": 5,
  "total_waypoints": 5,
  "mission_active": false,
  "mission_paused": false,
  "auto_mode": true,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

### 14. **manual_next**
When manual next waypoint triggered.

```json
{
  "timestamp": "2025-11-11T14:26:58.000000Z",
  "event_type": "manual_next",
  "message": "Manual next sequence triggered",
  "current_waypoint": 1,
  "total_waypoints": 5,
  "mission_active": true,
  "mission_paused": false,
  "auto_mode": false,
  "gps": {"lat": 37.4425, "lon": -122.1435}
}
```

---

## Frontend Integration

### Socket.IO Event
The backend forwards all mission status as a Socket.IO event:

```javascript
// Frontend listens for:
socket.on('mission_status', (statusReport) => {
  console.log(statusReport.event_type);
  console.log(statusReport.message);
  console.log(statusReport.distance_m);
  // Update UI with status
});
```

---

## Publishing Flow

```
Mission Controller Node
      ↓
publish_status() method
      ↓
JSON serialization
      ↓
/mission/status topic (ROS)
      ↓
MAVROS Bridge listener
      ↓
mission_status_callback()
      ↓
Server handler (_handle_mission_status_from_bridge)
      ↓
Socket.IO emit('mission_status', data)
      ↓
Frontend WebSocket listener
      ↓
UI Update
```

---

## API Endpoints to Consume Status

### Option 1: Real-Time (Recommended)
Listen to Socket.IO events:
```javascript
socket.on('mission_status', handleStatus);
```

### Option 2: Query Current Status
Call REST API:
```bash
GET /api/mission/status  # Returns latest status
```

---

## Status Report Summary Table

| Event Type | When Triggered | Key Fields | Frequency |
|-----------|---|---|---|
| `mission_loaded` | Mission file uploaded | waypoints count | Once |
| `mission_started` | `/api/mission/start` called | mission_active=true | Once |
| `navigating` | During waypoint flight | distance_m | Every 0.5s |
| `waypoint_reached` | GPS within 2m radius | current_waypoint | Once per WP |
| `spray_start` | Servo activated (PWM 1500) | - | Once per WP |
| `spray_stop` | Servo deactivated (PWM 1100) | - | Once per WP |
| `waypoint_completed` | After spray sequence | current_waypoint+1 | Once per WP |
| `mission_paused` | Pause or failure | mission_paused=true | On event |
| `mission_resumed` | Resume from pause | mission_paused=false | On event |
| `mission_stopped` | `/api/mission/stop` called | mission_active=false | Once |
| `mission_completed` | All waypoints done | current_waypoint=total | Once |

---

## Error Handling

If GPS data unavailable at status time:
```json
"gps": {
  "lat": null,
  "lon": null
}
```

If extra_data is not provided:
- No additional fields beyond base payload

If mission not loaded:
```json
{
  "total_waypoints": 0,
  "current_waypoint": 0
}
```

---

## Notes

- ⚠️ **Timestamp format**: Always UTC with 'Z' suffix (ISO 8601)
- ⚠️ **Distance updates**: Only in `navigating` event
- ⚠️ **GPS can be null**: During initial startup before first GPS fix
- ⚠️ **Periodic events**: `navigating` publishes every 0.5 seconds (high frequency)
- ✅ **Thread-safe**: All field access protected by locks
