# Mission Config API Implementation Complete

## Overview
The `/api/mission/config` endpoint now supports both GET and POST operations to read and apply mission servo configuration to the mission controller node.

**Port:** 5001

## Endpoints

### 1. GET `/api/mission/config`
**Purpose:** Retrieve current mission controller configuration

**URL:** `http://localhost:5001/api/mission/config`

**Method:** GET

**Response:**
```json
{
  "success": true,
  "data": {
    "sprayer_parameters": {
      "servo_channel": 10,
      "servo_pwm_start": 1500,
      "servo_pwm_stop": 1100,
      "spray_duration": 5.0,
      "delay_before_spray": 1.0,
      "delay_after_spray": 1.0,
      "gps_timeout": 30.0,
      "auto_mode": true
    },
    "mission_parameters": { ... },
    "safety_parameters": { ... }
  }
}
```

**Example:**
```bash
curl -X GET "http://localhost:5001/api/mission/config"
```

---

### 2. POST `/api/mission/config`
**Purpose:** Update mission controller servo configuration AND send command to mission controller node

**URL:** `http://localhost:5001/api/mission/config`

**Method:** POST

**Content-Type:** `application/json`

**Request Body:**
```json
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
```

**Required Parameters:**
- `servo_channel` (integer)
- `servo_pwm_start` (integer)
- `servo_pwm_stop` (integer)
- `spray_duration` (float)
- `delay_before_spray` (float)
- `delay_after_spray` (float)
- `gps_timeout` (float)
- `auto_mode` (boolean)

**Response:**
```json
{
  "success": true,
  "message": "Mission config updated and applied to mission controller"
}
```

**Example:**
```bash
curl -X POST "http://localhost:5001/api/mission/config" \
  -H "Content-Type: application/json" \
  -d '{
    "servo_channel": 10,
    "servo_pwm_start": 1500,
    "servo_pwm_stop": 1100,
    "spray_duration": 5.0,
    "delay_before_spray": 1.0,
    "delay_after_spray": 1.0,
    "gps_timeout": 30.0,
    "auto_mode": true
  }'
```

---

## What Happens Behind the Scenes

### Flow Diagram
```
User/Frontend
    ↓
POST /api/mission/config
    ↓
Backend Server (server.py)
    ├─ Validate parameters
    ├─ Save to mission_controller_config.json
    ├─ Get current_mission (waypoints)
    └─ Publish to /mission/command topic
         ↓
    mission_controller_node.py
    (receives load_mission command with new config)
         ↓
    Updates servo parameters in memory
    Ready to execute mission with new settings
```

### Command Published to `/mission/command` Topic

When a mission is loaded and config is updated, the server publishes:

```json
{
  "command": "load_mission",
  "waypoints": [
    {"lat": 13.071922, "lng": 80.2619957},
    {"lat": 13.072000, "lng": 80.2620000}
  ],
  "config": {
    "servo_channel": 10,
    "servo_pwm_start": 1500,
    "servo_pwm_stop": 1100,
    "spray_duration": 5.0,
    "delay_before_spray": 1.0,
    "delay_after_spray": 1.0,
    "gps_timeout": 30.0,
    "auto_mode": true
  }
}
```

---

## Usage Workflow

### Step 1: Load Mission
```bash
curl -X POST "http://localhost:5001/api/mission/load_controller" \
  -H "Content-Type: application/json" \
  -d '{"waypoints": [{"lat": 13.071922, "lng": 80.2619957}]}'
```

### Step 2: Get Current Config
```bash
curl -X GET "http://localhost:5001/api/mission/config"
```

### Step 3: Update Config (sends command to mission controller)
```bash
curl -X POST "http://localhost:5001/api/mission/config" \
  -H "Content-Type: application/json" \
  -d '{
    "servo_channel": 10,
    "servo_pwm_start": 1500,
    "servo_pwm_stop": 1100,
    "spray_duration": 5.0,
    "delay_before_spray": 1.0,
    "delay_after_spray": 1.0,
    "gps_timeout": 30.0,
    "auto_mode": true
  }'
```

### Step 4: Monitor Mission Controller
In another terminal, watch the commands being received:
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mission/command
```

---

## Error Handling

### Missing Required Parameters
```json
{
  "success": false,
  "error": "Missing required parameter: servo_channel",
  "type": "BadRequest"
}
```

### Mission Not Loaded
Response will still succeed, but server will log:
```
"No mission waypoints loaded, config saved but not applied to mission controller"
```

### Bridge Not Available
Response will still succeed, but server will log:
```
"Warning: publish_mission_command not available, config saved but not applied"
```

---

## Architecture

### Files Modified
- **Backend/server.py**
  - Added `POST /api/mission/config` endpoint (lines 3275-3355)
  - Modified `/api/config/sprayer` to send commands (lines 3491-3585)

### Mission Controller Integration
- **Backend/mission_controller_node.py**
  - Listens to `/mission/command` topic
  - Handles `load_mission` command
  - Updates `mission_config` with servo parameters

### Global State
- `current_mission`: Stores loaded waypoints
- Maintained across API calls
- Available to all endpoints

---

## Testing

### Run Test Script
```bash
chmod +x test_mission_config_api.sh
./test_mission_config_api.sh
```

### Manual Testing
```bash
# Terminal 1: Monitor mission controller commands
source /opt/ros/humble/setup.bash
ros2 topic echo /mission/command

# Terminal 2: Send API requests
# 1. Load mission
curl -X POST "http://localhost:5001/api/mission/load_controller" \
  -H "Content-Type: application/json" \
  -d '{"waypoints": [{"lat": 13.071922, "lng": 80.2619957}]}'

# 2. Update config (command will appear in Terminal 1)
curl -X POST "http://localhost:5001/api/mission/config" \
  -H "Content-Type: application/json" \
  -d '{
    "servo_channel": 10,
    "servo_pwm_start": 1500,
    "servo_pwm_stop": 1100,
    "spray_duration": 5.0,
    "delay_before_spray": 1.0,
    "delay_after_spray": 1.0,
    "gps_timeout": 30.0,
    "auto_mode": true
  }'
```

---

## Related Endpoints

### POST `/api/config/sprayer`
Also sends servo config to mission controller. Both this and `/api/mission/config` POST endpoint work the same way.

### POST `/api/mission/load_controller`
Loads waypoints to mission controller. Should be called before updating config.

### POST `/api/mission/start_controller`
Starts mission execution after loading and configuring.

---

## Status
✅ **COMPLETE** - Both endpoints now properly send servo configuration updates to the mission controller node.

Port: **5001**
