# Mission Config API - Quick Reference

## Port
**5001**

## Endpoints

### GET - Read Config
```bash
curl http://localhost:5001/api/mission/config
```

### POST - Update Config & Send to Mission Controller
```bash
curl -X POST http://localhost:5001/api/mission/config \
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

## Full Test Workflow

### Terminal 1: Monitor Topic
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mission/command
```

### Terminal 2: Load Mission & Update Config
```bash
# Step 1: Load mission
curl -X POST http://localhost:5001/api/mission/load_controller \
  -H "Content-Type: application/json" \
  -d '{"waypoints": [{"lat": 13.071922, "lng": 80.2619957}]}'

# Step 2: Get current config
curl http://localhost:5001/api/mission/config | jq

# Step 3: Update config (sends command to mission controller)
curl -X POST http://localhost:5001/api/mission/config \
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

# Terminal 1 will show the published command!
```

## What Gets Sent to /mission/command Topic

```json
{
  "command": "load_mission",
  "waypoints": [
    {"lat": 13.071922, "lng": 80.2619957}
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

## Files Modified
- `Backend/server.py` - Added POST endpoint for `/api/mission/config`
- `Backend/server.py` - Updated `/api/config/sprayer` to send commands

## Test Script
```bash
./test_mission_config_api.sh
```
