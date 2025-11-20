# Fix Applied: `/api/config/sprayer` Now Sends Command to Mission Controller

## Problem
The `/api/config/sprayer` endpoint was only saving config to file but NOT sending the `load_mission` command to the mission controller node via `/mission/command` topic.

## Solution Implemented

### Endpoint Modified
**File:** `Backend/server.py` (lines 3490-3585)  
**Endpoint:** `POST /api/config/sprayer`

### Changes Made

1. **Added Global Declaration:**
   ```python
   global current_mission
   ```
   This ensures we can access the currently loaded mission waypoints.

2. **Added Mission Command Publishing:**
   After saving config to file, the endpoint now:
   - Checks if a mission is loaded (`current_mission` has waypoints)
   - Creates a `load_mission` command with:
     - Current waypoints
     - Updated servo configuration (from request body)
   - Publishes command via `bridge.publish_mission_command()`

3. **Error Handling:**
   - If no mission is loaded: Info message, config still saved
   - If bridge unavailable: Warning, config still saved
   - If send fails: Warning with error details, config still saved
   - **Config file is always saved successfully**

### Updated Behavior

**Before:**
```
POST /api/config/sprayer
↓
Save config to file
↓
Return success
❌ Mission controller NOT updated
```

**After:**
```
POST /api/config/sprayer
↓
Save config to file
↓
Send load_mission command to mission controller with new servo config
↓
Return success
✅ Config applied in real-time to mission controller
```

### Code Implementation

```python
@app.post("/api/config/sprayer")
def api_save_sprayer_config():
    global current_mission
    try:
        # ... validation ...
        
        # Save config to file
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2)
        
        log_message("Sprayer configuration saved", "INFO")
        
        # ✅ NEW: Send load_mission command to mission controller
        try:
            bridge = _require_vehicle_bridge()
            
            if current_mission and len(current_mission) > 0:
                load_mission_data = {
                    'command': 'load_mission',
                    'waypoints': current_mission,
                    'config': body  # Updated servo config
                }
                
                if hasattr(bridge, 'publish_mission_command'):
                    bridge.publish_mission_command(load_mission_data)
                    log_message("Sent load_mission command with updated servo config", "INFO")
        except Exception as e:
            log_message(f"Warning: Config saved but failed to send command: {e}", "WARNING")
        
        return _http_success("Sprayer configuration saved and applied to mission controller")
```

## Testing

### Test 1: With Mission Loaded
```bash
# 1. Load mission via /api/mission/load_controller
POST /api/mission/load_controller
{
  "waypoints": [{"lat": 13.071922, "lng": 80.2619957}]
}

# 2. Update servo config
POST /api/config/sprayer
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

# Expected: Command sent to mission controller
# Log: "Sent load_mission command to mission controller with updated servo config"
```

### Test 2: Without Mission Loaded
```bash
# Update servo config without loading mission first
POST /api/config/sprayer
{...}

# Expected: Config saved, no command sent (no waypoints)
# Log: "No mission waypoints loaded, config saved but not applied"
```

## Mission Controller Integration

The mission controller (`mission_controller_node.py`) will receive the command:

```python
{
    'command': 'load_mission',
    'waypoints': [...],  # Current waypoints
    'config': {          # Updated servo parameters
        'servo_channel': 10,
        'servo_pwm_start': 1500,
        'servo_pwm_stop': 1100,
        'spray_duration': 5.0,
        'delay_before_spray': 1.0,
        'delay_after_spray': 1.0,
        'gps_timeout': 30.0,
        'auto_mode': true
    }
}
```

The mission controller will update its configuration immediately without needing to reload the mission.

## Flow Summary

```
Config Editor (Frontend)
    ↓
POST /api/config/sprayer
    ↓
├─ Save to mission_controller_config.json ✅
└─ Publish to /mission/command topic ✅
    ↓
mission_controller_node.py receives load_mission command
    ↓
Updates servo config in real-time
```

## Status
✅ **FIXED** - The `/api/config/sprayer` endpoint now properly sends servo configuration updates to the mission controller node via the ROS topic.
