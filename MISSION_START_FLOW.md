# Mission Start Flow: `/api/mission/start`

## Overview
When you call the `/api/mission/start` endpoint, here's what the controller node does:

---

## Flow Diagram

```
POST /api/mission/start (Flask API)
         ↓
_publish_controller_cmd_or_error()
         ↓
bridge.publish_mission_command({'command': 'start'})
         ↓
ROS2 Topic: /mission/command
         ↓
MissionControllerNode.command_callback()
         ↓
Processes 'start' command
         ↓
Mission begins execution
```

---

## Detailed Steps

### 1. **API Call** (`server.py`)
```python
@app.post("/api/mission/start")
def api_mission_start():
    ok, err = _publish_controller_cmd_or_error({'command': 'start'})
    if err:
        return err
    _record_mission_event('Mission controller started', status='MISSION_STARTED')
    return _http_success('Mission controller started')
```

### 2. **Publish Command** (`server.py`)
```python
def _publish_controller_cmd_or_error(cmd: dict):
    bridge = _require_vehicle_bridge()
    if not hasattr(bridge, 'publish_mission_command'):
        return None, _http_error("MAVROS bridge not available...", 500)
    
    bridge.publish_mission_command(cmd)  # Publishes to /mission/command topic
    return True, None
```

**Action**: Sends a JSON command `{'command': 'start'}` to the ROS2 topic `/mission/command`

### 3. **Mission Controller Node Receives Command** (`mission_controller_node.py`)

```python
def command_callback(self, msg: String) -> None:
    data = json.loads(msg.data)
    cmd = data.get('command')
    
    if cmd == 'start':
        if not self.waypoints:
            self.get_logger().error('start: no mission loaded')
            return
        
        self.mission_active = True
        self.mission_paused = False
        self.waiting_for_next = False
        
        # Create timer loop when mission starts
        if self._timer is None:
            self._timer = self.create_timer(0.1, self.mission_tick)
        
        self.get_logger().info('Mission started')
        self.publish_status('mission_started', 'Mission started')
```

**Actions**:
- ✅ Validates that waypoints are already loaded
- ✅ Sets `mission_active = True`
- ✅ Resets `mission_paused` and `waiting_for_next` flags
- ✅ Creates a **periodic timer** that calls `mission_tick()` every **0.1 seconds**
- ✅ Publishes status message: `'mission_started'`

### 4. **Mission Execution Loop** (`mission_controller_node.py`)

The `mission_tick()` runs every 0.1 seconds:

```python
def mission_tick(self) -> None:
    if not self.mission_active:
        return
    if self.mission_paused or self.waiting_for_next:
        return
    if self.current_wp_index >= len(self.waypoints):
        # Mission complete
        self.mission_active = False
        self.publish_status('mission_completed', 'All waypoints completed')
        return
    
    if self._exec_thread and self._exec_thread.is_alive():
        # Already executing a waypoint
        return
    
    # Start background thread for current waypoint
    self._exec_thread = threading.Thread(
        target=self.execute_waypoint_sequence, 
        daemon=True
    )
    self._exec_thread.start()
```

**Actions**:
- ✅ Checks mission is active and not paused
- ✅ Checks there are more waypoints to process
- ✅ Spawns a **background thread** to execute the current waypoint (if not already running)

### 5. **Waypoint Execution Sequence** (Per Waypoint)

The `execute_waypoint_sequence()` thread performs these steps for each waypoint:

```
1. Extract current waypoint coordinates (lat, lon)
2. Send MAV_CMD_NAV_WAYPOINT command via MAVROS
3. Monitor GPS distance until waypoint is reached (within acceptance_radius)
4. Set flight mode to HOLD
5. Delay (delay_before_spray from config)
6. Activate servo (PWM = servo_pwm_start)
7. Publish 'spray_start' status
8. Sleep for spray_duration seconds
9. Deactivate servo (PWM = servo_pwm_stop)
10. Publish 'spray_stop' status
11. Delay (delay_after_spray from config)
12. Set flight mode to GUIDED
13. Publish 'waypoint_completed' status
14. Increment to next waypoint
15. If manual mode: pause and wait for next_sequence command
```

**Navigation Details** (`goto_waypoint()`):
- Sends `MAV_CMD_NAV_WAYPOINT` (command=16) to MAVROS
- Monitors GPS position in a loop
- Calculates distance using Haversine formula
- Returns `True` when distance ≤ acceptance_radius (default 2.0m)
- Has GPS timeout protection (default 30 seconds)

---

## State Transitions

```
START
  ├─ mission_active = True
  ├─ mission_paused = False
  ├─ waiting_for_next = False
  └─ Create 0.1s timer → mission_tick()
       │
       ├─ Check mission_active && not mission_paused
       ├─ Spawn execute_waypoint_sequence() thread
       │
       ├─ [Auto Mode]
       │  └─ Immediately proceeds to next waypoint after completion
       │
       └─ [Manual Mode]
          └─ Sets waiting_for_next = True (waits for /api/mission/next)
```

---

## Key Configuration Parameters

From `mission_config` dictionary:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `servo_channel` | 10 | Which servo channel to activate |
| `servo_pwm_start` | 1500 | PWM value to activate spray |
| `servo_pwm_stop` | 1100 | PWM value to stop spray |
| `spray_duration` | 5.0 seconds | How long to spray per waypoint |
| `delay_before_spray` | 1.0 seconds | Wait time before activating spray |
| `delay_after_spray` | 1.0 seconds | Wait time after deactivating spray |
| `gps_timeout` | 30.0 seconds | Max wait for GPS fix |
| `auto_mode` | True | Auto (true) or Manual (false) waypoint advancement |

---

## Published Status Messages

The mission controller publishes status updates to `/mission/status`:

| Status | When Published | Purpose |
|--------|---|---|
| `mission_started` | After start command | Initial confirmation |
| `navigating` | Every 0.5s during waypoint navigation | Distance to waypoint |
| `waypoint_reached` | When distance ≤ acceptance_radius | Reached target location |
| `spray_start` | When servo activated | Spraying begins |
| `spray_stop` | When servo deactivated | Spraying ends |
| `waypoint_completed` | After spray cycle complete | Ready for next waypoint |
| `mission_paused` | When waiting for manual next_sequence | Waiting for user input |
| `mission_completed` | All waypoints processed | Mission done |
| `mission_stopped` | On stop command | Mission halted |

---

## Important Notes

- ⚠️ **Mission must be loaded first** (`/api/mission/load`) before calling start
- ⚠️ **GPS data is required** for navigation (monitored from MAVROS)
- ⚠️ **Waypoint execution is non-blocking** (runs in background thread)
- ⚠️ **Auto vs Manual Mode** affects how waypoints advance:
  - **Auto**: Next waypoint starts immediately after spray cycle
  - **Manual**: Waits for `/api/mission/next` API call to proceed
- ⚠️ **No retry on failure** - failed waypoints are skipped and mission continues
