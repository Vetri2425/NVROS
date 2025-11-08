# WP_MARK Robust Waypoint Marking Script

## 🎯 Overview

The **wpmark.py** script provides a robust, step-by-step waypoint marking system that:
1. Reads waypoints from the current mission
2. Navigates to each waypoint sequentially using GUIDED mode
3. Switches to HOLD mode at each waypoint
4. Activates servo (spray) with configurable timing
5. Deactivates servo and moves to next waypoint
6. Repeats until all waypoints are processed

---

## 📋 How It Works (Detailed Workflow)

### **Step 1: Load Mission Waypoints**
```
1. Connect to MAVROS
2. Pull waypoints from /mavros/mission/waypoints
3. Filter only navigation waypoints (command 16)
4. Validate mission has valid waypoints
```

### **Step 2-6: Process Each Waypoint**

For each waypoint in the mission:

#### **Step 2: Switch to GUIDED Mode**
```
- Set flight mode to GUIDED
- Vehicle takes GPS control
- Ready for waypoint navigation
```

#### **Step 3: Navigate to Waypoint**
```
- Set current waypoint using set_current_waypoint()
- Vehicle autonomously navigates
- Continuously monitor GPS position
```

#### **Step 4: Wait for Arrival**
```
- Calculate distance from target using Haversine formula
- Check if distance < waypoint_threshold (default: 2m)
- Timeout if not reached within navigation_timeout (default: 120s)
```

#### **Step 5: Switch to HOLD Mode**
```
- On arrival, switch mode to HOLD
- Vehicle stops and maintains position
- Stable platform for spraying
```

#### **Step 6: Spray Sequence**
```
Timeline at waypoint:
├─ T+0s:  Arrived (HOLD mode active)
├─ T+2s:  Servo ON (PWM 1500) ← delay_before_spray
├─ T+7s:  Servo OFF (PWM 1000) ← after spray_duration (5s)
└─ T+8s:  Ready for next WP ← delay_after_spray (1s)
```

#### **Step 7: Return to GUIDED & Next Waypoint**
```
- Switch back to GUIDED mode
- Move to next waypoint
- Repeat process
```

---

## 🔧 Configuration

Edit `/home/flash/NRP_ROS/Backend/servo_manager/config.json`:

```json
{
  "wpmark": {
    "servo_number": 10,           // Servo channel (1-16)
    "pwm_on": 1500,               // Spray ON PWM (μs)
    "pwm_off": 1000,              // Spray OFF PWM (μs)
    "delay_before_spray": 2.0,    // Wait after arrival (seconds)
    "spray_duration": 5.0,        // How long to spray (seconds)
    "delay_after_spray": 1.0,     // Wait before next WP (seconds)
    "waypoint_threshold": 2.0,    // Arrival distance (meters)
    "navigation_timeout": 120.0   // Max time to reach WP (seconds)
  }
}
```

### **Parameter Details:**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `servo_number` | int | 10 | 1-16 | Servo channel to control |
| `pwm_on` | int | 1500 | 1000-2000 | PWM for spray ON (μs) |
| `pwm_off` | int | 1000 | 1000-2000 | PWM for spray OFF (μs) |
| `delay_before_spray` | float | 2.0 | 0-60 | Wait after reaching waypoint |
| `spray_duration` | float | 5.0 | 0-60 | How long to spray |
| `delay_after_spray` | float | 1.0 | 0-60 | Wait before next waypoint |
| `waypoint_threshold` | float | 2.0 | 0.5-10 | Distance to consider arrived (m) |
| `navigation_timeout` | float | 120.0 | 10-600 | Max navigation time per WP (s) |

---

## 🚀 Usage

### **Start from UI:**

1. **Upload Mission** with waypoints to ArduPilot
2. **Arm** the vehicle
3. **Select "wpmark" mode** in Servo Control tab
4. **Configure parameters** in UI
5. **Click "Run"** to start

### **Start from Backend API:**

```bash
# Start wpmark mode
curl "http://localhost:5001/servo/run?mode=wpmark"

# Check status
curl "http://localhost:5001/servo/status"

# Stop if needed
curl "http://localhost:5001/servo/stop?mode=wpmark"
```

### **Start from Terminal (Testing):**

```bash
cd /home/flash/NRP_ROS/Backend/servo_manager
python3 wpmark.py
```

---

## 📊 Mission Flow Diagram

```
┌─────────────────────────────────────────────────┐
│ 1. LOAD WAYPOINTS                               │
│    - Connect MAVROS                             │
│    - Pull mission waypoints                     │
│    - Validate waypoints                         │
└─────────────┬───────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────┐
│ 2. FOR EACH WAYPOINT (WP0 → WPN)               │
└─────────────┬───────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────┐
│ 3. SWITCH TO GUIDED MODE                        │
│    - Set flight mode: GUIDED                    │
│    - Vehicle ready for GPS navigation           │
└─────────────┬───────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────┐
│ 4. NAVIGATE TO WAYPOINT                         │
│    - Set current WP: set_current_waypoint(i)    │
│    - Monitor GPS position                       │
│    - Calculate distance to target               │
└─────────────┬───────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────┐
│ 5. WAIT FOR ARRIVAL                             │
│    - Loop: Check distance < threshold (2m)      │
│    - Timeout if > navigation_timeout (120s)     │
│    - Log distance every 10 checks               │
└─────────────┬───────────────────────────────────┘
              │ ARRIVED
              ▼
┌─────────────────────────────────────────────────┐
│ 6. SWITCH TO HOLD MODE                          │
│    - Set flight mode: HOLD                      │
│    - Vehicle maintains position                 │
└─────────────┬───────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────┐
│ 7. SPRAY SEQUENCE                               │
│    T+0s : Wait delay_before_spray (2s)          │
│    T+2s : Servo ON (PWM 1500)                   │
│    T+7s : Wait spray_duration (5s)              │
│    T+7s : Servo OFF (PWM 1000)                  │
│    T+8s : Wait delay_after_spray (1s)           │
└─────────────┬───────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────┐
│ 8. NEXT WAYPOINT?                               │
│    YES → Go to step 3 with next WP              │
│    NO  → Mission Complete                       │
└─────────────────────────────────────────────────┘
```

---

## 🔍 Features

### **✅ Robust Error Handling**
- Connection failures → Retry logic
- Navigation timeouts → Continue to next WP
- Servo command failures → Log warning, continue
- User interruption → Graceful shutdown

### **✅ Thread-Safe Position Tracking**
- Real-time GPS monitoring via MAVROS telemetry
- Thread-safe position updates
- Accurate distance calculations (Haversine formula)

### **✅ Comprehensive Logging**
- Console output for real-time monitoring
- File logging: `/home/flash/NRP_ROS/Backend/servo_manager/logs/wpmark_<timestamp>.log`
- Detailed progress tracking
- Mission statistics on completion

### **✅ Safety Features**
- Automatic servo OFF on shutdown
- Mode change to HOLD on exit
- SIGTERM/SIGINT handling
- Cleanup on errors

---

## 📝 Example Mission

### **Scenario: 5-Point Field Perimeter Spray**

**Mission:** 5 waypoints around field perimeter

```
WP0: 12.345678°N, 77.123456°E
WP1: 12.345698°N, 77.123476°E
WP2: 12.345718°N, 77.123496°E
WP3: 12.345738°N, 77.123516°E
WP4: 12.345758°N, 77.123536°E
```

**Execution:**
```
[2025-11-06 14:30:00] Connecting to MAVROS...
[2025-11-06 14:30:01] MAVROS connected successfully
[2025-11-06 14:30:02] Loaded 5 waypoints from mission
[2025-11-06 14:30:03] === Processing Waypoint 1/5 ===
[2025-11-06 14:30:04] Setting mode to GUIDED...
[2025-11-06 14:30:05] Navigating to WP0...
[2025-11-06 14:30:10] Distance to WP0: 15.2m
[2025-11-06 14:30:15] Distance to WP0: 8.5m
[2025-11-06 14:30:20] Distance to WP0: 1.8m
[2025-11-06 14:30:21] Arrived at WP0 (distance=1.8m)
[2025-11-06 14:30:22] Setting mode to HOLD...
[2025-11-06 14:30:24] Setting servo 10 to PWM 1500...
[2025-11-06 14:30:24] Spraying for 5.0s...
[2025-11-06 14:30:29] Setting servo 10 to PWM 1000...
[2025-11-06 14:30:30] === Waypoint 1 completed (1/5) ===
...
[2025-11-06 14:35:00] === Mission Completed ===
[2025-11-06 14:35:00] Total waypoints: 5
[2025-11-06 14:35:00] Completed: 5
[2025-11-06 14:35:00] Failed: 0
[2025-11-06 14:35:00] Duration: 300.5s
```

---

## 🐛 Troubleshooting

### **Problem: "No waypoints received from mission"**
**Solution:**
```bash
# Upload waypoints first
# Via Mission Planner or UI
# Verify waypoints are loaded
curl http://localhost:5001/api/mission/download
```

### **Problem: "Navigation timeout"**
**Solution:**
- Increase `navigation_timeout` in config
- Check GPS fix quality
- Verify vehicle is in correct mode
- Check for obstacles

### **Problem: "Failed to connect to MAVROS"**
**Solution:**
```bash
# Check rosbridge is running
ps aux | grep rosbridge
# Restart rosbridge if needed
roslaunch rosbridge_server rosbridge_websocket.launch
```

### **Problem: "Servo command failed"**
**Solution:**
- Check servo channel (1-16)
- Verify PWM range (1000-2000)
- Check MAVROS connection
- Test manual servo command

---

## 📊 Comparison: Old vs New WP_MARK

| Feature | Old WP_MARK | **New wpmark.py** |
|---------|-------------|-------------------|
| **Waypoint Loading** | Manual config | ✅ Reads from mission |
| **Navigation** | Waypoint reached events | ✅ Active GUIDED navigation |
| **Mode Control** | No mode changes | ✅ GUIDED → HOLD → GUIDED |
| **Arrival Detection** | Event-based | ✅ GPS distance calculation |
| **Error Handling** | Basic | ✅ Comprehensive with retry |
| **Logging** | Console only | ✅ File + Console |
| **Progress Tracking** | None | ✅ Real-time statistics |
| **Timeout Handling** | None | ✅ Configurable timeouts |
| **Thread Safety** | No | ✅ Thread-safe GPS tracking |

---

## ✅ Success Criteria

Before each waypoint:
- ✅ GUIDED mode set successfully
- ✅ Waypoint set as current
- ✅ GPS position valid
- ✅ Distance calculated accurately

At each waypoint:
- ✅ Distance < threshold (default 2m)
- ✅ HOLD mode activated
- ✅ Servo ON command successful
- ✅ Spray duration completed
- ✅ Servo OFF command successful

Mission completion:
- ✅ All waypoints processed
- ✅ Statistics logged
- ✅ Servo in OFF state
- ✅ Mode set to HOLD

---

## 🎓 Key Improvements

1. **Active Navigation**: Uses GUIDED mode + set_current_waypoint() instead of passive event listening
2. **Mode Management**: Explicit HOLD mode at waypoints for stable spraying
3. **Distance-Based**: GPS distance calculation vs. event-based (more reliable)
4. **Mission Integration**: Reads waypoints directly from mission (no manual config)
5. **Comprehensive Logging**: Full mission statistics and detailed logs
6. **Error Recovery**: Continues on failures instead of stopping
7. **Thread Safety**: Proper locking for position updates
8. **Timeout Protection**: Prevents infinite waiting

---

**The new wpmark.py script is production-ready and provides robust, reliable waypoint marking!** 🎉
