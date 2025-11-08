# 🎯 How WP_MARK Servo Control Works - Simple Explanation

**WP_MARK** = **W**ay**P**oint **MARK**ing - Automated servo control at each waypoint during a mission.

---

## 📍 The 3-Step Process

### **Step 1: Rover Reaches Waypoint** 🚜
- Rover navigates autonomously to each waypoint in the mission
- When it gets within **2 meters** of the target waypoint, WP_MARK detects arrival
- GPS coordinates are compared: current position vs. waypoint position

```
Example:
Target Waypoint: 12.345678°N, 77.123456°E
Current Position: 12.345679°N, 77.123457°E
Distance: 1.5m ✅ (within 2m threshold)
→ Waypoint reached!
```

---

### **Step 2: Servo Activates (Spray ON)** 🚿
- **Wait** for configured delay (optional, default: 2 seconds)
- **Send PWM command** to servo (e.g., Servo 10 → 1500μs)
- This opens the spray nozzle or activates the mechanism
- **Spray duration**: Stays ON for configured time (e.g., 5 seconds)

```
Timeline at Waypoint:
├─ 0s:  Arrive at waypoint
├─ 2s:  Servo ON (PWM 1500) ← delay_before_start
├─ 7s:  Servo OFF (PWM 1000) ← after 5s spray duration
└─ 8s:  Ready for next waypoint ← delay_after_stop
```

---

### **Step 3: Move to Next Waypoint** ➡️
- **Servo turns OFF** (returns to PWM 1000 = closed position)
- **Brief pause** (optional, default: 1 second)
- Rover automatically navigates to the **next waypoint**
- Process repeats for each waypoint in the mission

```
Mission Flow:
WP 1 → [Arrive → Spray → Stop] → Navigate to WP 2
WP 2 → [Arrive → Spray → Stop] → Navigate to WP 3
WP 3 → [Arrive → Spray → Stop] → Navigate to WP 4
...
WP N → [Arrive → Spray → Stop] → Mission Complete ✅
```

---

## 🎮 Configuration Parameters

| Parameter | What It Does | Example |
|-----------|--------------|---------|
| **delay_before_start** | Wait time after arriving before spraying | 2.0 seconds |
| **pwm_start** | Servo PWM to turn ON (spray) | 1500 μs |
| **delay_before_stop** | How long to spray | 5.0 seconds |
| **pwm_stop** | Servo PWM to turn OFF (close) | 1000 μs |
| **delay_after_stop** | Wait time after stopping before moving | 1.0 second |

---

## 📊 Real-World Example

**Agricultural Spray Mission:**

```
Mission: 10 waypoints marking field perimeter
Servo Channel: 10 (spray nozzle)

At Each Waypoint:
1. Rover drives to GPS coordinate (auto-navigation)
2. Arrives within 2m → Trigger detected
3. Wait 2 seconds (settle)
4. Open spray valve (PWM 1500) for 5 seconds
5. Close spray valve (PWM 1000)
6. Wait 1 second
7. Drive to next waypoint

Result: Precise spray application at 10 GPS-marked locations
Total Time: ~15 minutes for complete mission
```

---

## 🔄 Behind the Scenes

```
┌────────────────────────────────────────────────────┐
│ MAVROS (Flight Controller)                         │
│ • GPS position tracking                            │
│ • Mission waypoint management                      │
│ • Servo PWM output to hardware                     │
└─────────────┬──────────────────────────────────────┘
              │ ROS2 Topics & Services
              ▼
┌────────────────────────────────────────────────────┐
│ WP_MARK Mission Controller (ROS2 Node)            │
│ • Monitors GPS position                            │
│ • Detects waypoint arrival (distance < 2m)        │
│ • Executes timed servo sequence                   │
│ • Manages mission state machine                   │
└─────────────┬──────────────────────────────────────┘
              │ Python API
              ▼
┌────────────────────────────────────────────────────┐
│ Flask REST API                                     │
│ • POST /wp_mark/start (configure & start)         │
│ • GET  /wp_mark/status (monitor progress)         │
│ • POST /wp_mark/stop (manual stop)                │
└────────────────────────────────────────────────────┘
              │ HTTP Requests
              ▼
┌────────────────────────────────────────────────────┐
│ Frontend UI                                        │
│ • Start/stop mission                              │
│ • Configure parameters                            │
│ • Monitor real-time progress                      │
└────────────────────────────────────────────────────┘
```

---

## ✅ Key Features

- ✅ **Fully Autonomous**: No manual intervention needed
- ✅ **GPS-Triggered**: Activates precisely at waypoints
- ✅ **Configurable Timing**: Adjust all delays and durations
- ✅ **Safety Checks**: Requires GPS fix, armed state, AUTO mode
- ✅ **Progress Tracking**: Real-time status updates
- ✅ **Error Recovery**: Handles GPS loss, timeouts gracefully

---

## 🚀 Quick Start

### Start WP_MARK Mission:
```bash
curl -X POST http://localhost:5001/wp_mark/start \
  -H "Content-Type: application/json" \
  -d '{
    "delay_before_start": 2.0,
    "pwm_start": 1500,
    "delay_before_stop": 5.0,
    "pwm_stop": 1000,
    "delay_after_stop": 1.0
  }'
```

### Check Status:
```bash
curl http://localhost:5001/wp_mark/status
```

### Stop Mission:
```bash
curl -X POST http://localhost:5001/wp_mark/stop
```

---

## 💡 That's It!

**WP_MARK in 3 Steps:**
1. 🚜 **Rover reaches waypoint** (GPS-based detection)
2. 🚿 **Servo activates** (timed spray sequence)
3. ➡️ **Move to next waypoint** (repeat for all waypoints)

Simple, precise, and fully automated! 🎯
