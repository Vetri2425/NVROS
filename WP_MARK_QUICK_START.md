# WP_MARK Quick Start Guide

## 🚀 Quick Setup

### Start the Backend
```bash
cd /home/flash/NRP_ROS/Backend
python3 server.py
```

### Verify Installation
```bash
curl http://localhost:5001/wp_mark/health
```

---

## 📡 API Quick Reference

### Start Mission
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

### Get Status
```bash
curl http://localhost:5001/wp_mark/status
```

### Stop Mission
```bash
curl -X POST http://localhost:5001/wp_mark/stop
```

---

## 📊 Parameter Reference

| Parameter | Type | Range | Unit | Description |
|-----------|------|-------|------|-------------|
| `delay_before_start` | float | 0-60 | seconds | Wait time after reaching waypoint |
| `pwm_start` | int | 1000-2000 | μs | Servo ON value (spray) |
| `delay_before_stop` | float | 0-60 | seconds | Spray duration |
| `pwm_stop` | int | 1000-2000 | μs | Servo OFF value |
| `delay_after_stop` | float | 0-60 | seconds | Wait time before next waypoint |

---

## 🔍 Mission Phases

- `idle` - No mission
- `initializing` - Starting
- `navigating` - Moving to waypoint
- `waiting_arrival` - Waiting to reach waypoint
- `delay_before_start` - Pre-spray delay
- `spraying` - Servo active
- `delay_after_stop` - Post-spray delay
- `completed` - Finished
- `error` - Failed

---

## 🛡️ Safety Requirements

✅ GPS: 3D fix required  
✅ Mode: AUTO or GUIDED  
✅ State: Armed  
✅ Waypoints: Must be uploaded  

---

## 📂 Files

```
Backend/
├── server.py                    # Main server (blueprint registered)
├── config/
│   └── wp_mark_config.json     # Configuration
├── logs/
│   ├── wp_mark_*.log           # Mission logs
│   └── mission_log.json        # Event log
└── servo_manager/wp_mark/
    ├── __init__.py             # Module init
    ├── validators.py           # Validation
    ├── utils.py                # Utilities
    ├── mission_controller.py   # ROS2 node
    └── api_routes.py           # API routes
```

---

## 🧪 Quick Test

```bash
# 1. Check health
curl http://localhost:5001/wp_mark/health

# 2. Upload waypoints to flight controller (via Mission Planner/QGC)

# 3. Start mission
curl -X POST http://localhost:5001/wp_mark/start \
  -H "Content-Type: application/json" \
  -d '{"delay_before_start": 2, "pwm_start": 1500, "delay_before_stop": 5, "pwm_stop": 1000, "delay_after_stop": 1}'

# 4. Monitor status
watch -n 2 curl -s http://localhost:5001/wp_mark/status

# 5. Stop if needed
curl -X POST http://localhost:5001/wp_mark/stop
```

---

## 📞 Status Response Example

```json
{
  "running": true,
  "current_waypoint": 3,
  "total_waypoints": 5,
  "current_phase": "spraying",
  "config": { ... },
  "uptime_seconds": 87.3,
  "last_action": "Spraying at WP 3"
}
```

---

## ⚠️ Troubleshooting

**Problem**: `ros2_initialized: false`  
**Solution**: Start ROS2 and MAVROS before backend

**Problem**: `No waypoints loaded`  
**Solution**: Upload mission via Mission Planner/QGC

**Problem**: `Insufficient GPS fix`  
**Solution**: Wait for 3D GPS lock

**Problem**: `Mission already running`  
**Solution**: Stop current mission first

---

## 📚 Full Documentation

See `WP_MARK_IMPLEMENTATION_COMPLETE.md` for complete details.
