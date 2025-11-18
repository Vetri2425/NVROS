# Rover Erratic Behavior - Root Cause Analysis

**Date:** 14 November 2025  
**Issue:** Rover moves forward/backward continuously in HOLD mode, manual mode not working properly

---

## 🔴 ROOT CAUSE IDENTIFIED

### **Problem: MAVROS ROS2 Node is NOT Running**

**Evidence:**
```bash
$ ros2 node list | grep mavros
# NO OUTPUT - MAVROS node is missing!
```

**Impact:**
- Mission controller cannot communicate with Pixhawk via ROS2
- Service calls timeout (`/mavros/mission/push`, `/mavros/set_mode`, etc.)
- System is using **rosbridge_websocket** for telemetry ONLY
- **No autopilot control through ROS2**

---

## 📊 Current System Architecture

### What's Running:
1. ✅ **rosbridge_websocket** - WebSocket bridge for telemetry
2. ✅ **Backend server (Python Flask)** - API and WebSocket server
3. ✅ **Mission controller node** - Our custom ROS2 node
4. ✅ **Telemetry node** - Data collection
5. ✅ **GPS altitude corrector** - GPS processing

### What's **NOT** Running:
1. ❌ **MAVROS ROS2 node** - Critical autopilot interface
2. ❌ **MAVLink router** - Message routing

---

## 🔍 Diagnosis Details

### 1. Mission Controller Behavior
From `/tmp/mission_controller.log`:
```
[INFO] Mission Controller initialized with state verification
[INFO] ✓ Flight mode changed: UNKNOWN → HOLD
[INFO] ✓ Arm state changed: False → True   # 17:29:49
[INFO] ✓ Arm state changed: True → False   # 17:29:52
[INFO] ✓ Arm state changed: False → True   # 17:29:56
[INFO] ✓ Arm state changed: True → False   # 17:30:00
```

**Analysis:**
- Mission controller is monitoring state changes
- Rapid ARM/DISARM cycles every 3-4 seconds
- No waypoint commands being sent
- **Symptom, not cause**

### 2. MAVROS Bridge (rosbridge)
- Using WebSocket connection to get telemetry
- Receiving GPS, battery, IMU data successfully
- **Cannot send commands** (no service interface)
- Only used for **reading data**, not controlling

### 3. The Real Problem

**Without MAVROS ROS2 node:**
```
❌ No /mavros/cmd/arming service
❌ No /mavros/set_mode service  
❌ No /mavros/mission/push service
❌ No /mavros/cmd/command service
❌ No autopilot command interface
```

**Result:**
- Pixhawk is receiving commands from **somewhere else** (RC? GCS? Old mission?)
- ROS2 system cannot override or control
- Erratic movement likely from:
  - Loaded mission on Pixhawk
  - RC transmitter interference
  - GCS (QGroundControl/Mission Planner) connected
  - Pixhawk failsafe behavior

---

## 🛠️ Solution Steps

### IMMEDIATE FIX (Choose ONE approach):

#### **Option A: Start MAVROS with System (Recommended)**

1. **Check if MAVROS is installed:**
```bash
ros2 pkg list | grep mavros
```

2. **Start MAVROS manually for testing:**
```bash
cd /home/flash/NRP_ROS
source /opt/ros/humble/setup.bash

# Start MAVROS for ArduPilot
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 &

# Wait 5 seconds for initialization
sleep 5

# Verify it's running
ros2 node list | grep mavros
```

3. **If successful, add MAVROS to start_service.sh:**

Edit `/home/flash/NRP_ROS/start_service.sh`, add after rosbridge launch:

```bash
# Start MAVROS for Pixhawk communication
log "Starting MAVROS..."
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 &
MAVROS_PID=$!
CHILD_PIDS+=($MAVROS_PID)
sleep 5  # Wait for MAVROS to initialize
```

4. **Restart service:**
```bash
sudo systemctl restart nrp-service
```

#### **Option B: Use Existing PyMAVLink (Fallback)**

If MAVROS cannot be installed/started, modify mission controller to use PyMAVLink directly instead of ROS2 services.

---

## 🔧 Temporary Workaround (Quick Test)

**Before fixing, test if this is the issue:**

1. **Stop mission controller:**
```bash
pkill -f mission_controller_node
```

2. **Connect to Pixhawk via QGroundControl/Mission Planner**

3. **Manually set mode to MANUAL**

4. **Check if rover behavior improves**

If YES → Confirms MAVROS missing is the issue  
If NO → Check for:
- RC transmitter active
- Existing mission on Pixhawk
- Parameter issues (EKF, fence, failsafe)

---

## ⚠️ Why This Happened

The mission controller node was updated to use:
```python
self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
self.command_long_client = self.create_client(CommandLong, '/mavros/cmd/command')
```

**These services require MAVROS ROS2 node to be running!**

The previous system might have been:
- Using rosbridge only (read-only telemetry)
- Controlling via RC transmitter
- Using external GCS

---

## 📝 Verification Checklist

After starting MAVROS, verify:

```bash
# 1. MAVROS node is running
ros2 node list | grep mavros
# Expected: /mavros

# 2. Services are available
ros2 service list | grep mavros | head -10
# Expected: /mavros/set_mode, /mavros/cmd/arming, etc.

# 3. Topics are publishing
ros2 topic hz /mavros/state
# Expected: ~10 Hz

# 4. Can set mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'HOLD'}"
# Expected: mode_sent: true

# 5. Mission controller connects
tail -f /tmp/mission_controller.log
# Expected: "All MAVROS services available"
```

---

## 🎯 Expected Behavior After Fix

1. **HOLD mode**: Rover stays completely still
2. **MANUAL mode**: Full RC control restored
3. **AUTO mode**: Follows missions correctly
4. **Mission controller**: Can arm, change modes, upload waypoints
5. **No erratic movement**: Stable operation

---

## 📋 Additional Checks

### Check for Loaded Mission on Pixhawk:
```bash
# Once MAVROS is running
ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull "{}"
ros2 topic echo /mavros/mission/waypoints --once
```

### Check RC Override:
```bash
ros2 topic echo /mavros/rc/override --once
```

### Check Current Mode:
```bash
ros2 topic echo /mavros/state --once | grep mode
```

---

## 🔄 Next Steps

1. **Install/Start MAVROS** (if not present)
2. **Test mode changes** via ROS2 services
3. **Verify manual control** works properly
4. **Test mission controller** with simple waypoint
5. **Update documentation** with MAVROS requirement

---

## 📞 Support Commands

```bash
# Check what's using the Pixhawk serial port
lsof /dev/ttyACM0

# Check if MAVLink messages are flowing
sudo cat /dev/ttyACM0 | hexdump -C | head -20

# Monitor MAVROS connection status
ros2 topic echo /mavros/state

# Check system health
ros2 topic echo /mavros/extended_state
```

---

**SUMMARY:** The rover behavior issue is caused by **missing MAVROS ROS2 node**. The mission controller node we created requires MAVROS services that are not currently running. Start MAVROS to restore proper autopilot control.
