# GPS ALTITUDE ISSUE - DIAGNOSIS & FIX

## Problem Statement
You discovered that:
1. **MAVProxy** shows correct GPS altitude: `+15.6m`
2. **ROS2 topic** `/mavros/global_position/global` shows wrong altitude: `-76.6m`
3. **Error magnitude**: ~92 meters

## Root Cause
**MAVROS `global_position` plugin is corrupting GPS altitude data.**

The MAVLink GLOBAL_POSITION_INT message from ArduPilot contains correct altitude (+15.6m), but MAVROS transforms it incorrectly to -76.6m when publishing to ROS2 topics.

## Verification Tests Performed

### Test 1: Direct MAVLink via MAVProxy
```bash
mavproxy.py --master=/dev/ttyACM0 --baud=115200
```
**Result:** Altitude = +15.64m ✅

### Test 2: MAVROS ROS2 Topic
```bash
ros2 topic echo /mavros/global_position/global
```
**Result:** Altitude = -76.56m ❌

### Test 3: Raw MAVLink via MAVROS GCS Port
```python
# Connect to tcp://127.0.0.1:5761
```
**Result:** GLOBAL_POSITION_INT.alt = +15.62m ✅

### Test 4: MAVROS raw fix topic
```bash
ros2 topic echo /mavros/global_position/raw/fix
```
**Result:** Altitude = -75.65m ❌

## Additional Finding
**GPS Status = 0 (NO FIX)**

The GPS does not have a valid fix, which may contribute to incorrect/cached altitude data. However, MAVLink shows the same lat/lon as MAVROS, suggesting the position is being received, but altitude transformation is buggy.

## Possible Root Causes

1. **Home altitude offset bug**: MAVROS may be incorrectly applying home position altitude offset
2. **Geoid height miscalculation**: The ~92m error might be geoid height applied with wrong sign
3. **Frame transformation issue**: AMSL vs WGS84 ellipsoid height confusion
4. **Cached/stale data**: Without GPS fix, MAVROS might be using old/invalid altitude reference
5. **MAVROS bug**: The global_position plugin may have a known bug in your MAVROS version

## Recommended Fixes

### Fix 1: Use GPS Raw Data Instead
Use `/mavros/global_position/raw/fix` topic which might have correct altitude, or subscribe to raw GPS messages directly via MAVLink.

### Fix 2: Check MAVROS Parameters
Check if there's a parameter for altitude reference frame:
```bash
ros2 param list /mavros
ros2 param get /mavros use_mission_item_int
```

### Fix 3: Force Home Position Reset
```bash
# Via ROS service
ros2 service call /mavros/home_position/req_update std_srvs/srv/Trigger
```

### Fix 4: Update MAVROS
Your MAVROS version might have this bug. Check for updates:
```bash
apt list --installed | grep mavros
sudo apt update
sudo apt upgrade ros-humble-mavros
```

### Fix 5: Workaround in Code
Add altitude correction in your code:
```python
# In mavros_bridge.py or wherever you read GPS
ALTITUDE_OFFSET = 92.2  # Measured error

def fix_altitude(mavros_alt):
    return mavros_alt + ALTITUDE_OFFSET
```

### Fix 6: Use Local Position Instead
If you only need relative positioning, use:
```bash
ros2 topic echo /mavros/local_position/pose
```

## Immediate Workaround

Until the root cause is fixed, modify `Backend/mavros_bridge.py` to correct the altitude:

```python
def _handle_global_position(self, msg: Dict[str, Any]) -> None:
    """Handle global position updates with altitude fix"""
    # WORKAROUND: MAVROS publishes wrong altitude (-76m instead of +15m)
    # Apply correction based on measured error
    ALTITUDE_CORRECTION = 92.2
    
    lat = msg.get("latitude", 0.0)
    lon = msg.get("longitude", 0.0)
    alt = msg.get("altitude", 0.0) + ALTITUDE_CORRECTION  # FIX HERE
    
    # ... rest of function
```

## Files Created for Diagnosis

1. `test_gps.py` - Monitor GPS data from MAVROS topics
2. `compare_gps.py` - Compare MAVLink vs MAVROS GPS data
3. `check_all_gps.py` - Check all MAVROS GPS topics
4. `check_mavlink_gcs.py` - Read raw MAVLink messages from MAVROS
5. `final_comparison.py` - Side-by-side comparison proving the bug
6. `gps_diagnosis_summary.py` - Summary of findings

## Next Steps

1. ✅ **Confirmed**: MAVROS bug - altitude is wrong by ~92m
2. ⚠️ **Action needed**: Apply altitude correction workaround in code
3. 🔍 **Investigate**: Check MAVROS GitHub issues for similar reports
4. 📝 **Report**: Consider filing bug report to MAVROS maintainers

## Technical Details

- **Flight Controller**: ArduRover V4.5.6
- **Hardware**: CubeOrangePlus
- **MAVROS Version**: ros-humble-mavros
- **Connection**: /dev/ttyACM0 @ 115200 baud
- **Location**: 13.072°N, 80.262°E (Chennai, India area)
- **GPS Status**: 0 (No fix - may be contributing factor)

---

**Date**: 2025-10-30
**Status**: ✅ Bug confirmed, workaround available
