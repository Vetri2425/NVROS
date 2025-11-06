# Test Scripts & Diagnostic Tools

This folder contains all test scripts, diagnostic tools, and monitoring utilities for the NRP ROS system.

## Test Scripts

### test_gps.py
**Purpose**: Monitor GPS data from MAVROS topics with correct QoS settings

**Usage**:
```bash
python3 summary/tests/test_gps.py
```

**What it does**:
- Subscribes to `/mavros/global_position/global`
- Uses BEST_EFFORT QoS (required for MAVROS)
- Displays GPS position updates in real-time

---

### test_gps_raw_integration.py ⭐
**Purpose**: Integration test for GPS raw topic handler

**Usage**:
```bash
python3 summary/tests/test_gps_raw_integration.py
```

**What it tests**:
- ✅ Lat/lon conversion (÷1e7)
- ✅ Altitude conversion (mm → meters)
- ✅ Accuracy conversion (cm → meters)
- ✅ Fix type mapping (6 → "RTK Fixed")
- ✅ Satellite count extraction
- ✅ Telemetry broadcast structure

**Expected Output**:
```
✅ GPS RAW integration test PASSED!
All position values correct!
All GPS fix values correct!
```

---

## Diagnostic Scripts

### check_all_gps.py
**Purpose**: Check multiple MAVROS GPS topics

**What it checks**:
- `/mavros/global_position/global`
- `/mavros/global_position/raw/fix`
- Other GPS-related topics

---

### check_ardupilot_gps.py
**Purpose**: Check GPS status directly from ArduPilot

**What it does**:
- Reads GPS_RAW_INT MAVLink messages
- Shows true GPS status from flight controller
- Bypasses MAVROS for ground truth

---

### check_mavlink_gcs.py
**Purpose**: Read raw MAVLink from MAVROS GCS port

**Usage**:
```bash
python3 summary/tests/check_mavlink_gcs.py
```

**What it does**:
- Connects to tcp:127.0.0.1:5761 (MAVROS GCS port)
- Reads GLOBAL_POSITION_INT messages
- Shows unmodified MAVLink data from ArduPilot

---

### compare_gps.py
**Purpose**: Compare MAVLink data vs MAVROS topics side-by-side

**What it compares**:
- MAVLink GLOBAL_POSITION_INT (ground truth)
- MAVROS /mavros/global_position/global (transformed)
- Highlights discrepancies

**Use Case**: Verify MAVROS transformation bugs

---

## Monitoring Scripts

### monitor_gps_updates.py
**Purpose**: Monitor whether GPS coordinates are updating

**Usage**:
```bash
python3 summary/tests/monitor_gps_updates.py
```

**What it monitors**:
- Coordinate changes over time
- Update frequency
- Variance in position

**Use Case**: Determine if "constant" coordinates are stale or just precise

---

### monitor_telemetry_data.py
**Purpose**: Monitor telemetry/RTK panel data for 5 samples

**Usage**:
```bash
python3 summary/tests/monitor_telemetry_data.py
```

**What it displays**:
- GPS position (lat, lon, alt)
- GPS fix quality (status, satellites, accuracy)
- RTK baseline data
- Vehicle state
- Velocity data

**Sample Output**:
```
📍 TELEMETRY PANEL - GPS Position #1
   latitude:  13.0720581°
   longitude: 80.2619324°
   altitude:  16.61 m
   status:    RTK Fixed
   satellites: 29
```

---

## Verification Scripts

### verify_altitude_fix.py
**Purpose**: Verify altitude correction calculation

**What it verifies**:
- MAVLink altitude (correct)
- MAVROS altitude (buggy)
- Correction offset (92.2m)

---

### verify_ros_topic_fix.py
**Purpose**: Compare original vs corrected GPS topics

**What it compares**:
- `/mavros/global_position/global` (buggy)
- `/mavros/global_position/global_corrected` (fixed)

**Note**: Deprecated - replaced by raw GPS topic

---

## Running All Tests

To verify system health:

```bash
cd /home/flash/NRP_ROS

# 1. Integration test
python3 summary/tests/test_gps_raw_integration.py

# 2. Monitor live GPS
python3 summary/tests/monitor_telemetry_data.py

# 3. Check raw GPS topic
ros2 topic echo /mavros/gpsstatus/gps1/raw --once
```

---

## Test Development Guidelines

### Creating New Test Scripts

1. **Name**: Use descriptive prefixes
   - `test_` - Unit/integration tests
   - `check_` - Diagnostic checks
   - `monitor_` - Live monitoring tools
   - `verify_` - Verification scripts
   - `compare_` - Comparison tools

2. **Structure**:
   ```python
   #!/usr/bin/env python3
   """
   Brief description of what this script does.
   """
   
   # Imports
   # Functions
   # Main execution
   
   if __name__ == "__main__":
       # Entry point
   ```

3. **Documentation**:
   - Add docstrings
   - Include usage examples
   - Document expected output

4. **Location**:
   - Place in `summary/tests/`
   - Update this README

---

## Test Results History

### October 30, 2025

**test_gps_raw_integration.py**:
```
✅ All position values correct!
  Latitude:  13.0720581° (expected: 13.0720581°)
  Longitude: 80.2619324° (expected: 80.2619324°)
  Altitude:  16.61 m (expected: 16.61 m)

✅ All GPS fix values correct!
  RTK Status: RTK Fixed (expected: RTK Fixed)
  Fix Type: 6 (expected: 6)
  Satellites Visible: 29 (expected: 29)
  Horizontal Accuracy: 0.70 m (expected: 0.70 m)
  Vertical Accuracy: 1.20 m (expected: 1.20 m)

✅ GPS RAW integration test PASSED!
```

---

**Folder**: `summary/tests/`  
**Last Updated**: October 30, 2025  
**Total Scripts**: 10  
**Test Coverage**: GPS system integration ✅
