#!/usr/bin/env python3
"""
DIAGNOSIS SUMMARY: GPS Data Discrepancy between MAVLink and MAVROS

This script summarizes the GPS data issue you discovered.
"""

print("""
================================================================================
GPS DATA DISCREPANCY DIAGNOSIS
================================================================================

ISSUE FOUND:
-----------
✅ MAVProxy (direct MAVLink): Shows CORRECT GPS altitude  = +15.64 m (MSL)
❌ MAVROS ROS2 Topics:         Shows WRONG GPS altitude   = -76.56 m (AMSL)

DIFFERENCE: ~92 meters!

WHAT WAS TESTED:
---------------
1. ros2 topic echo /mavros/global_position/global
   → Result: Altitude = -76 m ❌

2. mavproxy.py --master=/dev/ttyACM0 --baud=115200
   → Result: Altitude = +15.64 m ✅

3. MAVLink Direct (via pymavlink):
   → Result: Altitude = +15.64 m ✅

4. /mavros/global_position/raw/fix:
   → Result: Altitude = -75.65 m ❌

ROOT CAUSE:
----------
MAVROS is publishing GPS data, but the altitude value is incorrect/inverted.
The latitude and longitude are correct, but altitude has ~92m error.

Possible reasons:
1. MAVROS geoid height calculation is wrong
2. MAVROS is using wrong reference frame for altitude
3. Home position altitude offset is incorrectly applied
4. GLOBAL_POSITION_INT message from ArduPilot has wrong altitude field
5. MAVROS global_position plugin has a bug with altitude transformation

VERIFICATION:
------------
- MAVLink connection works fine (MAVProxy shows correct data)
- MAVROS receives MAVLink messages (connected to FCU)
- ROS2 topics are publishing at correct rate (~2Hz)
- QoS settings are correct (BEST_EFFORT)
- Lat/Lon data matches between all sources
- Only ALTITUDE differs

RECOMMENDATION:
--------------
1. Check MAVROS parameter for altitude reference frame
2. Check if geoid height offset is being applied incorrectly
3. Verify ArduPilot GPS_TYPE and AHRS_EKF_TYPE parameters
4. Check if home altitude is set correctly
5. Look into MAVROS global_position plugin source code
6. Try checking raw MAVLink GLOBAL_POSITION_INT message altitude field

STATUS: GPS STATUS = 0 (No GPS fix)
This might be related to the issue - without GPS fix, altitude may be invalid.

================================================================================
""")

# Check GPS status meanings
print("\nGPS STATUS CODES:")
print("  0 = NO_FIX      - No GPS fix acquired")
print("  1 = NO_FIX      - No GPS fix")  
print("  2 = 2D_FIX      - 2D GPS fix")
print("  3 = 3D_FIX      - 3D GPS fix")
print("  4 = DGPS        - DGPS/SBAS aided 3D fix")
print("  5 = RTK_FLOAT   - RTK float fix")
print("  6 = RTK_FIXED   - RTK fixed fix")
print("\n⚠️  Current GPS Status = 0 (NO FIX)")
print("   Without GPS fix, altitude data may be unreliable or cached!")
print("   The negative altitude suggests old/invalid/cached data.")
print("\n================================================================================\n")
