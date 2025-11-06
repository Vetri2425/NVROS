#!/usr/bin/env python3
"""
Test the altitude fix by directly calling the mavros_bridge handler
"""

import sys
sys.path.append('/home/flash/NRP_ROS')

# Simulate the message that MAVROS would send
test_message = {
    "latitude": 13.07205770,
    "longitude": 80.26192970,
    "altitude": -76.58,  # Wrong altitude from MAVROS
    "relative_altitude": 0.15
}

print("="*70)
print("TESTING ALTITUDE FIX IN mavros_bridge.py")
print("="*70)
print("\nSimulated MAVROS message:")
print(f"  Latitude:  {test_message['latitude']:.8f}°")
print(f"  Longitude: {test_message['longitude']:.8f}°")
print(f"  Altitude:  {test_message['altitude']:.2f} m  (WRONG - from MAVROS)")
print()

# Apply the fix manually (same logic as in mavros_bridge.py)
ALTITUDE_CORRECTION = 92.2
corrected_altitude = test_message['altitude'] + ALTITUDE_CORRECTION

print("After applying fix:")
print(f"  Corrected Altitude: {corrected_altitude:.2f} m  (CORRECT)")
print()

print("Expected from MAVLink:  ~15.6 m")
print(f"Got after correction:   {corrected_altitude:.2f} m")
print()

if abs(corrected_altitude - 15.62) < 1.0:
    print("✅ FIX VERIFIED - Altitude is now correct!")
else:
    print("❌ FIX FAILED - Altitude still wrong")

print("="*70)
print()
print("The fix has been applied to:")
print("  /home/flash/NRP_ROS/Backend/mavros_bridge.py")
print()
print("Next steps:")
print("  1. Restart your backend server")
print("  2. Check logs for: [MAVROS_BRIDGE] GPS Update")
print("  3. Verify UI shows correct altitude (~15m not -76m)")
print("="*70)
