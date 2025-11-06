#!/usr/bin/env python3
"""Check current RTK status from rover_data emissions."""

import socketio
import sys

sio = socketio.Client()

@sio.on('connect')
def on_connect():
    print("✅ Connected to backend\n")

@sio.on('rover_data')
def on_rover_data(data):
    print("="*60)
    print("CURRENT RTK STATUS FROM ROVER")
    print("="*60)
    print(f"rtk_fix_type:      {data.get('rtk_fix_type', 'N/A')}")
    print(f"rtk_status:        {data.get('rtk_status', 'N/A')}")
    print(f"rtk_base_linked:   {data.get('rtk_base_linked', 'N/A')}")
    print(f"rtk_baseline_age:  {data.get('rtk_baseline_age', 'N/A')}")
    print(f"rtk_baseline:      {data.get('rtk_baseline', 'N/A')}")
    print(f"satellites_visible: {data.get('satellites_visible', 'N/A')}")
    print()
    
    fix_type = data.get('rtk_fix_type', 0)
    fix_meaning = {
        0: "No GPS",
        1: "No Fix",
        2: "2D Fix",
        3: "3D Fix",
        4: "DGPS",
        5: "RTK Float",
        6: "RTK Fixed"
    }
    print(f"Fix Type Meaning:  {fix_meaning.get(fix_type, 'Unknown')}")
    print()
    
    if fix_type < 5:
        print("ℹ️  RTK is NOT active (fix_type < 5)")
        print("   → No RTCM corrections being received")
        print("   → /mavros/gps_rtk/rtk_baseline will be empty")
        print("   → To enable: Connect to NTRIP caster via frontend")
    else:
        print("✅ RTK is ACTIVE!")
        print("   → RTCM corrections flowing")
        print("   → GPS has RTK solution")
    
    print("="*60)
    sio.disconnect()

try:
    print("Connecting to backend at http://localhost:5001...\n")
    sio.connect('http://localhost:5001')
    sio.wait()
except KeyboardInterrupt:
    print("\n\nInterrupted")
    sys.exit(0)
except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)
