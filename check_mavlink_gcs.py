#!/usr/bin/env python3
"""
Connect to MAVROS GCS port and read raw MAVLink GLOBAL_POSITION_INT messages
to verify what data ArduPilot is actually sending
"""

from pymavlink import mavutil
import time

def main():
    print("\n" + "="*70)
    print("READING RAW MAVLINK MESSAGES FROM MAVROS GCS PORT")
    print("="*70)
    print("Connecting to tcp://127.0.0.1:5761 (MAVROS GCS output)...")
    
    try:
        # Connect to MAVROS GCS output port
        master = mavutil.mavlink_connection('tcp:127.0.0.1:5761')
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print(f"✅ Connected to system {master.target_system}, component {master.target_component}\n")
        
        print("Listening for GLOBAL_POSITION_INT messages (10 seconds)...\n")
        
        start_time = time.time()
        count = 0
        
        while time.time() - start_time < 10 and count < 5:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
            if msg:
                count += 1
                print(f"Message #{count}:")
                print(f"  Latitude:      {msg.lat / 1e7:.8f}°")
                print(f"  Longitude:     {msg.lon / 1e7:.8f}°")
                print(f"  Altitude (MSL): {msg.alt / 1000.0:.2f} m")
                print(f"  Relative Alt:  {msg.relative_alt / 1000.0:.2f} m")
                print(f"  Heading:       {msg.hdg / 100.0:.1f}°")
                print()
                time.sleep(1)
        
        if count == 0:
            print("❌ No GLOBAL_POSITION_INT messages received!")
            print("   Check if MAVROS is running and connected to FCU")
        else:
            print(f"✅ Received {count} GLOBAL_POSITION_INT messages")
            print("\nThis is the RAW data ArduPilot is sending via MAVLink")
            print("MAVROS should be publishing this exact data to ROS topics")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nPossible issues:")
        print("  - MAVROS not running")
        print("  - GCS port not configured (gcs_url:=tcp-l://0.0.0.0:5761)")
        print("  - Firewall blocking connection")
    
    print("="*70 + "\n")

if __name__ == '__main__':
    main()
