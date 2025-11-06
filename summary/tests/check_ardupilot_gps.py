#!/usr/bin/env python3
"""
Check GPS status directly from ArduPilot via MAVLink to understand the real GPS state.
"""

from pymavlink import mavutil
import time

def main():
    print("\n" + "="*80)
    print("CHECKING GPS STATUS FROM ARDUPILOT (via MAVLink GCS port)")
    print("="*80 + "\n")
    
    try:
        # Connect to MAVROS GCS port
        master = mavutil.mavlink_connection('tcp:127.0.0.1:5761')
        print("Connecting to tcp:127.0.0.1:5761...")
        master.wait_heartbeat()
        print(f"✅ Connected to system {master.target_system}, component {master.target_component}\n")
        
        print("Collecting GPS data for 10 seconds...\n")
        
        gps_raw_count = 0
        gps_status_count = 0
        global_pos_count = 0
        
        start_time = time.time()
        while time.time() - start_time < 10:
            # Check multiple message types
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == 'GPS_RAW_INT':
                    gps_raw_count += 1
                    if gps_raw_count <= 3:  # Show first 3
                        print(f"GPS_RAW_INT #{gps_raw_count}:")
                        print(f"  Fix Type: {msg.fix_type} (0=none, 1=no_fix, 2=2D, 3=3D, 4=DGPS, 5=RTK_float, 6=RTK_fixed)")
                        print(f"  Satellites: {msg.satellites_visible}")
                        print(f"  Lat: {msg.lat/1e7:.8f}, Lon: {msg.lon/1e7:.8f}")
                        print(f"  Alt: {msg.alt/1000:.2f}m")
                        print(f"  EPH: {msg.eph/100:.2f}m, EPV: {msg.epv/100:.2f}m")
                        print()
                
                elif msg_type == 'GPS_STATUS':
                    gps_status_count += 1
                    if gps_status_count <= 1:  # Show first one
                        print(f"GPS_STATUS:")
                        print(f"  Satellites visible: {msg.satellites_visible}")
                        print()
                
                elif msg_type == 'GLOBAL_POSITION_INT':
                    global_pos_count += 1
                    if global_pos_count <= 2:  # Show first 2
                        print(f"GLOBAL_POSITION_INT #{global_pos_count}:")
                        print(f"  Lat: {msg.lat/1e7:.8f}, Lon: {msg.lon/1e7:.8f}")
                        print(f"  Alt (MSL): {msg.alt/1000:.2f}m")
                        print(f"  Relative Alt: {msg.relative_alt/1000:.2f}m")
                        print()
        
        print("="*80)
        print("SUMMARY:")
        print("="*80)
        print(f"GPS_RAW_INT messages received: {gps_raw_count}")
        print(f"GPS_STATUS messages received: {gps_status_count}")
        print(f"GLOBAL_POSITION_INT messages received: {global_pos_count}")
        print()
        
        if gps_raw_count == 0:
            print("❌ No GPS_RAW_INT messages - GPS not working!")
        
        print("="*80 + "\n")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nMake sure MAVROS is running with GCS port enabled")


if __name__ == '__main__':
    main()
