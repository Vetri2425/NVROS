#!/usr/bin/env python3
"""
Monitor GPS coordinates to check if they are actually updating or stuck/constant.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time


class GPSMonitor(Node):
    def __init__(self):
        super().__init__('gps_monitor')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.readings = []
        
        self.subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos
        )
        
        self.get_logger().info('Monitoring GPS coordinates for changes...')
        
    def gps_callback(self, msg):
        reading = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'status': msg.status.status,
            'time': time.time()
        }
        self.readings.append(reading)
        
        # Log every reading with timestamp
        print(f"[{len(self.readings):3d}] Lat: {reading['lat']:.8f}  Lon: {reading['lon']:.8f}  "
              f"Alt: {reading['alt']:7.2f}m  Status: {reading['status']}")


def main():
    rclpy.init()
    node = GPSMonitor()
    
    print("\n" + "="*80)
    print("GPS COORDINATE UPDATE MONITORING")
    print("="*80)
    print("Watching /mavros/global_position/global for 15 seconds...")
    print("If coordinates are CONSTANT, GPS is not updating (stuck/cached data)")
    print("="*80 + "\n")
    
    start_time = time.time()
    try:
        while time.time() - start_time < 15:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    
    # Analyze the readings
    print("\n" + "="*80)
    print("ANALYSIS:")
    print("="*80)
    
    if len(node.readings) < 2:
        print("❌ Not enough GPS data received!")
        print("   Check if MAVROS is running and publishing data")
    else:
        print(f"✅ Received {len(node.readings)} GPS readings in 15 seconds")
        print(f"   Rate: ~{len(node.readings)/15:.1f} Hz\n")
        
        # Check for changes
        first = node.readings[0]
        last = node.readings[-1]
        
        lat_change = abs(last['lat'] - first['lat'])
        lon_change = abs(last['lon'] - first['lon'])
        alt_change = abs(last['alt'] - first['alt'])
        
        print("COORDINATE CHANGES over 15 seconds:")
        print(f"  Latitude:  {lat_change:.10f}° ({lat_change * 111000:.2f} meters)")
        print(f"  Longitude: {lon_change:.10f}° ({lon_change * 111000:.2f} meters)")
        print(f"  Altitude:  {alt_change:.2f} m")
        print()
        
        # Check if stuck
        lat_stuck = lat_change < 0.000001  # Less than ~0.1mm change
        lon_stuck = lon_change < 0.000001
        
        if lat_stuck and lon_stuck:
            print("❌ GPS COORDINATES ARE STUCK/CONSTANT!")
            print("   Lat/Lon have not changed significantly")
            print()
            print("Possible causes:")
            print("  1. GPS Status = 0 (No Fix) - GPS has no satellite lock")
            print("  2. MAVROS publishing cached/stale data")
            print("  3. GPS antenna disconnected or faulty")
            print("  4. Indoor/poor GPS signal")
            print("  5. ArduPilot not receiving GPS updates")
            print()
            print(f"Current GPS Status: {last['status']}")
            if last['status'] == 0:
                print("  → Status 0 = NO GPS FIX - This is the problem!")
                print("  → GPS needs clear view of sky to get satellite lock")
        else:
            print("✅ GPS coordinates are UPDATING")
            print(f"   Position changed by ~{lat_change * 111000:.2f}m")
        
        # Check altitude
        if alt_change < 0.1:
            print(f"\n⚠️  Altitude is constant: {first['alt']:.2f}m")
            print("   (May be normal if vehicle is stationary)")
        else:
            print(f"\n✅ Altitude is changing: {first['alt']:.2f}m → {last['alt']:.2f}m")
        
        # Show unique values
        unique_lats = len(set(f"{r['lat']:.7f}" for r in node.readings))
        unique_lons = len(set(f"{r['lon']:.7f}" for r in node.readings))
        print(f"\nUnique coordinate values:")
        print(f"  Latitudes:  {unique_lats} different values")
        print(f"  Longitudes: {unique_lons} different values")
        
        if unique_lats == 1 and unique_lons == 1:
            print("\n❌ COORDINATES ARE COMPLETELY CONSTANT - GPS NOT WORKING!")
        elif unique_lats < 3 and unique_lons < 3:
            print("\n⚠️  Very few coordinate changes - GPS may have poor lock")
        
    print("="*80 + "\n")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
