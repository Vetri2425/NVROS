#!/usr/bin/env python3
"""Check all MAVROS GPS-related topics"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time

class AllGPSTopics(Node):
    def __init__(self):
        super().__init__('all_gps_topics')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.global_gps = None
        self.raw_fix = None
        
        # Subscribe to multiple GPS topics
        self.sub1 = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.global_callback,
            qos
        )
        
        self.sub2 = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.raw_callback,
            qos
        )
        
        self.get_logger().info('Listening to MAVROS GPS topics...')
        
    def global_callback(self, msg):
        self.global_gps = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'status': msg.status.status
        }
    
    def raw_callback(self, msg):
        self.raw_fix = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'status': msg.status.status
        }

def main():
    rclpy.init()
    node = AllGPSTopics()
    
    # Spin for 5 seconds
    start = time.time()
    while time.time() - start < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("\n" + "="*60)
    print("MAVROS GPS TOPICS COMPARISON")
    print("="*60)
    
    if node.global_gps:
        print("\n📍 /mavros/global_position/global:")
        print(f"   Latitude:  {node.global_gps['lat']:.8f}")
        print(f"   Longitude: {node.global_gps['lon']:.8f}")
        print(f"   Altitude:  {node.global_gps['alt']:.2f} m  <-- AMSL (Above Mean Sea Level)")
        print(f"   Status:    {node.global_gps['status']}")
    else:
        print("\n❌ /mavros/global_position/global: No data")
    
    if node.raw_fix:
        print("\n📡 /mavros/global_position/raw/fix:")
        print(f"   Latitude:  {node.raw_fix['lat']:.8f}")
        print(f"   Longitude: {node.raw_fix['lon']:.8f}")
        print(f"   Altitude:  {node.raw_fix['alt']:.2f} m  <-- Raw GPS altitude (WGS84 ellipsoid)")
        print(f"   Status:    {node.raw_fix['status']}")
    else:
        print("\n❌ /mavros/global_position/raw/fix: No data")
    
    if node.global_gps and node.raw_fix:
        diff = node.global_gps['alt'] - node.raw_fix['alt']
        print(f"\n⚠️  Altitude Difference: {diff:.2f} m")
        print(f"   This is the geoid height offset for your location")
        print(f"   Raw GPS alt ({node.raw_fix['alt']:.2f}) + Geoid offset ({diff:.2f}) = AMSL ({node.global_gps['alt']:.2f})")
    
    print("="*60 + "\n")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
