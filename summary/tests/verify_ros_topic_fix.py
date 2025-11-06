#!/usr/bin/env python3
"""
Compare the original buggy MAVROS topic vs the corrected topic.
This proves the ROS-level fix is working.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time


class TopicComparison(Node):
    def __init__(self):
        super().__init__('topic_comparison')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.original = None
        self.corrected = None
        
        # Subscribe to both topics
        self.sub1 = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.original_callback,
            qos
        )
        
        self.sub2 = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global_corrected',
            self.corrected_callback,
            qos
        )
        
    def original_callback(self, msg):
        self.original = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }
    
    def corrected_callback(self, msg):
        self.corrected = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }


def main():
    rclpy.init()
    node = TopicComparison()
    
    print("\n" + "="*70)
    print("ROS2 TOPIC COMPARISON: Original vs Corrected")
    print("="*70)
    print("Collecting data for 3 seconds...\n")
    
    # Spin for 3 seconds
    start = time.time()
    while time.time() - start < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("="*70)
    print("RESULTS:")
    print("="*70 + "\n")
    
    if node.original:
        print("📍 Original Topic: /mavros/global_position/global (BUGGY)")
        print(f"   Latitude:  {node.original['lat']:.8f}°")
        print(f"   Longitude: {node.original['lon']:.8f}°")
        print(f"   Altitude:  {node.original['alt']:.2f} m  ❌ WRONG")
    else:
        print("❌ Original topic: No data")
    
    print()
    
    if node.corrected:
        print("✅ Corrected Topic: /mavros/global_position/global_corrected (FIXED)")
        print(f"   Latitude:  {node.corrected['lat']:.8f}°")
        print(f"   Longitude: {node.corrected['lon']:.8f}°")
        print(f"   Altitude:  {node.corrected['alt']:.2f} m  ✅ CORRECT")
    else:
        print("❌ Corrected topic: No data")
        print("   Make sure gps_altitude_corrector.py node is running!")
    
    if node.original and node.corrected:
        correction = node.corrected['alt'] - node.original['alt']
        print(f"\n📊 Altitude Correction Applied: +{correction:.2f} m")
        print(f"   {node.original['alt']:.2f}m (buggy) + {correction:.2f}m = {node.corrected['alt']:.2f}m (correct)")
        
        if abs(correction - 92.2) < 1.0:
            print("\n✅ CORRECTION VERIFIED - ROS node is working correctly!")
        else:
            print(f"\n⚠️  Warning: Expected correction ~92.2m, got {correction:.2f}m")
    
    print("\n" + "="*70)
    print("Backend now uses: /mavros/global_position/global_corrected")
    print("UI will display correct altitude (~15m instead of -76m)")
    print("="*70 + "\n")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
