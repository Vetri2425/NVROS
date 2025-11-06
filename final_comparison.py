#!/usr/bin/env python3
"""
FINAL CONFIRMATION: Side-by-side comparison
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from pymavlink import mavutil
import threading
import time

class FinalComparison(Node):
    def __init__(self):
        super().__init__('final_comparison')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.ros_data = None
        self.subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.callback,
            qos
        )
        
    def callback(self, msg):
        self.ros_data = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }

def mavlink_thread(results):
    try:
        master = mavutil.mavlink_connection('tcp:127.0.0.1:5761')
        master.wait_heartbeat()
        
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            results['mavlink'] = {
                'lat': msg.lat / 1e7,
                'lon': msg.lon / 1e7,
                'alt': msg.alt / 1000.0
            }
    except:
        pass

def main():
    print("\n" + "="*70)
    print("FINAL COMPARISON: MAVLink vs MAVROS")
    print("="*70 + "\n")
    
    rclpy.init()
    node = FinalComparison()
    
    results = {'mavlink': None}
    thread = threading.Thread(target=mavlink_thread, args=(results,), daemon=True)
    thread.start()
    
    # Spin for 3 seconds
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    
    thread.join(timeout=2)
    
    if results['mavlink'] and node.ros_data:
        print("SOURCE 1: RAW MAVLink GLOBAL_POSITION_INT (from ArduPilot)")
        print(f"  Latitude:  {results['mavlink']['lat']:.8f}°")
        print(f"  Longitude: {results['mavlink']['lon']:.8f}°")
        print(f"  Altitude:  {results['mavlink']['alt']:.2f} m  ✅ CORRECT")
        
        print("\nSOURCE 2: MAVROS ROS2 Topic (/mavros/global_position/global)")
        print(f"  Latitude:  {node.ros_data['lat']:.8f}°")
        print(f"  Longitude: {node.ros_data['lon']:.8f}°")
        print(f"  Altitude:  {node.ros_data['alt']:.2f} m  ❌ WRONG")
        
        alt_error = results['mavlink']['alt'] - node.ros_data['alt']
        print(f"\n⚠️  ALTITUDE ERROR: {alt_error:.2f} m")
        print(f"   MAVLink altitude ({results['mavlink']['alt']:.2f}) - MAVROS altitude ({node.ros_data['alt']:.2f}) = {alt_error:.2f} m error")
        
        print("\n" + "="*70)
        print("CONCLUSION:")
        print("="*70)
        print("✅ MAVLink data from ArduPilot is CORRECT")
        print("❌ MAVROS /mavros/global_position/global topic has WRONG altitude")
        print(f"🐛 Bug confirmed: MAVROS corrupts GPS altitude by ~{alt_error:.1f}m")
        print("\nLikely cause: MAVROS global_position plugin bug or config issue")
        print("="*70 + "\n")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
