#!/usr/bin/env python3
"""
Monitor the exact data being sent to Telemetry Panel and RTK Panel.
Shows 5 samples of each data type.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, BatteryState
from mavros_msgs.msg import State, RTKBaseline
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time


class TelemetryMonitor(Node):
    def __init__(self):
        super().__init__('telemetry_monitor')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Counters
        self.navsat_count = 0
        self.gps_fix_count = 0
        self.rtk_baseline_count = 0
        self.state_count = 0
        self.velocity_count = 0
        
        # Subscribe to all telemetry topics
        self.sub_navsat = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.navsat_callback, qos)
        
        self.sub_gps_fix = self.create_subscription(
            NavSatFix, '/mavros/global_position/raw/fix', self.gps_fix_callback, qos)
        
        self.sub_rtk = self.create_subscription(
            RTKBaseline, '/mavros/gps_rtk/rtk_baseline', self.rtk_callback, qos)
        
        self.sub_state = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos)
        
        self.sub_velocity = self.create_subscription(
            TwistStamped, '/mavros/global_position/raw/gps_vel', self.velocity_callback, qos)
        
        print("\n" + "="*80)
        print("TELEMETRY & RTK PANEL DATA MONITOR")
        print("="*80)
        print("Showing exactly what data is sent to the UI panels...")
        print("Collecting 5 samples of each data type...\n")
        
    def navsat_callback(self, msg):
        """GPS Position data → Telemetry Panel"""
        self.navsat_count += 1
        if self.navsat_count <= 5:
            print(f"📍 TELEMETRY PANEL - GPS Position #{self.navsat_count}")
            print(f"   latitude:  {msg.latitude:.8f}°")
            print(f"   longitude: {msg.longitude:.8f}°")
            print(f"   altitude:  {msg.altitude:.2f} m")
            print(f"   status:    {msg.status.status} (0=no_fix, 1=fix, 2=sbas, 3=gbas, 4=dgps, 5=rtk_float, 6=rtk_fixed)")
            print(f"   service:   {msg.status.service}")
            
            # Calculate covariance info
            cov = msg.position_covariance
            if len(cov) >= 5:
                hrms = (cov[0] + cov[4]) ** 0.5
                print(f"   hrms:      {hrms:.2f} m (horizontal accuracy)")
            print()
    
    def gps_fix_callback(self, msg):
        """GPS Fix Quality → RTK Panel"""
        self.gps_fix_count += 1
        if self.gps_fix_count <= 5:
            print(f"📡 RTK PANEL - GPS Fix Quality #{self.gps_fix_count}")
            
            fix_type = msg.status.status
            fix_names = {
                0: "No Fix",
                1: "No Fix",
                2: "2D Fix",
                3: "3D Fix",
                4: "DGPS/SBAS",
                5: "RTK Float",
                6: "RTK Fixed"
            }
            
            print(f"   rtk_status:         {fix_names.get(fix_type, 'Unknown')} (type={fix_type})")
            print(f"   fix_type:           {fix_type}")
            print(f"   satellites_visible: {msg.status.service} (from service field)")
            
            cov = msg.position_covariance
            if len(cov) >= 5:
                hrms = (cov[0] + cov[4]) ** 0.5
                print(f"   hrms:               {hrms:.2f} m")
            print()
    
    def rtk_callback(self, msg):
        """RTK Baseline data → RTK Panel"""
        self.rtk_baseline_count += 1
        if self.rtk_baseline_count <= 5:
            print(f"🛰️  RTK PANEL - RTK Baseline #{self.rtk_baseline_count}")
            
            # Convert mm to meters
            baseline_a = msg.baseline_a_mm / 1000.0
            baseline_b = msg.baseline_b_mm / 1000.0
            baseline_c = msg.baseline_c_mm / 1000.0
            
            # Calculate distance from base station
            baseline_distance = (baseline_a**2 + baseline_b**2 + baseline_c**2) ** 0.5
            
            print(f"   baseline_distance:  {baseline_distance:.2f} m (from base station)")
            print(f"   baseline_a:         {baseline_a:.2f} m")
            print(f"   baseline_b:         {baseline_b:.2f} m")
            print(f"   baseline_c:         {baseline_c:.2f} m")
            print(f"   accuracy:           {msg.accuracy} mm")
            print(f"   nsats:              {msg.nsats} satellites")
            print(f"   rtk_health:         {msg.rtk_health}")
            print(f"   rtk_rate:           {msg.rtk_rate} Hz")
            print()
    
    def state_callback(self, msg):
        """Vehicle State → Telemetry Panel"""
        self.state_count += 1
        if self.state_count <= 5:
            print(f"🚗 TELEMETRY PANEL - Vehicle State #{self.state_count}")
            print(f"   connected: {msg.connected}")
            print(f"   armed:     {msg.armed}")
            print(f"   mode:      {msg.mode}")
            print(f"   status:    {'armed' if msg.armed else 'disarmed'}")
            print()
    
    def velocity_callback(self, msg):
        """Velocity data → Telemetry Panel"""
        self.velocity_count += 1
        if self.velocity_count <= 5:
            speed = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2) ** 0.5
            print(f"🏃 TELEMETRY PANEL - Velocity #{self.velocity_count}")
            print(f"   linear_x:  {msg.twist.linear.x:.2f} m/s")
            print(f"   linear_y:  {msg.twist.linear.y:.2f} m/s")
            print(f"   linear_z:  {msg.twist.linear.z:.2f} m/s")
            print(f"   speed:     {speed:.2f} m/s ({speed*3.6:.1f} km/h)")
            print()


def main():
    rclpy.init()
    node = TelemetryMonitor()
    
    start_time = time.time()
    max_wait = 30  # Wait up to 30 seconds
    
    try:
        while rclpy.ok() and time.time() - start_time < max_wait:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Check if we have 5 samples of main data
            if (node.navsat_count >= 5 and 
                node.gps_fix_count >= 5 and 
                node.state_count >= 5):
                print("\n✅ Collected 5 samples of main telemetry data!")
                break
                
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    print("\n" + "="*80)
    print("SUMMARY OF DATA COLLECTED:")
    print("="*80)
    print(f"GPS Position (Telemetry):  {node.navsat_count} samples")
    print(f"GPS Fix (RTK Panel):       {node.gps_fix_count} samples")
    print(f"RTK Baseline (RTK Panel):  {node.rtk_baseline_count} samples")
    print(f"Vehicle State:             {node.state_count} samples")
    print(f"Velocity:                  {node.velocity_count} samples")
    print("="*80)
    
    if node.navsat_count == 0:
        print("\n⚠️  No GPS position data received!")
        print("   Check if MAVROS is running and publishing to /mavros/global_position/global")
    
    if node.rtk_baseline_count == 0:
        print("\n⚠️  No RTK baseline data received!")
        print("   This is normal if RTK base station is not configured")
    
    print()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
