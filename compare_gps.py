#!/usr/bin/env python3
"""Compare GPS data from MAVLink (direct) vs MAVROS (ROS2 topic)"""

from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import time

class GPSCompare(Node):
    def __init__(self):
        super().__init__('gps_compare')
        
        # MAVROS uses BEST_EFFORT QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.ros_gps = None
        self.subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos
        )
        
    def gps_callback(self, msg):
        self.ros_gps = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'status': msg.status.status
        }

def mavlink_thread_func(results):
    """Read GPS from MAVLink directly"""
    try:
        # Connect to flight controller
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        print("MAVLink: Waiting for heartbeat...")
        master.wait_heartbeat()
        print(f"MAVLink: Connected to system {master.target_system}, component {master.target_component}")
        
        # Request GPS data stream
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            2,  # 2 Hz
            1   # start
        )
        
        count = 0
        while results['running'] and count < 20:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if msg:
                results['mavlink'] = {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt': msg.alt / 1000.0,
                    'relative_alt': msg.relative_alt / 1000.0
                }
                count += 1
    except Exception as e:
        print(f"MAVLink Error: {e}")
        results['mavlink_error'] = str(e)

def main():
    print("\n" + "="*60)
    print("GPS DATA COMPARISON: MAVLink Direct vs MAVROS ROS2")
    print("="*60 + "\n")
    
    # Initialize ROS2
    rclpy.init()
    node = GPSCompare()
    
    # Shared results
    results = {
        'running': True,
        'mavlink': None,
        'mavlink_error': None
    }
    
    # Start MAVLink thread
    mav_thread = threading.Thread(target=mavlink_thread_func, args=(results,), daemon=True)
    mav_thread.start()
    
    # Spin ROS2 for 10 seconds
    start_time = time.time()
    print("Collecting data for 10 seconds...\n")
    
    try:
        while time.time() - start_time < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        results['running'] = False
        
    # Display results
    print("\n" + "="*60)
    print("RESULTS:")
    print("="*60 + "\n")
    
    if results.get('mavlink_error'):
        print(f"❌ MAVLink Error: {results['mavlink_error']}\n")
        print("   This is expected if MAVROS is already using /dev/ttyACM0")
        print("   (Both MAVLink and MAVROS cannot use the same port simultaneously)\n")
    elif results['mavlink']:
        print("📡 MAVLink Direct:")
        print(f"   Latitude:  {results['mavlink']['lat']:.8f}")
        print(f"   Longitude: {results['mavlink']['lon']:.8f}")
        print(f"   Altitude:  {results['mavlink']['alt']:.2f} m")
        print(f"   Rel Alt:   {results['mavlink']['relative_alt']:.2f} m")
    else:
        print("❌ MAVLink: No data received\n")
    
    print()
    
    if node.ros_gps:
        print("🤖 MAVROS ROS2 Topic (/mavros/global_position/global):")
        print(f"   Latitude:  {node.ros_gps['lat']:.8f}")
        print(f"   Longitude: {node.ros_gps['lon']:.8f}")
        print(f"   Altitude:  {node.ros_gps['alt']:.2f} m")
        print(f"   Status:    {node.ros_gps['status']} (0=no fix, 1=fix, 2=SBAS, >=3=RTK)")
    else:
        print("❌ MAVROS: No data received from ROS2 topic\n")
        print("   Possible issues:")
        print("   - MAVROS node not running")
        print("   - QoS mismatch (need BEST_EFFORT)")
        print("   - No GPS connection to flight controller")
    
    print("\n" + "="*60 + "\n")
    
    if node.ros_gps and results['mavlink'] and not results.get('mavlink_error'):
        lat_diff = abs(node.ros_gps['lat'] - results['mavlink']['lat'])
        lon_diff = abs(node.ros_gps['lon'] - results['mavlink']['lon'])
        alt_diff = abs(node.ros_gps['alt'] - results['mavlink']['alt'])
        
        print("COMPARISON:")
        print(f"  Latitude difference:  {lat_diff:.9f}° ({lat_diff * 111000:.2f} m)")
        print(f"  Longitude difference: {lon_diff:.9f}° ({lon_diff * 111000:.2f} m)")
        print(f"  Altitude difference:  {alt_diff:.2f} m")
        
        if lat_diff < 0.0001 and lon_diff < 0.0001:
            print("\n✅ DATA MATCHES - Both sources reporting same GPS position!")
        else:
            print("\n⚠️  DATA DIFFERS - Possible sync issue or different GPS sources")
    
    print("="*60 + "\n")
    
    node.destroy_node()
    rclpy.shutdown()
    mav_thread.join(timeout=2)

if __name__ == '__main__':
    main()
