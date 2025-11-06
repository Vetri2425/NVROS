#!/usr/bin/env python3
"""
Test script to check available MAVROS servo output topics
and verify channel availability for servo channel 10
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
import time


class ServoTopicTester(Node):
    def __init__(self):
        super().__init__('servo_topic_tester')
        
        self.get_logger().info("=== Servo Topic Tester ===")
        self.get_logger().info("Testing MAVROS servo output topics...")
        
        # Subscribe to /mavros/rc/out
        self.rc_out_sub = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.rc_out_callback,
            10
        )
        
        self.received_rc_out = False
        
    def rc_out_callback(self, msg):
        """Check what channels are available in /mavros/rc/out"""
        if not self.received_rc_out:
            self.received_rc_out = True
            self.get_logger().info("\n=== /mavros/rc/out Message ===")
            self.get_logger().info(f"Number of channels: {len(msg.channels)}")
            self.get_logger().info(f"Channels: {msg.channels}")
            
            # Check if channel 10 exists
            if len(msg.channels) >= 10:
                self.get_logger().info(f"✓ Channel 10 (index 9) value: {msg.channels[9]} PWM")
            else:
                self.get_logger().warning(f"✗ Channel 10 NOT available in /mavros/rc/out")
                self.get_logger().warning(f"  Only {len(msg.channels)} channels present")
                self.get_logger().warning(f"  Typically /mavros/rc/out has 8-16 channels")
            
            # Show all channels
            self.get_logger().info("\nAll channels:")
            for i, pwm in enumerate(msg.channels):
                self.get_logger().info(f"  Channel {i+1}: {pwm} PWM")


def main():
    print("\n" + "="*60)
    print("MAVROS Servo Output Topic Tester")
    print("="*60)
    print("\nThis script will:")
    print("1. Subscribe to /mavros/rc/out")
    print("2. Show how many channels are available")
    print("3. Check if servo channel 10 exists")
    print("\nWaiting for messages (10 seconds timeout)...\n")
    
    rclpy.init()
    node = ServoTopicTester()
    
    # Spin for 10 seconds to receive messages
    start_time = time.time()
    timeout = 10.0
    
    try:
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if node.received_rc_out:
                print("\n✓ Successfully received /mavros/rc/out message")
                break
        
        if not node.received_rc_out:
            print("\n✗ Timeout: No messages received on /mavros/rc/out")
            print("\nPossible reasons:")
            print("  1. MAVROS not running")
            print("  2. Flight controller not connected")
            print("  3. RC_CHANNELS/SERVO_OUTPUT_RAW not being streamed")
            print("\nTo enable, check:")
            print("  - Flight controller parameter: SR0_RC_CHAN (stream rate)")
            print("  - MAVROS launch: ensure proper stream rate configuration")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    print("\n" + "="*60)
    print("RECOMMENDATIONS:")
    print("="*60)
    
    if node.received_rc_out:
        print("\n✓ /mavros/rc/out is available")
        print("\nFor ArduPilot/PX4:")
        print("  - /mavros/rc/out typically has 16 channels")
        print("  - Channels 1-8: Main outputs (MAIN1-MAIN8)")
        print("  - Channels 9-16: Auxiliary outputs (AUX1-AUX8)")
        print("  - Servo channel 10 = AUX2 = index 9 in array")
        print("\nYour current implementation should work IF:")
        print("  - msg.channels array has at least 10 elements")
        print("  - Flight controller streams SERVO_OUTPUT_RAW or RC_CHANNELS_RAW")
    else:
        print("\n✗ /mavros/rc/out not available")
        print("\nAlternative approaches:")
        print("  1. Use MAVLink COMMAND_ACK to verify command was accepted")
        print("  2. Use mission item status")
        print("  3. Rely on timeout-based verification")
        print("  4. Enable SERVO_OUTPUT_RAW streaming on flight controller")
    
    print("\n")


if __name__ == '__main__':
    main()
