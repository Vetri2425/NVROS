#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from mavros_msgs.msg import State, GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MAV_STATE_LABELS = {
    0: 'UNINIT',
    1: 'BOOT',
    2: 'CALIBRATING',
    3: 'STANDBY',
    4: 'ACTIVE',
    5: 'CRITICAL',
    6: 'EMERGENCY',
    7: 'POWEROFF',
    8: 'TERMINATION',
}


def _map_mav_state(value: int) -> str:
    return MAV_STATE_LABELS.get(int(value), f'UNKNOWN({int(value)})')

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        
        # Configure QoS profile for reliable, real-time data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to MAVROS topics
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )
        
        self.position_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.position_callback,
            qos_profile
        )
        
        # Create a publisher for combined telemetry data
        self.telemetry_pub = self.create_publisher(
            String,
            '/nrp/telemetry',
            qos_profile
        )
        
        # Initialize telemetry data structure
        self.telemetry_data = {
            'state': {
                'mode': '',
                'armed': False,
                'system_status': 'STANDBY',
                'connected': False,
                'heartbeat_ts': 0
            },
            'position': {
                'latitude': 0.0,
                'longitude': 0.0,
                'altitude': 0.0
            }
        }
        
        # Create timer for publishing telemetry data
        self.timer = self.create_timer(0.1, self.publish_telemetry)  # 100ms interval
        self.get_logger().info('Telemetry node initialized')

    def state_callback(self, msg):
        self.telemetry_data['state']['mode'] = (msg.mode or '').upper()
        self.telemetry_data['state']['armed'] = msg.armed
        self.telemetry_data['state']['connected'] = msg.connected
        self.telemetry_data['state']['system_status'] = _map_mav_state(msg.system_status)
        self.telemetry_data['state']['heartbeat_ts'] = self.get_clock().now().nanoseconds // 1000000  # Convert to milliseconds
        
    def position_callback(self, msg):
        self.telemetry_data['position']['latitude'] = msg.latitude
        self.telemetry_data['position']['longitude'] = msg.longitude
        self.telemetry_data['position']['altitude'] = msg.altitude
        
    def publish_telemetry(self):
        # Create message and publish
        msg = String()
        msg.data = json.dumps(self.telemetry_data)
        self.telemetry_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    telemetry_node = TelemetryNode()
    
    try:
        rclpy.spin(telemetry_node)
    except KeyboardInterrupt:
        pass
    finally:
        telemetry_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
