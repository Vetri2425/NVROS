#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
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

def _map_gps_fix_type(fix_type: int) -> int:
    """Map GPS fix type to standard values
    0: No fix
    1: GPS fix
    2: DGPS
    3: RTK Float
    4: RTK Fixed
    """
    if fix_type < 0:
        return 0
    elif fix_type == 0:
        return 1
    elif fix_type == 1:
        return 2
    elif fix_type == 2:
        return 2
    return fix_type

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

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

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/global_position/raw/gps_vel',
            self.velocity_callback,
            qos_profile
        )

        # Subscribe to battery state published by MAVROS
        try:
            battery_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.battery_sub = self.create_subscription(
                BatteryState,
                '/mavros/battery',
                self.battery_callback,
                battery_qos
            )
        except Exception:
            # If BatteryState type isn't available or QoS fails, skip battery subscription
            self.get_logger().warning('BatteryState subscription unavailable; battery telemetry will be absent')

        self.telemetry_pub = self.create_publisher(
            String,
            '/nrp/telemetry',
            qos_profile
        )

        self.telemetry_data = {
            'state': {
                'mode': '',
                'armed': False,
                'system_status': 'STANDBY',
                'connected': False,
                'heartbeat_ts': 0
            },
            'global': {
                'latitude': 0.0,
                'longitude': 0.0,
                'altitude': 0.0,
                'vel': 0.0,
                'satellites_visible': 0
            },
            'rtk': {
                'fix_type': 0,
                'baseline_age': 0,
                'base_linked': False
            }
            ,
            'battery': {
                'percentage': None,
                'voltage': 0.0,
                'current': 0.0
            }
        }

        self.timer = self.create_timer(0.1, self.publish_telemetry)
        self.get_logger().info('Telemetry node initialized with GPS/RTK support')

    def state_callback(self, msg):
        self.telemetry_data['state']['mode'] = (msg.mode or '').upper()
        self.telemetry_data['state']['armed'] = msg.armed
        self.telemetry_data['state']['connected'] = msg.connected
        self.telemetry_data['state']['system_status'] = _map_mav_state(msg.system_status)
        self.telemetry_data['state']['heartbeat_ts'] = self.get_clock().now().nanoseconds // 1000000

    def position_callback(self, msg):
        self.telemetry_data['global']['latitude'] = msg.latitude
        self.telemetry_data['global']['longitude'] = msg.longitude
        self.telemetry_data['global']['altitude'] = msg.altitude
        
        if hasattr(msg, 'status'):
            self.telemetry_data['global']['satellites_visible'] = int(msg.status.service) if msg.status.service >= 0 else 0
            fix_type = _map_gps_fix_type(msg.status.status)
            self.telemetry_data['rtk']['fix_type'] = fix_type
            self.telemetry_data['rtk']['base_linked'] = fix_type >= 3

    def velocity_callback(self, msg):
        import math
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.telemetry_data['global']['vel'] = math.sqrt(vx*vx + vy*vy)

    def battery_callback(self, msg):
        try:
            import math
            # BatteryState.percentage is typically 0.0-1.0 (fraction)
            pct = msg.percentage if hasattr(msg, 'percentage') else None
            # Debug log to confirm callback is invoked and contents
            try:
                self.get_logger().info(f"Battery callback received: pct={pct}, volt={getattr(msg,'voltage',None)}, cur={getattr(msg,'current',None)}")
            except Exception:
                pass
            # Only set percentage if it's a valid number (not NaN or Inf)
            if pct is not None and not math.isnan(pct) and not math.isinf(pct):
                self.telemetry_data['battery']['percentage'] = float(pct)
            self.telemetry_data['battery']['voltage'] = float(msg.voltage) if hasattr(msg, 'voltage') and not math.isnan(msg.voltage) else 0.0
            self.telemetry_data['battery']['current'] = float(msg.current) if hasattr(msg, 'current') and not math.isnan(msg.current) else 0.0
        except Exception:
            pass

    def publish_telemetry(self):
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
