"""
Test Suite for Servo Verification and Enhanced Safety Features
Tests: Servo feedback, verification, mode switching, and pause-on-failure logic
"""

import unittest
import time
from unittest.mock import Mock, patch, MagicMock
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, RCOut
from mavros_msgs.srv import SetMode

from mission_controller import WPMarkMissionNode
from validators import WPMarkConfig


class TestServoVerification(unittest.TestCase):
    """Test servo verification and feedback mechanisms"""
    
    def setUp(self):
        """Setup test configuration"""
        self.config = WPMarkConfig(
            waypoint_file="test.plan",
            servo_channel=10,
            pwm_start=1900,
            pwm_stop=1100,
            delay_before_start=0.1,
            delay_before_stop=0.5,
            delay_after_stop=0.1
        )
    
    @patch('rclpy.init')
    @patch('rclpy.create_node')
    def test_servo_output_callback(self, mock_create_node, mock_init):
        """Test servo output feedback callback updates state correctly"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            node.settings = {'servo_channel': 10}
            node.state_lock = threading.Lock()
            node.current_servo_outputs = None
            node.last_servo_update = 0.0
            
            # Create RCOut message
            msg = RCOut()
            msg.channels = [1500] * 16
            msg.channels[9] = 1900  # Channel 10 (0-indexed)
            
            # Process callback
            node._servo_output_callback(msg)
            
            # Verify state updated
            self.assertIsNotNone(node.current_servo_outputs)
            self.assertEqual(node.current_servo_outputs.channels[9], 1900)
            self.assertGreater(node.last_servo_update, 0)
    
    @patch('rclpy.init')
    def test_verify_servo_state_success(self, mock_init):
        """Test servo verification succeeds when PWM matches"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            node.settings = {'servo_channel': 10}
            node.state_lock = threading.Lock()
            
            # Setup servo output data
            node.current_servo_outputs = RCOut()
            node.current_servo_outputs.channels = [1500] * 16
            node.current_servo_outputs.channels[9] = 1900  # Channel 10
            node.last_servo_update = time.time()
            
            # Verify servo ON (1900 PWM)
            result = node.verify_servo_state(1900, tolerance=50)
            
            self.assertTrue(result)
    
    @patch('rclpy.init')
    def test_verify_servo_state_failure(self, mock_init):
        """Test servo verification fails when PWM doesn't match"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            node.settings = {'servo_channel': 10}
            node.state_lock = threading.Lock()
            
            # Setup servo output data
            node.current_servo_outputs = RCOut()
            node.current_servo_outputs.channels = [1500] * 16
            node.current_servo_outputs.channels[9] = 1100  # Channel 10 is OFF
            node.last_servo_update = time.time()
            
            # Try to verify servo ON (1900 PWM) but it's at 1100
            result = node.verify_servo_state(1900, tolerance=50)
            
            self.assertFalse(result)
    
    @patch('rclpy.init')
    def test_verify_servo_state_stale_data(self, mock_init):
        """Test servo verification fails with stale data"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            node.settings = {'servo_channel': 10}
            node.state_lock = threading.Lock()
            
            # Setup stale servo output data
            node.current_servo_outputs = RCOut()
            node.current_servo_outputs.channels = [1500] * 16
            node.current_servo_outputs.channels[9] = 1900
            node.last_servo_update = time.time() - 2.0  # 2 seconds old
            
            # Verification should fail due to stale data
            result = node.verify_servo_state(1900, tolerance=50)
            
            self.assertFalse(result)


class TestFlightModeSwitch(unittest.TestCase):
    """Test flight mode switching functionality"""
    
    def setUp(self):
        """Setup test configuration"""
        self.config = WPMarkConfig(
            waypoint_file="test.plan",
            servo_channel=10,
            pwm_start=1900,
            pwm_stop=1100
        )
    
    @patch('rclpy.init')
    @patch('rclpy.spin_until_future_complete')
    def test_set_flight_mode_success(self, mock_spin, mock_init):
        """Test successful flight mode change to HOLD"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            
            # Mock SetMode service client
            mock_client = Mock()
            mock_client.wait_for_service = Mock(return_value=True)
            mock_future = Mock()
            mock_response = SetMode.Response()
            mock_response.mode_sent = True
            mock_future.result = Mock(return_value=mock_response)
            mock_client.call_async = Mock(return_value=mock_future)
            node.set_mode_client = mock_client
            
            # Test mode change
            result = node.set_flight_mode('HOLD')
            
            self.assertTrue(result)
            mock_client.call_async.assert_called_once()
    
    @patch('rclpy.init')
    @patch('rclpy.spin_until_future_complete')
    def test_set_flight_mode_rejected(self, mock_spin, mock_init):
        """Test flight mode change rejected by flight controller"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            
            # Mock SetMode service client
            mock_client = Mock()
            mock_client.wait_for_service = Mock(return_value=True)
            mock_future = Mock()
            mock_response = SetMode.Response()
            mock_response.mode_sent = False  # Rejected
            mock_future.result = Mock(return_value=mock_response)
            mock_client.call_async = Mock(return_value=mock_future)
            node.set_mode_client = mock_client
            
            # Test mode change
            result = node.set_flight_mode('HOLD')
            
            self.assertFalse(result)


class TestServoWithRetry(unittest.TestCase):
    """Test servo ON/OFF with retry and verification"""
    
    def setUp(self):
        """Setup test configuration"""
        self.config = WPMarkConfig(
            waypoint_file="test.plan",
            servo_channel=10,
            pwm_start=1900,
            pwm_stop=1100
        )
    
    @patch('rclpy.init')
    @patch('time.sleep')  # Mock sleep to speed up tests
    def test_servo_on_with_retry_success_first_attempt(self, mock_sleep, mock_init):
        """Test servo ON succeeds on first attempt"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            node.settings = {'spray_pwm_on': 1900, 'servo_channel': 10}
            
            # Mock successful servo command and verification
            node.set_servo = Mock(return_value=True)
            node.verify_servo_state = Mock(return_value=True)
            
            # Test servo ON
            result = node.servo_on_with_retry()
            
            self.assertTrue(result)
            node.set_servo.assert_called_once_with(1900, 10)
            node.verify_servo_state.assert_called_once_with(1900)
    
    @patch('rclpy.init')
    @patch('time.sleep')
    def test_servo_on_with_retry_success_third_attempt(self, mock_sleep, mock_init):
        """Test servo ON succeeds on third attempt"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            node.settings = {'spray_pwm_on': 1900, 'servo_channel': 10}
            
            # Mock: first 2 attempts fail, 3rd succeeds
            node.set_servo = Mock(return_value=True)
            node.verify_servo_state = Mock(side_effect=[False, False, True])
            
            # Test servo ON
            result = node.servo_on_with_retry()
            
            self.assertTrue(result)
            self.assertEqual(node.verify_servo_state.call_count, 3)
    
    @patch('rclpy.init')
    @patch('time.sleep')
    def test_servo_on_with_retry_all_attempts_fail(self, mock_sleep, mock_init):
        """Test servo ON fails after all retry attempts"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            node.settings = {'spray_pwm_on': 1900, 'servo_channel': 10}
            
            # Mock: all attempts fail
            node.set_servo = Mock(return_value=True)
            node.verify_servo_state = Mock(return_value=False)
            
            # Test servo ON
            result = node.servo_on_with_retry()
            
            self.assertFalse(result)
            self.assertEqual(node.verify_servo_state.call_count, 3)
    
    @patch('rclpy.init')
    @patch('time.sleep')
    def test_servo_off_with_safety_emergency_pwm(self, mock_sleep, mock_init):
        """Test servo OFF sends emergency PWM=0 after all retries fail"""
        with patch.object(WPMarkMissionNode, '__init__', lambda x, y: None):
            node = WPMarkMissionNode(self.config)
            node.logger = Mock()
            node.settings = {'spray_pwm_off': 1100, 'servo_channel': 10}
            
            # Mock: normal OFF fails, emergency PWM succeeds
            node.set_servo = Mock(return_value=True)
            node.verify_servo_state = Mock(side_effect=[False, False, False, True])
            
            # Test servo OFF
            result = node.servo_off_with_safety()
            
            # Should fail because it's an emergency scenario
            self.assertFalse(result)
            # Should have called set_servo 4 times: 3 retries + 1 emergency
            self.assertEqual(node.set_servo.call_count, 4)
            # Last call should be emergency PWM=0
            node.set_servo.assert_called_with(0, 10)


class TestMissionPauseLogic(unittest.TestCase):
    """Test mission pause behavior on failures"""
    
    def setUp(self):
        """Setup test configuration"""
        self.config = WPMarkConfig(
            waypoint_file="test.plan",
            servo_channel=10,
            pwm_start=1900,
            pwm_stop=1100,
            delay_before_start=0.0,
            delay_before_stop=0.1,
            delay_after_stop=0.0
        )
    
    @patch('rclpy.init')
    def test_mission_pauses_on_mode_change_failure(self, mock_init):
        """Test mission pauses if HOLD mode change fails"""
        # This would be an integration test requiring full mission setup
        # Placeholder for comprehensive testing
        pass
    
    @patch('rclpy.init')
    def test_mission_pauses_on_servo_on_failure(self, mock_init):
        """Test mission pauses if servo ON fails all retries"""
        # This would be an integration test requiring full mission setup
        # Placeholder for comprehensive testing
        pass
    
    @patch('rclpy.init')
    def test_mission_pauses_on_servo_off_failure(self, mock_init):
        """Test mission pauses if servo OFF fails all retries"""
        # This would be an integration test requiring full mission setup
        # Placeholder for comprehensive testing
        pass


if __name__ == '__main__':
    # Run tests
    import threading  # Need for Lock in tests
    unittest.main(verbosity=2)
