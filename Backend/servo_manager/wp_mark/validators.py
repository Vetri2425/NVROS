"""
WP_MARK Parameter Validation Module
Validates configuration parameters for waypoint-based servo control missions
"""

from typing import Dict, Tuple, Any
from dataclasses import dataclass


@dataclass
class WPMarkConfig:
    """WP_MARK mission configuration data class"""
    delay_before_start: float
    pwm_start: int
    delay_before_stop: float
    pwm_stop: int
    delay_after_stop: float
    servo_channel: int
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert config to dictionary"""
        return {
            'delay_before_start': self.delay_before_start,
            'pwm_start': self.pwm_start,
            'delay_before_stop': self.delay_before_stop,
            'pwm_stop': self.pwm_stop,
            'delay_after_stop': self.delay_after_stop,
            'servo_channel': self.servo_channel
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'WPMarkConfig':
        """Create config from dictionary"""
        return cls(
            delay_before_start=float(data['delay_before_start']),
            pwm_start=int(data['pwm_start']),
            delay_before_stop=float(data['delay_before_stop']),
            pwm_stop=int(data['pwm_stop']),
            delay_after_stop=float(data['delay_after_stop']),
            servo_channel=int(data.get('servo_channel', 10))  # Default to 10 if not provided
        )


def validate_parameters(params: Dict[str, Any]) -> Tuple[bool, str, WPMarkConfig | None]:
    """
    Validate WP_MARK configuration parameters
    
    Args:
        params: Dictionary containing mission parameters
        
    Returns:
        Tuple of (is_valid, error_message, config_object)
        - is_valid: True if all parameters are valid
        - error_message: Error description (empty if valid)
        - config_object: WPMarkConfig instance (None if invalid)
    """
    
    # Required fields
    required = [
        'delay_before_start',
        'pwm_start',
        'delay_before_stop',
        'pwm_stop',
        'delay_after_stop'
    ]
    
    # Check for missing required fields
    for field in required:
        if field not in params:
            return False, f"Missing required field: {field}", None
    
    try:
        # Extract and convert parameters
        delay_before_start = float(params['delay_before_start'])
        pwm_start = int(params['pwm_start'])
        delay_before_stop = float(params['delay_before_stop'])
        pwm_stop = int(params['pwm_stop'])
        delay_after_stop = float(params['delay_after_stop'])
        servo_channel = int(params.get('servo_channel', 10))  # Default to 10 if not provided
        
    except (ValueError, TypeError) as e:
        return False, f"Invalid parameter type: {str(e)}", None
    
    # Validate delay_before_start
    if not (0 <= delay_before_start <= 60):
        return False, "delay_before_start must be between 0-60 seconds", None
    
    # Validate delay_before_stop (spraying duration)
    if not (0 <= delay_before_stop <= 60):
        return False, "delay_before_stop must be between 0-60 seconds", None
    
    # Validate delay_after_stop
    if not (0 <= delay_after_stop <= 60):
        return False, "delay_after_stop must be between 0-60 seconds", None
    
    # Validate pwm_start (changed range from 1000-2000 to 100-2000)
    if not (100 <= pwm_start <= 2000):
        return False, "pwm_start must be between 100-2000 microseconds", None
    
    # Validate pwm_stop (changed range from 1000-2000 to 100-2000)
    if not (100 <= pwm_stop <= 2000):
        return False, "pwm_stop must be between 100-2000 microseconds", None
    
    # Validate servo_channel
    is_valid_channel, channel_error = validate_servo_channel(servo_channel)
    if not is_valid_channel:
        return False, channel_error, None
    
    # Additional validation: pwm_start should typically be different from pwm_stop
    if pwm_start == pwm_stop:
        return False, "pwm_start and pwm_stop should have different values for servo control", None
    
    # Create validated config object
    config = WPMarkConfig(
        delay_before_start=delay_before_start,
        pwm_start=pwm_start,
        delay_before_stop=delay_before_stop,
        pwm_stop=pwm_stop,
        delay_after_stop=delay_after_stop,
        servo_channel=servo_channel
    )
    
    return True, "", config


def validate_servo_channel(channel: int) -> Tuple[bool, str]:
    """
    Validate servo output channel number
    
    Args:
        channel: Servo channel number (typically 1-16 for ArduPilot)
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    if not isinstance(channel, int):
        return False, "Servo channel must be an integer"
    
    if not (1 <= channel <= 16):
        return False, "Servo channel must be between 1-16"
    
    return True, ""


def validate_waypoint_threshold(threshold: float) -> Tuple[bool, str]:
    """
    Validate waypoint arrival threshold in meters
    
    Args:
        threshold: Distance threshold in meters
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    try:
        threshold = float(threshold)
    except (ValueError, TypeError):
        return False, "Waypoint threshold must be a number"
    
    if not (0.1 <= threshold <= 100):
        return False, "Waypoint threshold must be between 0.1-100 meters"
    
    return True, ""


def validate_timeout(timeout: float) -> Tuple[bool, str]:
    """
    Validate timeout value in seconds
    
    Args:
        timeout: Timeout in seconds
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    try:
        timeout = float(timeout)
    except (ValueError, TypeError):
        return False, "Timeout must be a number"
    
    if not (1 <= timeout <= 3600):  # 1 second to 1 hour
        return False, "Timeout must be between 1-3600 seconds"
    
    return True, ""
