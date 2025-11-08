# Servo Test Configuration File
# Edit these values to customize your servo test parameters
# This file is sourced by the servo test scripts

# Servo channel to test (1-16, typically)
# Common channels: 1-8 for main motors/control surfaces, 9-16 for aux functions
SERVO_CHANNEL=10

# First PWM value in microseconds
# Typical servo range: 1000-2000 μs
# Center position: ~1500 μs  
# Use values appropriate for your servo/device
FIRST_PWM=600

# Second PWM value in microseconds
# Should be different from FIRST_PWM to observe movement
SECOND_PWM=1000

# Delay between PWM changes in seconds
# Allow time to observe servo movement
DELAY_SECONDS=2

# Service call timeout in seconds
# How long to wait for MAVROS to respond
TIMEOUT_SECONDS=5

# Rosbridge server port (usually 9090)
ROSBRIDGE_PORT=9090

# ============================================================================
# SAFETY NOTES:
# - Ensure the servo channel is properly configured in your autopilot
# - Verify no critical flight systems are connected to the test channel
# - PWM values should be within safe range for your servo/device
# - Test on the ground with propellers removed for safety
# ============================================================================

# ============================================================================
# COMMON SERVO PWM VALUES:
# - Standard servo: 1000-2000 μs (center ~1500 μs)
# - ESC throttle: 1000-2000 μs (1000=stop, 2000=full throttle)
# - Camera gimbal: varies by manufacturer
# - Landing gear: varies by system
# ============================================================================