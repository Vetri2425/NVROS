#!/bin/bash

# Servo Channel Test Runner
# Provides options to run servo test using different methods

# ============================================================================
# USER CONFIGURABLE VARIABLES - MODIFY THESE TO CHANGE TEST PARAMETERS
# ============================================================================

# Load configuration from external file if it exists
CONFIG_FILE="servo_test_config.sh"
if [ -f "$CONFIG_FILE" ]; then
    echo "Loading configuration from $CONFIG_FILE..."
    source "$CONFIG_FILE"
else
    echo "Using default configuration (create $CONFIG_FILE to customize)..."
fi

# Default values (used if not set in config file)
SERVO_CHANNEL=${SERVO_CHANNEL:-10}
FIRST_PWM=${FIRST_PWM:-1750}
SECOND_PWM=${SECOND_PWM:-2300}
DELAY_SECONDS=${DELAY_SECONDS:-3}
TIMEOUT_SECONDS=${TIMEOUT_SECONDS:-5}
ROSBRIDGE_PORT=${ROSBRIDGE_PORT:-9090}

# ============================================================================
# END USER CONFIGURABLE VARIABLES
# ============================================================================

echo "=== Servo Channel $SERVO_CHANNEL Test Runner ==="
echo ""

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if ROS2 is available
check_ros2() {
    if command_exists ros2; then
        echo "✓ ROS2 detected"
        return 0
    else
        echo "✗ ROS2 not found"
        return 1
    fi
}

# Function to check if rosbridge is running
check_rosbridge() {
    if netstat -tln | grep -q ":$ROSBRIDGE_PORT "; then
        echo "✓ Rosbridge server detected (port $ROSBRIDGE_PORT)"
        return 0
    else
        echo "✗ Rosbridge server not detected (port $ROSBRIDGE_PORT)"
        return 1
    fi
}

# Function to check MAVROS
check_mavros() {
    if command_exists ros2 && ros2 topic list 2>/dev/null | grep -q "/mavros/"; then
        echo "✓ MAVROS topics detected"
        return 0
    else
        echo "✗ MAVROS topics not detected"
        return 1
    fi
}

echo "System Check:"
echo "============="

ros2_available=false
rosbridge_available=false
mavros_available=false

if check_ros2; then
    ros2_available=true
fi

if check_rosbridge; then
    rosbridge_available=true
fi

if check_mavros; then
    mavros_available=true
fi

echo ""

# Determine which test to run
if [ "$1" = "--ros2" ] || [ "$1" = "-r" ]; then
    echo "Running ROS2 native version..."
    echo "Test parameters: Channel=$SERVO_CHANNEL, PWM1=$FIRST_PWM, PWM2=$SECOND_PWM, Delay=${DELAY_SECONDS}s"
    if [ "$ros2_available" = true ] && [ "$mavros_available" = true ]; then
        SERVO_CHANNEL=$SERVO_CHANNEL FIRST_PWM=$FIRST_PWM SECOND_PWM=$SECOND_PWM DELAY_SECONDS=$DELAY_SECONDS TIMEOUT_SECONDS=$TIMEOUT_SECONDS python3 test_servo_10_ros2.py
    else
        echo "❌ ROS2 and MAVROS are required for this test"
        exit 1
    fi
elif [ "$1" = "--bridge" ] || [ "$1" = "-b" ]; then
    echo "Running rosbridge version..."
    echo "Test parameters: Channel=$SERVO_CHANNEL, PWM1=$FIRST_PWM, PWM2=$SECOND_PWM, Delay=${DELAY_SECONDS}s"
    if [ "$rosbridge_available" = true ]; then
        SERVO_CHANNEL=$SERVO_CHANNEL FIRST_PWM=$FIRST_PWM SECOND_PWM=$SECOND_PWM DELAY_SECONDS=$DELAY_SECONDS TIMEOUT_SECONDS=$TIMEOUT_SECONDS python3 test_servo_10.py
    else
        echo "❌ Rosbridge server is required for this test"
        echo "Start it with: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
        exit 1
    fi
elif [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  --ros2, -r     Use ROS2 native service calls"
    echo "  --bridge, -b   Use rosbridge websocket (default)"
    echo "  --help, -h     Show this help message"
    echo ""
    echo "Test Details:"
    echo "- Servo Channel: 10"
    echo "- First PWM: 600 microseconds"
    echo "- Delay: 2 seconds"  
    echo "- Second PWM: 1000 microseconds"
    echo ""
    echo "Prerequisites:"
    echo "- MAVROS running"
    echo "- For --bridge: rosbridge_server on port $ROSBRIDGE_PORT"
    echo "- For --ros2: ROS2 environment sourced"
    exit 0
else
    # Auto-select best available option
    echo "Auto-selecting test method..."
    echo "Test parameters: Channel=$SERVO_CHANNEL, PWM1=$FIRST_PWM, PWM2=$SECOND_PWM, Delay=${DELAY_SECONDS}s"
    
    if [ "$rosbridge_available" = true ]; then
        echo "Using rosbridge version (recommended)..."
        SERVO_CHANNEL=$SERVO_CHANNEL FIRST_PWM=$FIRST_PWM SECOND_PWM=$SECOND_PWM DELAY_SECONDS=$DELAY_SECONDS TIMEOUT_SECONDS=$TIMEOUT_SECONDS python3 test_servo_10.py
    elif [ "$ros2_available" = true ] && [ "$mavros_available" = true ]; then
        echo "Using ROS2 native version..."
        SERVO_CHANNEL=$SERVO_CHANNEL FIRST_PWM=$FIRST_PWM SECOND_PWM=$SECOND_PWM DELAY_SECONDS=$DELAY_SECONDS TIMEOUT_SECONDS=$TIMEOUT_SECONDS python3 test_servo_10_ros2.py
    else
        echo "❌ No suitable test method available"
        echo ""
        echo "Requirements:"
        echo "- MAVROS must be running"
        echo "- Either rosbridge_server (port $ROSBRIDGE_PORT) OR ROS2 environment"
        echo ""
        echo "To start rosbridge:"
        echo "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
        exit 1
    fi
fi                                                                                              
