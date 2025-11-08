#!/bin/bash

# Example: How to run servo tests with custom parameters
# This script demonstrates different ways to configure and run servo tests

echo "=== Servo Test Examples ==="
echo ""

echo "1. Using default parameters:"
echo "   ./run_servo_test.sh"
echo ""

echo "2. Using environment variables (one-time):"
echo "   SERVO_CHANNEL=9 FIRST_PWM=1200 SECOND_PWM=1800 ./run_servo_test.sh"
echo ""

echo "3. Using configuration file (persistent):"
echo "   Edit servo_test_config.sh, then run ./run_servo_test.sh"
echo ""

echo "4. Force specific test method:"
echo "   ./run_servo_test.sh --ros2     # Use ROS2 native"
echo "   ./run_servo_test.sh --bridge   # Use rosbridge"
echo ""

echo "5. Custom test examples:"
echo ""

echo "   # Test servo 9 with ESC range (1000-2000)"
echo "   SERVO_CHANNEL=9 FIRST_PWM=1000 SECOND_PWM=2000 DELAY_SECONDS=3 ./run_servo_test.sh"
echo ""

echo "   # Test servo 11 with gimbal range"
echo "   SERVO_CHANNEL=11 FIRST_PWM=1200 SECOND_PWM=1800 DELAY_SECONDS=2 ./run_servo_test.sh"
echo ""

echo "   # Test servo 12 with landing gear (short pulses)"
echo "   SERVO_CHANNEL=12 FIRST_PWM=1100 SECOND_PWM=1900 DELAY_SECONDS=1 ./run_servo_test.sh"
echo ""

echo "6. Quick parameter override examples:"

# Example 1: Test servo 9
echo ""
echo "Example 1: Testing servo 9 with standard servo range"
read -p "Run this test? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    SERVO_CHANNEL=9 FIRST_PWM=1000 SECOND_PWM=2000 DELAY_SECONDS=3 ./run_servo_test.sh
fi

# Example 2: Test servo 11
echo ""
echo "Example 2: Testing servo 11 with camera gimbal range"
read -p "Run this test? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    SERVO_CHANNEL=11 FIRST_PWM=1200 SECOND_PWM=1800 DELAY_SECONDS=2 ./run_servo_test.sh
fi

# Example 3: Test servo 10 with custom range
echo ""
echo "Example 3: Testing servo 10 with custom range"
read -p "Run this test? (y/n): " -n 1 -r  
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    SERVO_CHANNEL=10 FIRST_PWM=600 SECOND_PWM=1000 DELAY_SECONDS=2 ./run_servo_test.sh
fi

echo ""
echo "=== Examples Complete ==="
echo ""
echo "To create your own test:"
echo "1. Copy servo_test_config.sh to servo_test_config_custom.sh"
echo "2. Edit the custom file with your parameters"
echo "3. Run: cp servo_test_config_custom.sh servo_test_config.sh"
echo "4. Run: ./run_servo_test.sh"