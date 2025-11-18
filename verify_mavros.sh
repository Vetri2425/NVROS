#!/bin/bash
# Quick verification script for MAVROS and mission controller

echo "=== MAVROS & Mission Controller Status ==="
echo ""

source /opt/ros/humble/setup.bash

echo "1. Checking ROS nodes..."
ros2 node list | grep -E "(mavros|mission)" && echo "✅ Nodes running" || echo "❌ Nodes missing"
echo ""

echo "2. Checking MAVROS services..."
ros2 service list | grep -E "(set_mode|arming|command)" | head -3 && echo "✅ Services available" || echo "❌ Services missing"
echo ""

echo "3. Testing mode change to HOLD..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'HOLD'}" 2>&1 | grep "mode_sent: True" && echo "✅ Mode control working" || echo "❌ Mode control failed"
echo ""

echo "4. Checking mission services..."
ros2 service list | grep "mission" | head -5 && echo "✅ Mission services ready" || echo "❌ Mission services missing"
echo ""

echo "=== Status Summary ==="
echo "If all checks show ✅, the system is ready!"
echo "The rover should now:"
echo "  - Stay still in HOLD mode"
echo "  - Respond to RC in MANUAL mode"
echo "  - Accept mission commands in AUTO mode"
