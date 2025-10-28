#!/bin/bash

cleanup() {
    echo -e "${BLUE}Cleaning up...${NC}"
    for pid in "$ROSBRIDGE_PID" "$MAVROS_PID" "$BACKEND_PID"; do
        if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    if [[ -n "${NPM_PID:-}" ]] && kill -0 "$NPM_PID" 2>/dev/null; then
        kill "$NPM_PID" 2>/dev/null || true
        wait "$NPM_PID" 2>/dev/null || true
    fi
}

handle_exit() {
    local code=$1
    trap - EXIT INT TERM
    cleanup
    exit "$code"
}

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Starting NRP Application...${NC}"

# 1. Source ROS 2 Humble
echo -e "${GREEN}1. Sourcing ROS 2 Humble...${NC}"
source /opt/ros/humble/setup.bash

# Track npm PID for cleanup
NPM_PID=""
BACKEND_PID=""
APP_STATUS=0

# Set up cleanup on script exit or interruption
trap 'handle_exit $?' EXIT
trap 'handle_exit 130' INT
trap 'handle_exit 143' TERM

# 2. Start rosbridge_server in background
echo -e "${GREEN}2. Starting rosbridge_server...${NC}"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!
sleep 5  # Wait for rosbridge to initialize

# 3. Start MAVROS in background
echo -e "${GREEN}3. Starting MAVROS...${NC}"
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 gcs_url:=tcp-l://0.0.0.0:5761 &
MAVROS_PID=$!
sleep 5  # Wait for MAVROS to initialize

# 4. Start the backend API
echo -e "${GREEN}4. Starting backend server...${NC}"
python3 -m Backend.server &
BACKEND_PID=$!
sleep 2  # Allow backend to bind to the port

# 5. Start the application
echo -e "${GREEN}5. Starting NRP frontend...${NC}"
npm run dev:frontend &
NPM_PID=$!
wait "$NPM_PID"
APP_STATUS=$?
exit "$APP_STATUS"
