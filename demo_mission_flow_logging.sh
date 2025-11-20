#!/bin/bash
# Demonstration of Mission Controller Full Flow Logging
# This script shows all logs with timestamps including:
# - Waypoint reception
# - Start command
# - Waypoint push to Pixhawk
# - Mode changes
# - 2Hz status updates during navigation

SERVER="http://localhost:5000"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "================================================================"
echo "          MISSION CONTROLLER FULL FLOW DEMONSTRATION"
echo "================================================================"
echo ""
echo "This demo will:"
echo "  1. Load a mission with 2 waypoints"
echo "  2. Start the mission"
echo "  3. Monitor ALL logs in journalctl including:"
echo "     - Command reception (COMMAND RECEIVED)"
echo "     - Waypoint validation"
echo "     - Waypoint push to Pixhawk"
echo "     - Arming sequence"
echo "     - Mode changes (AUTO, HOLD)"
echo "     - 2Hz status updates during navigation"
echo "     - Waypoint reached events"
echo "     - Hold periods"
echo "     - Mission completion"
echo ""
echo "================================================================"
echo ""

# Function to view logs
view_mission_logs() {
    echo ""
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}Mission Controller Logs (since last minute):${NC}"
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
    sudo journalctl -u nrp-service.service --since "1 minute ago" --no-pager | \
        grep "MISSION_CONTROLLER" | \
        grep --color=always -E "COMMAND RECEIVED|Received.*waypoint|Validating|MISSION LOADED|START command|PUSHING WAYPOINT|uploaded|ARMING|ARMED|MODE CHANGE|Setting.*mode|Status: WP|reached|HOLD|COMPLETED|$"
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
}

# Check if server is running
echo -e "${YELLOW}Checking if server is running...${NC}"
if ! curl -s "$SERVER/api/health" > /dev/null 2>&1; then
    echo -e "${RED}Error: Server is not running at $SERVER${NC}"
    echo "Please wait for the service to start completely."
    exit 1
fi
echo -e "${GREEN}✓ Server is running${NC}"
echo ""

# Step 1: Load Mission
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}STEP 1: Loading Mission with Waypoints${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"

curl -s -X POST "$SERVER/api/mission/command" \
  -H "Content-Type: application/json" \
  -d '{
    "command": "load_mission",
    "waypoints": [
      {"lat": 13.072100, "lng": 80.261950, "alt": 10},
      {"lat": 13.072150, "lng": 80.262000, "alt": 10}
    ],
    "config": {
      "waypoint_threshold": 2.0,
      "hold_duration": 3.0,
      "auto_mode": true
    }
  }' | jq -r '.message // .error // "No response"'

sleep 2

# Show logs for load_mission
view_mission_logs

echo ""
read -p "Press Enter to start the mission..."
echo ""

# Step 2: Start Mission
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}STEP 2: Starting Mission${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"

curl -s -X POST "$SERVER/api/mission/command" \
  -H "Content-Type: application/json" \
  -d '{"command": "start"}' | jq -r '.message // .error // "No response"'

sleep 2

# Show logs for mission start
view_mission_logs

echo ""
echo -e "${YELLOW}════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}Now monitoring live mission logs with 2Hz status updates...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop monitoring${NC}"
echo -e "${YELLOW}════════════════════════════════════════════════════════════════${NC}"
echo ""

# Follow logs in real-time
sudo journalctl -u nrp-service.service -f --no-pager | \
    grep --line-buffered "MISSION_CONTROLLER" | \
    grep --line-buffered --color=always -E "COMMAND RECEIVED|Received.*waypoint|Validating|MISSION LOADED|WP[0-9]:|START command|Waypoints loaded|PUSHING WAYPOINT|uploaded|ARMING|ARMED|MODE CHANGE|AUTO|HOLD|Status: WP|Distance:|reached|Hold period|COMPLETED|$"

