#!/bin/bash
# Complete Mission Flow Test with Full Logging
# Demonstrates all mission controller logs at 2Hz during navigation

SERVER="http://localhost:5001"

echo "============================================================"
echo "    COMPLETE MISSION FLOW - FULL LOGGING DEMONSTRATION"
echo "============================================================"
echo ""
echo "This test will demonstrate:"
echo "  ✓ Load mission with waypoints"
echo "  ✓ Set mission mode (manual/auto)"
echo "  ✓ Start mission"
echo "  ✓ Show ALL logs including:"
echo "    - Command reception"
echo "    - Waypoint validation"
echo "    - Mission state changes"
echo "    - Waypoint push to Pixhawk"
echo "    - Arming sequence"
echo "    - MODE changes (AUTO, HOLD)"
echo "    - 2Hz status updates during navigation"
echo "    - Waypoint reached events"
echo "    - Hold periods"
echo "    - Mission completion"
echo ""
echo "============================================================"
echo ""

# Check server
echo "Checking server availability..."
if ! curl -s "http://localhost:5001" > /dev/null 2>&1; then
    echo "ERROR: Server not responding on port 5001"
    exit 1
fi
echo "✓ Server is running"
echo ""

# Load mission
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "STEP 1: Loading Mission"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
curl -s -X POST "$SERVER/api/mission/load" \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {"lat": 13.072100, "lng": 80.261950, "alt": 10},
      {"lat": 13.072150, "lng": 80.262000, "alt": 10}
    ]
  }' | jq -r '.message // "Done"'

sleep 1
echo ""
echo "Mission Controller Logs:"
sudo journalctl -u nrp-service.service --since "5 seconds ago" --no-pager | grep "MISSION_CONTROLLER"
echo ""

# Set mode to auto
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "STEP 2: Setting Mission Mode to AUTO"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
curl -s -X POST "$SERVER/api/mission/mode" \
  -H "Content-Type: application/json" \
  -d '{"mode": "auto"}' | jq -r '.message // "Done"'

sleep 1
echo ""
echo "Mission Controller Logs:"
sudo journalctl -u nrp-service.service --since "3 seconds ago" --no-pager | grep "MISSION_CONTROLLER"
echo ""

read -p "Press Enter to start the mission..."
echo ""

# Start mission
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "STEP 3: Starting Mission"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
curl -s -X POST "$SERVER/api/mission/start" | jq -r '.message // "Done"'

sleep 2
echo ""
echo "Initial Mission Start Logs:"
sudo journalctl -u nrp-service.service --since "5 seconds ago" --no-pager | grep "MISSION_CONTROLLER"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Now showing LIVE mission logs with 2Hz status updates"
echo "Watch for:"
echo "  - Distance updates every 0.5 seconds"
echo "  - Mode changes"
echo "  - Waypoint reached events"
echo "  - Hold periods"
echo "Press Ctrl+C to stop"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Follow logs
sudo journalctl -u nrp-service.service -f --no-pager | \
    grep --line-buffered "MISSION_CONTROLLER" | \
    grep --line-buffered --color=always -E "COMMAND|MISSION|WAYPOINT|PUSHING|uploaded|ARMING|ARMED|MODE CHANGE|AUTO|HOLD|Status:|Distance:|reached|Hold period|COMPLETED|$"
