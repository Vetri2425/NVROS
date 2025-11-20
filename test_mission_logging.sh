#!/bin/bash
# Test script to demonstrate mission controller logging in journalctl

echo "=================================================="
echo "Mission Controller Logging Test"
echo "=================================================="
echo ""
echo "This script will:"
echo "1. Load a test mission with 2 waypoints"
echo "2. Start the mission"
echo "3. Monitor journalctl for mission flow logs"
echo ""
echo "Press Ctrl+C to stop monitoring at any time"
echo ""
read -p "Press Enter to continue..."

# Server endpoint
SERVER="http://localhost:5000"

echo ""
echo "Step 1: Loading test mission..."
echo "--------------------------------"

# Load mission with 2 waypoints
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
      "hold_duration": 5.0,
      "auto_mode": true
    }
  }' | jq .

echo ""
echo "Checking journalctl for MISSION LOADED logs..."
echo "=================================================="
sudo journalctl -u nrp-service.service --since "1 minute ago" --no-pager | grep -E "MISSION_CONTROLLER.*MISSION LOADED|MISSION_CONTROLLER.*WP[0-9]:"
echo "=================================================="
echo ""

sleep 2

echo ""
echo "Step 2: Starting mission..."
echo "--------------------------------"

# Start mission
curl -s -X POST "$SERVER/api/mission/command" \
  -H "Content-Type: application/json" \
  -d '{
    "command": "start"
  }' | jq .

echo ""
echo "Mission started! Monitoring journalctl for mission flow..."
echo ""
echo "=================================================="
echo "Looking for logs with the following patterns:"
echo "  - MISSION STARTED"
echo "  - EXECUTING WAYPOINT"
echo "  - Waypoint uploaded to Pixhawk"
echo "  - PIXHAWK ARMED"
echo "  - Setting AUTO mode"
echo "  - PIXHAWK MODE CHANGED"
echo "  - Navigating to WP"
echo "  - Waypoint reached"
echo "  - Setting HOLD mode"
echo "  - Hold period complete"
echo "  - MISSION COMPLETED"
echo "=================================================="
echo ""

# Monitor logs in real-time
echo "Live mission logs (Ctrl+C to stop):"
echo "=================================================="
sudo journalctl -u nrp-service.service -f --no-pager | grep --line-buffered -E "MISSION_CONTROLLER.*(\ud83d\ude80|\ud83d\udccd|\u2713|\u26a1|\u2705|\ud83e\udd16|\ud83d\uded1|\u23f1|\u27a1|\ud83c\udf89|MISSION|WAYPOINT|AUTO|ARMED|HOLD|WP[0-9])"

