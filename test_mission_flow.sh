#!/bin/bash

# Test script for refactored mission controller
# Tests the complete mission flow with the new one-by-one logic

set -e

API_URL="http://localhost:5001"

echo "================================"
echo "Mission Controller Flow Test"
echo "================================"
echo ""

# Step 1: Load mission
echo "Step 1: Loading mission with 2 waypoints..."
LOAD_RESPONSE=$(curl -s -X POST "${API_URL}/api/mission/load" \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {"lat": 13.072100, "lng": 80.262000, "alt": 10},
      {"lat": 13.072150, "lng": 80.262050, "alt": 10}
    ],
    "auto_mode": true,
    "waypoint_threshold": 2.0,
    "hold_duration": 5
  }')

echo "Load Response:"
echo "$LOAD_RESPONSE" | jq '.'
echo ""

# Check if load was successful
if echo "$LOAD_RESPONSE" | jq -e '.success == true' > /dev/null; then
    echo "✅ Mission loaded successfully"
else
    echo "❌ Failed to load mission"
    exit 1
fi

echo ""
echo "Waiting 2 seconds before starting mission..."
sleep 2

# Step 2: Start mission
echo ""
echo "Step 2: Starting mission..."
START_RESPONSE=$(curl -s -X POST "${API_URL}/api/mission/start")

echo "Start Response:"
echo "$START_RESPONSE" | jq '.'
echo ""

# Check if start was successful
if echo "$START_RESPONSE" | jq -e '.success == true' > /dev/null; then
    echo "✅ Mission started successfully"
else
    echo "❌ Failed to start mission"
    exit 1
fi

echo ""
echo "================================"
echo "Mission started! Monitoring logs..."
echo "================================"
echo ""
echo "Expected flow:"
echo "1. 🏠 Set HOME position (first time only)"
echo "2. 📤 Upload waypoint 1"
echo "3. ⚡ ARM if not armed"
echo "4. 🔄 Set AUTO mode"
echo "5. 📍 Monitor distance to waypoint"
echo "6. ✅ Waypoint reached → HOLD mode"
echo "7. ⏱ 5 second hold"
echo "8. ➡ Auto-proceed to waypoint 2 (if auto_mode=true)"
echo "9. Repeat steps 2-7 for waypoint 2"
echo "10. 🎉 Mission complete"
echo ""
echo "Press Ctrl+C to stop monitoring"
echo ""

# Monitor mission controller logs
journalctl -u nrp-service -f --no-pager | grep --line-buffered -E "(MISSION_CONTROLLER|WAYPOINT|AUTO mode|ARMED|HOME|Distance|reached|HOLD)"
