#!/bin/bash

BASE_URL="http://localhost:5001"

echo "=== Step 1: Load Mission ==="
curl -s -X POST "$BASE_URL/api/mission/load" \
  -H "Content-Type: application/json" \
  -d '{"waypoints": [{"lat": 11.0, "lng": 77.0, "alt": 10}]}' | jq .
echo ""

echo "=== Step 2: Get Status (should be READY) ==="
curl -s "$BASE_URL/api/mission/status" | jq '.status.mission_state'
echo ""

echo "=== Step 3: Start Mission ==="
curl -s -X POST "$BASE_URL/api/mission/start" | jq .
echo ""

echo "=== Step 4: Get Status (should be RUNNING) ==="
curl -s "$BASE_URL/api/mission/status" | jq '.status.mission_state'
echo ""

echo "=== Step 5: Stop Mission ==="
curl -s -X POST "$BASE_URL/api/mission/stop" | jq .
echo ""

echo "=== Step 6: Get Status (should be READY, not IDLE) ==="
curl -s "$BASE_URL/api/mission/status" | jq '.status.mission_state'
echo ""

echo "=== Step 7: Try to Start Again ==="
curl -s -X POST "$BASE_URL/api/mission/start" | jq .
echo ""

echo "=== Step 8: Final Status ==="
curl -s "$BASE_URL/api/mission/status" | jq '.status | {mission_state, current_waypoint, total_waypoints, waypoints: (.waypoints | length)}'
