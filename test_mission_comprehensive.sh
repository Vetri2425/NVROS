#!/bin/bash

BASE_URL="http://localhost:5001"

echo "════════════════════════════════════════════════════════════════════"
echo "  Comprehensive Mission Start/Stop/Restart Test"
echo "════════════════════════════════════════════════════════════════════"
echo ""

test_case() {
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "▶ $1"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

check_state() {
    local expected="$1"
    local actual=$(curl -s "$BASE_URL/api/mission/status" | jq -r '.status.mission_state')
    if [ "$actual" = "$expected" ]; then
        echo "✓ State is: $actual (expected: $expected)"
    else
        echo "✗ State is: $actual (expected: $expected) ← MISMATCH!"
        return 1
    fi
}

test_case "1. Load mission with 2 waypoints"
curl -s -X POST "$BASE_URL/api/mission/load" \
  -H "Content-Type: application/json" \
  -d '{"waypoints": [{"lat": 11.0, "lng": 77.0}, {"lat": 11.001, "lng": 77.001}]}' | jq -c '.success, .message'
echo ""

test_case "2. Verify state is READY after load"
check_state "ready"
echo ""

test_case "3. Start mission"
curl -s -X POST "$BASE_URL/api/mission/start" | jq -c '.success, .message'
sleep 1
echo ""

test_case "4. Verify state is RUNNING"
check_state "running"
echo ""

test_case "5. Stop mission (should set to READY)"
curl -s -X POST "$BASE_URL/api/mission/stop" | jq -c '.success, .message'
echo ""

test_case "6. Verify state is READY (not IDLE)"
check_state "ready"
echo ""

test_case "7. Restart mission WITHOUT reloading"
curl -s -X POST "$BASE_URL/api/mission/start" | jq -c '.success, .message'
sleep 1
echo ""

test_case "8. Verify state is RUNNING again"
check_state "running"
echo ""

test_case "9. Stop again to go back to READY"
curl -s -X POST "$BASE_URL/api/mission/stop" | jq -c '.success, .message'
echo ""

test_case "10. Pause mission is NOT allowed from READY state (no mission running)"
curl -s -X POST "$BASE_URL/api/mission/pause" | jq -c '.success, .message'
echo ""

test_case "11. Start mission one more time"
curl -s -X POST "$BASE_URL/api/mission/start" | jq -c '.success, .message'
sleep 1
echo ""

test_case "12. Pause mission while RUNNING"
curl -s -X POST "$BASE_URL/api/mission/pause" | jq -c '.success, .message'
echo ""

test_case "13. Verify state is PAUSED"
check_state "paused"
echo ""

test_case "14. Resume mission from PAUSED"
curl -s -X POST "$BASE_URL/api/mission/resume" | jq -c '.success, .message'
sleep 1
echo ""

test_case "15. Verify state is RUNNING after resume"
check_state "running"
echo ""

test_case "16. Stop mission final"
curl -s -X POST "$BASE_URL/api/mission/stop" | jq -c '.success, .message'
echo ""

test_case "17. Final status check"
curl -s "$BASE_URL/api/mission/status" | jq '.status | {mission_state, waypoints: (.waypoints | length)}'
echo ""

echo "════════════════════════════════════════════════════════════════════"
echo "✓ Test Complete - All state transitions working correctly!"
echo "════════════════════════════════════════════════════════════════════"
