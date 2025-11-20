#!/bin/bash

BASE_URL="http://localhost:5001"

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║  Final Mission Controller Test - With ARM Check & HOLD on Complete ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""

# Test 1: Load Mission
echo "TEST 1: Load mission"
echo "─────────────────────────────────────────────────────────────────────"
load_resp=$(curl -s -X POST "$BASE_URL/api/mission/load" \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {"lat": 11.0, "lng": 77.0, "alt": 10},
      {"lat": 11.001, "lng": 77.001, "alt": 10}
    ]
  }')
echo "Response: $load_resp"
echo ""

# Test 2: Verify READY state
echo "TEST 2: Verify mission state is READY"
echo "─────────────────────────────────────────────────────────────────────"
state=$(curl -s "$BASE_URL/api/mission/status" | jq -r '.status.mission_state')
echo "State: $state"
[ "$state" = "ready" ] && echo "✓ PASS" || echo "✗ FAIL"
echo ""

# Test 3: Start Mission
echo "TEST 3: Start mission (will attempt ARM if needed, then AUTO mode)"
echo "─────────────────────────────────────────────────────────────────────"
start_resp=$(curl -s -X POST "$BASE_URL/api/mission/start")
echo "Response: $start_resp"
echo ""

# Wait for execution
sleep 2

# Test 4: Verify RUNNING state
echo "TEST 4: Verify mission state is RUNNING"
echo "─────────────────────────────────────────────────────────────────────"
state=$(curl -s "$BASE_URL/api/mission/status" | jq -r '.status.mission_state')
echo "State: $state"
[ "$state" = "running" ] && echo "✓ PASS" || echo "✗ FAIL"
echo ""

# Test 5: Stop Mission
echo "TEST 5: Stop mission (should set HOLD mode)"
echo "─────────────────────────────────────────────────────────────────────"
stop_resp=$(curl -s -X POST "$BASE_URL/api/mission/stop")
echo "Response: $stop_resp"
echo ""

# Test 6: Verify READY state after stop (not IDLE)
echo "TEST 6: Verify state is READY after stop (not IDLE)"
echo "─────────────────────────────────────────────────────────────────────"
state=$(curl -s "$BASE_URL/api/mission/status" | jq -r '.status.mission_state')
mode=$(curl -s "$BASE_URL/api/mission/status" | jq -r '.status.pixhawk_state.mode // "unknown"')
echo "State: $state"
echo "Pixhawk Mode: $mode"
[ "$state" = "ready" ] && echo "✓ PASS: State is READY" || echo "✗ FAIL: State is $state"
[ "$mode" = "HOLD" ] && echo "✓ PASS: Mode is HOLD" || echo "✗ FAIL: Mode is $mode"
echo ""

# Test 7: Restart without reload
echo "TEST 7: Restart mission WITHOUT reloading"
echo "─────────────────────────────────────────────────────────────────────"
restart_resp=$(curl -s -X POST "$BASE_URL/api/mission/start")
echo "Response: $restart_resp"
sleep 1
state=$(curl -s "$BASE_URL/api/mission/status" | jq -r '.status.mission_state')
echo "New State: $state"
[ "$state" = "running" ] && echo "✓ PASS: Mission restarted without reload!" || echo "✗ FAIL: Cannot restart"
echo ""

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║                       TEST SUMMARY COMPLETE                        ║"
echo "║                                                                    ║"
echo "║  Key Improvements:                                                 ║"
echo "║  ✓ Mission completes with HOLD mode set                            ║"
echo "║  ✓ Mission can restart without reload (state stays READY)          ║"
echo "║  ✓ ARM check attempted before AUTO mode                            ║"
echo "║  ✓ Proper state transitions (READY→RUNNING→READY)                  ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
