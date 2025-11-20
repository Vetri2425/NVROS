#!/bin/bash

# Test script to verify mission start/stop/restart flow
# Uses proper HTTP POST commands

BASE_URL="http://localhost:5001"
TIMEOUT=5

echo "======================================================================"
echo "Mission Start/Stop/Restart Test"
echo "======================================================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_step() {
    echo -e "${BLUE}>>> $1${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}ℹ $1${NC}"
}

# Step 1: Check server health using mission status endpoint
print_step "Checking server connection..."
response=$(curl -s -w "\n%{http_code}" "$BASE_URL/api/mission/status")
http_code=$(echo "$response" | tail -n1)
if [ "$http_code" = "200" ]; then
    print_success "Server is running (HTTP $http_code)"
else
    print_error "Server not responding (HTTP $http_code)"
    exit 1
fi
echo ""

# Step 2: Load mission via POST
print_step "Loading mission with 2 waypoints..."
load_response=$(curl -s -X POST "$BASE_URL/api/mission/load" \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {"lat": 11.0, "lng": 77.0, "alt": 10},
      {"lat": 11.001, "lng": 77.001, "alt": 10}
    ],
    "config": {
      "auto_mode": true,
      "waypoint_threshold": 2.0,
      "hold_duration": 2.0
    }
  }')

echo "Response: $load_response"
if echo "$load_response" | grep -q "success"; then
    print_success "Mission loaded"
else
    print_error "Failed to load mission"
    echo "Response: $load_response"
fi
echo ""

# Step 3: Get mission status
print_step "Checking mission status..."
status_response=$(curl -s "$BASE_URL/api/mission/status")
echo "Status: $status_response" | head -c 200
echo ""
echo ""

# Step 4: Start mission via POST
print_step "Starting mission (POST to /api/mission/start)..."
start_response=$(curl -s -X POST "$BASE_URL/api/mission/start")
echo "Response: $start_response"
if echo "$start_response" | grep -q "success"; then
    print_success "Mission start command accepted"
else
    print_error "Failed to start mission"
fi
echo ""

# Step 5: Wait and check status
print_info "Waiting 2 seconds before checking status..."
sleep 2

print_step "Checking mission status after start..."
status_response=$(curl -s "$BASE_URL/api/mission/status")
echo "Status: $status_response" | head -c 300
echo ""
echo ""

# Step 6: Stop mission via POST
print_step "Stopping mission (POST to /api/mission/stop)..."
stop_response=$(curl -s -X POST "$BASE_URL/api/mission/stop")
echo "Response: $stop_response"
if echo "$stop_response" | grep -q "success"; then
    print_success "Mission stop command accepted"
else
    print_error "Failed to stop mission"
fi
echo ""

# Step 7: Check status after stop
print_step "Checking mission status after stop..."
status_response=$(curl -s "$BASE_URL/api/mission/status")
echo "Status: $status_response" | head -c 300
echo ""
echo ""

# Step 8: Restart mission (without reload) via POST
print_step "Restarting mission WITHOUT reloading (POST to /api/mission/start)..."
restart_response=$(curl -s -X POST "$BASE_URL/api/mission/start")
echo "Response: $restart_response"
if echo "$restart_response" | grep -q "success"; then
    print_success "Mission restart command accepted (no reload needed!)"
else
    print_error "Failed to restart mission"
    print_info "This indicates the state management needs fixing"
fi
echo ""

# Step 9: Final status
print_step "Final mission status..."
status_response=$(curl -s "$BASE_URL/api/mission/status")
echo "Status: $status_response" | head -c 300
echo ""
echo ""

echo "======================================================================"
echo "Test Complete"
echo "======================================================================"
echo ""
print_info "If all steps show ✓, the mission state management is working correctly"
