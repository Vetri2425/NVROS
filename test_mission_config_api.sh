#!/bin/bash
# Test script for mission config endpoint on port 5001

JETSON_API="http://localhost:5001"
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Mission Config API Test (Port 5001)${NC}"
echo -e "${BLUE}========================================${NC}\n"

# Step 1: Load mission first
echo -e "${YELLOW}Step 1: Loading mission to controller...${NC}"
LOAD_RESPONSE=$(curl -s -X POST "${JETSON_API}/api/mission/load_controller" \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {"lat": 13.071922, "lng": 80.2619957},
      {"lat": 13.072000, "lng": 80.2620000}
    ]
  }')

echo "$LOAD_RESPONSE" | jq .
echo ""

# Step 2: Get current config
echo -e "${YELLOW}Step 2: Getting current mission config (GET)...${NC}"
GET_RESPONSE=$(curl -s -X GET "${JETSON_API}/api/mission/config")
echo "$GET_RESPONSE" | jq .
echo ""

# Step 3: Update config via POST (this will send command to mission controller)
echo -e "${YELLOW}Step 3: Updating mission config and sending to mission controller (POST)...${NC}"
POST_RESPONSE=$(curl -s -X POST "${JETSON_API}/api/mission/config" \
  -H "Content-Type: application/json" \
  -d '{
    "servo_channel": 10,
    "servo_pwm_start": 1500,
    "servo_pwm_stop": 1100,
    "spray_duration": 5.0,
    "delay_before_spray": 1.0,
    "delay_after_spray": 1.0,
    "gps_timeout": 30.0,
    "auto_mode": true
  }')

echo "$POST_RESPONSE" | jq .
echo ""

# Step 4: Get updated config
echo -e "${YELLOW}Step 4: Getting updated mission config...${NC}"
UPDATED_RESPONSE=$(curl -s -X GET "${JETSON_API}/api/mission/config")
echo "$UPDATED_RESPONSE" | jq '.data.sprayer_parameters'
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Test Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "\n${BLUE}Check server logs for:${NC}"
echo -e "  - 'Sent load_mission command to mission controller with servo config'${NC}"
echo -e "\n${BLUE}Check ROS topic for message:${NC}"
echo -e "  ros2 topic echo /mission/command${NC}"
