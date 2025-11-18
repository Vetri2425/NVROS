#!/bin/bash
# Test script to verify the mission config endpoint sends commands

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

JETSON_API="http://localhost:8000"

echo -e "${BLUE}=== Testing Mission Config Endpoint Fix ===${NC}\n"

# Step 1: Load a mission
echo -e "${BLUE}Step 1: Loading mission to controller...${NC}"
curl -X POST "${JETSON_API}/api/mission/load_controller" \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {"lat": 13.071922, "lng": 80.2619957},
      {"lat": 13.072000, "lng": 80.2620000}
    ]
  }' 2>/dev/null | jq .

echo -e "\n${BLUE}Step 2: Updating servo config (should send command to mission controller)...${NC}"
curl -X POST "${JETSON_API}/api/config/sprayer" \
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
  }' 2>/dev/null | jq .

echo -e "\n${BLUE}Step 3: Verify config was saved...${NC}"
curl -X GET "${JETSON_API}/api/mission/config" 2>/dev/null | jq '.data.sprayer_parameters'

echo -e "\n${GREEN}Test complete! Check server logs for mission command publication.${NC}"
echo -e "${GREEN}Expected log: 'Sent load_mission command to mission controller with updated servo config'${NC}\n"
