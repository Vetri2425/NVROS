#!/bin/bash
# Script to restart nrp-service and verify POST endpoint is working

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}NRP Service Restart & Verification${NC}"
echo -e "${BLUE}========================================${NC}\n"

# Check current service status
echo -e "${YELLOW}Current Service Status:${NC}"
systemctl status nrp-service 2>&1 | head -5
echo ""

# Restart service
echo -e "${YELLOW}Restarting nrp-service...${NC}"
if sudo systemctl restart nrp-service; then
    echo -e "${GREEN}✓ Service restart initiated${NC}"
else
    echo -e "${RED}✗ Service restart failed${NC}"
    exit 1
fi

# Wait for service to start
echo -e "${YELLOW}Waiting for service to start...${NC}"
sleep 3

# Check service is running
if systemctl is-active --quiet nrp-service; then
    echo -e "${GREEN}✓ Service is running${NC}"
else
    echo -e "${RED}✗ Service is not running${NC}"
    systemctl status nrp-service
    exit 1
fi

# Wait a bit more for Flask to initialize
sleep 2

# Test GET endpoint (should work regardless)
echo -e "\n${YELLOW}Testing GET /api/mission/config:${NC}"
GET_RESPONSE=$(curl -s -X GET "http://localhost:5001/api/mission/config")
if echo "$GET_RESPONSE" | grep -q "success"; then
    echo -e "${GREEN}✓ GET endpoint working${NC}"
    echo "Response: $(echo $GET_RESPONSE | jq -c . 2>/dev/null || echo $GET_RESPONSE)"
else
    echo -e "${RED}✗ GET endpoint failed${NC}"
    echo "Response: $GET_RESPONSE"
fi

# Test POST endpoint
echo -e "\n${YELLOW}Testing POST /api/mission/config:${NC}"
POST_RESPONSE=$(curl -s -w "\n%{http_code}" -X POST "http://localhost:5001/api/mission/config" \
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

HTTP_CODE=$(echo "$POST_RESPONSE" | tail -1)
BODY=$(echo "$POST_RESPONSE" | head -1)

if [ "$HTTP_CODE" = "200" ]; then
    echo -e "${GREEN}✓ POST endpoint working (HTTP $HTTP_CODE)${NC}"
    echo "Response: $(echo $BODY | jq . 2>/dev/null || echo $BODY)"
else
    echo -e "${RED}✗ POST endpoint failed (HTTP $HTTP_CODE)${NC}"
    echo "Response: $BODY"
fi

echo -e "\n${BLUE}========================================${NC}"
echo -e "${GREEN}Verification Complete!${NC}"
echo -e "${BLUE}========================================${NC}\n"

if [ "$HTTP_CODE" = "200" ]; then
    echo -e "${GREEN}SUCCESS: POST endpoint is now active!${NC}"
    echo -e "\n${BLUE}Next Steps:${NC}"
    echo -e "1. Load mission: curl -X POST http://localhost:5001/api/mission/load_controller ..."
    echo -e "2. Monitor topic: ros2 topic echo /mission/command"
    echo -e "3. Update config: curl -X POST http://localhost:5001/api/mission/config ..."
else
    echo -e "${RED}WARNING: POST endpoint still not responding${NC}"
    echo -e "\n${YELLOW}Check service logs:${NC}"
    echo -e "systemctl status nrp-service"
    echo -e "sudo journalctl -u nrp-service -f"
fi
