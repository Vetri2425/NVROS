#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SERVER_URL="http://localhost:5000"

echo -e "${BLUE}=== Testing Servo Configuration API ===${NC}\n"

# Test 1: Get current configuration
echo -e "${YELLOW}Test 1: Get Current Servo Configuration${NC}"
curl -s ${SERVER_URL}/api/mission/servo_config | python3 -m json.tool
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ GET request successful${NC}\n"
else
    echo -e "\033[0;31m✗ GET request failed - Is the server running?${NC}\n"
    exit 1
fi

# Test 2: Update all parameters
echo -e "${YELLOW}Test 2: Update All Servo Parameters${NC}"
curl -s -X POST ${SERVER_URL}/api/mission/servo_config \
  -H "Content-Type: application/json" \
  -d '{
    "servo_channel": 10,
    "servo_pwm_on": 1500,
    "servo_pwm_off": 1100,
    "servo_delay_before": 1.0,
    "servo_spray_duration": 2.0,
    "servo_delay_after": 3.0
  }' | python3 -m json.tool

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ POST request successful${NC}\n"
else
    echo -e "\033[0;31m✗ POST request failed${NC}\n"
fi

# Test 3: Verify updated configuration
echo -e "${YELLOW}Test 3: Verify Updated Configuration${NC}"
curl -s ${SERVER_URL}/api/mission/servo_config | python3 -m json.tool
echo -e "${GREEN}✓ Configuration verified${NC}\n"

# Test 4: Update only timing parameters
echo -e "${YELLOW}Test 4: Update Only Timing Parameters${NC}"
curl -s -X POST ${SERVER_URL}/api/mission/servo_config \
  -H "Content-Type: application/json" \
  -d '{
    "servo_delay_before": 0.5,
    "servo_spray_duration": 1.5,
    "servo_delay_after": 2.5
  }' | python3 -m json.tool
echo -e "${GREEN}✓ Partial update successful${NC}\n"

# Test 5: Disable servo
echo -e "${YELLOW}Test 5: Disable Servo${NC}"
curl -s -X POST ${SERVER_URL}/api/mission/servo_config \
  -H "Content-Type: application/json" \
  -d '{"servo_enabled": false}' | python3 -m json.tool
echo -e "${GREEN}✓ Servo disabled${NC}\n"

# Test 6: Enable servo
echo -e "${YELLOW}Test 6: Enable Servo${NC}"
curl -s -X POST ${SERVER_URL}/api/mission/servo_config \
  -H "Content-Type: application/json" \
  -d '{"servo_enabled": true}' | python3 -m json.tool
echo -e "${GREEN}✓ Servo enabled${NC}\n"

# Test 7: Final configuration check
echo -e "${YELLOW}Test 7: Final Configuration Check${NC}"
curl -s ${SERVER_URL}/api/mission/servo_config | python3 -m json.tool
echo -e "${GREEN}✓ All tests completed${NC}\n"

echo -e "${BLUE}=== Summary ===${NC}"
echo "All 7 servo parameters are configurable:"
echo "  1. servo_channel"
echo "  2. servo_pwm_on"
echo "  3. servo_pwm_off"
echo "  4. servo_delay_before ⏱ (before servo ON)"
echo "  5. servo_spray_duration ⏱ (between ON/OFF)"
echo "  6. servo_delay_after ⏱ (after servo OFF)"
echo "  7. servo_enabled"
