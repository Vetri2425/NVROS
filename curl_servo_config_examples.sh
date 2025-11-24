#!/bin/bash
# Quick Servo Config API Reference

# ============================================
# GET CURRENT CONFIGURATION
# ============================================
curl -X GET http://localhost:5000/api/mission/servo_config | python3 -m json.tool

# ============================================
# UPDATE ALL PARAMETERS
# ============================================
curl -X POST http://localhost:5000/api/mission/servo_config \
  -H "Content-Type: application/json" \
  -d '{
    "servo_channel": 10,
    "servo_pwm_on": 1500,
    "servo_pwm_off": 1100,
    "servo_delay_before": 1.0,
    "servo_spray_duration": 2.0,
    "servo_delay_after": 3.0,
    "servo_enabled": true
  }'

# ============================================
# UPDATE ONLY TIMING
# ============================================
curl -X POST http://localhost:5000/api/mission/servo_config \
  -H "Content-Type: application/json" \
  -d '{
    "servo_delay_before": 0.5,
    "servo_spray_duration": 1.5,
    "servo_delay_after": 2.5
  }'

# ============================================
# UPDATE ONLY PWM VALUES
# ============================================
curl -X POST http://localhost:5000/api/mission/servo_config \
  -H "Content-Type: application/json" \
  -d '{
    "servo_pwm_on": 1800,
    "servo_pwm_off": 900
  }'

# ============================================
# DISABLE SERVO
# ============================================
curl -X POST http://localhost:5000/api/mission/servo_config \
  -H "Content-Type: application/json" \
  -d '{"servo_enabled": false}'

# ============================================
# ENABLE SERVO
# ============================================
curl -X POST http://localhost:5000/api/mission/servo_config \
  -H "Content-Type: application/json" \
  -d '{"servo_enabled": true}'

# ============================================
# LOAD MISSION WITH SERVO CONFIG
# ============================================
curl -X POST http://localhost:5000/api/mission/load_controller \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {"lat": 13.071922, "lng": 80.2619957, "alt": 10.0},
      {"lat": 13.071932, "lng": 80.2619967, "alt": 10.0}
    ],
    "servo_channel": 10,
    "servo_pwm_on": 1500,
    "servo_pwm_off": 1100,
    "servo_delay_before": 1.0,
    "servo_spray_duration": 2.0,
    "servo_delay_after": 3.0,
    "servo_enabled": true
  }'
