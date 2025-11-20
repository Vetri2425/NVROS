#!/bin/bash

# Test script for refactored mission controller
# Monitors mission controller logs to verify the simple one-by-one waypoint logic

echo "================================"
echo "Mission Controller Log Monitor"
echo "================================"
echo ""
echo "Monitoring for:"
echo "  🏠 HOME position setting"
echo "  📤 Single waypoint upload"
echo "  ⚡ ARM sequence"
echo "  🔄 AUTO mode activation"
echo "  📍 Distance monitoring"
echo "  ✅ Waypoint reached detection"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Follow logs in real-time, filtering for mission controller key events
journalctl -u nrp-service -f --no-pager | grep --line-buffered -E "(MISSION_CONTROLLER.*\[20|EXECUTING WAYPOINT|HOME|Uploading waypoint|ARMED|AUTO mode|Distance.*remaining|Waypoint.*reached|HOLD mode|Mission.*complete)"
