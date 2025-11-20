#!/bin/bash

# Simulation of refactored mission controller log output
# Shows what you'll see after restarting the service

echo "================================"
echo "Simulated Refactored Mission Flow"
echo "================================"
echo ""
echo "This shows what the logs will look like after restart"
echo ""

sleep 1

echo "📦 Loading mission with 2 waypoints..."
sleep 0.5
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.100] 📥 COMMAND RECEIVED: LOAD_MISSION"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.101] 📦 Received 2 waypoint(s) in load_mission command"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.102] ✓ Validating waypoints..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.103] ✓ All waypoints validated successfully"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.104] 📝 Setting mission state: IDLE → READY"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.105] ═══════════════════════════════════════════════════"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.106] MISSION LOADED: 2 waypoints"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.107] Mission Mode: auto"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.108] Waypoint Threshold: 2.0m"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.109] Hold Duration: 5.0s"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.110]   WP1: lat=13.072100, lng=80.262000, alt=10m"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.111]   WP2: lat=13.072150, lng=80.262050, alt=10m"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:00.112] ═══════════════════════════════════════════════════"
echo ""

sleep 2

echo "🚀 Starting mission..."
sleep 0.5
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:02.200] 📥 COMMAND RECEIVED: START"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:02.201] 🎬 START command received"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:02.202] Current state: ready"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:02.203] Waypoints loaded: 2"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:02.204] ═══════════════════════════════════════════════════"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:02.205] 🚀 MISSION STARTED"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:02.206] Starting from waypoint 1/2"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:02.207] Mission Mode: auto"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:02.208] ═══════════════════════════════════════════════════"
echo ""

sleep 1

echo "📍 Executing Waypoint 1..."
sleep 0.3
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:03.300] ───────────────────────────────────────────────────"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:03.301] 📍 EXECUTING WAYPOINT 1/2"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:03.302] Target: lat=13.072100, lng=80.262000, alt=10.0m"
echo ""

sleep 0.5

echo "🏠 Step 1: Setting HOME (first time only)..."
sleep 0.3
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:03.800] 🏠 Setting HOME position (first time only)..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:03.801] 🏠 HOME will be: lat=13.072060, lng=80.261957, alt=12.5m"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:03.802] ✓ ArduPilot will auto-set HOME on ARM at current position"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:03.803] ✅ HOME position set successfully"
echo ""

sleep 0.5

echo "📤 Step 2: Uploading waypoint 1..."
sleep 0.3
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:04.300] 📤 Uploading waypoint 1..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:04.301] 🗑️ Clearing existing waypoints..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:04.400] ✓ Cleared existing waypoints"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:04.401] 📤 Uploading waypoint to Pixhawk..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:04.550] ✅ Waypoint uploaded successfully"
echo ""

sleep 0.5

echo "⚡ Step 3: ARM check..."
sleep 0.3
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:05.100] ⚡ Attempting to arm Pixhawk..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:05.250] ✅ PIXHAWK ARMED SUCCESSFULLY"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:05.251] ✅ PIXHAWK ARMED"
echo ""

sleep 0.5

echo "🔄 Step 4: Setting AUTO mode..."
sleep 0.3
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:05.750] 🔄 Setting AUTO mode..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:05.751] 🔄 MODE CHANGE: GUIDED → AUTO"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:05.850] ✅ PIXHAWK MODE CHANGED TO: AUTO"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:05.851] ✅ AUTO mode activated - rover should move to waypoint"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:05.852] 📊 Started periodic status logging at 2Hz"
echo ""

sleep 1

echo "📍 Step 5: Monitoring distance to waypoint..."
sleep 0.5
for i in {5..1}; do
    echo "[MISSION_CONTROLLER] [2025-11-19 18:00:0$((6+5-i)).000] 📍 Status: WP1/2 | Distance: $i.50m | Mode: AUTO | Armed: YES | Pos: (13.072060, 80.261957)"
    sleep 0.5
done
echo ""

sleep 0.5

echo "✅ Waypoint 1 reached!"
sleep 0.3
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:11.500] ✓ Waypoint 1 reached! Distance: 1.82m (threshold: 2.0m)"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:11.501] 🛑 Setting HOLD mode"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:11.502] 🔄 MODE CHANGE: AUTO → HOLD"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:11.600] ✅ PIXHAWK MODE CHANGED TO: HOLD"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:11.601] ✅ WAYPOINT 1 REACHED"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:11.602] Position: lat=13.072100, lng=80.262000"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:11.603] ⏱ Starting 5.0s hold period at waypoint 1"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:11.604] ⏹ Stopped periodic status logging"
echo ""

sleep 2

echo "⏱ Hold period (5 seconds)..."
for i in {5..1}; do
    echo "  Holding... ${i}s remaining"
    sleep 1
done
echo ""

echo "[MISSION_CONTROLLER] [2025-11-19 18:00:16.610] ✓ Hold period complete for waypoint 1"
echo ""

sleep 0.5

echo "➡ Auto-proceeding to Waypoint 2..."
sleep 0.3
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:17.100] ➡ Proceeding to next waypoint (2/2)"
echo ""

sleep 1

echo "📍 Executing Waypoint 2..."
sleep 0.3
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:18.200] ───────────────────────────────────────────────────"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:18.201] 📍 EXECUTING WAYPOINT 2/2"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:18.202] Target: lat=13.072150, lng=80.262050, alt=10.0m"
echo ""

sleep 0.5

echo "⏩ HOME already set, skipping..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:18.700] 📤 Uploading waypoint 2..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:18.701] 🗑️ Clearing existing waypoints..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:18.800] ✓ Cleared existing waypoints"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:18.801] 📤 Uploading waypoint to Pixhawk..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:18.950] ✅ Waypoint uploaded successfully"
echo ""

sleep 0.5

echo "[MISSION_CONTROLLER] [2025-11-19 18:00:19.450] ✓ Pixhawk already armed"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:19.451] ✅ PIXHAWK ARMED"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:19.452] 🔄 Setting AUTO mode..."
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:19.550] ✅ AUTO mode activated - rover should move to waypoint"
echo ""

sleep 1

echo "📍 Monitoring distance..."
sleep 0.5
for i in {5..1}; do
    echo "[MISSION_CONTROLLER] [2025-11-19 18:00:$((20+5-i)).000] 📍 Status: WP2/2 | Distance: $i.30m | Mode: AUTO | Armed: YES | Pos: (13.072110, 80.262010)"
    sleep 0.5
done
echo ""

sleep 0.5

echo "✅ Waypoint 2 reached!"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:26.000] ✓ Waypoint 2 reached! Distance: 1.95m (threshold: 2.0m)"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:26.001] 🛑 Setting HOLD mode"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:26.100] ✅ WAYPOINT 2 REACHED"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:26.101] ⏱ Starting 5.0s hold period at waypoint 2"
echo ""

sleep 2

echo "⏱ Hold period..."
for i in {5..1}; do
    echo "  Holding... ${i}s remaining"
    sleep 1
done
echo ""

echo "[MISSION_CONTROLLER] [2025-11-19 18:00:31.110] ✓ Hold period complete for waypoint 2"
echo ""

sleep 0.5

echo "🎉 Mission Complete!"
sleep 0.3
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:31.600] ═══════════════════════════════════════════════════"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:31.601] 🎉 MISSION COMPLETED SUCCESSFULLY"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:31.602] Duration: 29.4 seconds (0.5 minutes)"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:31.603] Waypoints completed: 2"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:31.604] ═══════════════════════════════════════════════════"
echo "[MISSION_CONTROLLER] [2025-11-19 18:00:31.605] 🛑 Setting final HOLD mode"
echo ""

sleep 1

echo ""
echo "================================"
echo "✅ Simulation Complete!"
echo "================================"
echo ""
echo "Key Points:"
echo "  ✅ HOME set only ONCE (first waypoint)"
echo "  ✅ Single waypoint uploaded each time"
echo "  ✅ ARM check before AUTO mode"
echo "  ✅ Simple, clean flow"
echo "  ✅ No complex verification retries"
echo ""
echo "To see this for real:"
echo "  1. Restart service: bash start_service.sh"
echo "  2. Run test: ./test_mission_flow.sh"
echo ""
