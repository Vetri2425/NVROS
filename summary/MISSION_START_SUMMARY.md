# Mission Start: Complete Flow Summary

## What Happens When You Click "Start"

**Frontend → API → Mission Node → Vehicle**

1. **Frontend sends**: `POST /api/mission/start`
2. **API publishes**: `{'command': 'start'}` to `/mission/command` ROS topic
3. **Mission node receives**, sets `mission_active = True`, starts 0.1s timer
4. **Timer spawns thread** for each waypoint:
   - Sends `MAV_CMD_NAV_WAYPOINT` via MAVROS to vehicle
   - Monitors GPS distance until waypoint reached
   - Sets flight mode to **HOLD**
   - **Activates servo** (PWM 1500) → spray starts
   - Waits `spray_duration` (default 5s)
   - **Deactivates servo** (PWM 1100) → spray stops
   - Sets flight mode to **GUIDED**
   - Publishes status updates throughout
   - Moves to next waypoint

**Result**: Autonomous mission executes, spraying at each waypoint via MAVROS commands.
