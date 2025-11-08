# 🎯 Waypoint Marking - Simple Guide

## How It Works (3 Steps)

### **Step 1: Upload Mission**
Upload waypoints to ArduPilot using Mission Planner or the UI:
- Mark locations where you want to spray
- Save waypoints to vehicle

### **Step 2: Select WP_MARK Mode**
In the UI Servo Control tab:
- Select "Waypoint Mark" from mode dropdown
- Configure spray settings if needed

### **Step 3: Click Run**
Press the Run button and the script automatically:
- ✅ Navigates to each waypoint (GUIDED mode)
- ✅ Stops at waypoint (HOLD mode)
- ✅ Sprays for configured duration
- ✅ Moves to next waypoint
- ✅ Repeats until complete

---

## Configuration

All settings are in `Backend/servo_manager/config.json` under `wpmark`:

```json
{
  "wpmark": {
    "servo_number": 10,           // Which servo to control
    "pwm_on": 1500,               // Spray ON (1000-2000)
    "pwm_off": 1000,              // Spray OFF (1000-2000)
    "delay_before_spray": 2.0,    // Wait after arrival (seconds)
    "spray_duration": 5.0,        // How long to spray (seconds)
    "delay_after_spray": 1.0,     // Wait before next WP (seconds)
    "waypoint_threshold": 2.0,    // Arrival distance (meters)
    "navigation_timeout": 120.0   // Max time per WP (seconds)
  }
}
```

---

## Example Usage

**Scenario:** Spray 5 trees in an orchard

1. **Plan Mission:**
   - Drive to first tree, mark waypoint (WP0)
   - Drive to second tree, mark waypoint (WP1)
   - Continue for all 5 trees
   - Upload mission to ArduPilot

2. **Configure Spray:**
   - Set `spray_duration: 3.0` (3 seconds per tree)
   - Set `pwm_on: 1500` (spray valve open)
   - Set `pwm_off: 1000` (spray valve closed)

3. **Run:**
   - Arm rover
   - Select "Waypoint Mark" mode
   - Click "Run"
   - Rover automatically drives to each tree and sprays

---

## Timeline at Each Waypoint

```
T+0s  : Arrive at waypoint (switch to HOLD)
T+2s  : Spray ON (after delay_before_spray)
T+7s  : Spray OFF (after spray_duration of 5s)
T+8s  : Navigate to next waypoint (after delay_after_spray)
```

---

## Tips

✅ **Test first:** Start with 2-3 waypoints to test  
✅ **Check threshold:** Adjust `waypoint_threshold` based on GPS accuracy  
✅ **Monitor logs:** Watch console for real-time progress  
✅ **Adjust timing:** Change delays based on your spray equipment  

❌ **Don't:** Start without mission uploaded  
❌ **Don't:** Run in AUTO mode (use WPMARK mode)  
❌ **Don't:** Forget to arm the vehicle  

---

## Troubleshooting

**Problem:** "No waypoints in mission"  
**Solution:** Upload mission first using Mission Planner or UI

**Problem:** Rover doesn't navigate  
**Solution:** Ensure vehicle is armed and GPS has good fix

**Problem:** Spray doesn't activate  
**Solution:** Check servo_number and PWM values in config

**Problem:** Takes too long at waypoint  
**Solution:** Reduce `waypoint_threshold` or increase timeout

---

## Safety

- Script automatically switches to HOLD mode on completion
- Servo turns OFF on script exit (cleanup)
- Can be stopped anytime from UI
- Emergency stop available in UI

---

**Simple, Automated, Reliable Waypoint Marking!** 🚀
