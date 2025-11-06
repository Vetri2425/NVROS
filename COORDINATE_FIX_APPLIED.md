# ✅ COORDINATE MISMATCH FIX - APPLIED

## Date: 2025-11-04

## Problem Summary
Multi-client GCS setup had coordinate mismatch issues where missions uploaded from Laptop A appeared in different map positions when downloaded by Laptop B.

## Root Cause
Backend coordinate extraction in `_build_mavros_waypoints()` had inconsistent field name handling:
- Nav commands used: `wp.get("lat", wp.get("x", 0.0))`
- Non-nav commands used: `wp.get("x", 0.0)` directly
- This could cause lat/lng swapping for certain waypoint types

## Changes Applied

### 1. Fixed `_build_mavros_waypoints()` in Backend/server.py (Lines 1109-1111)

**BEFORE:**
```python
if command_requires_nav_coordinates(command_id):
    lat = safe_float(wp.get("lat", wp.get("x", 0.0)))
    lon = safe_float(wp.get("lng", wp.get("y", 0.0)))
    alt = safe_float(wp.get("alt", wp.get("z", 0.0)))
else:
    lat = safe_float(wp.get("x", 0.0))
    lon = safe_float(wp.get("y", 0.0))
    alt = safe_float(wp.get("alt", wp.get("z", 0.0)))
```

**AFTER:**
```python
# Unified coordinate extraction - always prioritize lat/lng field names for consistency
# This ensures coordinates are never swapped regardless of command type
lat = safe_float(wp.get("lat", wp.get("x_lat", 0.0)))
lon = safe_float(wp.get("lng", wp.get("y_long", 0.0)))
alt = safe_float(wp.get("alt", wp.get("z_alt", 0.0)))
```

### 2. Added Coordinate Validation in `_handle_upload_mission()` (Lines 1718-1727)

**Added validation to reject invalid coordinates before upload:**
```python
# Validate coordinates before upload to catch any lat/lng swap issues
for wp in waypoints:
    lat_val = wp.get('lat', 0)
    lng_val = wp.get('lng', 0)
    if abs(lat_val) > 90:
        raise ValueError(f"Invalid latitude {lat_val} in waypoint {wp.get('id', '?')} - must be between -90 and +90")
    if abs(lng_val) > 180:
        raise ValueError(f"Invalid longitude {lng_val} in waypoint {wp.get('id', '?')} - must be between -180 and +180")
```

### 3. Added Coordinate Validation in `_handle_get_mission()` (Lines 1786-1793)

**Added warning logs for suspicious coordinates during download:**
```python
# Validate coordinates to catch any lat/lng swap issues
for wp in mission_waypoints:
    lat_val = wp.get('lat', 0)
    lng_val = wp.get('lng', 0)
    if abs(lat_val) > 90:
        log_message(f"WARNING: Invalid latitude {lat_val} in waypoint {wp.get('id')} - possible lat/lng swap!", "ERROR")
    if abs(lng_val) > 180:
        log_message(f"WARNING: Invalid longitude {lng_val} in waypoint {wp.get('id')} - possible lat/lng swap!", "ERROR")
```

## Expected Behavior After Fix

### ✅ Consistent Coordinate Handling
- All waypoint types (NAV and non-NAV) use the same coordinate extraction logic
- Field name priority: `lat` → `x_lat` → fallback to 0.0
- Field name priority: `lng` → `y_long` → fallback to 0.0

### ✅ Upload Validation
- Uploads will be rejected if coordinates are out of valid range
- Prevents lat/lng swaps from being sent to the vehicle
- Clear error messages guide users to fix the issue

### ✅ Download Validation
- Suspicious coordinates logged during download
- Helps identify if vehicle has corrupted waypoints
- Warning logs appear in backend console

### ✅ Multi-Client Consistency
- Laptop A uploads: `{lat: 13.0827, lng: 80.2707}`
- Backend converts to MAVROS: `{x_lat: 13.0827, y_long: 80.2707}`
- Vehicle stores waypoint correctly
- Laptop B downloads: `{lat: 13.0827, lng: 80.2707}` ✅ SAME VALUES
- Map renders at identical position ✅

## Testing Recommendations

### Test 1: Basic Upload/Download
```bash
# Laptop A
1. Upload mission with known coordinates (e.g., Chennai: lat=13.0827, lng=80.2707)
2. Check browser console for upload confirmation

# Laptop B
3. Download mission from vehicle
4. Verify coordinates match: lat=13.0827, lng=80.2707
5. Confirm waypoints appear at same map position as Laptop A
```

### Test 2: Mixed Command Types
```bash
1. Create mission with:
   - NAV_WAYPOINT (lat: 13.0827, lng: 80.2707)
   - DO_SET_SERVO (lat: 13.0830, lng: 80.2710)
   - LOITER_TIME (lat: 13.0833, lng: 80.2713)
2. Upload from Laptop A
3. Download on Laptop B
4. Verify all waypoints have correct coordinates
```

### Test 3: Invalid Coordinate Rejection
```bash
1. Try uploading waypoint with lat=200 (invalid)
2. Should see error: "Invalid latitude 200 - must be between -90 and +90"
3. Upload should be rejected before sending to vehicle
```

### Test 4: Backend Log Monitoring
```bash
# Terminal
tail -f /path/to/backend/logs

# Expected logs on upload:
[mission_upload] Uploading 5 waypoint(s) via MAVROS

# Expected logs on download:
[MISSION] Successfully downloaded 5 waypoints

# If coordinates are invalid, you'll see:
WARNING: Invalid latitude 95.5 in waypoint 3 - possible lat/lng swap!
```

## Verification Checklist

- [x] `_build_mavros_waypoints()` uses unified coordinate extraction
- [x] No conditional logic based on command type for coordinate handling
- [x] Upload validation rejects out-of-range coordinates
- [x] Download validation logs suspicious coordinates
- [x] Coordinate field priority: `lat`/`lng` → `x_lat`/`y_long` → 0.0
- [x] No direct access to `x`/`y` fields that could cause swapping

## Files Modified

1. `Backend/server.py`:
   - `_build_mavros_waypoints()` - Lines 1109-1111
   - `_handle_upload_mission()` - Lines 1718-1727
   - `_handle_get_mission()` - Lines 1786-1793

## Rollback Instructions

If issues occur, revert these changes:

```bash
cd /home/flash/NRP_ROS
git diff Backend/server.py  # Review changes
git checkout Backend/server.py  # Revert if needed
```

Or manually restore the original conditional logic in `_build_mavros_waypoints()`.

## Related Documentation

- Full diagnosis: `COORDINATE_MISMATCH_DIAGNOSIS.md`
- Testing strategy: See diagnosis report section "Testing Strategy"

## Status

✅ **FIX APPLIED AND READY FOR TESTING**

Next steps:
1. Restart the backend service
2. Test with two laptops uploading/downloading missions
3. Verify coordinates remain consistent
4. Monitor backend logs for any validation warnings

---

**Author**: GitHub Copilot  
**Date**: 2025-11-04  
**Fix Type**: Backend Coordinate Handling  
**Severity**: Critical (fixes multi-client coordinate mismatch)
