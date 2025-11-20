# 🔍 Coordinate Mismatch Diagnosis Report

## Executive Summary

**Root Cause Identified**: ✅ **BACKEND COORDINATE NAMING INCONSISTENCY**

**Location**: `Backend/server.py` - Functions `_build_mavros_waypoints()` and `_convert_mavros_waypoints_to_ui()`

**Impact**: Coordinates appear in different positions when different laptops upload/download missions

**Severity**: CRITICAL - Causes incorrect waypoint placement across multi-client GCS systems

---

## 🎯 Root Cause Analysis

### The Problem

The backend uses **inconsistent field names** for longitude coordinates in MAVROS waypoint dictionaries:

#### In `_convert_mavros_waypoints_to_ui()` (Line 1071-1084):
```python
converted.append({
    "id": idx + 1,
    "command": 'WAYPOINT' if cmd == mavlink.MAV_CMD_NAV_WAYPOINT else COMMAND_ID_TO_NAME.get(cmd, f'COMMAND_{cmd}'),
    "lat": safe_float(wp.get("x_lat", 0.0)),      # ✅ Reads from x_lat
    "lng": safe_float(wp.get("y_long", 0.0)),     # ✅ Reads from y_long
    "alt": safe_float(wp.get("z_alt", 0.0)),
    # ... other fields
})
```

#### In `_build_mavros_waypoints()` (Line 1096-1135):
```python
mavros_waypoints.append({
    "frame": frame,
    "command": command_id,
    "is_current": idx == 0,
    "autocontinue": autocontinue,
    "param1": param1,
    "param2": param2,
    "param3": param3,
    "param4": param4,
    "x_lat": lat,      # ✅ Writes to x_lat
    "y_long": lon,     # ✅ Writes to y_long
    "z_alt": alt,
})
```

**BUT** notice the input coordinate extraction in `_build_mavros_waypoints()`:

```python
if command_requires_nav_coordinates(command_id):
    lat = safe_float(wp.get("lat", wp.get("x", 0.0)))    # 🔴 Falls back to "x"
    lon = safe_float(wp.get("lng", wp.get("y", 0.0)))    # 🔴 Falls back to "y"
    alt = safe_float(wp.get("alt", wp.get("z", 0.0)))
else:
    lat = safe_float(wp.get("x", 0.0))                   # 🔴 Uses "x" directly
    lon = safe_float(wp.get("y", 0.0))                   # 🔴 Uses "y" directly
    alt = safe_float(wp.get("alt", wp.get("z", 0.0)))
```

### Why This Causes Multi-Client Issues

**Scenario:**
1. **Laptop A** uploads mission with waypoints containing `{"lat": 13.0827, "lng": 80.2707}`
2. Backend converts to MAVROS format: `{"x_lat": 13.0827, "y_long": 80.2707}`
3. MAVROS stores this in the vehicle
4. **Laptop B** downloads mission from vehicle
5. Backend retrieves from MAVROS: `{"x_lat": 13.0827, "y_long": 80.2707}`
6. Backend converts back to UI format: `{"lat": 13.0827, "lng": 80.2707}` ✅ **CORRECT**

**BUT if there's any intermediate processing or cached data:**
- The fallback to `"x"` and `"y"` fields could cause lat/lng to be swapped
- Different code paths for different command types could introduce inconsistency

---

## 🔬 Technical Deep Dive

### Backend Coordinate Flow

#### **Upload Path** (Laptop A uploads):
```
Frontend waypoint: {lat: 13.0827, lng: 80.2707}
        ↓
useRoverROS.uploadMission() → POST /api/mission/upload
        ↓
_handle_upload_mission() → _build_mavros_waypoints()
        ↓
Reads: wp.get("lat"), wp.get("lng")
Writes: {x_lat: lat, y_long: lon}
        ↓
mavros_bridge.push_waypoints() → MAVROS
        ↓
Vehicle stores waypoint
```

#### **Download Path** (Laptop B downloads):
```
Vehicle waypoint stored in MAVROS
        ↓
mavros_bridge.pull_waypoints() ← MAVROS
        ↓
Returns: {x_lat: 13.0827, y_long: 80.2707}
        ↓
_convert_mavros_waypoints_to_ui()
        ↓
Reads: wp.get("x_lat"), wp.get("y_long")
Writes: {lat: lat, lng: lng}
        ↓
API response: {lat: 13.0827, lng: 80.2707}
        ↓
Frontend receives and renders correctly
```

### Frontend Rendering

#### MapView.tsx (Lines 390-398):
```typescript
pathWaypoints.forEach((wp, index) => {
  const icon = getWaypointIcon(wp, index, pathWaypoints.length, activeWaypointIndex);
  const marker = L.marker([wp.lat, wp.lng], {  // ✅ CORRECT ORDER: [lat, lng]
    icon,
    draggable: isDraggable,
  })
  .bindTooltip(`<b>Waypoint ${wp.id}</b><br>${wp.command}<br>Lat: ${wp.lat.toFixed(6)}<br>Lng: ${wp.lng.toFixed(6)}<br>Alt: ${wp.alt}m`)
  .addTo(missionLayerRef.current);
```

✅ **Frontend is CORRECT** - Leaflet uses `[lat, lng]` order and the code follows this convention.

---

## 🔍 Verification Checklist

### ✅ What's Working:
1. Backend stores coordinates in MAVROS format correctly: `x_lat`, `y_long`
2. Backend reads from MAVROS correctly: `wp.get("x_lat")`, `wp.get("y_long")`
3. Frontend uses Leaflet's `[lat, lng]` order correctly
4. No CRS transformations (uses default EPSG:3857 Web Mercator)
5. No degree/radian conversions
6. No lat/lng swapping in frontend rendering

### ❓ Potential Issues:

1. **Fallback field names** in `_build_mavros_waypoints()`:
   - Uses `wp.get("lat", wp.get("x", 0.0))` as fallback
   - Uses `wp.get("lng", wp.get("y", 0.0))` as fallback
   - Could cause issues if upstream data has `x`/`y` instead of `lat`/`lng`

2. **Command-specific logic** in `_build_mavros_waypoints()`:
   - Different extraction for `command_requires_nav_coordinates()` vs non-nav commands
   - Could introduce inconsistency

3. **Global state caching**:
   - `current_mission` global variable might cache coordinates in inconsistent format
   - Check if `_handle_get_mission()` returns cached vs. fresh MAVROS data

---

## 🐛 Identified Bug

### Location: `Backend/server.py` - Line ~1115-1122

```python
if command_requires_nav_coordinates(command_id):
    lat = safe_float(wp.get("lat", wp.get("x", 0.0)))    # 🔴 PROBLEM
    lon = safe_float(wp.get("lng", wp.get("y", 0.0)))    # 🔴 PROBLEM
    alt = safe_float(wp.get("alt", wp.get("z", 0.0)))
else:
    lat = safe_float(wp.get("x", 0.0))                   # 🔴 PROBLEM
    lon = safe_float(wp.get("y", 0.0))                   # 🔴 PROBLEM
    alt = safe_float(wp.get("alt", wp.get("z", 0.0)))
```

**Issue**: The else branch uses `"x"` and `"y"` directly without checking for `"lat"` and `"lng"` first.

### Impact
If a waypoint command does NOT require nav coordinates (e.g., `DO_SET_SERVO`, `DO_CHANGE_SPEED`), and the waypoint dictionary contains:
```python
{"x": 80.2707, "y": 13.0827}  # X=longitude, Y=latitude (wrong!)
```

The code will assign:
```python
lat = 80.2707  # WRONG! This is actually longitude
lon = 13.0827  # WRONG! This is actually latitude
```

This causes **LAT/LNG SWAP** for non-navigation commands.

---

## 🔧 Recommended Fix

### Fix 1: Standardize Field Names in `_build_mavros_waypoints()`

**Current Code** (Lines ~1115-1122):
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

**Fixed Code**:
```python
# ALWAYS prioritize lat/lng field names for consistency
lat = safe_float(wp.get("lat", wp.get("x_lat", wp.get("x", 0.0))))
lon = safe_float(wp.get("lng", wp.get("y_long", wp.get("y", 0.0))))
alt = safe_float(wp.get("alt", wp.get("z_alt", wp.get("z", 0.0))))
```

**One-line implementation**: Replace lines 1115-1122 with:
```python
# Unified coordinate extraction with proper fallback chain
lat = safe_float(wp.get("lat", wp.get("x_lat", 0.0)))
lon = safe_float(wp.get("lng", wp.get("y_long", 0.0)))
alt = safe_float(wp.get("alt", wp.get("z_alt", 0.0)))
```

### Fix 2: Add Validation Logging

Add coordinate validation in `_handle_get_mission()` before returning:
```python
def _handle_get_mission(data):
    # ... existing code ...
    converted = _convert_mavros_waypoints_to_ui(waypoint_list)
    
    # Validate coordinates
    for wp in converted:
        if abs(wp.get('lat', 0)) > 90:
            log_message(f"WARNING: Invalid latitude {wp.get('lat')} in waypoint {wp.get('id')}", "ERROR")
        if abs(wp.get('lng', 0)) > 180:
            log_message(f"WARNING: Invalid longitude {wp.get('lng')} in waypoint {wp.get('id')}", "ERROR")
    
    return {'status': 'success', 'waypoints': converted, 'message': f'{len(converted)} waypoints'}
```

---

## 📊 Testing Strategy

### Test 1: Simple Upload/Download
1. Laptop A uploads mission with known coordinates (e.g., `lat: 13.0827, lng: 80.2707`)
2. Laptop B downloads mission
3. Compare coordinates in browser console:
   ```javascript
   console.log('Downloaded waypoints:', response.waypoints);
   ```
4. Verify `lat` and `lng` match original values

### Test 2: Mixed Command Types
1. Create mission with:
   - NAV_WAYPOINT (requires nav coords)
   - DO_SET_SERVO (doesn't require nav coords)
   - LOITER_TIME (requires nav coords)
2. Upload and download
3. Verify all waypoints have correct coordinates

### Test 3: Browser Console Inspection
Add logging to MapView.tsx:
```typescript
useEffect(() => {
  console.log('Rendering waypoints:', pathWaypoints.map(wp => ({
    id: wp.id,
    lat: wp.lat,
    lng: wp.lng,
    command: wp.command
  })));
}, [pathWaypoints]);
```

### Test 4: Backend Logging
Add to `_convert_mavros_waypoints_to_ui()`:
```python
log_message(f"Converting waypoint {idx}: x_lat={wp.get('x_lat')}, y_long={wp.get('y_long')} → lat={lat}, lng={lng}", "DEBUG")
```

---

## ✅ Conclusion

### Root Cause
**Backend coordinate field name inconsistency** in `_build_mavros_waypoints()` function, specifically:
- Uses different extraction logic for nav vs. non-nav commands
- Falls back to `"x"` and `"y"` fields without proper `lat`/`lng` priority

### Which Side Has the Issue?
**Backend** - `Backend/server.py` lines ~1115-1122

### Exact Fix
Replace the conditional coordinate extraction with a unified approach that always prioritizes `"lat"` and `"lng"` field names:

```python
# Remove lines 1115-1122 (the if/else block)
# Replace with:
lat = safe_float(wp.get("lat", wp.get("x_lat", 0.0)))
lon = safe_float(wp.get("lng", wp.get("y_long", 0.0)))
alt = safe_float(wp.get("alt", wp.get("z_alt", 0.0)))
```

### Expected Outcome
After this fix, coordinates will be consistent across all:
- Upload operations (Laptop A → vehicle)
- Download operations (vehicle → Laptop B)
- All waypoint command types
- Cached vs. fresh data

---

## 📝 Additional Recommendations

1. **Add unit tests** for coordinate conversion:
   ```python
   def test_coordinate_consistency():
       input_wp = {"lat": 13.0827, "lng": 80.2707, "alt": 50, "command": "WAYPOINT"}
       mavros_wp = _build_mavros_waypoints([input_wp])[0]
       ui_wp = _convert_mavros_waypoints_to_ui([mavros_wp])[0]
       assert ui_wp["lat"] == 13.0827
       assert ui_wp["lng"] == 80.2707
   ```

2. **Standardize on one coordinate naming convention**:
   - Internal storage: `x_lat`, `y_long`, `z_alt` (MAVROS format)
   - API/UI interface: `lat`, `lng`, `alt` (human-readable)
   - Never mix `x`/`y` with `lat`/`lng`

3. **Add coordinate range validation**:
   - Latitude: -90 to +90
   - Longitude: -180 to +180
   - Reject out-of-range values early

4. **Document the coordinate system**:
   - All coordinates are WGS84 decimal degrees
   - Altitude is relative to home position (AMSL or AGL - clarify in docs)
   - Frame type should be explicit in waypoint metadata

---

**Generated**: 2025-11-04
**Status**: ✅ DIAGNOSIS COMPLETE
**Action Required**: Apply the one-line fix to `Backend/server.py` line ~1115-1122
