# Battery Status Fix Summary

## Overview
- Symptom: Frontend received `battery: -1` while rover reported `~0.975` (97.5%).
- Impact: UI showed no battery percentage even though the rover published valid data.

## Root Cause
1. MAVROS publishes `sensor_msgs/BatteryState.percentage` as a fraction in [0.0, 1.0].
2. Under some conditions, MAVROS can publish `NaN` (or `Inf`) for `percentage` when the value is temporarily unavailable.
3. The server logic attempted to convert the value to an integer percent, but:
   - When `percentage` was `NaN`, `int(round(NaN))` raised `ValueError`, which was caught, leaving `current_state.battery` unchanged at its default `-1`.
   - In the MAVROS bridge path, the bridge broadcasted the value under the key `battery` (already in percent after multiplying by 100), while the server only read `percentage` in some paths; this mismatch caused the server to skip updates.

## Fixes Implemented

### 1) MAVROS Bridge (`Backend/mavros_bridge.py`)
- Handler: `_handle_battery`
- Changes:
  - Guard `percentage` against `NaN/Inf`.
  - Multiply fraction to percent (×100) and broadcast under keys: `battery` (percent), `voltage`, `current` with `type="battery"`.
  - Added debug prints to trace incoming and outgoing values.

### 2) ROS2 Telemetry Node (`Backend/telemetry_node.py`)
- Handler: `battery_callback`
- Changes:
  - Validate `percentage`, `voltage`, `current` — ignore `NaN/Inf`.
  - Publish battery as `{ percentage: <fraction>, voltage, current }` on `/nrp/telemetry`.

### 3) Server (`Backend/server.py`)
- ROS2 path: `_merge_ros2_telemetry()`
  - Accept fractional `percentage` (0.0–1.0) or percent (0–100).
  - Convert to integer percentage, guard `NaN/Inf` and only update when valid.
- MAVROS path: `_handle_mavros_telemetry()`
  - Accept both `percentage` (fraction or percent) and `battery` (percent) fields.
  - Guard `NaN/Inf` and round to an integer percent.
  - Also record `voltage` and `current` if valid.
  - Added a safety fallback to process battery fields whenever present, even if the message `type` isn’t explicitly `"battery"`.
- Stability:
  - Avoid exceptions that previously left `current_state.battery` at `-1`.

## Validation
- Verified `/mavros/battery` contains valid data (example):
  - `percentage: 0.98`, `voltage: 0.216`, `current: -0.26`.
- Ran the test client `Backend/Test_Emit_Data.py`:
  - Before: `"battery": -1`.
  - After: `"battery": 98` (correctly converted and emitted).

## Notes
- Conversion rules:
  - If `percentage <= 1.0` → treat as fraction, multiply by 100, round to nearest integer.
  - Else → treat as already in percent, round to nearest integer.
- Invalid values (`NaN/Inf`) are skipped; the last valid battery reading is retained instead of resetting to `-1`.
- The server now supports both telemetry sources consistently:
  - ROS2 telemetry node via `/nrp/telemetry` (fraction-based `percentage`).
  - MAVROS bridge broadcast (percent-based `battery`).

## How to Reproduce/Verify
1. Confirm MAVROS battery topic:
   - `ros2 topic echo /mavros/battery --once`
   - Expect `percentage` ≈ `0.97..`.
2. Run test client:
   - `python3 Backend/Test_Emit_Data.py`
   - Expect `"battery": <positive integer like 98>` in the printed `rover_data`.

## Files Touched
- `Backend/mavros_bridge.py`: Robust battery parsing and debug logs.
- `Backend/telemetry_node.py`: Validate and publish sane battery metrics.
- `Backend/server.py`: Accept `percentage`/`battery`, guard invalid values, fallback processing.

