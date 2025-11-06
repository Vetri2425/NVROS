# GPS Issue - Quick Reference

## Problem
- ❌ Altitude showed **-76.5m** (wrong)
- ❌ GPS Status showed **"No Fix"** (wrong)
- ❌ Satellites showed **1** (wrong)

## Solution
Changed GPS data source from buggy NavSatFix topics to raw GPS topic.

## Fix Applied
**File**: `Backend/mavros_bridge.py`
- Subscribed to `/mavros/gpsstatus/gps1/raw`
- Created `_handle_gps_raw()` handler
- Converts: lat/lon (÷1e7), alt (÷1000), accuracy (÷100)

## Result
- ✅ Altitude: **+16.6m** (CORRECT!)
- ✅ GPS Status: **"RTK Fixed"** (CORRECT!)
- ✅ Satellites: **29** (CORRECT!)

## Test Command
```bash
python3 summary/tests/test_gps_raw_integration.py
```

## Status
✅ **FIXED** - October 30, 2025
