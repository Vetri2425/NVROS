# Summary Documentation

This folder contains organized documentation for the NRP ROS project, categorized by purpose.

## Folder Structure

### 📁 bugs/
**Purpose**: Bug reports and problem investigations

Contains detailed reports of issues discovered in the system, root cause analysis, and diagnostic findings.

**Currently Empty** - No standalone bug reports created. See `fixes/` for integrated problem-solution documentation.

---

### 📁 fixes/
**Purpose**: Solution documentation and implementation guides

Contains complete documentation of fixes applied to the system, including problem description, solution approach, and verification.

**Files**:
- `GPS_ALTITUDE_FIX_SUMMARY.md` - Initial altitude correction solution (deprecated)
- `GPS_ROS_TOPIC_FIX.md` - ROS topic-level fix approach
- `GPS_RAW_INTEGRATION_COMPLETE.md` - Final GPS raw topic integration (current solution)

---

### 📁 tests/
**Purpose**: Test scripts and diagnostic tools

Contains all Python scripts created for testing, diagnosis, verification, and monitoring of the system.

**Test Scripts**:
- `test_gps.py` - Monitor GPS with correct QoS settings
- `test_gps_raw_integration.py` - Integration test for GPS raw handler

**Diagnostic Scripts**:
- `check_all_gps.py` - Check multiple GPS topics
- `check_ardupilot_gps.py` - Check ArduPilot GPS status
- `check_mavlink_gcs.py` - Read raw MAVLink from GCS port
- `compare_gps.py` - Compare MAVLink vs MAVROS data
- `monitor_gps_updates.py` - Monitor GPS coordinate updates
- `monitor_telemetry_data.py` - Monitor telemetry/RTK panel data

**Verification Scripts**:
- `verify_altitude_fix.py` - Verify altitude correction calculation
- `verify_ros_topic_fix.py` - Compare original vs corrected topics

**Usage**:
```bash
# Run integration test
python3 summary/tests/test_gps_raw_integration.py

# Monitor GPS updates
python3 summary/tests/monitor_gps_updates.py

# Compare MAVLink vs MAVROS
python3 summary/tests/compare_gps.py
```

---

### 📁 setup/
**Purpose**: Setup guides and configuration documentation

Contains documentation for system setup, configuration, and service management.

**Files**:
- `QUICKSTART.md` - Quick start guide for the project
- `SERVICE_COMMANDS.md` - Systemd service management commands

---

### 📁 quick-summary/
**Purpose**: Quick reference cards and status pages

Contains concise, single-page summaries for quick reference during development or troubleshooting.

**Files**:
- `GPS_ISSUE_QUICK_REF.md` - One-page GPS issue reference
- `CURRENT_SYSTEM_STATUS.md` - Current system status snapshot

**When to Use**: Need quick answer without reading full documentation

---

### 📁 brief-summary/
**Purpose**: Technical summaries and architecture overviews

Contains medium-length technical summaries covering system architecture, major changes, and implementation details.

**Files**:
- `GPS_SYSTEM_FIX.md` - Complete GPS fix technical summary
- `SYSTEM_ARCHITECTURE.md` - System architecture and data flow

**When to Use**: Need technical details without diving into full implementation

---

### 📁 daily-reports/
**Purpose**: Chronological work logs and daily reports

Contains comprehensive daily reports documenting work completed, decisions made, and lessons learned.

**Files**:
- `2025-10-30_GPS_RAW_INTEGRATION.md` - GPS raw topic integration report

**Format**: Executive summary, work completed, results, impact, lessons learned

---

## Documentation Guidelines

### For Future Development

When creating new documentation, place files in appropriate folders:

1. **Bug Reports** → `bugs/`
   - Describe the problem
   - Include reproduction steps
   - Attach diagnostic data

2. **Solution Documentation** → `fixes/`
   - Problem description
   - Solution approach
   - Implementation details
   - Test results

3. **Test Scripts** → `tests/`
   - Diagnostic scripts
   - Integration tests
   - Monitoring tools
   - Verification scripts

4. **Setup Guides** → `setup/`
   - Installation instructions
   - Configuration guides
   - Service management

5. **Quick References** → `quick-summary/`
   - Single-page summaries
   - Status snapshots
   - Command cheat sheets

6. **Technical Summaries** → `brief-summary/`
   - Architecture docs
   - Technical overviews
   - Implementation summaries

7. **Daily Reports** → `daily-reports/`
   - Format: `YYYY-MM-DD_TOPIC.md`
   - Comprehensive work logs
   - Lessons learned

---

## Quick Access

### Current System Status
📄 `quick-summary/CURRENT_SYSTEM_STATUS.md`

### GPS Issue Reference
📄 `quick-summary/GPS_ISSUE_QUICK_REF.md`

### Latest Daily Report
📄 `daily-reports/2025-10-30_GPS_RAW_INTEGRATION.md`

### Run Tests
```bash
python3 summary/tests/test_gps_raw_integration.py
```

---

**Last Updated**: October 30, 2025  
**Project**: NRP ROS V2  
**Status**: Active development
