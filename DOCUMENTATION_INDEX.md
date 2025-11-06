# NRP ROS Documentation Index

**Project**: NRP ROS V2 (V3_Oct_29)  
**Last Updated**: October 30, 2025  
**Status**: Active Development

---

## 📚 Documentation Structure

```
summary/
├── README.md                    # This file - documentation index
├── bugs/                        # Bug reports and problem investigations
├── fixes/                       # Solution documentation
├── tests/                       # Test scripts and diagnostic tools
├── setup/                       # Setup guides and configuration
├── quick-summary/               # Quick reference cards (< 1 min read)
├── brief-summary/               # Technical summaries (5-15 min read)
└── daily-reports/               # Comprehensive daily work logs
```

---

## 🚀 Quick Start

**New to the project?** Start here:

1. **System Overview**: `brief-summary/SYSTEM_ARCHITECTURE.md`
2. **Current Status**: `quick-summary/CURRENT_SYSTEM_STATUS.md`
3. **Setup Guide**: `setup/QUICKSTART.md`

---

## 🔍 Find Documentation By Purpose

### I want to...

#### ...understand a problem
📁 **Go to**: `bugs/`
- Bug reports
- Root cause analysis
- Diagnostic findings

#### ...implement a fix
📁 **Go to**: `fixes/`
- Solution documentation
- Implementation guides
- Fix verification

#### ...run tests
📁 **Go to**: `tests/`
- Test scripts
- Diagnostic tools
- Monitoring utilities

#### ...set up the system
📁 **Go to**: `setup/`
- Installation guides
- Configuration docs
- Service management

#### ...get quick info
📁 **Go to**: `quick-summary/`
- One-page references
- Status snapshots
- Quick commands

#### ...understand architecture
📁 **Go to**: `brief-summary/`
- System design
- Component overviews
- Technical summaries

#### ...see what was done recently
📁 **Go to**: `daily-reports/`
- Daily work logs
- Progress reports
- Lessons learned

---

## 📂 Detailed Folder Guide

### bugs/ - Problem Reports
**Purpose**: Document issues and investigations

**Currently**: Empty (bugs documented in fixes/)

**Use When**:
- Discovered a new bug
- Investigating an issue
- Documenting root cause

**README**: `bugs/README.md`

---

### fixes/ ⭐ - Solution Documentation
**Purpose**: Complete fix documentation

**Current Files**:
- `GPS_ALTITUDE_FIX_SUMMARY.md` (deprecated)
- `GPS_ROS_TOPIC_FIX.md` (deprecated)
- `GPS_RAW_INTEGRATION_COMPLETE.md` ✅ **Current solution**

**Use When**:
- Need to understand how something was fixed
- Implementing similar solutions
- Verifying fix approaches

**README**: `fixes/README.md`

---

### tests/ - Test Scripts & Diagnostics
**Purpose**: Testing and diagnostic tools

**Test Scripts** (2):
- `test_gps.py` - Monitor GPS with correct QoS
- `test_gps_raw_integration.py` ⭐ - Integration test

**Diagnostic Scripts** (4):
- `check_all_gps.py` - Check multiple GPS topics
- `check_ardupilot_gps.py` - Check ArduPilot GPS
- `check_mavlink_gcs.py` - Read raw MAVLink
- `compare_gps.py` - Compare MAVLink vs MAVROS

**Monitoring Scripts** (2):
- `monitor_gps_updates.py` - Monitor coordinate updates
- `monitor_telemetry_data.py` - Monitor telemetry panels

**Verification Scripts** (2):
- `verify_altitude_fix.py` - Verify correction
- `verify_ros_topic_fix.py` - Compare topics

**Quick Test**:
```bash
python3 summary/tests/test_gps_raw_integration.py
```

**README**: `tests/README.md`

---

### setup/ - Configuration & Setup
**Purpose**: Setup and service management

**Files**:
- `QUICKSTART.md` - Quick start guide
- `SERVICE_COMMANDS.md` - Service management

**Common Commands**:
```bash
sudo systemctl status rosbridge
sudo systemctl restart rosbridge
journalctl -u rosbridge -f
```

**README**: `setup/README.md`

---

### quick-summary/ - Quick References
**Purpose**: Rapid reference (< 1 minute reads)

**Files**:
- `GPS_ISSUE_QUICK_REF.md` ⭐ - GPS fix one-pager
- `CURRENT_SYSTEM_STATUS.md` 📊 - System status snapshot

**Use When**:
- Need quick answer
- Status check
- Before starting work

**README**: `quick-summary/README.md`

---

### brief-summary/ - Technical Overviews
**Purpose**: Technical summaries (5-15 minute reads)

**Files**:
- `GPS_SYSTEM_FIX.md` - GPS fix technical summary
- `SYSTEM_ARCHITECTURE.md` ⭐ - System architecture overview

**Use When**:
- Onboarding developers
- Understanding design
- Planning features

**README**: `brief-summary/README.md`

---

### daily-reports/ - Work Logs
**Purpose**: Comprehensive daily reports

**Files**:
- `2025-10-30_GPS_RAW_INTEGRATION.md` ⭐ - GPS integration report

**Use When**:
- Reviewing recent work
- Understanding decisions
- Learning from experience

**README**: `daily-reports/README.md`

---

## 🎯 Common Tasks

### Check System Status
```bash
cat summary/quick-summary/CURRENT_SYSTEM_STATUS.md
```

### Run GPS Test
```bash
python3 summary/tests/test_gps_raw_integration.py
```

### View Latest Daily Report
```bash
ls -lt summary/daily-reports/ | head -2
```

### Restart Services
```bash
sudo systemctl restart rosbridge
```

### Monitor GPS Data
```bash
ros2 topic echo /mavros/gpsstatus/gps1/raw --once
```

---

## 📊 Documentation Statistics

| Category | Files | Purpose |
|----------|-------|---------|
| Bugs | 0 | Problem reports |
| Fixes | 3 | Solution docs |
| Tests | 10 | Test scripts |
| Setup | 2 | Setup guides |
| Quick Summary | 2 | Quick refs |
| Brief Summary | 2 | Tech overviews |
| Daily Reports | 1 | Work logs |
| **Total** | **20** | **All documentation** |

---

## 🔄 Update Guidelines

### When to Update Documentation

1. **After Major Changes**
   - Update relevant fix documentation
   - Create/update daily report
   - Update quick summaries

2. **New Features**
   - Add to brief summary
   - Create test scripts
   - Update status page

3. **Bug Fixes**
   - Document in fixes/
   - Create tests
   - Update quick refs

4. **Daily Work**
   - Create daily report
   - Update status page
   - Add test scripts

### Documentation Workflow

```
Work Done
    ↓
Create/Update Daily Report
    ↓
Update Quick Summary (if status changed)
    ↓
Update Brief Summary (if architecture changed)
    ↓
Create/Update Fix Documentation
    ↓
Add Test Scripts
```

---

## 📝 Documentation Templates

All folders contain README files with templates:

- `bugs/README.md` - Bug report template
- `fixes/README.md` - Fix documentation template
- `brief-summary/README.md` - Technical summary template
- `daily-reports/README.md` - Daily report template

---

## 🔗 External References

### Source Code
- Backend: `/home/flash/NRP_ROS/Backend/`
- Frontend: `/home/flash/NRP_ROS/src/`

### Configuration
- Service: `/home/flash/NRP_ROS/start_service.sh`
- Main README: `/home/flash/NRP_ROS/README.md`

### Project Documentation
- Project Report: `/home/flash/NRP_ROS/PROJECT_REPORT.html`
- Integration Summary: `/home/flash/NRP_ROS/UI_ROS_BACKEND_INTEGRATION_SUMMARY.md`

---

## 📞 Documentation Maintenance

**Maintained By**: Development Team  
**Review Frequency**: After significant changes  
**Last Major Update**: October 30, 2025 (GPS Raw Integration)

---

## 🎓 Best Practices

1. **Keep It Organized**
   - Files in correct folders
   - Follow naming conventions
   - Use templates

2. **Keep It Current**
   - Update after changes
   - Review regularly
   - Archive old versions

3. **Keep It Useful**
   - Write for readers
   - Include examples
   - Link related docs

4. **Keep It Discoverable**
   - Clear filenames
   - Good README files
   - This index!

---

**Index**: `summary/README.md`  
**Created**: October 30, 2025  
**Purpose**: Central documentation navigation
