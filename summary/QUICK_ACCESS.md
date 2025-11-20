# Documentation Quick Access Cheat Sheet

**Location**: `/home/flash/NRP_ROS/summary/`

---

## 📍 I Need To...

### ...check system status RIGHT NOW
```bash
cat summary/quick-summary/CURRENT_SYSTEM_STATUS.md
```

### ...understand the GPS issue
```bash
cat summary/quick-summary/GPS_ISSUE_QUICK_REF.md
```

### ...run the GPS test
```bash
python3 summary/tests/test_gps_raw_integration.py
```

### ...see live GPS data
```bash
ros2 topic echo /mavros/gpsstatus/gps1/raw --once
```

### ...restart the services
```bash
sudo systemctl restart rosbridge
sudo systemctl status rosbridge
```

### ...find all documentation
```bash
cat DOCUMENTATION_INDEX.md
```

### ...understand system architecture
```bash
cat summary/brief-summary/SYSTEM_ARCHITECTURE.md
```

### ...see what was done today
```bash
cat summary/daily-reports/2025-10-30_GPS_RAW_INTEGRATION.md
```

### ...read the GPS fix details
```bash
cat summary/fixes/GPS_RAW_INTEGRATION_COMPLETE.md
```

### ...create a new bug report
```bash
# Use template in:
cat summary/bugs/README.md
```

### ...create a new fix document
```bash
# Use template in:
cat summary/fixes/README.md
```

### ...monitor telemetry
```bash
python3 summary/tests/monitor_telemetry_data.py
```

### ...view service logs
```bash
journalctl -u rosbridge -f
```

---

## 📂 Folder Purposes (Quick Ref)

| Folder | What's In It | When To Use |
|--------|--------------|-------------|
| `bugs/` | Problem reports | Found a bug? Document it here |
| `fixes/` | Solution docs | Fixed something? Document it here |
| `tests/` | Test scripts | Need to test? Scripts are here |
| `setup/` | Setup guides | Setting up? Start here |
| `quick-summary/` | 1-page refs | Need fast answer? Check here |
| `brief-summary/` | Tech overviews | Need tech details? Read here |
| `daily-reports/` | Work logs | What was done? Check here |

---

## 🔑 Most Important Files

1. **DOCUMENTATION_INDEX.md** → Master navigation
2. **summary/README.md** → Summary navigation
3. **summary/quick-summary/CURRENT_SYSTEM_STATUS.md** → System status
4. **summary/brief-summary/SYSTEM_ARCHITECTURE.md** → Architecture
5. **summary/tests/test_gps_raw_integration.py** → Main test

---

## ⚡ Emergency Commands

```bash
# Service crashed? Restart it
sudo systemctl restart rosbridge

# GPS not working? Check raw topic
ros2 topic echo /mavros/gpsstatus/gps1/raw --once

# Backend issues? Check logs
journalctl -u rosbridge -n 50 --no-pager

# Tests failing? Run integration test
python3 summary/tests/test_gps_raw_integration.py

# Need all topics? List them
ros2 topic list | grep mavros
```

---

**Keep this file handy for quick access!**

**File**: `/home/flash/NRP_ROS/QUICK_ACCESS.md`
