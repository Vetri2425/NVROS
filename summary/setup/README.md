# Setup & Configuration Documentation

This folder contains setup guides, configuration instructions, and service management documentation for the NRP ROS system.

## Files

### QUICKSTART.md
**Purpose**: Quick start guide for getting the system running

**Contents**:
- System requirements
- Installation steps
- First-time setup
- Basic usage instructions

**When to Use**: Setting up the system for the first time or after a fresh install

---

### SERVICE_COMMANDS.md
**Purpose**: Service management commands reference

**Contents**:
- Systemd service commands
- Start/stop/restart procedures
- Status checking
- Log viewing

**Common Commands**:
```bash
# Start the service
sudo systemctl start rosbridge

# Stop the service
sudo systemctl stop rosbridge

# Restart the service
sudo systemctl restart rosbridge

# Check status
sudo systemctl status rosbridge

# View logs
journalctl -u rosbridge -f
```

---

## System Architecture

### Service Stack

```
rosbridge.service (systemd)
    └─ start_service.sh
        ├─ rosbridge_websocket (port 9090)
        ├─ mavros_node (/mavros)
        ├─ gps_altitude_corrector.py (optional)
        ├─ telemetry_node.py
        └─ server.py (Flask backend)
```

### Data Flow

```
GPS Hardware → ArduPilot → MAVROS → ROS Topics → Backend → UI
```

---

## Quick Setup Reference

### Prerequisites

- Ubuntu (tested on 22.04)
- ROS2 Humble
- Python 3.10+
- Node.js 18+

### Install Dependencies

```bash
# ROS2 packages
sudo apt install ros-humble-mavros ros-humble-rosbridge-server

# Python packages
cd /home/flash/NRP_ROS
pip3 install -r Backend/requirements.txt

# Node packages
npm install
```

### Configure Services

```bash
# Enable service autostart
sudo systemctl enable rosbridge

# Start the service
sudo systemctl start rosbridge

# Check status
sudo systemctl status rosbridge
```

### Verify System

```bash
# Check ROS topics
ros2 topic list | grep mavros

# Check GPS data
ros2 topic echo /mavros/gpsstatus/gps1/raw --once

# Check backend logs
journalctl -u rosbridge -f
```

---

## Configuration Files

### Backend Configuration

**Backend/server.py**:
- Flask server configuration
- WebSocket settings
- CORS configuration

**Backend/mavros_bridge.py**:
- ROS topic subscriptions
- Telemetry handlers
- Service clients

### System Configuration

**start_service.sh**:
- Service startup script
- Process management
- Environment variables

**systemd/rosbridge.service**:
- System service definition
- Auto-start configuration
- Process monitoring

---

## Troubleshooting

### Service Won't Start

```bash
# Check service status
sudo systemctl status rosbridge

# View detailed logs
journalctl -u rosbridge -n 100 --no-pager

# Check for port conflicts
sudo netstat -tlnp | grep 9090
```

### GPS Data Issues

```bash
# Check MAVROS connection
ros2 topic echo /mavros/state --once

# Check GPS raw topic
ros2 topic echo /mavros/gpsstatus/gps1/raw --once

# Run diagnostic tests
python3 summary/tests/test_gps_raw_integration.py
```

### Backend Connection Issues

```bash
# Check rosbridge
ros2 node list | grep rosbridge

# Check backend process
ps aux | grep server.py

# Restart service
sudo systemctl restart rosbridge
```

---

## For Future Setup Documentation

When adding new setup documentation, include:

1. **Prerequisites**
   - Required software/hardware
   - Version requirements
   - Dependencies

2. **Step-by-Step Instructions**
   - Clear, numbered steps
   - Copy-pasteable commands
   - Expected outputs

3. **Verification**
   - How to verify each step
   - What success looks like
   - Common errors

4. **Troubleshooting**
   - Common issues
   - Error messages
   - Solutions

---

## Related Documentation

- System Architecture: `summary/brief-summary/SYSTEM_ARCHITECTURE.md`
- Current Status: `summary/quick-summary/CURRENT_SYSTEM_STATUS.md`
- Daily Reports: `summary/daily-reports/`

---

**Folder**: `summary/setup/`  
**Last Updated**: October 30, 2025  
**Status**: Production configuration
