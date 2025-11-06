# NRP ROS System - Brief Technical Summary

## System Architecture

### Hardware Stack
- **Flight Controller**: CubeOrangePlus (ArduRover V4.5.6)
- **GPS**: RTK-capable GPS receiver
- **Connection**: Serial /dev/ttyACM0 @ 115200 baud
- **Location**: Chennai, India (13.072°N, 80.262°E)

### Software Stack
- **OS**: Ubuntu (Linux)
- **ROS**: ROS2 Humble
- **MAVROS**: ArduPilot MAVLink bridge
- **Backend**: Python Flask + roslibpy
- **Frontend**: React/TypeScript (Vite)
- **Communication**: WebSocket (rosbridge port 9090)

### Service Architecture
```
┌─────────────────┐
│ Flight Controller│
│  (ArduRover)    │
└────────┬────────┘
         │ MAVLink
┌────────▼────────┐
│     MAVROS      │
│  (ROS2 Topics)  │
└────────┬────────┘
         │
┌────────▼────────┐
│  Rosbridge WS   │
│   (Port 9090)   │
└────────┬────────┘
         │
┌────────▼────────┐
│  Flask Backend  │
│  (server.py)    │
└────────┬────────┘
         │ WebSocket
┌────────▼────────┐
│   React UI      │
│  (Frontend)     │
└─────────────────┘
```

## Key Components

### Backend Services
1. **MAVROS Node** (`/mavros`)
   - Publishes vehicle telemetry to ROS topics
   - Handles MAVLink ↔ ROS2 conversion
   - Topics: `/mavros/state`, `/mavros/gpsstatus/gps1/raw`, etc.

2. **Rosbridge Server**
   - WebSocket bridge between ROS and web applications
   - Port 9090
   - Enables browser-based ROS topic access

3. **Flask Server** (`Backend/server.py`)
   - HTTP/WebSocket server
   - Subscribes to ROS topics via roslibpy
   - Broadcasts telemetry to connected clients
   - Handles mission management, commands

4. **Telemetry Node** (`Backend/telemetry_node.py`)
   - ROS2 node for custom telemetry processing
   - Coordinates with Flask backend

5. **MAVROS Bridge** (`Backend/mavros_bridge.py`)
   - Abstraction layer for MAVROS topics/services
   - Handles GPS, position, RTK, battery, RC data
   - Broadcasts structured telemetry to Flask

### Data Flow

**GPS Example**:
```
GPS Hardware
  ↓ (NMEA/UBX)
ArduPilot
  ↓ (MAVLink GLOBAL_POSITION_INT)
MAVROS
  ↓ (ROS Topic /mavros/gpsstatus/gps1/raw)
MAVROS Bridge (_handle_gps_raw)
  ↓ (Telemetry callback)
Flask Server
  ↓ (WebSocket emit)
React UI
  ↓
Display on map/panels
```

## Current Status

### Operational
- ✅ ROS2 system running
- ✅ MAVROS connected to flight controller
- ✅ Rosbridge active (port 9090)
- ✅ Backend services running
- ✅ GPS: RTK Fixed (29 satellites)
- ✅ Telemetry streaming to UI

### Recent Updates (Oct 30, 2025)
1. **GPS Raw Integration**
   - Fixed altitude bug (-76m → +16m)
   - Fixed GPS status display (No Fix → RTK Fixed)
   - Added accuracy metrics (eph, epv)

### Known Issues
- ⚠️ MAVROS NavSatFix topics have transformation bugs (workaround: use raw GPS topic)

## Development Files

### Source Code
- `Backend/server.py` - Main Flask application
- `Backend/mavros_bridge.py` - ROS topic abstraction
- `Backend/telemetry_node.py` - Custom ROS2 node
- `Backend/mavros_bridge.py` - MAVLink utilities
- `src/` - React frontend components

### Configuration
- `start_service.sh` - System startup script
- `SERVICE_COMMANDS.md` - Service management commands
- `Backend/requirements.txt` - Python dependencies
- `package.json` - Node.js dependencies

### Documentation
- `summary/` - Organized documentation
- `QUICKSTART.md` - Quick start guide
- `README.md` - Project overview

---
**System Version**: V3_Oct_29  
**Last Updated**: October 30, 2025  
**Status**: Production-ready
