# 🚀 NRP_ROS Service Management Commands
**Complete Command Reference for Navigation Rover Platform**  
*Last Updated: October 29, 2025*

---

## 📑 Table of Contents
1. [System Service Control](#system-service-control)
2. [Frontend Management](#frontend-management)
3. [Backend Management](#backend-management)
4. [ROS 2 System](#ros-2-system)
5. [MAVROS Bridge](#mavros-bridge)
6. [Database & Logs](#database--logs)
7. [Network & Ports](#network--ports)
8. [Troubleshooting](#troubleshooting)
9. [Health Checks](#health-checks)

---

## 🎛️ System Service Control

### **Primary Service (rosbridge.service)**
The main systemd service that manages all components.

```bash
# ✅ START - Start all services
sudo systemctl start rosbridge.service

# ⏹️ STOP - Stop all services
sudo systemctl stop rosbridge.service

# 🔄 RESTART - Restart all services (apply updates)
sudo systemctl restart rosbridge.service

# 📊 STATUS - Check if service is running
sudo systemctl status rosbridge.service

# 📋 LOGS - View live logs
sudo journalctl -u rosbridge.service -f

# 📜 RECENT LOGS - Last 100 lines
sudo journalctl -u rosbridge.service -n 100

# 🔍 SEARCH LOGS - Find errors
sudo journalctl -u rosbridge.service | grep -i error

# 🚀 ENABLE - Auto-start on boot
sudo systemctl enable rosbridge.service

# ⛔ DISABLE - Prevent auto-start
sudo systemctl disable rosbridge.service

# ❓ CHECK AUTO-START - Is it enabled?
sudo systemctl is-enabled rosbridge.service

# 🔁 RELOAD DAEMON - After changing .service file
sudo systemctl daemon-reload
```

---

## 🎨 Frontend Management

### **Vite Development Server (Port 5173)**

```bash
# 📂 Navigate to project
cd /home/flash/NRP_ROS

# ▶️ START - Development server
npm run dev

# 🏗️ BUILD - Production build
npm run build

# 👀 PREVIEW - Preview production build
npm run preview

# 🧹 CLEAN - Remove node_modules and reinstall
rm -rf node_modules package-lock.json
npm install

# 🔍 CHECK - Is frontend running?
curl http://localhost:5173

# 📦 UPDATE DEPENDENCIES
npm update

# 🛡️ AUDIT - Check for vulnerabilities
npm audit
npm audit fix

# 📊 PROCESS CHECK - Find Vite process
ps aux | grep vite

# ⏹️ KILL FRONTEND - Stop Vite server
pkill -f "vite"
# OR find and kill by PID
lsof -ti:5173 | xargs kill -9
```

### **Frontend URLs**
- Local: `http://localhost:5173`
- Network: `http://<rover-ip>:5173`

---

## 🐍 Backend Management

### **Flask + Socket.IO Server (Port 5001)**

```bash
# 📂 Navigate to backend
cd /home/flash/NRP_ROS/Backend

# ▶️ START - Run backend manually
python3 server.py

# ▶️ START - With output logging
python3 server.py 2>&1 | tee backend.log

# 🔍 CHECK - Is backend running?
curl http://localhost:5001/api/health

# 📊 PROCESS CHECK - Find backend process
ps aux | grep "server.py"

# ⏹️ KILL BACKEND - Stop Flask server
pkill -f "server.py"
# OR find and kill by PID
lsof -ti:5001 | xargs kill -9

# 🧪 TEST API - Test REST endpoint
curl http://localhost:5001/api/status

# 🔌 TEST WEBSOCKET - Check Socket.IO
curl http://localhost:5001/socket.io/

# 📦 INSTALL DEPENDENCIES
pip3 install -r requirements.txt

# 🐛 DEBUG MODE - Run with verbose output
FLASK_ENV=development python3 server.py
```

### **Backend API Endpoints**
- Health Check: `http://localhost:5001/api/health`
- Status: `http://localhost:5001/api/status`
- Mission Upload: `POST http://localhost:5001/api/mission/upload`
- Mission Download: `GET http://localhost:5001/api/mission/download`

---

## 🤖 ROS 2 System

### **ROS 2 Humble Environment**

```bash
# 🔧 SOURCE ROS 2 - Load environment
source /opt/ros/humble/setup.bash

# 📋 LIST NODES - All active ROS nodes
ros2 node list

# 📊 NODE INFO - Details about specific node
ros2 node info /rosbridge_websocket
ros2 node info /mavros

# 📡 LIST TOPICS - All active topics
ros2 topic list

# 👁️ ECHO TOPIC - Monitor topic data
ros2 topic echo /mavros/state
ros2 topic echo /mavros/global_position/global
ros2 topic echo /mavros/rc/out

# 📈 TOPIC INFO - Topic details
ros2 topic info /mavros/state -v

# 🔍 TOPIC HZ - Check publish rate
ros2 topic hz /mavros/state

# 🎯 LIST SERVICES - All ROS services
ros2 service list

# 📞 CALL SERVICE - Test service
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"

# 🔍 SERVICE TYPE - Get service message type
ros2 service type /mavros/mission/push

# 📦 INTERFACE SHOW - View message structure
ros2 interface show mavros_msgs/msg/State
ros2 interface show mavros_msgs/srv/WaypointPush

# 🏃 RUN NODE - Launch individual node
ros2 run rosbridge_server rosbridge_websocket

# 🚀 LAUNCH FILE - Start launch configuration
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 launch mavros apm.launch
```

---

## 🔌 MAVROS Bridge

### **MAVROS Node Management (Port 9090)**

```bash
# ▶️ START - Launch MAVROS manually
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200

# 🔍 CHECK CONNECTION - MAVROS state
ros2 topic echo /mavros/state --once

# 📊 GPS STATUS - Check GPS fix
ros2 topic echo /mavros/global_position/raw/fix --once

# 🛰️ SERVO OUTPUT - Monitor servo PWM values
ros2 topic echo /mavros/rc/out

# 🎯 MISSION WAYPOINTS - Current mission
ros2 topic echo /mavros/mission/waypoints

# 📡 BATTERY - Check battery status
ros2 topic echo /mavros/battery

# 🧭 COMPASS - Check heading
ros2 topic echo /mavros/global_position/compass_hdg

# ⚙️ PARAMETERS - List MAVROS parameters
ros2 param list /mavros

# 🔧 SET PARAMETER - Change MAVROS parameter
ros2 param set /mavros system_id 1
```

### **rosbridge_server (WebSocket Port 9090)**

```bash
# ▶️ START - Launch rosbridge manually
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 🔍 CHECK - Test WebSocket connection
wscat -c ws://localhost:9090

# 📊 PROCESS CHECK
ps aux | grep rosbridge

# ⏹️ KILL ROSBRIDGE
pkill -f rosbridge

# 🔌 PORT CHECK
lsof -i:9090
```

---

## 📚 Database & Logs

### **Mission Logs**

```bash
# 📂 View mission logs directory
ls -lh /home/flash/NRP_ROS/Backend/logs/

# 📋 List recent mission logs
ls -lt /home/flash/NRP_ROS/Backend/logs/ | head -20

# 👁️ VIEW LOG - Read mission log
cat /home/flash/NRP_ROS/Backend/logs/mission_2025-10-29.log

# 🔍 SEARCH LOGS - Find specific events
grep "waypoint" /home/flash/NRP_ROS/Backend/logs/*.log
grep -i "error" /home/flash/NRP_ROS/Backend/logs/*.log

# 📊 LOG SIZE - Check log file sizes
du -sh /home/flash/NRP_ROS/Backend/logs/*

# 🧹 CLEAN OLD LOGS - Remove logs older than 30 days
find /home/flash/NRP_ROS/Backend/logs/ -name "*.log" -mtime +30 -delete
```

### **System Logs**

```bash
# 📜 SERVICE LOGS - rosbridge.service
sudo journalctl -u rosbridge.service -n 100

# 🔴 ERROR LOGS - Show only errors
sudo journalctl -u rosbridge.service -p err

# 📅 DATE RANGE - Logs from today
sudo journalctl -u rosbridge.service --since today

# ⏰ TIME RANGE - Last hour
sudo journalctl -u rosbridge.service --since "1 hour ago"

# 💾 EXPORT LOGS - Save to file
sudo journalctl -u rosbridge.service > rosbridge_logs.txt
```

---

## 🌐 Network & Ports

### **Port Status Checks**

```bash
# 🔍 CHECK ALL PORTS - NRP system ports
sudo lsof -i :5173  # Frontend (Vite)
sudo lsof -i :5001  # Backend (Flask)
sudo lsof -i :9090  # rosbridge WebSocket
sudo lsof -i :5761  # MAVROS (optional)

# 📊 ALL LISTENING PORTS
sudo netstat -tulpn | grep LISTEN

# 🔌 SPECIFIC PORT - What's using port?
sudo lsof -i :5173
sudo fuser 5173/tcp

# ⏹️ KILL PROCESS ON PORT
sudo lsof -ti:5173 | xargs kill -9
sudo fuser -k 5001/tcp

# 🌐 NETWORK INTERFACES
ip addr show
ifconfig
```

### **Firewall (if enabled)**

```bash
# 🔥 CHECK FIREWALL STATUS
sudo ufw status

# ✅ ALLOW PORTS
sudo ufw allow 5173/tcp  # Frontend
sudo ufw allow 5001/tcp  # Backend
sudo ufw allow 9090/tcp  # rosbridge

# 🔁 RELOAD FIREWALL
sudo ufw reload
```

---

## 🔧 Troubleshooting

### **Common Issues**

```bash
# ❌ PROBLEM: Service won't start
# 🔍 SOLUTION: Check logs for errors
sudo journalctl -u rosbridge.service -n 50
# Check if ports are already in use
sudo lsof -i :5001 -i :5173 -i :9090

# ❌ PROBLEM: "Port already in use"
# 🔍 SOLUTION: Kill processes on ports
sudo lsof -ti:5173 | xargs kill -9
sudo lsof -ti:5001 | xargs kill -9
sudo lsof -ti:9090 | xargs kill -9

# ❌ PROBLEM: MAVROS not connecting
# 🔍 SOLUTION: Check serial device
ls -l /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0
# Check connection
ros2 topic echo /mavros/state --once

# ❌ PROBLEM: Frontend can't connect to backend
# 🔍 SOLUTION: Check backend is running
curl http://localhost:5001/api/health
# Check CORS settings
grep CORS /home/flash/NRP_ROS/Backend/server.py

# ❌ PROBLEM: No GPS data
# 🔍 SOLUTION: Check MAVROS topics
ros2 topic list | grep gps
ros2 topic echo /mavros/global_position/global --once

# ❌ PROBLEM: Dependencies missing
# 🔍 SOLUTION: Reinstall
cd /home/flash/NRP_ROS
pip3 install -r Backend/requirements.txt
npm install
```

### **Emergency Reset**

```bash
# 🚨 FULL SYSTEM RESTART
sudo systemctl stop rosbridge.service
sleep 5
sudo lsof -ti:5001,5173,9090 | xargs kill -9 2>/dev/null
sleep 2
sudo systemctl start rosbridge.service
```

---

## 🏥 Health Checks

### **Complete System Check**

```bash
# 📊 FULL STATUS CHECK - Run all checks
echo "=== SERVICE STATUS ==="
sudo systemctl is-active rosbridge.service

echo "=== FRONTEND STATUS ==="
curl -s http://localhost:5173 > /dev/null && echo "✅ Frontend Running" || echo "❌ Frontend Down"

echo "=== BACKEND STATUS ==="
curl -s http://localhost:5001/api/health > /dev/null && echo "✅ Backend Running" || echo "❌ Backend Down"

echo "=== ROS 2 NODES ==="
source /opt/ros/humble/setup.bash
ros2 node list

echo "=== MAVROS CONNECTION ==="
ros2 topic echo /mavros/state --once 2>/dev/null | grep connected || echo "❌ MAVROS Disconnected"

echo "=== PORT STATUS ==="
sudo lsof -i :5173,5001,9090

echo "=== DISK USAGE ==="
df -h /home/flash/NRP_ROS

echo "=== MEMORY USAGE ==="
free -h
```

### **Quick Health Script**

Create a health check script:
```bash
#!/bin/bash
# Save as: /home/flash/NRP_ROS/health_check.sh

echo "🏥 NRP_ROS System Health Check"
echo "================================"

# Service Status
if systemctl is-active --quiet rosbridge.service; then
    echo "✅ rosbridge.service: RUNNING"
else
    echo "❌ rosbridge.service: STOPPED"
fi

# Frontend
if curl -s http://localhost:5173 > /dev/null 2>&1; then
    echo "✅ Frontend (5173): ACCESSIBLE"
else
    echo "❌ Frontend (5173): DOWN"
fi

# Backend
if curl -s http://localhost:5001/api/health > /dev/null 2>&1; then
    echo "✅ Backend (5001): HEALTHY"
else
    echo "❌ Backend (5001): DOWN"
fi

# rosbridge
if lsof -i:9090 > /dev/null 2>&1; then
    echo "✅ rosbridge (9090): LISTENING"
else
    echo "❌ rosbridge (9090): NOT RUNNING"
fi

# ROS 2 Nodes
source /opt/ros/humble/setup.bash > /dev/null 2>&1
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
echo "📊 ROS 2 Nodes Active: $NODE_COUNT"

echo "================================"
```

Make executable and run:
```bash
chmod +x /home/flash/NRP_ROS/health_check.sh
./health_check.sh
```

---

## 📋 Quick Reference Card

```bash
# 🚀 START EVERYTHING
sudo systemctl start rosbridge.service

# ⏹️ STOP EVERYTHING
sudo systemctl stop rosbridge.service

# 🔄 RESTART (after code changes)
sudo systemctl restart rosbridge.service

# 📊 CHECK STATUS
sudo systemctl status rosbridge.service

# 📋 VIEW LOGS
sudo journalctl -u rosbridge.service -f

# 🏥 HEALTH CHECK
curl http://localhost:5001/api/health
curl http://localhost:5173

# 🔍 ROS STATUS
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list

# 📡 MAVROS STATE
ros2 topic echo /mavros/state --once

# 🧹 CLEAN RESTART
sudo systemctl stop rosbridge.service
sudo lsof -ti:5001,5173,9090 | xargs kill -9
sudo systemctl start rosbridge.service
```

---

## 🎯 Development Workflow

### **After Making Code Changes**

```bash
# Frontend changes (React/TypeScript)
# ✅ Vite auto-reloads, no action needed

# Backend changes (Python)
# 🔄 Restart service
sudo systemctl restart rosbridge.service

# ROS launch file changes
# 🔄 Restart service
sudo systemctl restart rosbridge.service
```

### **Testing New Features**

```bash
# 1. Check frontend in browser
open http://localhost:5173

# 2. Test backend API
curl http://localhost:5001/api/status

# 3. Check ROS topics
ros2 topic list

# 4. Monitor logs
sudo journalctl -u rosbridge.service -f
```

---

**📝 Notes:**
- All commands assume you're in `/home/flash/NRP_ROS` directory
- Replace `<rover-ip>` with actual rover IP address for network access
- Use `Ctrl+C` to stop following logs (`-f` flag)
- ROS 2 commands require sourcing: `source /opt/ros/humble/setup.bash`

**💡 Tips:**
- Add aliases to `~/.bashrc` for frequently used commands
- Create custom scripts for common operations
- Use `tmux` or `screen` for persistent terminal sessions
- Monitor resource usage with `htop` during operation

---

*Document Version: 1.0*  
*Project: NRP_ROS - Navigation Rover Platform*  
*Last Updated: October 29, 2025*
