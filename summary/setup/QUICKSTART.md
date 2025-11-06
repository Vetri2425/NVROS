# 🚀 NRP_ROS Quick Start Guide
**Fast Reference for Common Operations**

---

## ⚡ Most Common Commands

### **Daily Operations**
```bash
# Check system status
./health_check.sh

# View live logs
sudo journalctl -u rosbridge.service -f

# Restart after code changes
sudo systemctl restart rosbridge.service

# Access web interface
http://localhost:5173
```

---

## 🎯 Quick Actions

| Action | Command |
|--------|---------|
| **Start System** | `sudo systemctl start rosbridge.service` |
| **Stop System** | `sudo systemctl stop rosbridge.service` |
| **Restart System** | `sudo systemctl restart rosbridge.service` |
| **System Status** | `sudo systemctl status rosbridge.service` |
| **Health Check** | `./health_check.sh` |
| **View Logs** | `sudo journalctl -u rosbridge.service -f` |
| **Kill All** | `sudo systemctl stop rosbridge.service && sudo lsof -ti:5001,5173,9090 \| xargs kill -9` |

---

## 📍 URLs & Endpoints

| Service | URL |
|---------|-----|
| **Web Interface** | http://localhost:5173 |
| **Backend API** | http://localhost:5001 |
| **Health Check** | http://localhost:5001/api/health |
| **rosbridge WS** | ws://localhost:9090 |

---

## 🔍 Troubleshooting One-Liners

```bash
# Check if backend is responding
curl http://localhost:5001/api/health

# Check MAVROS connection
ros2 topic echo /mavros/state --once

# See what's using ports
sudo lsof -i :5001,5173,9090

# Restart everything cleanly
sudo systemctl restart rosbridge.service

# View recent errors
sudo journalctl -u rosbridge.service -p err -n 20

# Check ROS nodes
source /opt/ros/humble/setup.bash && ros2 node list
```

---

## 📂 Important Files & Locations

| Item | Path |
|------|------|
| **Project Root** | `/home/flash/NRP_ROS` |
| **Backend Code** | `/home/flash/NRP_ROS/Backend` |
| **Frontend Code** | `/home/flash/NRP_ROS/src` |
| **Service File** | `/etc/systemd/system/rosbridge.service` |
| **Start Script** | `/home/flash/NRP_ROS/start_service.sh` |
| **Mission Logs** | `/home/flash/NRP_ROS/Backend/logs` |
| **Health Check** | `/home/flash/NRP_ROS/health_check.sh` |
| **Commands Ref** | `/home/flash/NRP_ROS/SERVICE_COMMANDS.md` |

---

## 🎨 Development Workflow

```bash
# 1. Make code changes in your editor

# 2. Frontend changes (auto-reload, no action needed)
#    Just refresh browser

# 3. Backend changes (restart required)
sudo systemctl restart rosbridge.service

# 4. Check logs for errors
sudo journalctl -u rosbridge.service -f

# 5. Test in browser
open http://localhost:5173
```

---

## 📊 System Monitoring

```bash
# Watch system resources
htop

# Monitor specific process
watch -n 1 'ps aux | grep -E "vite|server.py|rosbridge|mavros"'

# Check disk space
df -h

# Monitor network connections
watch -n 1 'sudo lsof -i :5001,5173,9090'
```

---

## 🆘 Emergency Procedures

### **System Unresponsive**
```bash
sudo systemctl stop rosbridge.service
sleep 5
sudo lsof -ti:5001,5173,9090 | xargs kill -9
sleep 2
sudo systemctl start rosbridge.service
```

### **Port Conflicts**
```bash
# Kill processes on specific ports
sudo lsof -ti:5173 | xargs kill -9  # Frontend
sudo lsof -ti:5001 | xargs kill -9  # Backend
sudo lsof -ti:9090 | xargs kill -9  # rosbridge
```

### **Complete Clean Start**
```bash
# Stop everything
sudo systemctl stop rosbridge.service

# Clear all processes
pkill -f vite
pkill -f server.py
pkill -f rosbridge
pkill -f mavros

# Wait
sleep 3

# Restart
sudo systemctl start rosbridge.service
```

---

## 🧪 Testing Features

### **Mission Planning Tools**
1. Open: http://localhost:5173
2. Go to **Planning** tab
3. Try these buttons:
   - 📐 **Survey Grid** - Area coverage pattern
   - 🌀 **Circle** - Circular pattern
   - 🗺️ **Polygon** - Custom area survey
   - 〰️ **Spline** - Smooth curves

### **Live Mission Control**
1. Upload a mission
2. Write to rover
3. Go to **Live** tab
4. Test buttons:
   - ⏩ Skip waypoint
   - ⏪ Go back
   - ⏸️ Pause mission
   - ▶️ Resume mission

---

## 📱 Remote Access

```bash
# Find rover IP address
hostname -I

# Access from another device on same network
http://<rover-ip>:5173
```

---

## 🔧 Useful Aliases

Add to `~/.bashrc`:
```bash
alias nrp-start='sudo systemctl start rosbridge.service'
alias nrp-stop='sudo systemctl stop rosbridge.service'
alias nrp-restart='sudo systemctl restart rosbridge.service'
alias nrp-status='sudo systemctl status rosbridge.service'
alias nrp-logs='sudo journalctl -u rosbridge.service -f'
alias nrp-health='/home/flash/NRP_ROS/health_check.sh'
alias nrp-ros='source /opt/ros/humble/setup.bash'
```

Then run: `source ~/.bashrc`

---

**💡 Pro Tips:**
- Use `tmux` for persistent terminal sessions
- Bookmark http://localhost:5173 in browser
- Keep logs open in separate terminal: `sudo journalctl -u rosbridge.service -f`
- Run health check after restart: `./health_check.sh`

---

*For detailed command reference, see: `SERVICE_COMMANDS.md`*
