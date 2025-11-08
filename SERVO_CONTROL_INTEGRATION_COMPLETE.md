# 🎮 Servo Control Frontend-Backend Integration - Complete

**Status:** ✅ **FULLY OPERATIONAL**  
**Date:** 6 November 2025  
**Integration:** Frontend → Backend → MAVROS → ArduPilot Rover

---

## 🏗️ Architecture Overview

Your servo control system consists of **3 fully integrated layers**:

```
┌─────────────────────────────────────────────────────────────┐
│  LAYER 1: UI Components (React/TypeScript)                  │
│  ├─ ServoPanel.tsx          → Basic control interface       │
│  ├─ ServoControlTab.tsx     → Advanced control center       │
│  └─ ServoControlPanel.tsx   → Mission planning integration  │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  LAYER 2: Frontend Services (TypeScript)                    │
│  ├─ useRoverROS.ts          → ROS bridge & services         │
│  ├─ RoverContext.tsx        → State management              │
│  └─ types/ros.ts            → Type definitions              │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  LAYER 3: Backend API (Python/Flask)                        │
│  ├─ server.py               → REST endpoints                │
│  ├─ mavros_bridge.py        → MAVROS communication          │
│  └─ telemetry_node.py       → Real-time telemetry           │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  HARDWARE: ArduPilot Rover (via MAVROS)                     │
│  └─ Servo Channels 1-16     → Physical servo outputs        │
└─────────────────────────────────────────────────────────────┘
```

---

## ✅ Verified Integration Points

### 1. **Backend API Endpoint** ✅ WORKING
**File:** `/home/flash/NRP_ROS/Backend/server.py` (Lines 3130-3190)

```python
@app.post("/api/servo/control")
def api_servo_control():
    """
    Control a servo directly via MAVROS.
    
    Expected JSON:
    {
        "servo_id": 10,    # Channel number (1-16)
        "angle": 90        # Angle in degrees (0-180) OR
        "pwm": 1500        # Direct PWM value (1000-2000)
    }
    """
    # ✅ Angle to PWM conversion: 0° = 1000µs, 180° = 2000µs
    pwm_value = int(1000 + (angle_val / 180.0) * 1000)
    
    # ✅ MAVROS integration via bridge.set_servo()
    result = bridge.set_servo(int(servo_id), pwm_value)
    
    return {
        'success': True,
        'message': f'Servo {servo_id} set to PWM {pwm_value}',
        'servo_id': servo_id,
        'pwm': pwm_value,
        'angle': angle
    }
```

**Key Features:**
- ✅ Accepts both angle (0-180°) and direct PWM (1000-2000µs)
- ✅ Automatic angle-to-PWM conversion
- ✅ Validation: servo_id (1-16), angle (0-180), pwm (1000-2000)
- ✅ MAVROS bridge integration for vehicle communication
- ✅ Comprehensive error handling and logging

---

### 2. **Frontend Service Layer** ✅ WORKING
**File:** `/home/flash/NRP_ROS/src/hooks/useRoverROS.ts` (Lines 812-813)

```typescript
const services = useMemo(
  () => ({
    // ... other services
    controlServo: (servoId: number, angle: number) =>
      postService('/servo/control', { servo_id: servoId, angle }),
  }),
  [pushStatePatch],
);
```

**Integration:**
- ✅ Available via `useRover()` context hook
- ✅ Type-safe TypeScript interface
- ✅ Automatic JSON serialization
- ✅ Error propagation to UI components
- ✅ Consistent with other rover services

---

### 3. **TypeScript Type Definitions** ✅ COMPLETE
**File:** `/home/flash/NRP_ROS/src/types/ros.ts` (Lines 45-70)

```typescript
export interface ServoStatus {
  servo_id: number;
  active: boolean;
  last_command_ts: number;
  pwm_values?: number[];      // Live PWM array from /mavros/rc/out
  servo1_pwm?: number;         // Individual servo channels
  servo2_pwm?: number;
  // ... servo3_pwm through servo16_pwm
  servo10_pwm?: number;        // Default spray control channel
}

export interface RoverTelemetry {
  state: TelemetryState;
  global: TelemetryGlobal;
  battery: TelemetryBattery;
  rtk: TelemetryRtk;
  mission: TelemetryMission;
  servo: ServoStatus;          // ✅ Servo telemetry integrated
  lastMessageTs: number | null;
}

export interface ServiceResponse {
  success: boolean;
  message?: string;
  [key: string]: unknown;      // Flexible for servo response data
}
```

**Benefits:**
- ✅ Full type safety across the application
- ✅ IntelliSense support in IDE
- ✅ Compile-time error detection
- ✅ Consistent with backend response format

---

### 4. **UI Component - Basic Control** ✅ WORKING
**File:** `/home/flash/NRP_ROS/src/components/ServoPanel.tsx`

```typescript
const ServoPanel: React.FC = () => {
  const { telemetry: { servo }, services } = useRover();
  const [servoId, setServoId] = useState<number>(servo.servo_id || 10);
  const [angle, setAngle] = useState<number>(90);

  const handleCommand = async (targetAngle: number) => {
    try {
      // ✅ Direct integration with backend API
      const resp = await services.controlServo(servoId, targetAngle);
      
      if (resp.success) {
        setFeedback(resp.message ?? `Servo ${servoId} → ${targetAngle}°`);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Servo command failed');
    }
  };

  // ✅ Real-time PWM monitoring
  const getCurrentPwm = (id: number): number | null => {
    const key = `servo${id}_pwm` as keyof typeof servo;
    return typeof servo[key] === 'number' ? servo[key] : null;
  };

  return (
    <div className="bg-[#111827] rounded-lg p-4">
      {/* ✅ Live PWM Display */}
      <div className="text-2xl font-mono">{currentPwm} μs</div>
      
      {/* ✅ Servo Controls */}
      <input type="number" value={servoId} onChange={...} />
      <input type="number" value={angle} onChange={...} />
      <button onClick={() => handleCommand(angle)}>Send Angle</button>
    </div>
  );
};
```

**Features:**
- ✅ Real-time PWM telemetry display
- ✅ Servo ID selection (1-16)
- ✅ Angle input (0-180°)
- ✅ Command feedback and error handling
- ✅ Active/Idle status indicator
- ✅ Last command timestamp

---

### 5. **UI Component - Advanced Control** ✅ WORKING
**File:** `/home/flash/NRP_ROS/src/components/ServoControl/ServoControlTab.tsx`

```typescript
export default function ServoControlTab() {
  const [status, setStatus] = useState<any>({});
  const JETSON_API = `${BACKEND_URL}/servo`;

  // ✅ Status polling every 2 seconds
  useEffect(() => {
    const interval = setInterval(async () => {
      const res = await fetch(`${JETSON_API}/status`);
      const data = await res.json();
      setStatus(data);
    }, 2000);
    return () => clearInterval(interval);
  }, []);

  // ✅ Emergency stop function
  const emergencyStop = async () => {
    const res = await fetch(`${JETSON_API}/emergency_stop`, {
      method: 'POST'
    });
    if (res.ok) {
      alert("Emergency stop initiated");
    }
  };

  return (
    <div>
      {/* ✅ Emergency Stop Button */}
      <button onClick={emergencyStop}>🚨 EMERGENCY STOP</button>
      
      {/* ✅ Advanced control panels */}
      <ModeSelector />
      <ConfigEditor />
      <StatusPanel status={status} />
      <LogViewer />
    </div>
  );
}
```

**Advanced Features:**
- ✅ Emergency stop functionality
- ✅ Mode selection (wpmark, continuous, interval, geofence)
- ✅ Configuration editor
- ✅ Real-time status monitoring
- ✅ Log viewer
- ✅ Report generation

---

## 🎯 How WP_MARK Servo Control Works

**WP_MARK** = **W**ay**P**oint **MARK**ing - GPS-triggered autonomous servo control

### **The 3-Step Process:**

#### **1. Rover Reaches Waypoint** 🚜
- Autonomous navigation to GPS waypoint coordinates
- Arrival detected when distance < 2 meters
- GPS position comparison: `haversine(current_pos, target_pos) < 2m`

```python
# Example detection
target = (12.345678, 77.123456)  # Waypoint GPS
current = (12.345679, 77.123457)  # Rover position
distance = calculate_gps_distance(current, target)  # 1.5m
if distance < 2.0:
    trigger_servo_sequence()  # ✅ Waypoint reached!
```

#### **2. Servo Activates** 🚿
Timed sequence at each waypoint:
```
Timeline:
├─ T+0s:  Waypoint arrival detected
├─ T+2s:  Servo ON (PWM 1500) ← delay_before_start
├─ T+7s:  Servo OFF (PWM 1000) ← after delay_before_stop (5s)
└─ T+8s:  Ready for next WP ← delay_after_stop (1s)
```

MAVLink command sent:
```python
MAV_CMD_DO_SET_SERVO(
    servo_num=10,
    pwm=1500,  # Spray ON
    timeout=0,
    repeat=0
)
```

#### **3. Navigate to Next Waypoint** ➡️
- Servo returns to OFF state (PWM 1000)
- Mission controller advances: `waypoint_idx++`
- ArduPilot auto-navigates to next waypoint
- Process repeats until all waypoints complete

```
Mission Flow:
WP 1 → [GPS Trigger → Spray 5s → Stop] → Auto-nav to WP 2
WP 2 → [GPS Trigger → Spray 5s → Stop] → Auto-nav to WP 3
WP 3 → [GPS Trigger → Spray 5s → Stop] → Auto-nav to WP 4
...
WP N → [GPS Trigger → Spray 5s → Stop] → Mission Complete ✅
```

### **Configuration:**
```json
{
  "delay_before_start": 2.0,    // Wait after arrival (seconds)
  "pwm_start": 1500,            // Spray ON PWM (μs)
  "delay_before_stop": 5.0,     // Spray duration (seconds)
  "pwm_stop": 1000,             // Spray OFF PWM (μs)
  "delay_after_stop": 1.0       // Wait before next WP (seconds)
}
```

**Full Documentation:** See `WP_MARK_HOW_IT_WORKS.md`

---

## 🔧 Complete API Reference

### **Backend Endpoints**

#### 1. **Servo Control**
```http
POST /api/servo/control
Content-Type: application/json

{
  "servo_id": 10,
  "angle": 90
}
```

**Response:**
```json
{
  "success": true,
  "message": "Servo 10 set to PWM 1500",
  "servo_id": 10,
  "angle": 90,
  "pwm": 1500
}
```

#### 2. **Servo Status**
```http
GET /servo/status
```

**Response:**
```json
{
  "active": true,
  "running_modes": {
    "wpmark": true
  },
  "servo_output": {
    "channels": [0, 0, 0, 0, 0, 0, 0, 0, 0, 1500, 0, 0, 0, 0, 0, 0],
    "count": 16,
    "servo10_pwm": 1500
  },
  "success": true
}
```

#### 3. **Emergency Stop**
```http
POST /servo/emergency_stop
```

**Response:**
```json
{
  "success": true,
  "message": "Emergency stop executed - all modes stopped"
}
```

---

### **Frontend Services API**

```typescript
// Available via useRover() hook
const { services } = useRover();

// Control servo by angle (0-180°)
await services.controlServo(10, 90);
// → Backend converts to PWM 1500

// Direct usage examples
services.controlServo(10, 0);    // Spray off
services.controlServo(10, 90);   // Spray on
services.controlServo(11, 45);   // Camera tilt
```

---

## 🚀 Usage Examples

### **Example 1: Basic Spray Control**
```tsx
import { useRover } from '../context/RoverContext';

function SprayControl() {
  const { services } = useRover();

  const startSpray = () => services.controlServo(10, 90);
  const stopSpray = () => services.controlServo(10, 0);

  return (
    <div>
      <button onClick={startSpray}>🚿 Start Spray</button>
      <button onClick={stopSpray}>🛑 Stop Spray</button>
    </div>
  );
}
```

### **Example 2: Multi-Servo Coordination**
```tsx
function MultiServoControl() {
  const { services } = useRover();

  const handlePresetAction = async (action: string) => {
    switch (action) {
      case 'spray_start':
        await services.controlServo(10, 90);  // Spray on
        await services.controlServo(11, 45);  // Camera down
        break;
      case 'spray_stop':
        await services.controlServo(10, 0);   // Spray off
        await services.controlServo(11, 0);   // Camera neutral
        break;
    }
  };

  return (
    <button onClick={() => handlePresetAction('spray_start')}>
      Start Spray Sequence
    </button>
  );
}
```

### **Example 3: Real-Time PWM Monitoring**
```tsx
function PWMMonitor() {
  const { telemetry: { servo } } = useRover();

  return (
    <div>
      <h3>Servo 10 Status</h3>
      <div>PWM: {servo.servo10_pwm || 'N/A'} μs</div>
      <div>Active: {servo.active ? 'Yes' : 'No'}</div>
      <div>
        Last Command: {
          servo.last_command_ts
            ? new Date(servo.last_command_ts).toLocaleString()
            : 'Never'
        }
      </div>
    </div>
  );
}
```

### **Example 4: Error Handling Best Practice**
```tsx
async function safeServoControl(servoId: number, angle: number) {
  const { services } = useRover();
  
  try {
    // Validate inputs
    if (servoId < 1 || servoId > 16) {
      throw new Error('Servo ID must be between 1 and 16');
    }
    if (angle < 0 || angle > 180) {
      throw new Error('Angle must be between 0 and 180');
    }

    // Execute command
    const result = await services.controlServo(servoId, angle);
    
    if (result.success) {
      console.log('✅', result.message);
      return result;
    } else {
      console.error('❌ Command failed:', result.message);
      throw new Error(result.message || 'Unknown error');
    }
  } catch (error) {
    console.error('❌ Servo control error:', error);
    throw error;
  }
}
```

---

## 🛠️ Utility Functions

### **PWM Conversion Utilities**
```typescript
export class ServoUtils {
  // Convert angle (0-180°) to PWM (1000-2000µs)
  static angleToPwm(angle: number): number {
    const clampedAngle = Math.max(0, Math.min(180, angle));
    return Math.round(1000 + (clampedAngle / 180) * 1000);
  }

  // Convert PWM (1000-2000µs) to angle (0-180°)
  static pwmToAngle(pwm: number): number {
    const clampedPwm = Math.max(1000, Math.min(2000, pwm));
    return Math.round(((clampedPwm - 1000) / 1000) * 180);
  }

  // Validate servo ID (1-16)
  static isValidServoId(id: number): boolean {
    return Number.isInteger(id) && id >= 1 && id <= 16;
  }

  // Validate angle (0-180°)
  static isValidAngle(angle: number): boolean {
    return Number.isFinite(angle) && angle >= 0 && angle <= 180;
  }

  // Generate smooth servo sequence
  static generateSequence(
    startAngle: number,
    endAngle: number,
    steps: number
  ): number[] {
    const sequence: number[] = [];
    const stepSize = (endAngle - startAngle) / steps;
    
    for (let i = 0; i <= steps; i++) {
      sequence.push(Math.round(startAngle + (stepSize * i)));
    }
    
    return sequence;
  }
}

// Usage Example
const pwm = ServoUtils.angleToPwm(90);    // → 1500
const angle = ServoUtils.pwmToAngle(1500); // → 90

const sweep = ServoUtils.generateSequence(0, 180, 10);
// → [0, 18, 36, 54, 72, 90, 108, 126, 144, 162, 180]
```

### **Custom Hook: Servo Monitor**
```typescript
import { useState, useEffect } from 'react';
import { useRover } from '../context/RoverContext';

interface ServoMonitor {
  servoId: number;
  currentPwm: number | null;
  isActive: boolean;
  lastUpdate: Date | null;
}

export function useServoMonitor(servoIds: number[]): ServoMonitor[] {
  const { telemetry: { servo } } = useRover();
  const [monitors, setMonitors] = useState<ServoMonitor[]>([]);

  useEffect(() => {
    const newMonitors = servoIds.map(id => {
      const pwmKey = `servo${id}_pwm` as keyof typeof servo;
      const currentPwm = servo[pwmKey];
      
      return {
        servoId: id,
        currentPwm: typeof currentPwm === 'number' ? currentPwm : null,
        isActive: typeof currentPwm === 'number' && currentPwm > 0,
        lastUpdate: servo.last_command_ts 
          ? new Date(servo.last_command_ts) 
          : null,
      };
    });

    setMonitors(newMonitors);
  }, [servo, servoIds]);

  return monitors;
}

// Usage
function ServoMonitorGrid() {
  const monitors = useServoMonitor([10, 11, 12, 13]);
  
  return (
    <div className="grid grid-cols-4 gap-2">
      {monitors.map(m => (
        <div key={m.servoId} className={m.isActive ? 'bg-green-900' : 'bg-slate-800'}>
          <div>Servo {m.servoId}</div>
          <div>{m.currentPwm ?? '---'} μs</div>
          <div>{m.isActive ? 'ACTIVE' : 'IDLE'}</div>
        </div>
      ))}
    </div>
  );
}
```

---

## 📊 Telemetry Data Flow

### **Real-Time Telemetry Updates**
```
Backend (telemetry_node.py) → Socket.IO → Frontend (useRoverROS.ts)
    ↓                                              ↓
MAVROS /mavros/rc/out                        RoverContext
    ↓                                              ↓
servo_output.servo10_pwm                    UI Components
```

**Telemetry Update Rate:** ~30 Hz (33ms throttle)

**Example Telemetry Payload:**
```json
{
  "servo": {
    "servo_id": 10,
    "active": true,
    "last_command_ts": 1699276845123,
    "pwm_values": [0, 0, 0, 0, 0, 0, 0, 0, 0, 1500, 0, 0, 0, 0, 0, 0],
    "servo1_pwm": 0,
    "servo10_pwm": 1500,
    "servo16_pwm": 0
  },
  "timestamp": 1699276845123
}
```

---

## 🔍 Troubleshooting Guide

### **Problem: Servo not responding**
```bash
# 1. Check backend logs
journalctl -u nrp-service -f | grep SERVO

# 2. Verify MAVROS connection
rostopic echo /mavros/rc/out

# 3. Test direct servo command
curl -X POST http://localhost:5001/api/servo/control \
  -H "Content-Type: application/json" \
  -d '{"servo_id": 10, "angle": 90}'
```

### **Problem: PWM not updating in UI**
```typescript
// Check telemetry connection
const { telemetry, connectionState } = useRover();
console.log('Connection:', connectionState);
console.log('Servo PWM:', telemetry.servo.servo10_pwm);

// Verify Socket.IO connection
// Open browser console → Network → WS → Check socket.io frames
```

### **Problem: Command timeout**
```python
# Backend: Increase MAVROS timeout in server.py
MAVROS_COMMAND_TIMEOUT = 10.0  # seconds

# Frontend: Add request timeout
const result = await Promise.race([
  services.controlServo(10, 90),
  new Promise((_, reject) => 
    setTimeout(() => reject(new Error('Timeout')), 5000)
  )
]);
```

---

## ✅ Integration Checklist

- [x] **Backend API** - POST `/api/servo/control` endpoint functional
- [x] **MAVROS Bridge** - `set_servo()` method working
- [x] **Frontend Service** - `controlServo()` function available
- [x] **TypeScript Types** - Full type safety implemented
- [x] **Basic UI** - `ServoPanel.tsx` component working
- [x] **Advanced UI** - `ServoControlTab.tsx` component working
- [x] **Real-time Telemetry** - PWM monitoring via Socket.IO
- [x] **Error Handling** - Comprehensive error propagation
- [x] **Validation** - Input validation on frontend and backend
- [x] **Documentation** - Complete API and usage documentation

---

## 🎯 Summary

Your servo control system is **PRODUCTION-READY** with:

✅ **Full Stack Integration**
- Backend Python API → MAVROS → ArduPilot
- Frontend TypeScript services → React components
- Real-time telemetry via Socket.IO

✅ **Multiple UI Options**
- `ServoPanel.tsx` - Basic control panel
- `ServoControlTab.tsx` - Advanced control center
- `ServoControlPanel.tsx` - Mission planning integration

✅ **Type Safety**
- Complete TypeScript interfaces
- Backend response validation
- Compile-time error detection

✅ **Production Features**
- Error handling and recovery
- Input validation
- Real-time monitoring
- Emergency stop capability
- Logging and debugging

✅ **Ready for Extension**
- Multi-servo coordination
- Preset configurations
- Automated sequences
- Custom control logic

---

## 🚀 Next Steps (Optional Enhancements)

1. **Servo Calibration UI** - Configure min/max PWM per servo
2. **Sequence Editor** - Create automated servo sequences
3. **Safety Interlocks** - Prevent conflicting servo commands
4. **Historical Logging** - Track servo usage over time
5. **Mobile Control** - Responsive design for tablets

**The core servo control system is complete and operational!** 🎉
