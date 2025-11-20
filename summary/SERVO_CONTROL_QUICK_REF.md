# 🎮 Servo Control - Quick Reference Guide

**Quick Access:** Ready-to-use components and functions for servo control

---

## 🚀 Quick Start

### **1. Basic Servo Control (Simplest)**

```tsx
import { useRover } from '../context/RoverContext';

function SprayControl() {
  const { services } = useRover();

  return (
    <div>
      <button onClick={() => services.controlServo(10, 90)}>
        🚿 Spray ON
      </button>
      <button onClick={() => services.controlServo(10, 0)}>
        🛑 Spray OFF
      </button>
    </div>
  );
}
```

---

## 📦 Available Components

### **1. ServoPanel** (Basic UI)
**Location:** `src/components/ServoPanel.tsx`  
**Use Case:** Simple servo control with PWM monitoring

```tsx
import ServoPanel from './components/ServoPanel';

function App() {
  return <ServoPanel />;
}
```

**Features:**
- ✅ Single servo control
- ✅ Live PWM display
- ✅ Angle input (0-180°)
- ✅ Active/Idle status
- ✅ Command feedback

---

### **2. MultiServoControl** (Advanced Multi-Servo)
**Location:** `src/components/ServoControl/MultiServoControl.tsx`  
**Use Case:** Control multiple servos simultaneously

```tsx
import MultiServoControl from './components/ServoControl/MultiServoControl';

function App() {
  return <MultiServoControl />;
}
```

**Features:**
- ✅ 4 preset servo configurations
- ✅ Individual servo controls
- ✅ Coordinated multi-servo actions
- ✅ Preset buttons (Spray Start/Stop, All Default/Neutral)

---

### **3. ServoMonitorDisplay** (Real-Time Monitoring)
**Location:** `src/components/ServoControl/ServoMonitorDisplay.tsx`  
**Use Case:** Monitor multiple servo channels in real-time

```tsx
import ServoMonitorDisplay from './components/ServoControl/ServoMonitorDisplay';

function App() {
  return <ServoMonitorDisplay servoIds={[10, 11, 12, 13]} />;
}
```

**Features:**
- ✅ Grid display of servo status
- ✅ Real-time PWM values
- ✅ Active/Idle indicators
- ✅ Last update timestamps

---

### **4. ServoSequenceControl** (Automated Sequences)
**Location:** `src/components/ServoControl/ServoSequenceControl.tsx`  
**Use Case:** Execute automated servo movements

```tsx
import ServoSequenceControl from './components/ServoControl/ServoSequenceControl';

function App() {
  return <ServoSequenceControl />;
}
```

**Features:**
- ✅ Quick Test (0° → 180° → 0°)
- ✅ Full Sweep (smooth 0-180-0 movement)
- ✅ Calibration Test (test key positions)
- ✅ Smooth Transitions (gradual movements)
- ✅ Progress tracking

---

### **5. ServoControlTab** (Complete Control Center)
**Location:** `src/components/ServoControl/ServoControlTab.tsx`  
**Use Case:** Full-featured servo control interface

```tsx
import ServoControlTab from './components/ServoControl/ServoControlTab';

function App() {
  return <ServoControlTab />;
}
```

**Features:**
- ✅ Emergency stop button
- ✅ Mode selection (wpmark, continuous, interval, geofence)
- ✅ Configuration editor
- ✅ Status panel
- ✅ Log viewer
- ✅ Report generation

---

## 🔧 Utility Functions

### **ServoUtils Class**
**Location:** `src/utils/ServoUtils.ts`

```tsx
import { ServoUtils } from '../utils/ServoUtils';

// Convert angle to PWM
const pwm = ServoUtils.angleToPwm(90);  // → 1500

// Convert PWM to angle
const angle = ServoUtils.pwmToAngle(1500);  // → 90

// Validate inputs
ServoUtils.isValidServoId(10);   // → true
ServoUtils.isValidAngle(90);     // → true
ServoUtils.isValidPwm(1500);     // → true

// Generate sequences
const sweep = ServoUtils.generateSweep(180, 10);
// → [0, 18, 36, ..., 162, 180, 162, ..., 18, 0]

const sequence = ServoUtils.generateSequence(0, 180, 5);
// → [0, 36, 72, 108, 144, 180]

// Interpolation
const mid = ServoUtils.interpolateAngle(0, 180, 0.5);  // → 90
```

---

### **useServoMonitor Hook**
**Location:** `src/hooks/useServoMonitor.ts`

```tsx
import { useServoMonitor } from '../hooks/useServoMonitor';

function MyComponent() {
  const monitors = useServoMonitor([10, 11, 12, 13]);
  
  return (
    <div>
      {monitors.map(m => (
        <div key={m.servoId}>
          Servo {m.servoId}: {m.currentPwm} μs
          Status: {m.isActive ? 'ACTIVE' : 'IDLE'}
        </div>
      ))}
    </div>
  );
}
```

---

## 📋 Common Use Cases

### **Use Case 1: Simple Spray Control**
```tsx
function SprayButton() {
  const { services } = useRover();
  const [spraying, setSpraying] = useState(false);

  const toggleSpray = async () => {
    const angle = spraying ? 0 : 90;
    await services.controlServo(10, angle);
    setSpraying(!spraying);
  };

  return (
    <button onClick={toggleSpray}>
      {spraying ? '🛑 Stop Spray' : '🚿 Start Spray'}
    </button>
  );
}
```

---

### **Use Case 2: Camera Gimbal Control**
```tsx
function CameraGimbal() {
  const { services } = useRover();

  return (
    <div>
      <button onClick={() => services.controlServo(11, 0)}>
        ⬆️ Look Up
      </button>
      <button onClick={() => services.controlServo(11, 45)}>
        ➡️ Look Forward
      </button>
      <button onClick={() => services.controlServo(11, 90)}>
        ⬇️ Look Down
      </button>
    </div>
  );
}
```

---

### **Use Case 3: Coordinated Multi-Servo Action**
```tsx
function SpraySequence() {
  const { services } = useRover();

  const startSpraySequence = async () => {
    // Step 1: Position camera
    await services.controlServo(11, 45);
    await new Promise(r => setTimeout(r, 500));
    
    // Step 2: Open spray nozzle
    await services.controlServo(10, 90);
    
    console.log('Spray sequence started');
  };

  const stopSpraySequence = async () => {
    // Stop spray first
    await services.controlServo(10, 0);
    await new Promise(r => setTimeout(r, 500));
    
    // Reset camera
    await services.controlServo(11, 0);
    
    console.log('Spray sequence stopped');
  };

  return (
    <div>
      <button onClick={startSpraySequence}>Start Sequence</button>
      <button onClick={stopSpraySequence}>Stop Sequence</button>
    </div>
  );
}
```

---

### **Use Case 4: Smooth Servo Animation**
```tsx
function SmoothServoMove() {
  const { services } = useRover();
  const [isAnimating, setIsAnimating] = useState(false);

  const animateServo = async () => {
    setIsAnimating(true);
    
    const sequence = ServoUtils.generateSequence(0, 180, 20);
    
    for (const angle of sequence) {
      await services.controlServo(10, angle);
      await new Promise(r => setTimeout(r, 100));
    }
    
    setIsAnimating(false);
  };

  return (
    <button onClick={animateServo} disabled={isAnimating}>
      {isAnimating ? 'Animating...' : 'Animate Servo'}
    </button>
  );
}
```

---

### **Use Case 5: Real-Time PWM Display**
```tsx
function PWMDisplay() {
  const { telemetry: { servo } } = useRover();

  return (
    <div className="bg-slate-800 p-4 rounded">
      <h3>Servo 10 Status</h3>
      <div className="text-2xl font-mono">
        {servo.servo10_pwm || '---'} μs
      </div>
      <div className={servo.active ? 'text-green-400' : 'text-slate-500'}>
        {servo.active ? '● ACTIVE' : '○ IDLE'}
      </div>
      {servo.last_command_ts && (
        <div className="text-xs text-slate-400">
          Last: {new Date(servo.last_command_ts).toLocaleTimeString()}
        </div>
      )}
    </div>
  );
}
```

---

### **Use Case 6: Emergency Stop Integration**
```tsx
function EmergencyStopButton() {
  const { services } = useRover();

  const handleEmergencyStop = async () => {
    if (window.confirm('Emergency stop all servos?')) {
      try {
        // Stop all critical servos
        await Promise.all([
          services.controlServo(10, 0),  // Spray
          services.controlServo(11, 0),  // Camera
          services.controlServo(12, 0),  // Aux 1
          services.controlServo(13, 0),  // Aux 2
        ]);
        alert('✅ All servos stopped');
      } catch (err) {
        alert('❌ Emergency stop failed');
      }
    }
  };

  return (
    <button 
      onClick={handleEmergencyStop}
      className="bg-red-600 hover:bg-red-700 text-white px-6 py-3 rounded-lg font-bold"
    >
      🚨 EMERGENCY STOP
    </button>
  );
}
```

---

### **Use Case 7: Servo Status Dashboard**
```tsx
function ServosDashboard() {
  const monitors = useServoMonitor([10, 11, 12, 13]);
  const { services } = useRover();

  const resetAllServos = async () => {
    await Promise.all(
      monitors.map(m => services.controlServo(m.servoId, 0))
    );
  };

  return (
    <div className="p-4 bg-slate-900 rounded-lg">
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-xl font-bold text-white">Servo Dashboard</h2>
        <button 
          onClick={resetAllServos}
          className="bg-slate-700 text-white px-4 py-2 rounded"
        >
          Reset All
        </button>
      </div>
      
      <div className="grid grid-cols-2 gap-3">
        {monitors.map(m => (
          <div 
            key={m.servoId}
            className={`p-3 rounded ${
              m.isActive ? 'bg-green-900' : 'bg-slate-800'
            }`}
          >
            <div className="text-sm text-slate-400">Servo {m.servoId}</div>
            <div className="text-xl font-mono text-white">
              {m.currentPwm ?? '---'} μs
            </div>
            <div className="text-xs text-slate-500">
              {m.isActive ? '● Active' : '○ Idle'}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
```

---

## 🎯 Integration Examples

### **Add to Main Dashboard**
```tsx
import ServoPanel from './components/ServoPanel';
import ServoMonitorDisplay from './components/ServoControl/ServoMonitorDisplay';

function Dashboard() {
  return (
    <div className="grid grid-cols-3 gap-4">
      {/* Existing panels */}
      <BatteryPanel />
      <GPSPanel />
      
      {/* Add servo control */}
      <ServoPanel />
      
      {/* Add servo monitoring */}
      <div className="col-span-3">
        <ServoMonitorDisplay servoIds={[10, 11, 12, 13]} />
      </div>
    </div>
  );
}
```

---

### **Add to Tab Interface**
```tsx
import ServoControlTab from './components/ServoControl/ServoControlTab';

function App() {
  const [tab, setTab] = useState('map');

  return (
    <div>
      <nav>
        <button onClick={() => setTab('map')}>Map</button>
        <button onClick={() => setTab('servos')}>Servos</button>
      </nav>
      
      <main>
        {tab === 'map' && <MapView />}
        {tab === 'servos' && <ServoControlTab />}
      </main>
    </div>
  );
}
```

---

## 🔌 Backend API Reference

### **Control Servo**
```bash
curl -X POST http://localhost:5001/api/servo/control \
  -H "Content-Type: application/json" \
  -d '{"servo_id": 10, "angle": 90}'
```

### **Get Status**
```bash
curl http://localhost:5001/servo/status
```

### **Emergency Stop**
```bash
curl -X POST http://localhost:5001/servo/emergency_stop
```

---

## 📊 PWM Reference Table

| Angle | PWM (μs) | Position |
|-------|----------|----------|
| 0°    | 1000     | Minimum  |
| 45°   | 1250     | Quarter  |
| 90°   | 1500     | Center   |
| 135°  | 1750     | 3/4      |
| 180°  | 2000     | Maximum  |

---

## 🎯 How WP_MARK Works (3 Simple Steps)

**WP_MARK** automates servo control at GPS waypoints during missions:

1. **🚜 Rover Reaches Waypoint**
   - GPS detects arrival (within 2m of target)
   - Example: Target at 12.345°N, 77.123°E

2. **🚿 Servo Activates**
   - Wait 2s → Spray ON (PWM 1500) → Spray 5s → Spray OFF (PWM 1000)
   - Configurable timing and PWM values

3. **➡️ Move to Next Waypoint**
   - Brief pause → Auto-navigate to next waypoint
   - Process repeats for entire mission

**See `WP_MARK_HOW_IT_WORKS.md` for detailed explanation!**

---

## ✅ Implementation Checklist

- [x] Backend API endpoint working
- [x] Frontend service integrated
- [x] TypeScript types defined
- [x] Basic UI component (`ServoPanel`)
- [x] Advanced UI component (`ServoControlTab`)
- [x] Multi-servo control (`MultiServoControl`)
- [x] Real-time monitoring (`ServoMonitorDisplay`)
- [x] Sequence control (`ServoSequenceControl`)
- [x] Utility functions (`ServoUtils`)
- [x] Custom hooks (`useServoMonitor`)
- [x] Error handling implemented
- [x] Real-time telemetry working

---

## 🎉 You're Ready!

All servo control components are **production-ready** and fully integrated with your rover system. Choose the components that fit your needs and start controlling servos! 🚀
