export interface LogEntry {
  timestamp: string;
  lat: number | null;
  lng: number | null;
  event: string;
  waypointId?: number | null;
  status?: string | null;
  servoAction?: string | null;
}

export interface BackendLogEntry {
  timestamp: string;
  message: string;
  lat?: number | null;
  lng?: number | null;
  waypointId?: number | null;
  status?: string | null;
  servoAction?: string | null;
}

export interface MissionFileInfo {
  name: string;
  size: number;
  type: string;
  uploadedAt: string;
  waypointCount: number;
  source?: 'file' | 'generated' | 'drawn' | 'downloaded';
}

export interface MissionLog {
  id: string;
  name: string;
  status: 'Completed' | 'Incomplete' | 'In Progress';
  timestamp: Date;
  entries: LogEntry[];
}

/**
 * A single, unified data structure for all rover data,
 * combining real telemetry with UI-specific values.
 */
export interface RoverData {
  // --- Core Connection & Mode ---
  connected: boolean;
  mode: string;
  status: 'armed' | 'disarmed';
  
  // --- Position & Navigation ---
  position: { lat: number; lng: number } | null;
  latitude?: number;
  longitude?: number;
  altitude?: number;
  relative_altitude?: number;
  heading: number;
  groundspeed?: number;
  
  // --- GPS & RTK Status ---
  rtk_status: string;
  fix_type?: number;
  satellites_visible?: number;
  hrms: string | number;
  vrms: string | number;
  
  // --- System Status ---
  battery: number;
  voltage?: number;
  current?: number;
  cpu_load?: number;
  drop_rate?: number;
  rc_connected: boolean;
  signal_strength: string;
  imu_status: string;
  
  // --- Mission Progress ---
  mission_progress?: {
    current: number;
    total: number;
  };
  current_waypoint_id: number | null;
  activeWaypointIndex: number | null;
  completedWaypointIds: number[];
  distanceToNext: number;
  
  // --- Servo Output (PWM Values) ---
  servo_output?: {
    channels: number[];
    count: number;
    servo1_pwm?: number;
    servo2_pwm?: number;
    servo3_pwm?: number;
    servo4_pwm?: number;
    servo5_pwm?: number;
    servo6_pwm?: number;
    servo7_pwm?: number;
    servo8_pwm?: number;
    servo9_pwm?: number;
    servo10_pwm?: number;
    servo11_pwm?: number;
    servo12_pwm?: number;
    servo13_pwm?: number;
    servo14_pwm?: number;
    servo15_pwm?: number;
    servo16_pwm?: number;
  };
  
  // --- Metadata ---
  lastUpdate: string | number;
  telemetryAgeMs?: number;
}

export type Waypoint = {
  id: number;
  command: string;
  lat: number;
  lng: number;
  alt: number;
  frame: number;
  current?: number; // 0 or 1
  autocontinue?: number; // 0 or 1
  param1?: number;
  param2?: number;
  param3?: number;
  param4?: number;
  action?: string;
};

export type ViewMode = 'dashboard' | 'planning' | 'live' | 'logs' | 'map' | 'setup' | 'servo';
