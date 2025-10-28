export interface TelemetryState {
  armed: boolean;
  mode: string;
  system_status: string;
  heartbeat_ts: number;
}

export interface TelemetryGlobal {
  lat: number;
  lon: number;
  alt_rel: number;
  vel: number;
  satellites_visible: number;
}

export interface TelemetryBattery {
  voltage: number;
  current: number;
  percentage: number;
}

export interface TelemetryRtk {
  fix_type: number;
  baseline_age: number;
  base_linked: boolean;
}

export interface TelemetryMission {
  total_wp: number;
  current_wp: number;
  status: string;
  progress_pct: number;
}

export interface ServoStatus {
  servo_id: number;
  active: boolean;
  last_command_ts: number;
}

export interface TelemetryEnvelope {
  state?: TelemetryState;
  global?: TelemetryGlobal;
  battery?: TelemetryBattery;
  rtk?: TelemetryRtk;
  mission?: TelemetryMission;
  servo?: ServoStatus;
  timestamp?: number;
}

export interface ServiceResponse {
  success: boolean;
  message?: string;
  [key: string]: unknown;
}

export interface RoverTelemetry {
  state: TelemetryState;
  global: TelemetryGlobal;
  battery: TelemetryBattery;
  rtk: TelemetryRtk;
  mission: TelemetryMission;
  servo: ServoStatus;
  lastMessageTs: number | null;
}
