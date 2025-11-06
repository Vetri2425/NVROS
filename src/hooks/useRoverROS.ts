import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { io, type ManagerOptions, type Socket, type SocketOptions } from 'socket.io-client';
import { BACKEND_URL } from '../config';
import type { Waypoint } from '../types';
import type {
  RoverTelemetry,
  ServiceResponse,
  ServoStatus,
  TelemetryBattery,
  TelemetryEnvelope,
  TelemetryGlobal,
  TelemetryMission,
  TelemetryRtk,
  TelemetryState,
} from '../types/ros';

export type ConnectionState = 'connecting' | 'connected' | 'disconnected' | 'error';

const DEFAULT_HTTP_BASE = BACKEND_URL.replace(/\/$/, '');

const SOCKET_PATH = import.meta.env.VITE_ROS_SOCKET_PATH ?? '/socket.io';
const SOCKET_TRANSPORTS = (
  import.meta.env.VITE_ROS_SOCKET_TRANSPORTS ?? 'websocket,polling'
)
  .split(',')
  .map((t) => t.trim())
  .filter(Boolean);

const SOCKET_OPTIONS: Partial<ManagerOptions & SocketOptions> = {
  path: SOCKET_PATH,
  transports: SOCKET_TRANSPORTS.length > 0 ? SOCKET_TRANSPORTS : ['websocket', 'polling'],
  autoConnect: false,
  withCredentials: false,
  reconnection: true,
  reconnectionAttempts: Infinity,
  reconnectionDelay: 1000,
  reconnectionDelayMax: 5000,
  timeout: 20000,
};

const API_BASE = `${DEFAULT_HTTP_BASE}/api`;

const THROTTLE_MS = 33; // ~30 Hz
const MAX_BACKOFF_MS = 8000;
const INITIAL_BACKOFF_MS = 1000;

interface MutableTelemetry {
  telemetry: RoverTelemetry;
  lastEnvelopeTs: number | null;
}

const DEFAULT_STATE: TelemetryState = {
  armed: false,
  mode: 'UNKNOWN',
  system_status: 'STANDBY', // Changed from UNKNOWN to STANDBY as per telemetry_node.py
  heartbeat_ts: 0,
};

const DEFAULT_GLOBAL: TelemetryGlobal = {
  lat: 0,
  lon: 0,
  alt_rel: 0,
  vel: 0,
  satellites_visible: 0,
};

const DEFAULT_BATTERY: TelemetryBattery = {
  voltage: 0,
  current: 0,
  percentage: 0,
};

const DEFAULT_RTK: TelemetryRtk = {
  fix_type: 0,
  baseline_age: 0,
  base_linked: false,
};

const DEFAULT_MISSION: TelemetryMission = {
  total_wp: 0,
  current_wp: 0,
  status: 'IDLE',
  progress_pct: 0,
};

const DEFAULT_SERVO: ServoStatus = {
  servo_id: 0,
  active: false,
  last_command_ts: 0,
};

const createDefaultTelemetry = (): RoverTelemetry => ({
  state: { ...DEFAULT_STATE },
  global: { ...DEFAULT_GLOBAL },
  battery: { ...DEFAULT_BATTERY },
  rtk: { ...DEFAULT_RTK },
  mission: { ...DEFAULT_MISSION },
  servo: { ...DEFAULT_SERVO },
  lastMessageTs: null,
});

async function fetchJson<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(path, {
    headers: {
      'Content-Type': 'application/json',
      ...(init?.headers ?? {}),
    },
    ...init,
  });
  if (!response.ok) {
    const text = await response.text().catch(() => '');
    throw new Error(`Request failed (${response.status}): ${text}`);
  }
  return (await response.json()) as T;
}

async function postService(path: string, body?: Record<string, unknown>): Promise<ServiceResponse> {
  return fetchJson<ServiceResponse>(`${API_BASE}${path}`, {
    method: 'POST',
    body: body ? JSON.stringify(body) : undefined,
  });
}

async function getService<T extends ServiceResponse = ServiceResponse>(path: string): Promise<T> {
  return fetchJson<T>(`${API_BASE}${path}`);
}

// Map legacy string RTK status to numeric fix type (GPS GPSRAW standard)
// 0-1: No Fix, 2: 2D Fix, 3: 3D Fix, 4: DGPS, 5: RTK Float, 6: RTK Fixed
const mapRtkStatusToFixType = (status?: string | null): number => {
  const normalized = (status ?? '').toUpperCase();
  switch (normalized) {
    case 'RTK FIXED':
    case 'FIX':
      return 6;  // ✅ Corrected from 4 to 6
    case 'RTK FLOAT':
    case 'FLOAT':
      return 5;  // ✅ Corrected from 3 to 5
    case 'DGPS':
      return 4;  // ✅ Corrected from 2 to 4
    case '3D FIX':
    case '3D':
      return 3;
    case '2D FIX':
    case '2D':
      return 2;
    case 'GPS FIX':
    case 'GPS':
      return 1;
    default:
      return 0;
  }
};

const toTelemetryEnvelopeFromRoverData = (data: any): TelemetryEnvelope | null => {
  if (!data || typeof data !== 'object') {
    return null;
  }

  // Debug: Log incoming rover_data (only RTK-related fields to reduce noise)
  if (data.rtk_fix_type !== undefined || data.rtk_status) {
    console.log('[useRoverROS] rover_data RTK fields:', {
      rtk_fix_type: data.rtk_fix_type,
      rtk_status: data.rtk_status,
      rtk_base_linked: data.rtk_base_linked,
    });
  }

  const envelope: Partial<TelemetryEnvelope> = {
    timestamp: Date.now(),
  };
  let touched = false;

  if (data.mode || data.status || data.last_heartbeat != null) {
    const status = typeof data.status === 'string' ? data.status : 'UNKNOWN';
    const heartbeatSeconds =
      typeof data.last_heartbeat === 'number' ? data.last_heartbeat : undefined;
    envelope.state = {
      armed: String(status).toLowerCase() === 'armed',
      mode: typeof data.mode === 'string' ? data.mode : 'UNKNOWN',
      system_status: status.toUpperCase(),
      heartbeat_ts: heartbeatSeconds ? Math.floor(heartbeatSeconds * 1000) : Date.now(),
    };
    touched = true;
  }

  if (data.position && typeof data.position === 'object') {
    const { lat, lng } = data.position as { lat?: number; lng?: number };
    // DEBUG: Log position parsing
    console.log('[toTelemetryEnvelope] Parsing position:', { lat, lng });
    envelope.global = {
      lat: typeof lat === 'number' ? lat : 0,
      lon: typeof lng === 'number' ? lng : 0,
      alt_rel: typeof data.distanceToNext === 'number' ? data.distanceToNext : 0,
      // Prefer an explicit 'groundspeed' field from backend if present
      vel: typeof data.groundspeed === 'number' ? data.groundspeed : 0,
      satellites_visible: typeof data.satellites_visible === 'number' ? data.satellites_visible : 0,
    };
    console.log('[toTelemetryEnvelope] Created envelope.global:', envelope.global);
    touched = true;
  }

  if (data.battery != null || data.voltage != null || data.current != null) {
    envelope.battery = {
      percentage: typeof data.battery === 'number' ? data.battery : 0,
      voltage: typeof data.voltage === 'number' ? data.voltage : 0,
      current: typeof data.current === 'number' ? data.current : 0,
    };
    touched = true;
  }

  // ✅ PRIORITY FIX: Check numeric rtk_fix_type FIRST (most accurate)
  if (typeof data.rtk_fix_type === 'number') {
    // Direct RTK fix_type from backend (preferred - accurate numeric value)
    console.log('[useRoverROS] ✅ Using rtk_fix_type:', data.rtk_fix_type);
    envelope.rtk = {
      fix_type: data.rtk_fix_type,
      baseline_age: typeof data.rtk_baseline_age === 'number' ? data.rtk_baseline_age : 0,
      base_linked: typeof data.rtk_base_linked === 'boolean' ? data.rtk_base_linked : data.rtk_fix_type >= 5,
    };
    touched = true;
  } else if (data.rtk_status) {
    // Fallback: Map string rtk_status to numeric fix_type (legacy path)
    const fixType = mapRtkStatusToFixType(data.rtk_status);
    console.log('[useRoverROS] Fallback to rtk_status mapping:', {
      rtk_status: data.rtk_status,
      mapped_fix_type: fixType,
    });
    envelope.rtk = {
      fix_type: fixType,
      baseline_age: typeof data.rtk_baseline_age === 'number' ? data.rtk_baseline_age : 0,
      base_linked: typeof data.rtk_base_linked === 'boolean' ? data.rtk_base_linked : fixType >= 5,
    };
    touched = true;
  }
  
  // Handle RTK baseline data from mavros_bridge
  if (data.rtk_baseline && typeof data.rtk_baseline === 'object') {
    const baseline = data.rtk_baseline;
    if (!envelope.rtk) {
      envelope.rtk = { ...DEFAULT_RTK };
    }
    envelope.rtk = {
      ...envelope.rtk,
      baseline_distance: typeof baseline.baseline_distance === 'number' ? baseline.baseline_distance : undefined,
      baseline_coords: {
        a_mm: typeof baseline.baseline_a_mm === 'number' ? baseline.baseline_a_mm : 0,
        b_mm: typeof baseline.baseline_b_mm === 'number' ? baseline.baseline_b_mm : 0,
        c_mm: typeof baseline.baseline_c_mm === 'number' ? baseline.baseline_c_mm : 0,
      },
      accuracy: typeof baseline.accuracy === 'number' ? baseline.accuracy : undefined,
      nsats: typeof baseline.nsats === 'number' ? baseline.nsats : undefined,
      rtk_health: typeof baseline.rtk_health === 'number' ? baseline.rtk_health : undefined,
      rtk_rate: typeof baseline.rtk_rate === 'number' ? baseline.rtk_rate : undefined,
    };
    touched = true;
  }

  const activeIndex =
    typeof data.activeWaypointIndex === 'number' && data.activeWaypointIndex >= 0
      ? data.activeWaypointIndex
      : null;
  const completedCount = Array.isArray(data.completedWaypointIds)
    ? data.completedWaypointIds.length
    : 0;
  const currentWp = activeIndex != null ? activeIndex + 1 : 0;
  const inferredTotal = Math.max(
    currentWp,
    completedCount,
    typeof data.current_waypoint_id === 'number' ? data.current_waypoint_id : 0,
  );

  if (activeIndex != null || completedCount > 0) {
    const total = inferredTotal || (currentWp > 0 ? currentWp : completedCount);
    const progress = total > 0 ? (currentWp / total) * 100 : 0;
    envelope.mission = {
      total_wp: total,
      current_wp: currentWp,
      status: currentWp > 0 ? 'ACTIVE' : 'IDLE',
      progress_pct: progress,
    };
    touched = true;
  }

  // Handle servo_output telemetry
  if (data.servo_output && typeof data.servo_output === 'object') {
    const servoOutput = data.servo_output;
    const servoPwmValues: Partial<ServoStatus> = {
      servo_id: 0,
      active: false,
      last_command_ts: 0,
    };

    // Map individual servo PWM values
    if (Array.isArray(servoOutput.channels)) {
      servoPwmValues.pwm_values = servoOutput.channels;
      for (let i = 1; i <= 16; i++) {
        const key = `servo${i}_pwm` as keyof ServoStatus;
        if (servoOutput[key] !== undefined) {
          (servoPwmValues as any)[key] = servoOutput[key];
        }
      }
    }

    envelope.servo = servoPwmValues as ServoStatus;
    touched = true;
  }

  return touched ? (envelope as TelemetryEnvelope) : null;
};

const toTelemetryEnvelopeFromBridge = (data: any): TelemetryEnvelope | null => {
  if (!data || typeof data !== 'object') {
    return null;
  }

  const envelope: Partial<TelemetryEnvelope> = {
    timestamp: Date.now(),
  };
  let touched = false;

  if (data.state && typeof data.state === 'object') {
    envelope.state = {
      armed: Boolean(data.state.armed),
      mode: typeof data.state.mode === 'string' ? data.state.mode : 'UNKNOWN',
      system_status:
        typeof data.state.system_status === 'string'
          ? data.state.system_status
          : 'UNKNOWN',
      heartbeat_ts:
        typeof data.state.heartbeat_ts === 'number'
          ? data.state.heartbeat_ts
          : Date.now(),
    };
    touched = true;
  }

  if (data.position && typeof data.position === 'object') {
    envelope.global = {
      lat: typeof data.position.latitude === 'number' ? data.position.latitude : 0,
      lon: typeof data.position.longitude === 'number' ? data.position.longitude : 0,
      alt_rel: typeof data.position.altitude === 'number' ? data.position.altitude : 0,
      vel: 0,
      satellites_visible: 0,
    };
    touched = true;
  }

  return touched ? (envelope as TelemetryEnvelope) : null;
};

export interface RoverServices {
  armVehicle: () => Promise<ServiceResponse>;
  disarmVehicle: () => Promise<ServiceResponse>;
  setMode: (mode: string) => Promise<ServiceResponse>;
  uploadMission: (waypoints: Waypoint[]) => Promise<ServiceResponse>;
  downloadMission: () => Promise<ServiceResponse & { waypoints?: Waypoint[] }>;
  clearMission: () => Promise<ServiceResponse>;
  setCurrentWaypoint: (wpSeq: number) => Promise<ServiceResponse>;
  pauseMission: () => Promise<ServiceResponse>;
  resumeMission: () => Promise<ServiceResponse>;
  injectRTK: (ntripUrl: string) => Promise<ServiceResponse>;
  stopRTK: () => Promise<ServiceResponse>;
  getRTKStatus: () => Promise<ServiceResponse & { running?: boolean; caster?: any; total_bytes?: number }>;
  controlServo: (servoId: number, angle: number) => Promise<ServiceResponse>;
}

export interface MissionEventPayload {
  timestamp: number;
  message: string;
  status?: string | null;
  waypointId?: number | null;
  servoAction?: string | null;
  lat?: number | null;
  lng?: number | null;
}

export interface UseRoverROSResult {
  telemetry: RoverTelemetry;
  connectionState: ConnectionState;
  reconnect: () => void;
  services: RoverServices;
  setMissionEventHandler: (handler: ((event: MissionEventPayload) => void) | null) => void;
}

export function useRoverROS(): UseRoverROSResult {
  const [telemetrySnapshot, setTelemetrySnapshot] = useState<RoverTelemetry>(createDefaultTelemetry);
  const [connectionState, setConnectionState] = useState<ConnectionState>('connecting');

  const socketRef = useRef<Socket | null>(null);
  const manualDisconnectRef = useRef(false);
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const backoffRef = useRef<number>(INITIAL_BACKOFF_MS);
  const connectDelayRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const mutableRef = useRef<MutableTelemetry>({
    telemetry: createDefaultTelemetry(),
    lastEnvelopeTs: null,
  });
  const lastDispatchRef = useRef<number>(0);
  const pendingDispatchRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const connectSocketRef = useRef<() => void>(() => {});
  const mountedRef = useRef(true);
  const missionEventHandlerRef = useRef<((event: MissionEventPayload) => void) | null>(null);

  const applyEnvelope = useCallback((envelope: TelemetryEnvelope) => {
    const mutable = mutableRef.current;
    const next = { ...mutable.telemetry };

    if (envelope.state) {
      next.state = { ...next.state, ...envelope.state };
    }
    if (envelope.global) {
      next.global = { ...next.global, ...envelope.global };
    }
    if (envelope.battery) {
      next.battery = { ...next.battery, ...envelope.battery };
    }
    if (envelope.rtk) {
      next.rtk = { ...next.rtk, ...envelope.rtk };
    }
    if (envelope.mission) {
      next.mission = { ...next.mission, ...envelope.mission };
    }
    if (envelope.servo) {
      next.servo = { ...next.servo, ...envelope.servo };
    }

    next.lastMessageTs = envelope.timestamp ?? Date.now();

    mutable.telemetry = next;
    mutable.lastEnvelopeTs = Date.now();

    const now = performance.now();
    const elapsed = now - lastDispatchRef.current;

    if (elapsed >= THROTTLE_MS) {
      lastDispatchRef.current = now;
      setTelemetrySnapshot(next);
    } else {
      if (pendingDispatchRef.current) {
        clearTimeout(pendingDispatchRef.current);
      }
      pendingDispatchRef.current = setTimeout(() => {
        lastDispatchRef.current = performance.now();
        setTelemetrySnapshot(mutableRef.current.telemetry);
        pendingDispatchRef.current = null;
      }, THROTTLE_MS - elapsed);
    }
  }, []);

  const pingIntervalRef = useRef<ReturnType<typeof setInterval> | null>(null);

  const clearReconnectTimer = useCallback(() => {
    if (reconnectTimerRef.current) {
      clearTimeout(reconnectTimerRef.current);
      reconnectTimerRef.current = null;
    }
  }, []);

  const teardownSocket = useCallback(() => {
    clearReconnectTimer();

    if (connectDelayRef.current) {
      clearTimeout(connectDelayRef.current);
      connectDelayRef.current = null;
    }
    // Clear ping interval if it exists
    if (pingIntervalRef.current) {
      clearInterval(pingIntervalRef.current);
      pingIntervalRef.current = null;
    }

    if (socketRef.current) {
      manualDisconnectRef.current = true;
      socketRef.current.removeAllListeners();
      socketRef.current.disconnect();
      setTimeout(() => {
        manualDisconnectRef.current = false;
      }, 0);
      socketRef.current = null;
    }

    if (pendingDispatchRef.current) {
      clearTimeout(pendingDispatchRef.current);
      pendingDispatchRef.current = null;
    }
  }, [clearReconnectTimer]);

  const scheduleReconnect = useCallback(() => {
    if (reconnectTimerRef.current) {
      return;
    }
    const delay = Math.min(backoffRef.current, MAX_BACKOFF_MS);
    reconnectTimerRef.current = setTimeout(() => {
      reconnectTimerRef.current = null;
      backoffRef.current = Math.min(Math.floor(backoffRef.current * 1.5), MAX_BACKOFF_MS);
      connectSocketRef.current();
    }, delay);
  }, []);

  const handleBridgeTelemetry = useCallback(
    (payload: any) => {
      const envelope = toTelemetryEnvelopeFromBridge(payload);
      if (envelope) {
        applyEnvelope(envelope);
      }
    },
    [applyEnvelope],
  );

  const handleRoverData = useCallback(
    (payload: any) => {
      // DEBUG: Log received rover_data
      if (payload?.position) {
        console.log('[useRoverROS] Received rover_data with position:', payload.position);
      } else {
        console.log('[useRoverROS] Received rover_data WITHOUT position', payload);
      }
      
      const envelope = toTelemetryEnvelopeFromRoverData(payload);
      if (envelope) {
        // DEBUG: Log parsed envelope
        if (envelope.global) {
          console.log('[useRoverROS] Parsed envelope.global:', envelope.global);
        }
        applyEnvelope(envelope);
      } else {
        console.warn('[useRoverROS] Failed to parse rover_data into envelope');
      }
    },
    [applyEnvelope],
  );

  const handleMissionEvent = useCallback((payload: any) => {
    if (missionEventHandlerRef.current && payload) {
      try {
        missionEventHandlerRef.current(payload);
      } catch (error) {
        console.error('Error in mission event handler:', error);
      }
    }
  }, []);

  const connectSocket = useCallback(() => {
    teardownSocket();

    if (connectDelayRef.current) {
      clearTimeout(connectDelayRef.current);
    }

    setConnectionState('connecting');

    connectDelayRef.current = setTimeout(() => {
      connectDelayRef.current = null;

      if (!mountedRef.current) {
        return;
      }

      try {
        const socket = io(DEFAULT_HTTP_BASE, SOCKET_OPTIONS);
        socketRef.current = socket;

        socket.on('connect', () => {
          console.log('Socket connected successfully');
          clearReconnectTimer();
          backoffRef.current = INITIAL_BACKOFF_MS;
          setConnectionState('connected');
          socket.emit('ping');
        });

        socket.on('pong', () => {
          console.log('Received pong from server');
        });

        socket.on('connect_error', (error) => {
          console.error('Socket.IO connection error:', error);
          setConnectionState('error');
          scheduleReconnect();
        });

        socket.on('disconnect', (reason) => {
          console.warn('Socket disconnected:', reason);
          if (manualDisconnectRef.current) {
            manualDisconnectRef.current = false;
            return;
          }
          setConnectionState('disconnected');
          if (reason === 'io server disconnect') {
            socket.connect();
          } else {
            scheduleReconnect();
          }
        });

        socket.on('error', (error) => {
          console.error('Socket error:', error);
          setConnectionState('error');
        });

        socket.io.on('reconnect', (attempt) => {
          console.log('Socket reconnected after', attempt, 'attempts');
          clearReconnectTimer();
          backoffRef.current = INITIAL_BACKOFF_MS;
          setConnectionState('connected');
        });

        socket.io.on('reconnect_attempt', () => {
          console.log('Attempting to reconnect...');
          setConnectionState('connecting');
        });

        socket.io.on('reconnect_error', (error) => {
          console.error('Reconnection error:', error);
          setConnectionState('error');
        });

        socket.io.on('reconnect_failed', () => {
          console.error('Failed to reconnect');
          clearReconnectTimer();
          setConnectionState('error');
        });

        socket.on('telemetry', handleBridgeTelemetry);
        socket.on('rover_data', handleRoverData);
        socket.on('mission_event', handleMissionEvent);

        socket.connect();

        const pingInterval = setInterval(() => {
          if (socket.connected) {
            socket.emit('ping');
          }
        }, 5000);

        pingIntervalRef.current = pingInterval;
      } catch (error) {
        console.error('Failed to initialize Socket.IO connection:', error);
        setConnectionState('error');
        scheduleReconnect();
      }
    }, 100);
  }, [clearReconnectTimer, handleBridgeTelemetry, handleRoverData, handleMissionEvent, scheduleReconnect, teardownSocket]);

  const reconnect = useCallback(() => {
    clearReconnectTimer();
    backoffRef.current = INITIAL_BACKOFF_MS;
    connectSocket();
  }, [clearReconnectTimer, connectSocket]);

  useEffect(() => {
    connectSocketRef.current = connectSocket;
  }, [connectSocket]);

  useEffect(() => {
    mountedRef.current = true;

    const handleVisibilityChange = () => {
      if (document.visibilityState === 'visible') {
        console.log('Page became visible, checking connection...');
        if (socketRef.current?.disconnected) {
          console.log('Socket was disconnected, reconnecting...');
          reconnect();
        }
      }
    };

    const handleOnline = () => {
      console.log('Network connection restored, reconnecting...');
      reconnect();
    };

    // Initial connection
    connectSocket();

    // Add event listeners for visibility and network changes
    document.addEventListener('visibilitychange', handleVisibilityChange);
    window.addEventListener('online', handleOnline);

    return () => {
      mountedRef.current = false;
      // Clean up all listeners and connections
      document.removeEventListener('visibilitychange', handleVisibilityChange);
      window.removeEventListener('online', handleOnline);
      
      clearReconnectTimer();
      teardownSocket();
    };
  }, [clearReconnectTimer, connectSocket, teardownSocket, reconnect]);

const pushStatePatch = useCallback(
  (patch: Partial<TelemetryState>) => {
    const baseState = mutableRef.current.telemetry.state;
    const envelope: TelemetryEnvelope = {
      timestamp: Date.now(),
      state: {
        ...baseState,
        ...patch,
        heartbeat_ts: patch.heartbeat_ts ?? Date.now(),
      },
    };
    applyEnvelope(envelope);
  },
  [applyEnvelope],
);

const services = useMemo<RoverServices>(
  () => ({
    armVehicle: async () => {
      const response = await postService('/arm', { value: true });
      if (response.success) {
        pushStatePatch({ armed: true, system_status: 'ARMED' });
      }
      return response;
    },
    disarmVehicle: async () => {
      const response = await postService('/arm', { value: false });
      if (response.success) {
        pushStatePatch({ armed: false, system_status: 'DISARMED' });
      }
      return response;
    },
    setMode: (mode: string) => postService('/set_mode', { mode }),
    uploadMission: (waypoints: Waypoint[]) => postService('/mission/upload', { waypoints }),
    downloadMission: () => getService('/mission/download'),
    clearMission: () => postService('/mission/clear'),
    setCurrentWaypoint: (wpSeq: number) => postService('/mission/set_current', { wp_seq: wpSeq }),
    pauseMission: () => postService('/mission/pause'),
    resumeMission: () => postService('/mission/resume'),
    injectRTK: (ntripUrl: string) => postService('/rtk/inject', { ntrip_url: ntripUrl }),
    stopRTK: () => postService('/rtk/stop'),
    getRTKStatus: () => getService('/rtk/status'),
    controlServo: (servoId: number, angle: number) =>
      postService('/servo/control', { servo_id: servoId, angle }),
  }),
  [pushStatePatch],
);

  const setMissionEventHandler = useCallback((handler: ((event: MissionEventPayload) => void) | null) => {
    missionEventHandlerRef.current = handler;
  }, []);

  return {
    telemetry: telemetrySnapshot,
    connectionState,
    reconnect,
    services,
    setMissionEventHandler,
  };
}

export default useRoverROS;
