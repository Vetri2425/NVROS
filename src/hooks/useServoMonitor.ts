// src/hooks/useServoMonitor.ts
import { useState, useEffect } from 'react';
import { useRover } from '../context/RoverContext';

export interface ServoMonitor {
  servoId: number;
  currentPwm: number | null;
  isActive: boolean;
  lastUpdate: Date | null;
}

/**
 * Custom hook to monitor multiple servo channels in real-time
 * 
 * @param servoIds - Array of servo IDs to monitor (1-16)
 * @returns Array of ServoMonitor objects with current status
 * 
 * @example
 * ```tsx
 * function MyComponent() {
 *   const monitors = useServoMonitor([10, 11, 12, 13]);
 *   
 *   return (
 *     <div>
 *       {monitors.map(m => (
 *         <div key={m.servoId}>
 *           Servo {m.servoId}: {m.currentPwm} μs
 *         </div>
 *       ))}
 *     </div>
 *   );
 * }
 * ```
 */
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
        lastUpdate: servo.last_command_ts ? new Date(servo.last_command_ts) : null,
      };
    });

    setMonitors(newMonitors);
  }, [servo, servoIds]);

  return monitors;
}
