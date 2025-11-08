// src/components/ServoControl/ServoMonitorDisplay.tsx
import React from 'react';
import { useServoMonitor } from '../../hooks/useServoMonitor';

interface ServoMonitorDisplayProps {
  servoIds?: number[];
  className?: string;
}

/**
 * Display component for monitoring multiple servos in a grid layout
 * Shows real-time PWM values and active status for each servo
 */
const ServoMonitorDisplay: React.FC<ServoMonitorDisplayProps> = ({ 
  servoIds = [10, 11, 12, 13],
  className = '' 
}) => {
  const monitors = useServoMonitor(servoIds);
  
  return (
    <div className={`grid grid-cols-4 gap-2 ${className}`}>
      {monitors.map(monitor => (
        <div 
          key={monitor.servoId} 
          className={`p-2 rounded text-center transition-colors ${
            monitor.isActive ? 'bg-green-900 border-green-500' : 'bg-slate-800 border-slate-700'
          } border`}
        >
          <div className="text-xs text-slate-400 mb-1">
            Servo {monitor.servoId}
          </div>
          <div className="text-sm font-mono text-white font-bold">
            {monitor.currentPwm ?? '---'} μs
          </div>
          <div className={`text-xs mt-1 ${
            monitor.isActive ? 'text-green-400' : 'text-slate-500'
          }`}>
            {monitor.isActive ? '● ACTIVE' : '○ IDLE'}
          </div>
          {monitor.lastUpdate && (
            <div className="text-xs text-slate-500 mt-1">
              {monitor.lastUpdate.toLocaleTimeString()}
            </div>
          )}
        </div>
      ))}
    </div>
  );
};

export default ServoMonitorDisplay;
