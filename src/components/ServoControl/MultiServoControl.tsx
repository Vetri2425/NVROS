// src/components/ServoControl/MultiServoControl.tsx
import React, { useState } from 'react';
import { useRover } from '../../context/RoverContext';

interface ServoControlProps {
  className?: string;
}

const MultiServoControl: React.FC<ServoControlProps> = ({ className = '' }) => {
  const { telemetry: { servo }, services } = useRover();
  const [isBusy, setIsBusy] = useState(false);
  const [feedback, setFeedback] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  // Preset Servo Configurations
  const servoPresets = [
    { id: 10, name: 'Spray Nozzle', defaultAngle: 90, color: 'bg-blue-600' },
    { id: 11, name: 'Camera Gimbal', defaultAngle: 45, color: 'bg-green-600' },
    { id: 12, name: 'Auxiliary 1', defaultAngle: 0, color: 'bg-purple-600' },
    { id: 13, name: 'Auxiliary 2', defaultAngle: 0, color: 'bg-orange-600' },
  ];

  // Send Command to Multiple Servos
  const handleMultiCommand = async (commands: Array<{id: number, angle: number}>) => {
    setIsBusy(true);
    setFeedback(null);
    setError(null);

    try {
      const results = await Promise.allSettled(
        commands.map(cmd => services.controlServo(cmd.id, cmd.angle))
      );

      const successful = results.filter(r => r.status === 'fulfilled').length;
      const failed = results.length - successful;

      if (failed === 0) {
        setFeedback(`✅ All ${successful} servos commanded successfully`);
      } else {
        setError(`⚠️ ${successful} succeeded, ${failed} failed`);
      }
    } catch (err) {
      setError('Multi-servo command failed');
    } finally {
      setIsBusy(false);
    }
  };

  // Preset Actions
  const handlePresetAction = (action: string) => {
    switch (action) {
      case 'spray_start':
        handleMultiCommand([
          { id: 10, angle: 90 },  // Spray nozzle open
          { id: 11, angle: 45 },  // Camera down for monitoring
        ]);
        break;
      case 'spray_stop':
        handleMultiCommand([
          { id: 10, angle: 0 },   // Spray nozzle closed
          { id: 11, angle: 0 },   // Camera neutral
        ]);
        break;
      case 'all_neutral':
        handleMultiCommand(
          servoPresets.map(preset => ({ id: preset.id, angle: 0 }))
        );
        break;
      case 'all_default':
        handleMultiCommand(
          servoPresets.map(preset => ({ id: preset.id, angle: preset.defaultAngle }))
        );
        break;
    }
  };

  return (
    <div className={`bg-[#111827] rounded-lg p-4 ${className}`}>
      <h3 className="text-white font-semibold mb-4">Multi-Servo Control</h3>

      {/* Individual Servo Controls */}
      <div className="grid grid-cols-2 gap-3 mb-4">
        {servoPresets.map(preset => {
          const currentPwm = servo[`servo${preset.id}_pwm` as keyof typeof servo];
          return (
            <div key={preset.id} className="bg-[#1F2937] rounded-md p-3">
              <div className="flex items-center justify-between mb-2">
                <span className="text-xs font-semibold text-white">{preset.name}</span>
                <span className="text-xs text-slate-400">ID: {preset.id}</span>
              </div>
              {typeof currentPwm === 'number' && (
                <div className="text-sm font-mono text-green-400 mb-2">
                  {currentPwm} μs
                </div>
              )}
              <div className="flex gap-2">
                <button
                  onClick={() => services.controlServo(preset.id, preset.defaultAngle)}
                  disabled={isBusy}
                  className={`flex-1 px-2 py-1 rounded text-xs ${preset.color} hover:opacity-80 text-white disabled:opacity-50`}
                >
                  Default
                </button>
                <button
                  onClick={() => services.controlServo(preset.id, 0)}
                  disabled={isBusy}
                  className="flex-1 px-2 py-1 rounded text-xs bg-slate-600 hover:bg-slate-500 text-white disabled:opacity-50"
                >
                  Off
                </button>
              </div>
            </div>
          );
        })}
      </div>

      {/* Preset Action Buttons */}
      <div className="grid grid-cols-2 gap-2 mb-4">
        <button
          onClick={() => handlePresetAction('spray_start')}
          disabled={isBusy}
          className="px-3 py-2 bg-green-600 hover:bg-green-500 text-white rounded-md text-sm font-semibold disabled:opacity-50"
        >
          🚿 Start Spray
        </button>
        <button
          onClick={() => handlePresetAction('spray_stop')}
          disabled={isBusy}
          className="px-3 py-2 bg-red-600 hover:bg-red-500 text-white rounded-md text-sm font-semibold disabled:opacity-50"
        >
          🛑 Stop Spray
        </button>
        <button
          onClick={() => handlePresetAction('all_default')}
          disabled={isBusy}
          className="px-3 py-2 bg-blue-600 hover:bg-blue-500 text-white rounded-md text-sm font-semibold disabled:opacity-50"
        >
          🎯 All Default
        </button>
        <button
          onClick={() => handlePresetAction('all_neutral')}
          disabled={isBusy}
          className="px-3 py-2 bg-slate-600 hover:bg-slate-500 text-white rounded-md text-sm font-semibold disabled:opacity-50"
        >
          ⭕ All Neutral
        </button>
      </div>

      {/* Status */}
      {feedback && <div className="text-xs text-green-300 mb-2">{feedback}</div>}
      {error && <div className="text-xs text-red-300">{error}</div>}
    </div>
  );
};

export default MultiServoControl;
