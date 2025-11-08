// src/components/ServoControl/ServoSequenceControl.tsx
import React, { useState } from 'react';
import { useRover } from '../../context/RoverContext';
import { ServoUtils } from '../../utils/ServoUtils';

interface ServoSequenceControlProps {
  className?: string;
}

/**
 * Advanced servo control component with sequence generation and smooth movement
 * Allows creating and executing automated servo sequences
 */
const ServoSequenceControl: React.FC<ServoSequenceControlProps> = ({ className = '' }) => {
  const { services } = useRover();
  const [servoId, setServoId] = useState<number>(10);
  const [isRunning, setIsRunning] = useState(false);
  const [progress, setProgress] = useState<number>(0);
  const [status, setStatus] = useState<string>('');

  /**
   * Execute a servo sequence with delays between steps
   */
  const runServoSequence = async (id: number, sequence: number[], delayMs: number = 200) => {
    setIsRunning(true);
    setProgress(0);
    setStatus(`Running sequence of ${sequence.length} steps...`);

    try {
      for (let i = 0; i < sequence.length; i++) {
        const angle = sequence[i];
        await services.controlServo(id, angle);
        setProgress(((i + 1) / sequence.length) * 100);
        
        if (i < sequence.length - 1) {
          await new Promise(resolve => setTimeout(resolve, delayMs));
        }
      }
      setStatus('✅ Sequence complete');
    } catch (error) {
      setStatus('❌ Sequence failed: ' + (error instanceof Error ? error.message : 'Unknown error'));
    } finally {
      setIsRunning(false);
      setTimeout(() => {
        setProgress(0);
        setStatus('');
      }, 3000);
    }
  };

  // Preset sequences
  const handleSweepTest = () => {
    const sequence = ServoUtils.generateSweep(180, 10);
    runServoSequence(servoId, sequence);
  };

  const handleCalibrationTest = () => {
    const sequence = [0, 45, 90, 135, 180, 90, 0];
    runServoSequence(servoId, sequence, 500);
  };

  const handleQuickTest = () => {
    const sequence = [0, 180, 0];
    runServoSequence(servoId, sequence, 300);
  };

  const handleSmoothTransition = () => {
    const sequence = ServoUtils.generateSequence(0, 180, 20);
    runServoSequence(servoId, sequence, 100);
  };

  return (
    <div className={`bg-[#111827] rounded-lg p-4 ${className}`}>
      <h3 className="text-white font-semibold mb-4">Servo Sequence Control</h3>

      {/* Servo Selection */}
      <div className="mb-4">
        <label className="flex flex-col gap-1">
          <span className="text-xs uppercase text-slate-400">Target Servo</span>
          <input
            type="number"
            min={1}
            max={16}
            value={servoId}
            onChange={(e) => setServoId(Number(e.target.value))}
            disabled={isRunning}
            className="bg-[#1F2937] border border-slate-600 rounded-md px-3 py-2 text-sm text-white focus:outline-none focus:ring-2 focus:ring-indigo-500 disabled:opacity-50"
          />
        </label>
      </div>

      {/* Progress Bar */}
      {isRunning && (
        <div className="mb-4">
          <div className="bg-[#1F2937] rounded-full h-2 overflow-hidden">
            <div 
              className="bg-blue-500 h-full transition-all duration-200"
              style={{ width: `${progress}%` }}
            />
          </div>
          <div className="text-xs text-slate-400 mt-1 text-center">
            {Math.round(progress)}%
          </div>
        </div>
      )}

      {/* Preset Sequences */}
      <div className="grid grid-cols-2 gap-2 mb-4">
        <button
          onClick={handleQuickTest}
          disabled={isRunning}
          className="px-3 py-2 bg-blue-600 hover:bg-blue-500 text-white rounded-md text-sm font-semibold disabled:opacity-50 disabled:cursor-not-allowed"
        >
          ⚡ Quick Test
        </button>
        <button
          onClick={handleSweepTest}
          disabled={isRunning}
          className="px-3 py-2 bg-purple-600 hover:bg-purple-500 text-white rounded-md text-sm font-semibold disabled:opacity-50 disabled:cursor-not-allowed"
        >
          🔄 Full Sweep
        </button>
        <button
          onClick={handleCalibrationTest}
          disabled={isRunning}
          className="px-3 py-2 bg-green-600 hover:bg-green-500 text-white rounded-md text-sm font-semibold disabled:opacity-50 disabled:cursor-not-allowed"
        >
          🎯 Calibration
        </button>
        <button
          onClick={handleSmoothTransition}
          disabled={isRunning}
          className="px-3 py-2 bg-indigo-600 hover:bg-indigo-500 text-white rounded-md text-sm font-semibold disabled:opacity-50 disabled:cursor-not-allowed"
        >
          ➡️ Smooth Move
        </button>
      </div>

      {/* Status Display */}
      {status && (
        <div className={`text-xs p-2 rounded ${
          status.includes('✅') ? 'bg-green-900 text-green-200' :
          status.includes('❌') ? 'bg-red-900 text-red-200' :
          'bg-blue-900 text-blue-200'
        }`}>
          {status}
        </div>
      )}

      {/* Sequence Descriptions */}
      <div className="mt-4 text-xs text-slate-400 space-y-1">
        <div><strong>Quick Test:</strong> 0° → 180° → 0°</div>
        <div><strong>Full Sweep:</strong> Complete 0-180-0 sweep in steps</div>
        <div><strong>Calibration:</strong> Test key positions with delays</div>
        <div><strong>Smooth Move:</strong> Gradual 0° → 180° transition</div>
      </div>
    </div>
  );
};

export default ServoSequenceControl;
