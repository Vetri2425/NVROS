import React, { useState } from 'react';
import { useRover } from '../context/RoverContext';

const ServoPanel: React.FC = () => {
  const {
    telemetry: { servo },
    services,
  } = useRover();
  const [servoId, setServoId] = useState<number>(() => servo.servo_id || 10);
  const [angle, setAngle] = useState<number>(90);
  const [isBusy, setIsBusy] = useState(false);
  const [feedback, setFeedback] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const handleCommand = async (targetAngle: number) => {
    setIsBusy(true);
    setFeedback(null);
    setError(null);
    try {
      const resp = await services.controlServo(servoId, targetAngle);
      if (resp.success) {
        setFeedback(resp.message ?? `Servo ${servoId} → ${targetAngle}`);
      } else {
        setError(resp.message ?? 'Servo command failed');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Servo command failed');
    } finally {
      setIsBusy(false);
    }
  };

  return (
    <div className="bg-[#111827] rounded-lg p-4 flex flex-col gap-4">
      <header className="flex items-center justify-between">
        <h3 className="text-white font-semibold uppercase tracking-wide text-sm">Servo Control</h3>
        <span className={`text-xs font-semibold ${servo.active ? 'text-green-300' : 'text-slate-400'}`}>
          {servo.active ? 'Active' : 'Idle'}
        </span>
      </header>

      <div className="grid grid-cols-2 gap-3 text-sm text-slate-200">
        <label className="flex flex-col gap-1">
          <span className="text-xs uppercase text-slate-400">Servo ID</span>
          <input
            type="number"
            min={1}
            max={16}
            value={servoId}
            onChange={(e) => setServoId(Number(e.target.value))}
            className="bg-[#1F2937] border border-slate-600 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-500"
          />
        </label>
        <label className="flex flex-col gap-1">
          <span className="text-xs uppercase text-slate-400">Angle</span>
          <input
            type="number"
            min={0}
            max={180}
            value={angle}
            onChange={(e) => setAngle(Number(e.target.value))}
            className="bg-[#1F2937] border border-slate-600 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-500"
          />
        </label>
      </div>

      <div className="grid grid-cols-2 gap-2">
        <button
          onClick={() => handleCommand(angle)}
          disabled={isBusy}
          className="px-3 py-2 rounded-md text-sm font-semibold bg-blue-600 hover:bg-blue-500 text-white disabled:opacity-50 disabled:cursor-not-allowed"
        >
          Send Angle
        </button>
        <button
          onClick={() => handleCommand(0)}
          disabled={isBusy}
          className="px-3 py-2 rounded-md text-sm font-semibold bg-slate-700 hover:bg-slate-600 text-white disabled:opacity-50 disabled:cursor-not-allowed"
        >
          Reset
        </button>
      </div>

      <footer className="text-xs text-slate-400 flex flex-col gap-1">
        <span>
          Last command: {servo.last_command_ts ? new Date(servo.last_command_ts).toLocaleTimeString() : '—'}
        </span>
        {feedback && <span className="text-green-300">{feedback}</span>}
        {error && <span className="text-red-300">{error}</span>}
      </footer>
    </div>
  );
};

export default ServoPanel;
