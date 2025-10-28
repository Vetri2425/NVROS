import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { useRover } from '../context/RoverContext';

const AVAILABLE_MODES = ['AUTO', 'GUIDED', 'HOLD', 'MANUAL', 'RTL', 'STEERING'];

const ModeSelector: React.FC = () => {
  const {
    telemetry: { state },
    services,
  } = useRover();
  const [isLoading, setIsLoading] = useState(false);
  const [message, setMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);

  const normalizedMode = useMemo(
    () => state.mode?.toUpperCase?.() ?? 'UNKNOWN',
    [state.mode],
  );

  const options = useMemo(() => {
    if (normalizedMode !== 'UNKNOWN' && !AVAILABLE_MODES.includes(normalizedMode)) {
      return [...AVAILABLE_MODES, normalizedMode];
    }
    return AVAILABLE_MODES;
  }, [normalizedMode]);

  const [selectedMode, setSelectedMode] = useState(normalizedMode);

  useEffect(() => {
    setSelectedMode(normalizedMode);
  }, [normalizedMode]);

  const runAction = useCallback(
    async (
      action: () => Promise<{ success: boolean; message?: string }>,
      buildDefaultMessage: (success: boolean) => string,
    ) => {
      setMessage(null);
      setIsLoading(true);
      try {
        const resp = await action();
        if (resp.success) {
          setMessage({
            type: 'success',
            text: resp.message ?? buildDefaultMessage(true),
          });
        } else {
          setMessage({
            type: 'error',
            text: resp.message ?? buildDefaultMessage(false),
          });
        }
      } catch (err) {
        setMessage({
          type: 'error',
          text: err instanceof Error ? err.message : 'Command failed',
        });
      } finally {
        setIsLoading(false);
      }
    },
    [],
  );

  const requestModeChange = useCallback(async () => {
    if (!selectedMode) {
      return;
    }
    await runAction(
      () => services.setMode(selectedMode),
      (success) => (success ? `Mode change requested: ${selectedMode}` : 'Failed to set mode'),
    );
  }, [runAction, selectedMode, services]);

  const toggleArmState = useCallback(async () => {
    const currentlyArmed = Boolean(state.armed);
    await runAction(
      () => (currentlyArmed ? services.disarmVehicle() : services.armVehicle()),
      (success) => {
        if (!success) {
          return currentlyArmed ? 'Failed to disarm vehicle' : 'Failed to arm vehicle';
        }
        return currentlyArmed ? 'Disarm request sent' : 'Arm request sent';
      },
    );
  }, [runAction, services, state.armed]);

  return (
    <div className="bg-[#111827] rounded-lg p-4 flex flex-col gap-3">
      <header className="flex items-center justify-between">
        <h3 className="text-white font-semibold uppercase tracking-wide text-sm">Mode</h3>
        <span className="text-xs text-slate-400 font-mono">{normalizedMode}</span>
      </header>

      <div className="flex items-center gap-3">
        <select
          value={selectedMode}
          onChange={(event) => {
            setMessage(null);
            setSelectedMode(event.target.value);
          }}
          disabled={isLoading}
          className="bg-[#1F2937] border border-slate-600 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-500"
        >
          {options.map((mode) => (
            <option key={mode} value={mode}>
              {mode}
            </option>
          ))}
        </select>
        <button
          onClick={requestModeChange}
          disabled={isLoading || !selectedMode || selectedMode === normalizedMode}
          className="px-3 py-2 rounded-md text-sm font-semibold bg-blue-600 hover:bg-blue-500 text-white disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
        >
          Set Mode
        </button>
      </div>

      <div className="flex items-center gap-2">
        <span
          className={`inline-flex items-center gap-1 text-xs px-2 py-1 rounded ${
            state.armed ? 'bg-green-900 text-green-200' : 'bg-slate-700 text-slate-300'
          }`}
        >
          {state.armed ? 'Armed' : 'Disarmed'}
        </span>
        <button
          onClick={toggleArmState}
          disabled={isLoading}
          className={`px-3 py-2 rounded-md text-sm font-semibold transition-colors ${
            state.armed
              ? 'bg-red-600 hover:bg-red-500 text-white'
              : 'bg-green-600 hover:bg-green-500 text-white'
          } disabled:opacity-50 disabled:cursor-not-allowed`}
        >
          {state.armed ? 'Disarm' : 'Arm'}
        </button>
      </div>

      {message && (
        <p
          className={`text-xs ${
            message.type === 'success' ? 'text-green-300' : 'text-red-300'
          }`}
        >
          {message.text}
        </p>
      )}
    </div>
  );
};

export default ModeSelector;
