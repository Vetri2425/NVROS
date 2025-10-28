import React from 'react';
import { useRover } from '../context/RoverContext';

const formatTimestamp = (ts: number | null): string => {
  if (!ts) {
    return '—';
  }
  const date = new Date(ts);
  return Number.isNaN(date.getTime()) ? '—' : date.toLocaleTimeString();
};

const safeFixed = (value: number, digits: number): string => {
  if (!Number.isFinite(value)) {
    return '—';
  }
  return value.toFixed(digits);
};

const TelemetryPanel: React.FC = () => {
  const { telemetry, connectionState } = useRover();
  const { state, global, battery, mission, rtk, lastMessageTs } = telemetry;

  const armed = state.armed ? 'Yes' : 'No';
  const rtkLabel = (() => {
    switch (rtk.fix_type) {
      case 4:
        return 'RTK Fixed';
      case 3:
        return 'RTK Float';
      case 2:
        return 'DGPS';
      case 1:
        return 'GPS Fix';
      default:
        return 'No Fix';
    }
  })();

  return (
    <div className="bg-[#111827] rounded-lg overflow-hidden flex flex-col">
      <header className="bg-indigo-700 px-4 py-3 flex items-center justify-between">
        <span className="font-semibold text-white tracking-wide">Telemetry</span>
        <span className="text-xs text-indigo-100 uppercase">{connectionState}</span>
      </header>

      <div className="p-4 flex flex-col gap-4 text-sm text-slate-200">
        <section className="grid grid-cols-2 gap-3">
          <div>
            <p className="text-xs uppercase text-slate-400">Mode</p>
            <p className="font-semibold">{state.mode || 'UNKNOWN'}</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Armed</p>
            <p className={`font-semibold ${state.armed ? 'text-green-400' : 'text-red-400'}`}>
              {armed}
            </p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Heartbeat</p>
            <p className="font-semibold font-mono">{formatTimestamp(state.heartbeat_ts)}</p>
          </div>
        </section>

        <section className="grid grid-cols-2 gap-3">
          <div>
            <p className="text-xs uppercase text-slate-400">Latitude</p>
            <p className="font-mono">{safeFixed(global.lat, 7)}</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Longitude</p>
            <p className="font-mono">{safeFixed(global.lon, 7)}</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Altitude (rel)</p>
            <p className="font-semibold">{safeFixed(global.alt_rel, 1)} m</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Ground Speed</p>
            <p className="font-semibold">{safeFixed(global.vel, 2)} m/s</p>
          </div>
        </section>

        <section className="grid grid-cols-3 gap-3">
          <div>
            <p className="text-xs uppercase text-slate-400">Battery</p>
            <p className="font-semibold">{safeFixed(battery.percentage, 1)}%</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Voltage</p>
            <p className="font-semibold">{safeFixed(battery.voltage, 1)} V</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Current</p>
            <p className="font-semibold">{safeFixed(battery.current, 1)} A</p>
          </div>
        </section>

        <section className="grid grid-cols-2 gap-3">
          <div>
            <p className="text-xs uppercase text-slate-400">Mission</p>
            <p className="font-semibold">{mission.status}</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Progress</p>
            <p className="font-semibold">
              {safeFixed(mission.progress_pct, 1)}% ({mission.current_wp}/{mission.total_wp})
            </p>
          </div>
        </section>

        <section className="grid grid-cols-2 gap-3">
          <div>
            <p className="text-xs uppercase text-slate-400">RTK</p>
            <p className="font-semibold">{rtkLabel}</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Satellites</p>
            <p className="font-semibold">{global.satellites_visible}</p>
          </div>
        </section>

        <footer className="text-xs text-slate-500 flex justify-between">
          <span>Last update: {formatTimestamp(lastMessageTs)}</span>
          <span>Telemetry age: {lastMessageTs ? `${Date.now() - lastMessageTs} ms` : '—'}</span>
        </footer>
      </div>
    </div>
  );
};

export default TelemetryPanel;
