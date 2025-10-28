import React from 'react';
import { useRover } from '../context/RoverContext';

const FIX_LABELS: Record<number, string> = {
  0: 'No Fix',
  1: 'GPS Fix',
  2: 'DGPS',
  3: 'RTK Float',
  4: 'RTK Fixed',
};

const RTKPanel: React.FC = () => {
  const {
    telemetry: { rtk, global },
  } = useRover();

  const fixLabel = FIX_LABELS[rtk.fix_type] ?? `Fix ${rtk.fix_type}`;
  const fixClass =
    rtk.fix_type >= 4 ? 'text-green-400' : rtk.fix_type >= 1 ? 'text-yellow-300' : 'text-red-400';

  return (
    <div className="bg-[#111827] rounded-lg p-4 flex flex-col gap-3">
      <header className="flex items-center justify-between">
        <h3 className="text-white font-semibold uppercase tracking-wide text-sm">RTK</h3>
        <span className={`text-xs font-semibold ${fixClass}`}>{fixLabel}</span>
      </header>

      <div className="grid grid-cols-2 gap-3 text-sm text-slate-200">
        <div>
          <p className="text-xs uppercase text-slate-400">Base Linked</p>
          <p className={rtk.base_linked ? 'text-green-300' : 'text-red-300'}>
            {rtk.base_linked ? 'Connected' : 'Disconnected'}
          </p>
        </div>
        <div>
          <p className="text-xs uppercase text-slate-400">Baseline Age</p>
          <p>{rtk.baseline_age.toFixed(1)} s</p>
        </div>
        <div>
          <p className="text-xs uppercase text-slate-400">Satellites</p>
          <p>{global.satellites_visible}</p>
        </div>
        <div>
          <p className="text-xs uppercase text-slate-400">Ground Speed</p>
          <p>{global.vel.toFixed(2)} m/s</p>
        </div>
      </div>
    </div>
  );
};

export default RTKPanel;
