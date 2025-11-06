import React from 'react';
import { useRover } from '../context/RoverContext';
import { TelemetryRtk } from '../types/ros';

// GPS fix type mapping according to GPSRAW message standard:
// 0-1: No Fix, 2: 2D Fix, 3: 3D Fix, 4: DGPS, 5: RTK Float, 6: RTK Fixed
const FIX_LABELS: Record<number, string> = {
  0: 'No Fix',
  1: 'No Fix',
  2: '2D Fix',
  3: '3D Fix',
  4: 'DGPS',
  5: 'RTK Float',
  6: 'RTK Fixed',
};

const RTKPanel: React.FC = () => {
  const {
    telemetry: { rtk, global },
  } = useRover();

  // Debug logging for RTK fix type (only when it changes)
  React.useEffect(() => {
    console.log('[RTKPanel] RTK fix_type changed:', {
      fix_type: rtk.fix_type,
      base_linked: rtk.base_linked,
      baseline_distance: rtk.baseline_distance,
      nsats: rtk.nsats,
    });
  }, [rtk.fix_type, rtk.base_linked]);

  const fixLabel = FIX_LABELS[rtk.fix_type] ?? `Fix ${rtk.fix_type}`;
  const fixClass =
    rtk.fix_type >= 5 ? 'text-green-400' : rtk.fix_type >= 1 ? 'text-yellow-300' : 'text-red-400';

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
          <p className="text-xs uppercase text-slate-400">Satellites</p>
          <p>{rtk.nsats ?? global.satellites_visible}</p>
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
