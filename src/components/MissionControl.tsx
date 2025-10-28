import React, { useMemo, useState } from 'react';
import { useRover } from '../context/RoverContext';
import { Waypoint } from '../types';

type MissionControlProps = {
  missionWaypoints: Waypoint[];
  onMissionDownloaded?: (waypoints: Waypoint[]) => void;
  onMissionCleared?: () => void;
};

const MissionControl: React.FC<MissionControlProps> = ({
  missionWaypoints,
  onMissionDownloaded,
  onMissionCleared,
}) => {
  const { telemetry, services } = useRover();
  const [isBusy, setIsBusy] = useState(false);
  const [feedback, setFeedback] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const hasMissionBuffer = missionWaypoints.length > 0;

  const missionSummary = useMemo(() => {
    const progress = telemetry.mission.progress_pct ?? 0;
    const status = telemetry.mission.status ?? 'UNKNOWN';
    return {
      progress,
      status,
      text: `${telemetry.mission.current_wp}/${telemetry.mission.total_wp}`,
    };
  }, [telemetry.mission]);

  const runCommand = async (handler: () => Promise<unknown>) => {
    setFeedback(null);
    setError(null);
    setIsBusy(true);
    try {
      const result = await handler();
      if (result && typeof result === 'object' && 'success' in result) {
        const resp = result as { success: boolean; message?: string };
        if (resp.success) {
          setFeedback(resp.message ?? 'Command accepted');
        } else {
          setError(resp.message ?? 'Command rejected');
        }
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Command failed');
    } finally {
      setIsBusy(false);
    }
  };

  return (
    <div className="bg-[#111827] rounded-lg p-4 flex flex-col gap-4">
      <header className="flex items-center justify-between">
        <div>
          <h3 className="text-white font-semibold uppercase tracking-wide text-sm">Mission Control</h3>
          <p className="text-xs text-slate-400">Live status from /rover/telemetry/mission</p>
        </div>
        <span className="text-xs font-semibold text-indigo-300">{missionSummary.status}</span>
      </header>

      <div>
        <div className="flex items-center justify-between text-sm text-slate-300 mb-2">
          <span>Progress</span>
          <span>
            {missionSummary.progress.toFixed(1)}% · {missionSummary.text}
          </span>
        </div>
        <div className="w-full h-2 bg-slate-700 rounded-full overflow-hidden">
          <div
            className="h-full bg-indigo-500 transition-all duration-200"
            style={{ width: `${Math.min(100, Math.max(0, missionSummary.progress))}%` }}
          />
        </div>
      </div>

      <div className="grid grid-cols-1 gap-2">
        <button
          onClick={() => {
            if (!hasMissionBuffer) {
              setError('Load a mission in the planner before uploading.');
              return;
            }
            setFeedback('Mission upload requested');
            runCommand(() => services.uploadMission(missionWaypoints));
          }}
          disabled={!hasMissionBuffer || isBusy}
          className="px-3 py-2 rounded-md text-sm font-semibold bg-blue-600 hover:bg-blue-500 text-white disabled:bg-slate-600 disabled:text-slate-400 disabled:cursor-not-allowed"
        >
          Upload Mission ({missionWaypoints.length} waypoints)
        </button>

        <button
          onClick={() =>
            runCommand(async () => {
              const response = await services.downloadMission();
              if (response.success && Array.isArray((response as any).waypoints)) {
                onMissionDownloaded?.(((response as any).waypoints as Waypoint[]) ?? []);
                setFeedback(`Downloaded ${(response as any).waypoints.length} waypoints`);
              }
              return response;
            })
          }
          disabled={isBusy}
          className="px-3 py-2 rounded-md text-sm font-semibold bg-slate-700 hover:bg-slate-600 text-white disabled:opacity-50 disabled:cursor-not-allowed"
        >
          Download Mission
        </button>

        <button
          onClick={() =>
            runCommand(async () => {
              const response = await services.clearMission();
              if (response.success) {
                onMissionCleared?.();
              }
              return response;
            })
          }
          disabled={isBusy}
          className="px-3 py-2 rounded-md text-sm font-semibold bg-red-600 hover:bg-red-500 text-white disabled:opacity-50 disabled:cursor-not-allowed"
        >
          Clear Mission
        </button>
      </div>

      {feedback && <p className="text-xs text-green-300">{feedback}</p>}
      {error && <p className="text-xs text-red-300">{error}</p>}
    </div>
  );
};

export default MissionControl;
