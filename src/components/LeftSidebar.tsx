import React from 'react';
import TelemetryPanel from './TelemetryPanel';
import ModeSelector from './ModeSelector';
import RTKPanel from './RTKPanel';
import LogsPanel from './LogsPanel';
import { MissionLog } from '../types';

type LeftSidebarProps = {
  missionLogs: MissionLog[];
};

const LeftSidebar: React.FC<LeftSidebarProps> = ({
  missionLogs
}) => {
  return (
    <aside className="w-80 max-w-xs flex flex-col gap-4 min-h-0">
      <TelemetryPanel />
      <ModeSelector />
      <RTKPanel />
      <LogsPanel missionLogs={missionLogs} />
    </aside>
  );
};

export default LeftSidebar;
