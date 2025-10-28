import React from 'react';
import { MissionLog } from '../../types';
import LogManager from '../LogManager';
import { LogsIcon } from '../icons/LogsIcon';

// Define the type for the component's props
type LogsPanelProps = {
  missionLogs: MissionLog[];
};

/**
 * LogsPanel serves as a dedicated container in the sidebar for displaying
 * mission logs via the LogManager component. It is designed to fill
 * the remaining vertical space.
 */
const LogsPanel: React.FC<LogsPanelProps> = ({ missionLogs }) => {
  return (
    // This container uses Flexbox to grow and fill available space.
    // min-h-0 is crucial for flex children to shrink properly in a container.
    <div className="bg-[#111827] rounded-lg flex flex-col flex-1 min-h-0">
      
      {/* Panel Header */}
      <h2 className="bg-indigo-700 text-white text-md font-bold p-3 flex items-center gap-3 flex-shrink-0">
        <LogsIcon className="w-5 h-5" />
        LOGS
      </h2>

      {/* Scrollable Content Area */}
      <div className="p-4 flex-1 min-h-0 overflow-y-auto">
        <LogManager logs={missionLogs} />
      </div>
    </div>
  );
};

export default LogsPanel;
