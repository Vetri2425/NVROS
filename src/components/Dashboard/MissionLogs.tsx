import React from 'react';
import { LogsIcon } from '../icons/LogsIcon';
import { LogEntry } from '../../types';

type MissionLogsProps = {
    logEntries: LogEntry[];
    onDownload: () => void;
    onClear: () => void;
};

const MissionLogs: React.FC<MissionLogsProps> = ({ logEntries, onDownload, onClear }) => {
  const headers = ['#', 'Timestamp', 'Waypoint', 'Latitude', 'Longitude', 'Status', 'Servo', 'Remark'];

  const missionEvents = logEntries;
  const hasLogs = missionEvents.length > 0;

  return (
    <div className="bg-[#111827] p-4 rounded-lg h-full flex flex-col">
      <div className="flex items-center justify-between mb-3 flex-shrink-0">
        <div className="flex items-center gap-3">
          <LogsIcon className="w-5 h-5 text-gray-300"/>
          <h2 className="text-lg font-bold text-white">Mission Logs</h2>
        </div>
        <div className="flex items-center gap-2">
          <button
            onClick={onClear}
            disabled={!hasLogs}
            className={`px-3 py-1.5 rounded-md text-sm font-semibold transition-colors ${
              hasLogs
                ? 'bg-red-600 text-white hover:bg-red-500'
                : 'bg-red-900 text-red-300 cursor-not-allowed'
            }`}
          >
            Clear Logs
          </button>
          <button
            onClick={onDownload}
            disabled={!hasLogs}
            className={`px-3 py-1.5 rounded-md text-sm font-semibold transition-colors ${
              hasLogs
                ? 'bg-gray-700 text-white hover:bg-gray-600'
                : 'bg-gray-600 text-gray-400 cursor-not-allowed'
            }`}
          >
            Download CSV
          </button>
        </div>
      </div>
      <div className="overflow-y-auto flex-1">
        <table className="w-full text-sm text-left text-gray-300">
          <thead className="text-xs text-gray-400 uppercase bg-[#111827] sticky top-0">
            <tr>
              {headers.map(header => (
                <th key={header} scope="col" className="px-4 py-2 whitespace-nowrap">
                  {header}
                </th>
              ))}
            </tr>
          </thead>
          <tbody>
            {missionEvents.length === 0 ? (
                <tr>
                    <td colSpan={headers.length} className="text-center py-8 text-gray-500">
                        No mission logs to display.
                    </td>
                </tr>
            ) : (
                missionEvents.map((entry, index) => {
                    const timestamp = new Date(entry.timestamp).toLocaleString();
                    const waypoint = entry.waypointId ?? '-';
                    const lat = entry.lat != null ? entry.lat.toFixed(6) : '-';
                    const lng = entry.lng != null ? entry.lng.toFixed(6) : '-';
                    const status = entry.status ? entry.status.toUpperCase() : '-';
                    const servo = entry.servoAction ? entry.servoAction.toUpperCase() : '-';
                    const remark = entry.event;
                    return (
                        <tr key={`${entry.timestamp}-${index}`} className="border-b border-gray-700 hover:bg-gray-800 text-xs">
                            <td className="px-4 py-1">{index + 1}</td>
                            <td className="px-4 py-1 whitespace-nowrap font-mono">{timestamp}</td>
                            <td className="px-4 py-1 text-center">{waypoint}</td>
                            <td className="px-4 py-1 font-mono">{lat}</td>
                            <td className="px-4 py-1 font-mono">{lng}</td>
                            <td className="px-4 py-1">{status}</td>
                            <td className="px-4 py-1">{servo}</td>
                            <td className="px-4 py-1 truncate max-w-[260px]" title={remark}>{remark}</td>
                        </tr>
                    )
                })
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default MissionLogs;
