// src/components/ServoControl/StatusPanel.tsx
import React from "react";

export default function StatusPanel({ status, selectedMode, onRefresh }: any) {
  const handleRefresh = () => {
    if (onRefresh) onRefresh();
  };

  return (
    <div className="bg-slate-800 border border-slate-700 p-4 rounded-lg shadow mb-4">
      <div className="flex justify-between items-center mb-2">
        <h3 className="font-semibold text-slate-200">Running Status</h3>
        <button
          onClick={handleRefresh}
          className="px-2 py-1 bg-blue-600 hover:bg-blue-700 text-white text-xs rounded transition-colors"
          title="Refresh status"
        >
          Refresh
        </button>
      </div>
      {Object.keys(status).length === 0 ? (
        <p className="text-slate-400">Loading...</p>
      ) : (
        <div className="space-y-1">
          {Object.entries(status as any).map(([mode, info]: any) => (
            <div key={mode} className={`flex justify-between border-b border-slate-700 py-2 text-slate-200 ${
              mode === selectedMode ? 'bg-slate-700 px-2 rounded' : ''
            }`}>
              <span className={mode === selectedMode ? 'text-orange-400 font-semibold' : ''}>{String(mode).toUpperCase()}</span>
              <span className={info?.running ? "text-green-400" : "text-slate-500"}>
                {info?.running ? "RUNNING ✅" : "Stopped ❌"}
              </span>
            </div>
          ))}
          {selectedMode && !status[selectedMode] && (
            <div className="flex justify-between border-b border-slate-700 py-2 text-slate-200 bg-slate-700 px-2 rounded">
              <span className="text-orange-400 font-semibold">{String(selectedMode).toUpperCase()}</span>
              <span className="text-slate-500">Ready ⏳</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
