// Basic ReportPanel.tsx
import React from "react";

export default function ReportPanel({ status, selectedMode }: any) {
  const entries = Object.entries((status ?? {}) as Record<string, any>);
  const active = entries.find(([, s]) => s?.running) as [string, any] | undefined;

  return (
    <div className="bg-slate-800 border border-slate-700 p-4 rounded-lg shadow">
      <h3 className="font-semibold mb-2 text-slate-200">Live Report</h3>
      {!active ? (
        <p className="text-slate-400">No mode currently running.</p>
      ) : (
        <div className="text-slate-200">
          <p><strong>Mode:</strong> {active[0].toUpperCase()}</p>
          <p><strong>PID:</strong> {active[1].pid}</p>
          <p><strong>Started:</strong> {new Date(active[1].start * 1000).toLocaleTimeString()}</p>
          <p><strong>Log:</strong> <span className="text-blue-400 text-xs font-mono">{active[1].log}</span></p>
        </div>
      )}
    </div>
  );
}
