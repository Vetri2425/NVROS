// src/components/ServoControl/ReportPanel.tsx
import React from "react";

export default function ReportPanel({ status }: any) {
  const entries = Object.entries((status ?? {}) as Record<string, any>);
  const active = entries.find(([, s]) => s?.running) as [string, any] | undefined;
  return (
    <div className="bg-slate-800 border border-slate-700 p-4 rounded-lg mt-4">
      <h3 className="font-semibold mb-2 text-slate-200">Report Summary</h3>
      {active ? (
        <>
          <p className="text-slate-300"><b className="text-slate-200">Active Mode:</b> {String(active[0])}</p>
          {active[1]?.pid != null && <p className="text-slate-300"><b className="text-slate-200">PID:</b> {String(active[1]?.pid)}</p>}
          {active[1] && (active[1] as any).start != null && (
            <p className="text-slate-300"><b className="text-slate-200">Start Time:</b> {new Date(Number((active[1] as any).start) * 1000).toLocaleTimeString()}</p>
          )}
        </>
      ) : (
        <p className="text-slate-400">No mode currently running.</p>
      )}
    </div>
  );
}
