// src/components/ServoControl/ServoControlTab.tsx
import React, { useEffect, useState } from "react";
import ModeSelector from "./ModeSelector";
import ConfigEditor from "./ConfigEditor";
import StatusPanel from "./StatusPanel";
import LogViewer from "./LogViewer";
import ReportPanel from "./ReportPanel";
import { BACKEND_URL } from "../../config";

export default function ServoControlTab() {
  const [status, setStatus] = useState<any>({});
  const [selectedMode, setSelectedMode] = useState<string>("wpmark");
  const [logText, setLogText] = useState<string>("");

  // Use the same dynamic backend URL as the rest of the app
  const JETSON_API = `${BACKEND_URL}/servo`;

  // Helper to fetch status
  const refreshStatus = async () => {
    try {
      const res = await fetch(`${JETSON_API}/status`);
      if (!res.ok) return;
      const data = await res.json();
      setStatus(data);
    } catch (_) {
      /* ignore */
    }
  };

  // Fetch status every 500ms (2 Hz)
  useEffect(() => {
    refreshStatus();
    const interval = setInterval(refreshStatus, 500);
    return () => clearInterval(interval);
  }, [JETSON_API]);

  return (
    <div className="p-4 grid grid-cols-2 gap-4 w-full text-white">
      <div className="flex flex-col gap-4">
        <h2 className="text-xl font-bold mb-2 text-orange-400">Servo Control Center</h2>
        
        <ModeSelector
          selectedMode={selectedMode}
          setSelectedMode={setSelectedMode}
          JETSON_API={JETSON_API}
          status={status}
          onRefresh={refreshStatus}
        />
        <ConfigEditor
          selectedMode={selectedMode}
          JETSON_API={JETSON_API}
          status={status}
          onRefresh={refreshStatus}
        />
      </div>

      <div className="flex flex-col gap-4">
        <StatusPanel status={status} />
        <LogViewer logText={logText} setLogText={setLogText} JETSON_API={JETSON_API} status={status} />
        <ReportPanel status={status} />
      </div>
    </div>
  );
}
