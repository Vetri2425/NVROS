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

  // Fetch status every 2s
  useEffect(() => {
    refreshStatus();
    const interval = setInterval(refreshStatus, 2000);
    return () => clearInterval(interval);
  }, [JETSON_API]);

  // Emergency stop function
  const emergencyStop = async () => {
    if (!window.confirm("Are you sure you want to EMERGENCY STOP all servo operations? This will immediately stop all spraying.")) {
      return;
    }
    try {
      const res = await fetch(`${JETSON_API}/emergency_stop`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
      });
      if (res.ok) {
        alert("Emergency stop initiated - all servo operations stopped");
        refreshStatus();
      } else {
        alert("Failed to initiate emergency stop");
      }
    } catch (error) {
      alert("Error initiating emergency stop");
    }
  };

  return (
    <div className="p-4 grid grid-cols-2 gap-4 w-full text-white">
      <div className="flex flex-col gap-4">
        <h2 className="text-xl font-bold mb-2 text-orange-400">Servo Control Center</h2>
        
        {/* Emergency Stop Button */}
        <div className="bg-red-900 border-2 border-red-600 rounded-lg p-4">
          <button
            onClick={emergencyStop}
            className="w-full bg-red-600 hover:bg-red-700 text-white font-bold py-3 px-6 rounded-lg text-lg transition-colors duration-200"
            disabled={!status.active}
          >
            🚨 EMERGENCY STOP
          </button>
          <p className="text-red-200 text-sm mt-2 text-center">
            Immediately stops all servo operations
          </p>
        </div>

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
