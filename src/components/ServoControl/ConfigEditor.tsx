// src/components/ServoControl/ConfigEditor.tsx
import React, { useState } from "react";

type Props = {
  selectedMode: string;
  JETSON_API: string;
  status?: any;
  onRefresh?: () => void;
};

export default function ConfigEditor({ selectedMode, JETSON_API, status, onRefresh }: any) {
  const [inputs, setInputs] = useState<Record<string, string>>({});
  const [saving, setSaving] = useState(false);
  const [restarting, setRestarting] = useState(false);

  const handleChange = (key: string, value: string) => {
    setInputs((prev) => ({ ...prev, [key]: value }));
  };

  const saveConfig = async (): Promise<boolean> => {
    try {
      setSaving(true);
      const res = await fetch(`${JETSON_API}/edit`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(inputs),
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error((data as any).error || 'Failed to update config');
      alert('Config updated!');
      return true;
    } catch (e: any) {
      alert(e?.message || 'Failed to update config');
      return false;
    } finally {
      setSaving(false);
    }
  };

  const saveAndRestart = async () => {
    const ok = await saveConfig();
    if (!ok) return;
    try {
      setRestarting(true);
      const running = !!status && !!status[selectedMode]?.running;
      // Stop if already running
      if (running) {
        await fetch(`${JETSON_API}/stop?mode=${encodeURIComponent(selectedMode)}`).catch(() => undefined);
        // small delay before restart
        await new Promise(r => setTimeout(r, 500));
      }
      const res = await fetch(`${JETSON_API}/run?mode=${encodeURIComponent(selectedMode)}`);
      const data = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error((data as any).error || 'Failed to start');
      alert(data.status || `Restarted ${selectedMode}`);
    } catch (e: any) {
      alert(e?.message || `Failed to restart ${selectedMode}`);
    } finally {
      setRestarting(false);
      onRefresh && onRefresh();
    }
  };

  return (
    <div className="bg-slate-800 border border-slate-700 p-4 rounded-lg mt-4">
      <h3 className="font-semibold mb-2 text-slate-200">Edit Parameters ({selectedMode})</h3>

      {selectedMode === "wpmark" && (
        <>
          <div className="grid grid-cols-3 gap-3 mb-2">
            <div>
              <label className="block text-xs text-slate-400">Servo Number</label>
              <input
                type="number"
                min={1}
                max={99}
                placeholder="10"
                onChange={(e) => handleChange("wp_mark.servo_number", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              />
            </div>
            <div>
              <label className="block text-xs text-slate-400">PWM ON</label>
              <input
                type="number"
                min={100}
                max={2500}
                placeholder="650"
                onChange={(e) => handleChange("wp_mark.pwm_on", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              />
            </div>
            <div>
              <label className="block text-xs text-slate-400">PWM OFF</label>
              <input
                type="number"
                min={100}
                max={2500}
                placeholder="1000"
                onChange={(e) => handleChange("wp_mark.pwm_off", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              />
            </div>
          </div>
          <label className="block text-xs text-slate-400">Delay Before ON:</label>
          <input onChange={(e) => handleChange("wp_mark.delay_before_on", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white mb-2" />
          <label className="block text-xs text-slate-400">Spray Duration:</label>
          <input onChange={(e) => handleChange("wp_mark.spray_duration", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white mb-2" />
          <label className="block text-xs text-slate-400">Delay After OFF:</label>
          <input onChange={(e) => handleChange("wp_mark.delay_after_off", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
        </>
      )}

      {selectedMode === "continuous" && (
        <>
          <div className="grid grid-cols-3 gap-3 mb-2">
            <div>
              <label className="block text-xs text-slate-400">Servo Number</label>
              <input
                type="number"
                min={1}
                max={99}
                placeholder="10"
                onChange={(e) => handleChange("continuous_line.servo_number", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              />
            </div>
            <div>
              <label className="block text-xs text-slate-400">PWM ON</label>
              <input
                type="number"
                min={100}
                max={2500}
                placeholder="650"
                onChange={(e) => handleChange("continuous_line.pwm_on", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              />
            </div>
            <div>
              <label className="block text-xs text-slate-400">PWM OFF</label>
              <input
                type="number"
                min={100}
                max={2500}
                placeholder="1000"
                onChange={(e) => handleChange("continuous_line.pwm_off", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              />
            </div>
          </div>
          <div className="grid grid-cols-2 gap-3 mb-2">
            <div>
              <label className="block text-xs text-slate-400">Start Waypoint</label>
              <input onChange={(e) => handleChange("continuous_line.start_wp", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
            </div>
            <div>
              <label className="block text-xs text-slate-400">End Waypoint</label>
              <input onChange={(e) => handleChange("continuous_line.end_wp", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
            </div>
          </div>
          <div className="grid grid-cols-2 gap-3">
            <div>
              <label className="block text-xs text-slate-400">Delay Before ON (s)</label>
              <input onChange={(e) => handleChange("continuous_line.delay_before_on", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
            </div>
            <div>
              <label className="block text-xs text-slate-400">Delay After OFF (s)</label>
              <input onChange={(e) => handleChange("continuous_line.delay_after_off", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
            </div>
          </div>
        </>
      )}

      {selectedMode === "interval" && (
        <>
          <div className="grid grid-cols-3 gap-3 mb-2">
            <div>
              <label className="block text-xs text-slate-400">Servo Number</label>
              <input
                type="number"
                min={1}
                max={99}
                placeholder="10"
                onChange={(e) => handleChange("interval_spray.servo_number", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              />
            </div>
            <div>
              <label className="block text-xs text-slate-400">PWM ON</label>
              <input
                type="number"
                min={100}
                max={2500}
                placeholder="650"
                onChange={(e) => handleChange("interval_spray.pwm_on", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              />
            </div>
            <div>
              <label className="block text-xs text-slate-400">PWM OFF</label>
              <input
                type="number"
                min={100}
                max={2500}
                placeholder="1000"
                onChange={(e) => handleChange("interval_spray.pwm_off", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              />
            </div>
          </div>

          <div className="grid grid-cols-2 gap-3 mb-2">
            <div>
              <label className="block text-xs text-slate-400">Start WP (optional)</label>
              <input onChange={(e) => handleChange("interval_spray.start_wp", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
            </div>
            <div>
              <label className="block text-xs text-slate-400">End WP (optional)</label>
              <input onChange={(e) => handleChange("interval_spray.end_wp", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
            </div>
          </div>

          <div className="grid grid-cols-3 gap-3 mb-2">
            <div className="col-span-3">
              <label className="block text-xs text-slate-400">Toggle Mode</label>
              <select
                defaultValue="timer"
                onChange={(e) => handleChange("interval_spray.toggle_mode", e.target.value)}
                className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
              >
                <option value="timer">Timer</option>
                <option value="distance">Distance</option>
              </select>
            </div>
            <div className="col-span-3">
              <div className="grid grid-cols-2 gap-3">
                <div>
                  <label className="block text-xs text-slate-400">ON Time (s)</label>
                  <input onChange={(e) => handleChange("interval_spray.on_time_s", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
                </div>
                <div>
                  <label className="block text-xs text-slate-400">OFF Time (s)</label>
                  <input onChange={(e) => handleChange("interval_spray.off_time_s", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
                </div>
              </div>
            </div>
            <div className="col-span-3">
              <label className="block text-xs text-slate-400">Distance Interval (m)</label>
              <input onChange={(e) => handleChange("interval_spray.distance_interval_m", e.target.value)} className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white" />
            </div>
          </div>
        </>
      )}

      <div className="flex gap-2 mt-3">
        <button
          onClick={saveConfig}
          disabled={saving || restarting}
          className={`px-3 py-2 rounded text-white ${saving ? 'bg-sky-700 opacity-70' : 'bg-sky-600 hover:bg-sky-700'}`}
        >
          {saving ? 'Saving…' : 'Save Config'}
        </button>
        <button
          onClick={saveAndRestart}
          disabled={saving || restarting}
          className={`px-3 py-2 rounded text-white ${restarting ? 'bg-orange-700 opacity-70' : 'bg-orange-600 hover:bg-orange-700'}`}
          title="Save parameters and (re)start the selected mode"
        >
          {restarting ? 'Saving & Restarting…' : 'Save & Restart'}
        </button>
      </div>
    </div>
  );
}
