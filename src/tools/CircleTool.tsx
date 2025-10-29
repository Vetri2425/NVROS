// File: src/tools/CircleTool.tsx
// Enhanced Circle Tool with Mission Planner-style controls

import React, { useState } from "react";
import { Waypoint } from "../types";

type CircleToolProps = {
  onGenerate: (waypoints: Waypoint[]) => void;
  onClose: () => void;
};

/**
 * Generate circular mission with enhanced controls
 */
function generateEnhancedCircleMission(
  center: { lat: number; lng: number },
  radiusM: number,
  numPoints: number,
  altitude: number,
  startAngleDeg: number,
  clockwise: boolean
): Waypoint[] {
  const waypoints: Waypoint[] = [];
  const R = 6378137; // Earth radius in meters
  
  // Direction multiplier
  const direction = clockwise ? 1 : -1;
  
  for (let i = 0; i < numPoints; i++) {
    // Calculate angle for this waypoint
    const angleStep = (360 / numPoints) * direction;
    const angleDeg = startAngleDeg + (i * angleStep);
    const angleRad = (angleDeg * Math.PI) / 180;
    
    // Calculate lat/lng offset
    const dLat = (radiusM * Math.cos(angleRad)) / R * (180 / Math.PI);
    const dLng = (radiusM * Math.sin(angleRad)) / (R * Math.cos((center.lat * Math.PI) / 180)) * (180 / Math.PI);
    
    waypoints.push({
      id: i + 1,
      lat: center.lat + dLat,
      lng: center.lng + dLng,
      alt: altitude,
      frame: 3,
      command: 'WAYPOINT',
      current: i === 0 ? 1 : 0,
      autocontinue: 1,
      param1: 0,
      param2: 2, // Acceptance radius
      param3: 0,
      param4: 0,
      action: 'NONE'
    });
  }
  
  // Add closing waypoint to complete circle
  if (waypoints.length > 0) {
    waypoints.push({ ...waypoints[0], id: numPoints + 1, current: 0 });
  }
  
  return waypoints;
}

const CircleTool: React.FC<CircleToolProps> = ({ onGenerate, onClose }) => {
  const [centerLat, setCenterLat] = useState(13.0764);
  const [centerLng, setCenterLng] = useState(80.1559);
  const [radius, setRadius] = useState(30);
  const [points, setPoints] = useState(12);
  const [altitude, setAltitude] = useState(30);
  const [startAngle, setStartAngle] = useState(0);
  const [clockwise, setClockwise] = useState(true);

  const handleGenerate = () => {
    if (points < 3) return alert("At least 3 waypoints required");
    if (radius <= 0) return alert("Radius must be positive");
    
    const waypoints = generateEnhancedCircleMission(
      { lat: centerLat, lng: centerLng },
      radius,
      points,
      altitude,
      startAngle,
      clockwise
    );
    onGenerate(waypoints);
    onClose();
  };

  // Calculate circle stats
  const circumference = 2 * Math.PI * radius;
  const spacing = circumference / points;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 backdrop-blur-sm flex items-center justify-center z-[2000]">
      <div className="bg-slate-800 rounded-xl shadow-lg p-6 w-[460px] text-white">
        <h2 className="text-lg font-bold mb-4 text-center flex items-center justify-center gap-2">
          <span>🌀</span>
          Circle Mission Generator
          <span className="text-xs bg-sky-600 px-2 py-1 rounded ml-2">Enhanced</span>
        </h2>

        <div className="space-y-3">
          {/* Center Point */}
          <div className="p-3 bg-slate-700 rounded-lg">
            <h3 className="text-xs font-semibold mb-2 text-gray-300">Circle Center</h3>
            <div className="grid grid-cols-2 gap-3">
              <div>
                <label className="block text-xs mb-1 text-gray-400">Latitude</label>
                <input
                  type="number"
                  step="0.000001"
                  value={centerLat}
                  onChange={(e) => setCenterLat(parseFloat(e.target.value))}
                  className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                />
              </div>
              <div>
                <label className="block text-xs mb-1 text-gray-400">Longitude</label>
                <input
                  type="number"
                  step="0.000001"
                  value={centerLng}
                  onChange={(e) => setCenterLng(parseFloat(e.target.value))}
                  className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                />
              </div>
            </div>
          </div>

          {/* Circle Parameters */}
          <div className="p-3 bg-slate-700 rounded-lg">
            <h3 className="text-xs font-semibold mb-2 text-gray-300">Circle Parameters</h3>
            <div className="grid grid-cols-2 gap-3 mb-3">
              <div>
                <label className="block text-xs mb-1 text-gray-400">Radius (m)</label>
                <input
                  type="number"
                  value={radius}
                  onChange={(e) => setRadius(parseFloat(e.target.value))}
                  className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                />
              </div>
              <div>
                <label className="block text-xs mb-1 text-gray-400">Waypoints</label>
                <input
                  type="number"
                  min="3"
                  value={points}
                  onChange={(e) => setPoints(parseInt(e.target.value))}
                  className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                />
              </div>
            </div>
            <div className="grid grid-cols-2 gap-3">
              <div>
                <label className="block text-xs mb-1 text-gray-400">Start Angle (°)</label>
                <input
                  type="number"
                  min="0"
                  max="359"
                  value={startAngle}
                  onChange={(e) => setStartAngle(parseFloat(e.target.value))}
                  className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                />
                <p className="text-xs text-gray-500 mt-1">0°=North</p>
              </div>
              <div>
                <label className="block text-xs mb-1 text-gray-400">Altitude (m)</label>
                <input
                  type="number"
                  value={altitude}
                  onChange={(e) => setAltitude(parseFloat(e.target.value))}
                  className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                />
              </div>
            </div>
          </div>

          {/* Direction Control */}
          <div className="p-3 bg-slate-700 rounded-lg">
            <h3 className="text-xs font-semibold mb-2 text-gray-300">Direction</h3>
            <div className="flex gap-2">
              <button
                onClick={() => setClockwise(true)}
                className={`flex-1 py-2 px-3 rounded text-sm font-medium transition ${
                  clockwise
                    ? 'bg-blue-600 text-white'
                    : 'bg-slate-600 text-gray-300 hover:bg-slate-500'
                }`}
              >
                ↻ Clockwise
              </button>
              <button
                onClick={() => setClockwise(false)}
                className={`flex-1 py-2 px-3 rounded text-sm font-medium transition ${
                  !clockwise
                    ? 'bg-blue-600 text-white'
                    : 'bg-slate-600 text-gray-300 hover:bg-slate-500'
                }`}
              >
                ↺ Counter-CW
              </button>
            </div>
          </div>

          {/* Statistics */}
          <div className="p-3 bg-blue-900 bg-opacity-30 rounded-lg border border-blue-500">
            <h3 className="text-xs font-semibold mb-2 text-blue-300">Circle Statistics</h3>
            <div className="grid grid-cols-2 gap-2 text-xs">
              <div className="flex justify-between">
                <span className="text-gray-400">Circumference:</span>
                <span className="font-semibold text-white">{circumference.toFixed(1)} m</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">WP Spacing:</span>
                <span className="font-semibold text-white">{spacing.toFixed(1)} m</span>
              </div>
              <div className="flex justify-between col-span-2">
                <span className="text-gray-400">Total Waypoints:</span>
                <span className="font-semibold text-white">{points + 1} (incl. close)</span>
              </div>
            </div>
          </div>
        </div>

        <div className="flex justify-end gap-3 mt-4">
          <button
            onClick={onClose}
            className="px-4 py-2 rounded-md bg-gray-600 hover:bg-gray-700 transition text-sm"
          >
            Cancel
          </button>
          <button
            onClick={handleGenerate}
            className="px-4 py-2 rounded-md bg-green-600 hover:bg-green-700 transition font-semibold text-sm"
          >
            ✓ Generate Circle
          </button>
        </div>
      </div>
    </div>
  );
};

export default CircleTool;
