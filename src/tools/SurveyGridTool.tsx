// File: src/tools/SurveyGridTool.tsx
// Mission Planner-style Survey Grid Tool for autonomous area coverage

import React, { useState, useEffect } from "react";
import { Waypoint } from "../types";

type SurveyGridToolProps = {
  onGenerate: (waypoints: Waypoint[]) => void;
  onClose: () => void;
};

/**
 * Generate survey grid mission with serpentine (lawnmower) pattern
 * Matches Mission Planner's Auto WP → Survey (Grid) functionality
 */
function generateSurveyGrid(
  centerLat: number,
  centerLng: number,
  widthM: number,
  heightM: number,
  laneSpacingM: number,
  angleDegree: number,
  altitude: number,
  overlapPercent: number = 0
): Waypoint[] {
  const waypoints: Waypoint[] = [];
  
  // Earth radius for lat/lng calculations
  const R = 6378137; // meters
  
  // Convert angle to radians
  const angleRad = (angleDegree * Math.PI) / 180;
  
  // Calculate effective spacing with overlap
  const effectiveSpacing = laneSpacingM * (1 - overlapPercent / 100);
  
  // Calculate number of lanes
  const numLanes = Math.max(2, Math.ceil(widthM / effectiveSpacing));
  
  // Calculate waypoints per lane
  const waypointsPerLane = 2; // Start and end of each lane
  
  // Calculate half dimensions
  const halfWidth = widthM / 2;
  const halfHeight = heightM / 2;
  
  let wpId = 1;
  
  for (let lane = 0; lane < numLanes; lane++) {
    // Calculate offset from center for this lane
    const laneOffset = -halfWidth + (lane * effectiveSpacing);
    
    // Determine direction (alternate for serpentine pattern)
    const forward = lane % 2 === 0;
    
    // Calculate start and end points for this lane
    const startY = forward ? -halfHeight : halfHeight;
    const endY = forward ? halfHeight : -halfHeight;
    
    // Create waypoints for this lane (start and end)
    for (const y of forward ? [startY, endY] : [startY, endY]) {
      // Rotate and translate relative to grid center
      const rotatedX = laneOffset * Math.cos(angleRad) - y * Math.sin(angleRad);
      const rotatedY = laneOffset * Math.sin(angleRad) + y * Math.cos(angleRad);
      
      // Convert meters to lat/lng offset
      const latOffset = (rotatedY / R) * (180 / Math.PI);
      const lngOffset = (rotatedX / (R * Math.cos((centerLat * Math.PI) / 180))) * (180 / Math.PI);
      
      const lat = centerLat + latOffset;
      const lng = centerLng + lngOffset;
      
      waypoints.push({
        id: wpId,
        lat,
        lng,
        alt: altitude,
        frame: 3, // MAV_FRAME_GLOBAL_RELATIVE_ALT
        command: 'WAYPOINT', // MAV_CMD_NAV_WAYPOINT
        current: wpId === 1 ? 1 : 0,
        autocontinue: 1,
        param1: 0, // Hold time (seconds)
        param2: 2, // Acceptance radius (meters)
        param3: 0, // Pass radius
        param4: 0, // Yaw angle
        action: 'NONE'
      });
      
      wpId++;
    }
  }
  
  return waypoints;
}

const SurveyGridTool: React.FC<SurveyGridToolProps> = ({ onGenerate, onClose }) => {
  // Grid center (can be updated to use map click)
  const [centerLat, setCenterLat] = useState(13.0764);
  const [centerLng, setCenterLng] = useState(80.1559);
  
  // Grid dimensions
  const [widthM, setWidthM] = useState(100);
  const [heightM, setHeightM] = useState(100);
  
  // Lane spacing (distance between parallel flight lines)
  const [laneSpacing, setLaneSpacing] = useState(10);
  
  // Grid rotation angle (0 = North, 90 = East)
  const [angle, setAngle] = useState(0);
  
  // Flight altitude
  const [altitude, setAltitude] = useState(30);
  
  // Overlap percentage for camera/sensor coverage
  const [overlap, setOverlap] = useState(20);
  
  // Calculated stats
  const [numLanes, setNumLanes] = useState(0);
  const [numWaypoints, setNumWaypoints] = useState(0);
  const [totalDistance, setTotalDistance] = useState(0);
  const [estimatedTime, setEstimatedTime] = useState(0);
  
  // Recalculate stats when parameters change
  useEffect(() => {
    const effectiveSpacing = laneSpacing * (1 - overlap / 100);
    const lanes = Math.max(2, Math.ceil(widthM / effectiveSpacing));
    const waypoints = lanes * 2; // 2 waypoints per lane
    const distance = (lanes * heightM) + ((lanes - 1) * laneSpacing); // Total path length
    const time = distance / 5; // Assume 5 m/s ground speed
    
    setNumLanes(lanes);
    setNumWaypoints(waypoints);
    setTotalDistance(distance);
    setEstimatedTime(time);
  }, [widthM, heightM, laneSpacing, overlap]);
  
  const handleGenerate = () => {
    if (widthM <= 0 || heightM <= 0) {
      alert("Width and height must be positive values");
      return;
    }
    
    if (laneSpacing <= 0) {
      alert("Lane spacing must be positive");
      return;
    }
    
    const waypoints = generateSurveyGrid(
      centerLat,
      centerLng,
      widthM,
      heightM,
      laneSpacing,
      angle,
      altitude,
      overlap
    );
    
    onGenerate(waypoints);
    onClose();
  };
  
  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 backdrop-blur-sm flex items-center justify-center z-[2000]">
      <div className="bg-slate-800 rounded-xl shadow-lg p-6 w-[520px] text-white max-h-[90vh] overflow-y-auto">
        <h2 className="text-lg font-bold mb-4 text-center flex items-center justify-center gap-2">
          <span className="text-2xl">📐</span>
          Survey Grid Generator
          <span className="text-xs bg-green-600 px-2 py-1 rounded ml-2">Mission Planner Style</span>
        </h2>
        
        {/* Grid Center */}
        <div className="mb-4 p-3 bg-slate-700 rounded-lg">
          <h3 className="text-sm font-semibold mb-2 text-gray-300">Grid Center</h3>
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
        
        {/* Grid Dimensions */}
        <div className="mb-4 p-3 bg-slate-700 rounded-lg">
          <h3 className="text-sm font-semibold mb-2 text-gray-300">Grid Dimensions</h3>
          <div className="grid grid-cols-2 gap-3">
            <div>
              <label className="block text-xs mb-1 text-gray-400">Width (m)</label>
              <input
                type="number"
                value={widthM}
                onChange={(e) => setWidthM(parseFloat(e.target.value))}
                className="w-full p-2 rounded bg-slate-600 text-white text-sm"
              />
            </div>
            <div>
              <label className="block text-xs mb-1 text-gray-400">Height (m)</label>
              <input
                type="number"
                value={heightM}
                onChange={(e) => setHeightM(parseFloat(e.target.value))}
                className="w-full p-2 rounded bg-slate-600 text-white text-sm"
              />
            </div>
          </div>
        </div>
        
        {/* Flight Parameters */}
        <div className="mb-4 p-3 bg-slate-700 rounded-lg">
          <h3 className="text-sm font-semibold mb-2 text-gray-300">Flight Parameters</h3>
          <div className="grid grid-cols-2 gap-3 mb-3">
            <div>
              <label className="block text-xs mb-1 text-gray-400">Lane Spacing (m)</label>
              <input
                type="number"
                value={laneSpacing}
                onChange={(e) => setLaneSpacing(parseFloat(e.target.value))}
                className="w-full p-2 rounded bg-slate-600 text-white text-sm"
              />
            </div>
            <div>
              <label className="block text-xs mb-1 text-gray-400">Overlap (%)</label>
              <input
                type="number"
                min="0"
                max="80"
                value={overlap}
                onChange={(e) => setOverlap(parseFloat(e.target.value))}
                className="w-full p-2 rounded bg-slate-600 text-white text-sm"
              />
            </div>
          </div>
          <div className="grid grid-cols-2 gap-3">
            <div>
              <label className="block text-xs mb-1 text-gray-400">Grid Angle (°)</label>
              <input
                type="number"
                min="0"
                max="359"
                value={angle}
                onChange={(e) => setAngle(parseFloat(e.target.value))}
                className="w-full p-2 rounded bg-slate-600 text-white text-sm"
              />
              <p className="text-xs text-gray-500 mt-1">0°=N, 90°=E</p>
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
        
        {/* Mission Statistics */}
        <div className="mb-4 p-3 bg-blue-900 bg-opacity-30 rounded-lg border border-blue-500">
          <h3 className="text-sm font-semibold mb-2 text-blue-300">Mission Statistics</h3>
          <div className="grid grid-cols-2 gap-2 text-xs">
            <div className="flex justify-between">
              <span className="text-gray-400">Lanes:</span>
              <span className="font-semibold text-white">{numLanes}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Waypoints:</span>
              <span className="font-semibold text-white">{numWaypoints}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Distance:</span>
              <span className="font-semibold text-white">{totalDistance.toFixed(0)} m</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Est. Time:</span>
              <span className="font-semibold text-white">{Math.ceil(estimatedTime / 60)} min</span>
            </div>
            <div className="flex justify-between col-span-2">
              <span className="text-gray-400">Area Coverage:</span>
              <span className="font-semibold text-white">{((widthM * heightM) / 10000).toFixed(2)} ha</span>
            </div>
          </div>
        </div>
        
        {/* Info Box */}
        <div className="mb-4 p-2 bg-yellow-900 bg-opacity-20 rounded border border-yellow-600 text-xs text-yellow-200">
          <strong>ℹ️ Pattern:</strong> Serpentine (lawnmower) pattern for efficient coverage. 
          Overlap ensures no gaps in sensor/camera coverage.
        </div>
        
        {/* Action Buttons */}
        <div className="flex justify-end gap-3">
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
            ✓ Generate Grid Mission
          </button>
        </div>
      </div>
    </div>
  );
};

export default SurveyGridTool;
