// File: src/tools/SplineWaypointTool.tsx
// Convert standard waypoints to spline waypoints for smooth curved paths

import React, { useState } from "react";
import { Waypoint } from "../types";

type SplineWaypointToolProps = {
  currentWaypoints: Waypoint[];
  onConvert: (waypoints: Waypoint[]) => void;
  onClose: () => void;
};

const SplineWaypointTool: React.FC<SplineWaypointToolProps> = ({ 
  currentWaypoints, 
  onConvert, 
  onClose 
}) => {
  const [convertAll, setConvertAll] = useState(true);
  const [startIndex, setStartIndex] = useState(0);
  const [endIndex, setEndIndex] = useState(currentWaypoints.length - 1);

  const handleConvert = () => {
    if (currentWaypoints.length === 0) {
      alert("No waypoints to convert");
      return;
    }

    const converted = currentWaypoints.map((wp, idx) => {
      // Determine if this waypoint should be converted
      const shouldConvert = convertAll || (idx >= startIndex && idx <= endIndex);
      
      if (shouldConvert && wp.command === 'WAYPOINT') {
        return {
          ...wp,
          command: 'SPLINE_WP',
        };
      }
      return wp;
    });

    const convertedCount = converted.filter(wp => wp.command === 'SPLINE_WP').length;
    if (convertedCount === 0) {
      alert("No waypoints were converted. Make sure you have WAYPOINT commands selected.");
      return;
    }

    onConvert(converted);
    onClose();
  };

  const totalWaypoints = currentWaypoints.length;
  const estimatedConversion = convertAll 
    ? currentWaypoints.filter(wp => wp.command === 'WAYPOINT').length
    : currentWaypoints.slice(startIndex, endIndex + 1).filter(wp => wp.command === 'WAYPOINT').length;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 backdrop-blur-sm flex items-center justify-center z-[2000]">
      <div className="bg-slate-800 rounded-xl shadow-lg p-6 w-[480px] text-white">
        <h2 className="text-lg font-bold mb-4 text-center flex items-center justify-center gap-2">
          <span>〰️</span>
          Convert to Spline Waypoints
          <span className="text-xs bg-indigo-600 px-2 py-1 rounded ml-2">Smooth Curves</span>
        </h2>

        <div className="space-y-4">
          {/* Info Box */}
          <div className="p-3 bg-blue-900 bg-opacity-30 rounded-lg border border-blue-500 text-sm">
            <strong>ℹ️ Spline Waypoints:</strong> Creates smooth curved paths instead of sharp corners. 
            Better for camera work, sensor coverage, and passenger comfort. The rover will follow a curved 
            spline through the waypoints rather than stopping at each point.
          </div>

          {/* Current Mission Stats */}
          <div className="p-3 bg-slate-700 rounded-lg">
            <h3 className="text-xs font-semibold mb-2 text-gray-300">Current Mission</h3>
            <div className="grid grid-cols-2 gap-2 text-xs">
              <div className="flex justify-between">
                <span className="text-gray-400">Total Waypoints:</span>
                <span className="font-semibold text-white">{totalWaypoints}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Standard WPs:</span>
                <span className="font-semibold text-white">
                  {currentWaypoints.filter(wp => wp.command === 'WAYPOINT').length}
                </span>
              </div>
            </div>
          </div>

          {/* Conversion Options */}
          <div className="p-3 bg-slate-700 rounded-lg">
            <h3 className="text-xs font-semibold mb-3 text-gray-300">Conversion Options</h3>
            
            {/* Convert All Toggle */}
            <div className="flex items-center gap-3 mb-3">
              <input
                type="checkbox"
                id="convertAll"
                checked={convertAll}
                onChange={(e) => setConvertAll(e.target.checked)}
                className="form-checkbox bg-gray-600 border-gray-500 text-indigo-500 focus:ring-indigo-500"
              />
              <label htmlFor="convertAll" className="text-sm text-gray-200">
                Convert all WAYPOINT commands to spline
              </label>
            </div>

            {/* Range Selection */}
            {!convertAll && (
              <div className="space-y-2">
                <div className="grid grid-cols-2 gap-3">
                  <div>
                    <label className="block text-xs mb-1 text-gray-400">Start WP #</label>
                    <input
                      type="number"
                      min="0"
                      max={totalWaypoints - 1}
                      value={startIndex}
                      onChange={(e) => setStartIndex(Math.max(0, parseInt(e.target.value) || 0))}
                      className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                    />
                  </div>
                  <div>
                    <label className="block text-xs mb-1 text-gray-400">End WP #</label>
                    <input
                      type="number"
                      min={startIndex}
                      max={totalWaypoints - 1}
                      value={endIndex}
                      onChange={(e) => setEndIndex(Math.min(totalWaypoints - 1, parseInt(e.target.value) || totalWaypoints - 1))}
                      className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                    />
                  </div>
                </div>
                <p className="text-xs text-gray-400">
                  Convert waypoints {startIndex} through {endIndex} ({endIndex - startIndex + 1} waypoints)
                </p>
              </div>
            )}
          </div>

          {/* Preview */}
          <div className="p-3 bg-green-900 bg-opacity-20 rounded-lg border border-green-500">
            <h3 className="text-xs font-semibold mb-2 text-green-300">Conversion Preview</h3>
            <div className="text-sm text-green-200">
              <strong>{estimatedConversion}</strong> waypoints will be converted to spline type
            </div>
          </div>

          {/* Warning */}
          {totalWaypoints === 0 && (
            <div className="p-3 bg-yellow-900 bg-opacity-20 rounded border border-yellow-600 text-xs text-yellow-200">
              <strong>⚠️ Warning:</strong> No waypoints in current mission. Please load or create a mission first.
            </div>
          )}
        </div>

        {/* Action Buttons */}
        <div className="flex justify-end gap-3 mt-6">
          <button
            onClick={onClose}
            className="px-4 py-2 rounded-md bg-gray-600 hover:bg-gray-700 transition text-sm"
          >
            Cancel
          </button>
          <button
            onClick={handleConvert}
            disabled={totalWaypoints === 0}
            className="px-4 py-2 rounded-md bg-indigo-600 hover:bg-indigo-700 disabled:opacity-50 disabled:cursor-not-allowed transition font-semibold text-sm"
          >
            ✓ Convert to Spline
          </button>
        </div>
      </div>
    </div>
  );
};

export default SplineWaypointTool;
