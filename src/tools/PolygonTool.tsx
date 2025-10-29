// File: src/tools/PolygonTool.tsx
// Enhanced Polygon Survey Tool with proper serpentine pattern

import React, { useState } from "react";
import { Waypoint } from "../types";

type PolygonToolProps = {
  onGenerate: (waypoints: Waypoint[]) => void;
  onClose: () => void;
};

/**
 * Calculate if point is inside polygon using ray casting algorithm
 */
function pointInPolygon(point: { lat: number; lng: number }, polygon: { lat: number; lng: number }[]): boolean {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].lng, yi = polygon[i].lat;
    const xj = polygon[j].lng, yj = polygon[j].lat;
    const intersect = ((yi > point.lat) !== (yj > point.lat))
      && (point.lng < (xj - xi) * (point.lat - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

/**
 * Generate polygon survey with serpentine pattern
 */
function generatePolygonSurvey(
  polygon: { lat: number; lng: number }[],
  spacingM: number,
  altitude: number,
  angleDeg: number
): Waypoint[] {
  if (polygon.length < 3) throw new Error('Polygon must have at least 3 vertices');
  
  const R = 6378137; // Earth radius
  const angleRad = (angleDeg * Math.PI) / 180;
  
  // Find bounding box
  const minLat = Math.min(...polygon.map(p => p.lat));
  const maxLat = Math.max(...polygon.map(p => p.lat));
  const minLng = Math.min(...polygon.map(p => p.lng));
  const maxLng = Math.max(...polygon.map(p => p.lng));
  
  // Calculate polygon center
  const centerLat = (minLat + maxLat) / 2;
  const centerLng = (minLng + maxLng) / 2;
  
  // Calculate bounding box dimensions in meters
  const latDiffM = (maxLat - minLat) * (R * Math.PI / 180);
  const lngDiffM = (maxLng - minLng) * (R * Math.cos(centerLat * Math.PI / 180) * Math.PI / 180);
  const maxDim = Math.max(latDiffM, lngDiffM);
  
  // Number of survey lines
  const numLines = Math.ceil(maxDim / spacingM) + 2;
  
  const waypoints: Waypoint[] = [];
  let wpId = 1;
  
  // Generate survey lines
  for (let i = 0; i < numLines; i++) {
    const offset = -maxDim / 2 + (i * spacingM);
    const linePoints: { lat: number; lng: number }[] = [];
    
    // Generate points along this survey line
    const numPointsPerLine = 50; // Sample points along line
    for (let j = 0; j <= numPointsPerLine; j++) {
      const along = -maxDim / 2 + (j * maxDim / numPointsPerLine);
      
      // Calculate point position (rotated)
      const rotX = offset * Math.cos(angleRad) - along * Math.sin(angleRad);
      const rotY = offset * Math.sin(angleRad) + along * Math.cos(angleRad);
      
      // Convert to lat/lng
      const lat = centerLat + (rotY / R) * (180 / Math.PI);
      const lng = centerLng + (rotX / (R * Math.cos(centerLat * Math.PI / 180))) * (180 / Math.PI);
      
      // Check if point is inside polygon
      if (pointInPolygon({ lat, lng }, polygon)) {
        linePoints.push({ lat, lng });
      }
    }
    
    // If we have points on this line, add first and last as waypoints
    if (linePoints.length >= 2) {
      // Alternate direction for serpentine pattern
      const points = i % 2 === 0 ? [linePoints[0], linePoints[linePoints.length - 1]] : [linePoints[linePoints.length - 1], linePoints[0]];
      
      points.forEach(pt => {
        waypoints.push({
          id: wpId++,
          lat: pt.lat,
          lng: pt.lng,
          alt: altitude,
          frame: 3,
          command: 'WAYPOINT',
          current: wpId === 1 ? 1 : 0,
          autocontinue: 1,
          param1: 0,
          param2: 2,
          param3: 0,
          param4: 0,
          action: 'NONE'
        });
      });
    }
  }
  
  return waypoints;
}

const PolygonTool: React.FC<PolygonToolProps> = ({ onGenerate, onClose }) => {
  const [polygonText, setPolygonText] = useState(
    "13.0764,80.1559\n13.0764,80.1569\n13.0754,80.1569\n13.0754,80.1559"
  );
  const [spacing, setSpacing] = useState(5);
  const [altitude, setAltitude] = useState(30);
  const [direction, setDirection] = useState(90);

  const handleGenerate = () => {
    const points = polygonText
      .split("\n")
      .map((line) => line.trim())
      .filter(Boolean)
      .map((line) => {
        const [latStr, lngStr] = line.split(/[,\s]+/);
        return { lat: parseFloat(latStr), lng: parseFloat(lngStr) };
      });

    if (points.length < 3) {
      alert("A polygon must have at least 3 vertices.");
      return;
    }

    // Validate coordinates
    const invalid = points.some(p => isNaN(p.lat) || isNaN(p.lng));
    if (invalid) {
      alert("Invalid coordinates detected. Please check your input.");
      return;
    }

    try {
      const waypoints = generatePolygonSurvey(points, spacing, altitude, direction);
      if (waypoints.length === 0) {
        alert("No waypoints generated. Check polygon size and spacing.");
        return;
      }
      onGenerate(waypoints);
      onClose();
    } catch (err) {
      console.error("Polygon generation error:", err);
      alert("Error generating polygon mission. Check coordinates or spacing.");
    }
  };

  // Calculate polygon stats
  const vertexCount = polygonText.split("\n").filter(l => l.trim()).length;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 backdrop-blur-sm flex items-center justify-center z-[2000]">
      <div className="bg-slate-800 rounded-xl shadow-lg p-6 w-[520px] text-white max-h-[90vh] overflow-y-auto">
        <h2 className="text-lg font-bold mb-4 text-center flex items-center justify-center gap-2">
          <span>🗺️</span>
          Polygon Survey Generator
          <span className="text-xs bg-purple-600 px-2 py-1 rounded ml-2">Enhanced</span>
        </h2>

        <div className="space-y-3">
          {/* Polygon Vertices */}
          <div className="p-3 bg-slate-700 rounded-lg">
            <h3 className="text-xs font-semibold mb-2 text-gray-300">Polygon Vertices</h3>
            <label className="block text-xs mb-1 text-gray-400">
              Coordinates (lat, lng per line)
            </label>
            <textarea
              rows={6}
              value={polygonText}
              onChange={(e) => setPolygonText(e.target.value)}
              className="w-full p-2 rounded bg-slate-600 text-white font-mono text-xs"
              placeholder="13.0764, 80.1559&#10;13.0764, 80.1569&#10;13.0754, 80.1569&#10;13.0754, 80.1559"
            />
            <p className="text-xs text-gray-500 mt-2">
              Enter one coordinate pair per line. Polygon will be automatically closed.
            </p>
            <div className="mt-2 text-xs text-blue-300">
              ✓ {vertexCount} vertices entered
            </div>
          </div>

          {/* Survey Parameters */}
          <div className="p-3 bg-slate-700 rounded-lg">
            <h3 className="text-xs font-semibold mb-2 text-gray-300">Survey Parameters</h3>
            <div className="grid grid-cols-3 gap-3">
              <div>
                <label className="block text-xs mb-1 text-gray-400">Lane Spacing (m)</label>
                <input
                  type="number"
                  value={spacing}
                  onChange={(e) => setSpacing(parseFloat(e.target.value))}
                  className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                />
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
              <div>
                <label className="block text-xs mb-1 text-gray-400">Angle (°)</label>
                <input
                  type="number"
                  min="0"
                  max="359"
                  value={direction}
                  onChange={(e) => setDirection(parseFloat(e.target.value))}
                  className="w-full p-2 rounded bg-slate-600 text-white text-sm"
                />
              </div>
            </div>
            <p className="text-xs text-gray-500 mt-2">
              Survey angle: 0°=North, 90°=East. Serpentine pattern for efficiency.
            </p>
          </div>

          {/* Info */}
          <div className="p-2 bg-yellow-900 bg-opacity-20 rounded border border-yellow-600 text-xs text-yellow-200">
            <strong>ℹ️ Pattern:</strong> Generates parallel survey lines covering the polygon area with alternating (serpentine) direction for efficient coverage.
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
            ✓ Generate Survey
          </button>
        </div>
      </div>
    </div>
  );
};

export default PolygonTool;
