
import React, { useEffect, useRef, useState, useCallback, useMemo } from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { CrosshairIcon } from './icons/CrosshairIcon';
import { ExpandIcon } from './icons/ExpandIcon';
import { FullScreenToggleIcon } from './icons/FullScreenToggleIcon';
import { RoverIcon } from './icons/RoverIcon';
import { NorthArrowIcon } from './icons/NorthArrowIcon';
import { Waypoint, ViewMode } from '../types';
import { RulerIcon } from './icons/RulerIcon';
import { LineIcon } from './icons/LineIcon';
import { RectangleIcon } from './icons/RectangleIcon';
import { CircleIcon } from './icons/CircleIcon';
import { PolygonIcon } from './icons/PolygonIcon';
import { HexagonIcon } from './icons/HexagonIcon';
import { generateCircleWaypoints, generateRegularPolygonWaypoints, calculateDistance, calculateBearing } from '../utils/geo';
import DrawingInstructions from './DrawingInstructions';
import { useFrameTicker } from '../hooks/useFrameTicker';


declare var L: any;

type MapViewProps = {
  missionWaypoints: Waypoint[];
  onMapClick: (lat: number, lng: number) => void;
  roverPosition?: { lat: number; lng: number } | null;
  activeWaypointIndex?: number | null;
  heading?: number | null;
  viewMode: ViewMode;
  isFullScreen: boolean;
  onNewMissionDrawn: (points: { lat: number, lng: number }[]) => void;
  isConnectedToRover: boolean;
  onUpdateWaypointPosition: (waypointId: number, newPosition: { lat: number, lng: number }) => void;
};

type Tool = 'measure' | 'profile' | 'line' | 'rectangle' | 'circle' | 'polygon' | 'hexagon' | null;

const MAP_COMMANDS = new Set<string>([
  'WAYPOINT',
  'SPLINE_WAYPOINT',
  'TAKEOFF',
  'LAND',
  'RETURN_TO_LAUNCH',
  'LOITER_TIME',
  'LOITER_TURNS',
]);

// Top-down rover icon (bird's eye view) with clear front direction
const ROVER_SVG = `
<svg viewBox="0 0 64 64" xmlns="http://www.w3.org/2000/svg" width="200" height="200" style="display:block">
  <!-- Main body -->
  <rect x="18" y="10" width="28" height="44" rx="4" ry="4" fill="#2563eb" stroke="#1e40af" stroke-width="2" />
  
  <!-- Front windshield (top = front) -->
  <path d="M 20 14 L 32 18 L 44 14 L 44 22 L 20 22 Z" fill="#93c5fd" opacity="0.7" />
  
  <!-- Headlights -->
  <circle cx="24" cy="12" r="2" fill="#fef08a" />
  <circle cx="40" cy="12" r="2" fill="#fef08a" />
  
  <!-- Side mirrors -->
  <rect x="14" y="22" width="4" height="8" rx="1" fill="#1e40af" />
  <rect x="46" y="22" width="4" height="8" rx="1" fill="#1e40af" />
  
  <!-- Wheels -->
  <rect x="14" y="18" width="5" height="10" rx="2" fill="#1f2937" stroke="#000" stroke-width="1" />
  <rect x="45" y="18" width="5" height="10" rx="2" fill="#1f2937" stroke="#000" stroke-width="1" />
  <rect x="14" y="36" width="5" height="10" rx="2" fill="#1f2937" stroke="#000" stroke-width="1" />
  <rect x="45" y="36" width="5" height="10" rx="2" fill="#1f2937" stroke="#000" stroke-width="1" />
  
  <!-- Rear window -->
  <rect x="22" y="48" width="20" height="4" rx="1" fill="#93c5fd" opacity="0.5" />
</svg>
`;

const getWaypointIcon = (waypoint: Waypoint, index: number, total: number, activeId: number | null | undefined): any => {
    const isActive = waypoint.id === activeId;
    const isStart = index === 0;
    const isEnd = index === total - 1 && total > 1;

    let fill = '#f97316'; // Default orange
    if (isStart) fill = '#16a34a';
    if (isEnd) fill = '#dc2626';
    if (isActive) fill = '#22c55e';

    const size = isActive ? 32 : 24;

    const svgIcon = `
      <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="${size}" height="${size}" fill="${fill}" class="drop-shadow-lg">
        <path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z"/>
        <text x="12" y="10.5" font-family="sans-serif" font-size="8" font-weight="bold" fill="white" text-anchor="middle" dy=".3em">${waypoint.id}</text>
      </svg>
    `;

    return L.divIcon({
        html: svgIcon,
        className: 'bg-transparent border-0',
        iconSize: [size, size],
        iconAnchor: [size / 2, size]
    });
};

// Helper: Calculate endpoint of a line given start point, bearing (degrees), and distance (in degrees)
const calculateEndPoint = (lat: number, lng: number, bearing: number, distance: number) => {
  const bearingRad = (bearing * Math.PI) / 180;
  const endLat = lat + distance * Math.cos(bearingRad);
  const endLng = lng + distance * Math.sin(bearingRad) / Math.cos((lat * Math.PI) / 180);
  return { lat: endLat, lng: endLng };
};

const MapView: React.FC<MapViewProps> = ({
  missionWaypoints,
  onMapClick,
  roverPosition,
  activeWaypointIndex,
  heading,
  viewMode,
  isFullScreen,
  onNewMissionDrawn,
  isConnectedToRover,
  onUpdateWaypointPosition,
  }) => {
  const pathWaypoints = useMemo(
    () =>
      {
        return missionWaypoints.filter(
          (wp) => MAP_COMMANDS.has(wp.command) &&
            Number.isFinite(wp.lat) &&
            Number.isFinite(wp.lng)
        );
      },
    [missionWaypoints]
  );

  const mapWrapperRef = useRef<HTMLDivElement>(null);
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<any>(null);
  const userLocationMarkerRef = useRef<any>(null);
  const missionLayerRef = useRef<any>(null);
  const missionMarkersRef = useRef<any[]>([]);
  const roverMarkerRef = useRef<any>(null);
  const traveledPathLayerRef = useRef<any>(null);
  const pathSegmentsRef = useRef<Array<{ polyline: any, timestamp: number, points: any[] }>>([]);
  const headingLineRef = useRef<any>(null);
  const northLineRef = useRef<any>(null);  // Static north reference line
  const roverHeadingLineRef = useRef<any>(null);  // Dynamic rover heading line
  const drawingLayerRef = useRef<any>(null);
  const isDrawingRef = useRef(false);
  const lastSmoothedHeadingRef = useRef<number>(0);  // For smooth heading transitions

  // Keep previous and last rover samples for interpolation
  const prevSampleRef = useRef<{ t: number, lat: number, lng: number, heading: number } | null>(null);
  const lastSampleRef = useRef<{ t: number, lat: number, lng: number, heading: number } | null>(null);
  const lastIconUpdateRef = useRef<number>(0);
  const frameNow = useFrameTicker();
  // Track whether we've already auto-fitted to the current mission to avoid repeated zoom jitter
  const hasFittedMissionRef = useRef(false);
  // Stable signature for mission geometry to reset auto-fit when a new mission loads/changes
  const missionKey = useMemo(
    () => pathWaypoints.map((wp) => `${wp.lat.toFixed(6)},${wp.lng.toFixed(6)}`).join('|'),
    [pathWaypoints]
  );

  const [isMapFullScreen, setIsMapFullScreen] = useState(false);
  const [activeTool, setActiveTool] = useState<Tool>(null);
  const [drawnPoints, setDrawnPoints] = useState<any[]>([]);
  const [mousePos, setMousePos] = useState<any>(null);
  const [measurementText, setMeasurementText] = useState<string | null>(null);

  // Debug: Log when roverPosition changes
  useEffect(() => {
    if (roverPosition) {
      console.log('[MapView] Rover position updated:', {
        lat: roverPosition.lat,
        lng: roverPosition.lng,
        timestamp: new Date().toISOString()
      });
    } else {
      console.log('[MapView] No rover position available');
    }
  }, [roverPosition]);

  const invalidateAndFitBounds = useCallback(() => {
    if (!mapRef.current) return;
    setTimeout(() => {
      if (!mapRef.current) return;
      mapRef.current.invalidateSize();
      if (pathWaypoints.length > 0) {
        const missionBounds = L.latLngBounds(pathWaypoints.map((wp) => [wp.lat, wp.lng]));
        if (missionBounds.isValid()) {
          mapRef.current.fitBounds(missionBounds.pad(0.2));
        }
      }
    }, 150);
  }, [pathWaypoints]);

  useEffect(() => {
    if (mapContainerRef.current && !mapRef.current) {
      const map = L.map(mapContainerRef.current, { zoomControl: true, maxZoom: 22 }).setView([13.0827, 80.2707], 13);
      mapRef.current = map;
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
          attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      }).addTo(map);
    }
  }, []);

  useEffect(() => {
    const onFullScreenChange = () => setIsMapFullScreen(document.fullscreenElement === mapWrapperRef.current);
    document.addEventListener('fullscreenchange', onFullScreenChange);
    return () => document.removeEventListener('fullscreenchange', onFullScreenChange);
  }, []);

  useEffect(() => {
    invalidateAndFitBounds();
  }, [viewMode, isFullScreen, isMapFullScreen, invalidateAndFitBounds]);

  // Reset auto-fit state whenever the mission geometry changes
  useEffect(() => {
    hasFittedMissionRef.current = false;
  }, [missionKey]);

  // Manage map event listeners based on active tool
  useEffect(() => {
    if (!mapRef.current) return;
    const map = mapRef.current;
    
    map.off('click').off('mousedown').off('mousemove').off('mouseup').off('dblclick');
    map.getContainer().style.cursor = 'grab';

    const handleSimpleClick = (e: any) => onMapClick(e.latlng.lat, e.latlng.lng);

    const handleMouseDown = (e: any) => {
        isDrawingRef.current = true;
        setDrawnPoints([e.latlng]);
        setMousePos(e.latlng);
    };

    const handleMouseMove = (e: any) => {
        setMousePos(e.latlng);
        if (isDrawingRef.current) {
          if (activeTool === 'rectangle' || activeTool === 'circle' || activeTool === 'hexagon') {
             setDrawnPoints(prev => [prev[0], e.latlng]);
          }
        }
    };
    
    const handleMouseUp = (e: any) => {
        isDrawingRef.current = false;
        if (drawnPoints.length === 0) return;

        const endPoint = e.latlng;
        const startPoint = drawnPoints[0];
        const radius = calculateDistance(startPoint, endPoint);
        let waypoints: {lat: number, lng: number}[] = [];
        
        if (activeTool === 'rectangle') {
            const bounds = L.latLngBounds(startPoint, endPoint);
            waypoints = [
                bounds.getSouthWest(),
                bounds.getNorthWest(),
                bounds.getNorthEast(),
                bounds.getSouthEast(),
            ];
        } else if (activeTool === 'circle') {
            waypoints = generateCircleWaypoints(startPoint, radius);
        } else if (activeTool === 'hexagon') {
            const bearing = calculateBearing(startPoint, endPoint);
            waypoints = generateRegularPolygonWaypoints(startPoint, radius, 6, bearing);
        }

        if (waypoints.length > 0) {
            onNewMissionDrawn(waypoints);
        }
        handleCancelTool();
    };

    const handlePointClick = (e: any) => {
      setDrawnPoints(prev => [...prev, e.latlng]);
    };
    
    const handleFinishWithDoubleClick = () => {
       if (activeTool === 'measure') {
         setActiveTool(null);
       } else {
         handleFinishDrawing();
       }
    };


    switch (activeTool) {
        case 'line':
        case 'polygon':
        case 'measure':
            map.on('click', handlePointClick);
            map.on('dblclick', handleFinishWithDoubleClick);
            map.getContainer().style.cursor = 'crosshair';
            break;
        case 'rectangle':
        case 'circle':
        case 'hexagon':
            map.on('mousedown', handleMouseDown);
            map.on('mousemove', handleMouseMove);
            map.on('mouseup', handleMouseUp);
            map.getContainer().style.cursor = 'crosshair';
            break;
        default:
            map.on('click', handleSimpleClick);
            break;
    }
    
    return () => {
        map.off('click').off('mousedown').off('mousemove').off('mouseup').off('dblclick');
        map.getContainer().style.cursor = 'grab';
    };
}, [activeTool, onMapClick]);

  // Effect to manage drawing/measuring visuals
  useEffect(() => {
    if (!mapRef.current) return;
    if (!drawingLayerRef.current) drawingLayerRef.current = L.layerGroup().addTo(mapRef.current);
    drawingLayerRef.current.clearLayers();

    if (drawnPoints.length > 0) {
        if (['line', 'polygon', 'rectangle', 'circle', 'hexagon'].includes(activeTool || '')) {
            const drawOptions = { color: '#22c55e', weight: 3, dashArray: '5, 5' };
            if (activeTool === 'rectangle' && drawnPoints.length === 2) {
                L.rectangle(L.latLngBounds(drawnPoints[0], drawnPoints[1]), drawOptions).addTo(drawingLayerRef.current);
            } else if ((activeTool === 'circle' || activeTool === 'hexagon') && drawnPoints.length > 0 && mousePos) {
                const radius = calculateDistance(drawnPoints[0], mousePos);
                L.circle(drawnPoints[0], { ...drawOptions, radius }).addTo(drawingLayerRef.current);
            } else {
                 drawnPoints.forEach(p => L.circleMarker(p, { radius: 4, color: '#22c55e', fillOpacity: 1 }).addTo(drawingLayerRef.current));
                 if (mousePos && drawnPoints.length > 0) {
                    const previewPoints = [...drawnPoints, mousePos];
                     L.polyline(previewPoints, drawOptions).addTo(drawingLayerRef.current);
                     if (activeTool === 'polygon') {
                        // Also draw a closing line for polygons
                        L.polyline([drawnPoints[drawnPoints.length - 1], drawnPoints[0]], drawOptions).addTo(drawingLayerRef.current);
                     }
                 }
            }
        } else if (activeTool === 'measure') {
            let totalDistance = 0;
            drawnPoints.forEach((p, i) => {
                L.circleMarker(p, { radius: 4, color: '#3b82f6', fillOpacity: 1 }).addTo(drawingLayerRef.current);
                if (i > 0) {
                    const dist = mapRef.current.distance(p, drawnPoints[i - 1]);
                    totalDistance += dist;
                    L.tooltip({ permanent: true, direction: 'center', className: 'measurement-tooltip' })
                      .setLatLng(L.latLngBounds(p, drawnPoints[i - 1]).getCenter())
                      .setContent(`${(dist / 1000).toFixed(2)} km`)
                      .addTo(drawingLayerRef.current);
                }
            });
            if (drawnPoints.length > 1) {
                L.polyline(drawnPoints, { color: '#3b82f6' }).addTo(drawingLayerRef.current);
                setMeasurementText(`Total: ${(totalDistance / 1000).toFixed(2)} km`);
            } else if (drawnPoints.length === 1 && mousePos) {
                const dist = mapRef.current.distance(drawnPoints[0], mousePos);
                setMeasurementText(`Dist: ${(dist/1000).toFixed(2)} km`);
                L.polyline([drawnPoints[0], mousePos], { color: '#3b82f6', dashArray: '5, 5' }).addTo(drawingLayerRef.current);
            } else {
                setMeasurementText('Click to add points');
            }
        }
    }
  }, [drawnPoints, activeTool, mousePos]);

  useEffect(() => {
    if (!mapRef.current) return;
    if (!missionLayerRef.current) missionLayerRef.current = L.layerGroup().addTo(mapRef.current);
    missionLayerRef.current.clearLayers();
    missionMarkersRef.current = [];
    if (traveledPathLayerRef.current) traveledPathLayerRef.current.remove();
    traveledPathLayerRef.current = null;
    // Clean up all path segments
    pathSegmentsRef.current.forEach(segment => segment.polyline.remove());
    pathSegmentsRef.current = [];
    if (headingLineRef.current) headingLineRef.current.remove();
    headingLineRef.current = null;
    if (northLineRef.current) northLineRef.current.remove();
    northLineRef.current = null;
    if (roverHeadingLineRef.current) roverHeadingLineRef.current.remove();
    roverHeadingLineRef.current = null;

    if (pathWaypoints.length > 0) {
      const latLngs = pathWaypoints.map((wp) => [wp.lat, wp.lng]);
      const isDraggable = viewMode === 'planning';
      
      pathWaypoints.forEach((wp, index) => {
        const icon = getWaypointIcon(wp, index, pathWaypoints.length, activeWaypointIndex);
        const marker = L.marker([wp.lat, wp.lng], { 
          icon,
          draggable: isDraggable,
        })
          .bindTooltip(`<b>Waypoint ${wp.id}</b><br>${wp.command}<br>Lat: ${wp.lat.toFixed(6)}<br>Lng: ${wp.lng.toFixed(6)}<br>Alt: ${wp.alt}m`)
          .addTo(missionLayerRef.current);

        if (isDraggable) {
          marker.on('dragend', (event: any) => {
            const newPos = event.target.getLatLng();
            onUpdateWaypointPosition(wp.id, { lat: newPos.lat, lng: newPos.lng });
          });
        }
        
        missionMarkersRef.current.push(marker);
      });
      L.polyline(latLngs, { color: 'orange' }).addTo(missionLayerRef.current);
      // Only auto-fit once for a given mission to prevent oscillation with other updates
      if (!hasFittedMissionRef.current) {
        invalidateAndFitBounds();
        hasFittedMissionRef.current = true;
      }
    }
  }, [pathWaypoints, viewMode, activeWaypointIndex, invalidateAndFitBounds, onUpdateWaypointPosition]);

  useEffect(() => {
    if (!mapRef.current || missionMarkersRef.current.length === 0) return;
    pathWaypoints.forEach((wp, index) => {
      if (missionMarkersRef.current[index]) {
        missionMarkersRef.current[index].setIcon(
          getWaypointIcon(wp, index, pathWaypoints.length, activeWaypointIndex)
        );
      }
    });
  }, [activeWaypointIndex, pathWaypoints]);

  useEffect(() => {
    if (!mapRef.current) return;
  if (roverPosition) {
    // Smooth heading to prevent oscillation/jitter
    const rawHeading = heading || 0;
    const lastHeading = lastSmoothedHeadingRef.current;
    
    // Apply exponential smoothing (alpha = 0.15 for very smooth transitions)
    let headingDiff = ((rawHeading - lastHeading + 540) % 360 - 180);
    
    // Only update if change is significant (> 2 degrees) to avoid jitter
    if (Math.abs(headingDiff) < 2) {
      headingDiff = 0;
    }
    
    let smoothedHeading = lastHeading + 0.15 * headingDiff;
    smoothedHeading = (smoothedHeading + 360) % 360;
    lastSmoothedHeadingRef.current = smoothedHeading;
    
    // Top-down car icon with smoothed rotation - use transform with will-change for GPU acceleration
    const roverIconHTML = `<div style="width:200px;height:200px;display:flex;align-items:center;justify-content:center;transform:rotate(${smoothedHeading}deg);will-change:transform;transition:transform 0.3s cubic-bezier(0.4, 0.0, 0.2, 1);">${ROVER_SVG}</div>`;
    const roverIcon = L.divIcon({ 
      html: roverIconHTML, 
      className: 'bg-transparent border-0', 
      iconSize: [200, 200], 
      iconAnchor: [100, 100] 
    });
    
        if (!roverMarkerRef.current) {
            roverMarkerRef.current = L.marker([roverPosition.lat, roverPosition.lng], { icon: roverIcon, zIndexOffset: 1000 }).addTo(mapRef.current);
        } else {
            // Only update position, not icon (to prevent size oscillation)
            const currentPos = roverMarkerRef.current.getLatLng();
            if (Math.abs(currentPos.lat - roverPosition.lat) > 0.0000001 || 
                Math.abs(currentPos.lng - roverPosition.lng) > 0.0000001) {
                roverMarkerRef.current.setLatLng([roverPosition.lat, roverPosition.lng]);
            }
            
            // Update icon only if heading changed significantly
            if (Math.abs(headingDiff) > 0.1) {
                roverMarkerRef.current.setIcon(roverIcon);
            }
        }
        
        // Add/update North reference line (green dashed - always points north)
        const northEndPoint = calculateEndPoint(roverPosition.lat, roverPosition.lng, 0, 0.00008);
        const northLinePoints = [[roverPosition.lat, roverPosition.lng], [northEndPoint.lat, northEndPoint.lng]];
        if (!northLineRef.current) {
            northLineRef.current = L.polyline(northLinePoints, { 
                color: '#10b981',
                weight: 3, 
                opacity: 0.8,
                dashArray: '10, 5'
            }).addTo(mapRef.current);
        } else {
            northLineRef.current.setLatLngs(northLinePoints);
        }
        
        // Add/update Rover heading line (red solid - points in rover's direction with smoothing)
        const headingEndPoint = calculateEndPoint(roverPosition.lat, roverPosition.lng, smoothedHeading, 0.00008);
        const headingLinePoints = [[roverPosition.lat, roverPosition.lng], [headingEndPoint.lat, headingEndPoint.lng]];
        if (!roverHeadingLineRef.current) {
            roverHeadingLineRef.current = L.polyline(headingLinePoints, { 
                color: '#ef4444',
                weight: 3, 
                opacity: 0.9
            }).addTo(mapRef.current);
        } else {
            roverHeadingLineRef.current.setLatLngs(headingLinePoints);
        }
        
        // shift samples for interpolation
        const now = performance.now();
        prevSampleRef.current = lastSampleRef.current;
        lastSampleRef.current = { t: now, lat: roverPosition.lat, lng: roverPosition.lng, heading: smoothedHeading };
    } else if (roverMarkerRef.current) {
        roverMarkerRef.current.remove();
        roverMarkerRef.current = null;
        if (northLineRef.current) {
            northLineRef.current.remove();
            northLineRef.current = null;
        }
        if (roverHeadingLineRef.current) {
            roverHeadingLineRef.current.remove();
            roverHeadingLineRef.current = null;
        }
    }
  }, [roverPosition, heading]);

  // Animation frame: interpolate marker between samples for smooth motion
  useEffect(() => {
    if (!mapRef.current || !roverMarkerRef.current) return;
    const a = prevSampleRef.current;
    const b = lastSampleRef.current;
    if (!b) return;
    const now = performance.now();
    let lat = b.lat, lng = b.lng, hdg = b.heading;
    if (a) {
      const dt = Math.max(0.0001, b.t - a.t);
      const t = Math.max(0, Math.min(1, (now - a.t) / dt));
      const lerp = (x: number, y: number, t: number) => x + (y - x) * t;
      lat = lerp(a.lat, b.lat, t);
      lng = lerp(a.lng, b.lng, t);
      // shortest-angle interpolation for heading
      const d = ((((b.heading - a.heading) + 540) % 360) - 180);
      hdg = a.heading + d * t;
    }
    roverMarkerRef.current.setLatLng([lat, lng]);
    // throttle icon updates to ~10 Hz
    if (now - lastIconUpdateRef.current > 100) {
      const roverIconHTML = `<div style="width:32px;height:32px;display:flex;align-items:center;justify-content:center;transform:rotate(${hdg || 0}deg);">${ROVER_SVG}</div>`;
      const roverIcon = L.divIcon({ html: roverIconHTML, className: 'bg-transparent border-0', iconSize: [32, 32], iconAnchor: [16, 16] });
      roverMarkerRef.current.setIcon(roverIcon);
      lastIconUpdateRef.current = now;
    }
  }, [frameNow]);
  
  // Fading travel path - creates new segment every 3 seconds, old segments fade out
  useEffect(() => {
      if (!mapRef.current) return;
      
      if (roverPosition) {
          const now = Date.now();
          
          // Add point to current segment or create new one
          if (!traveledPathLayerRef.current || (pathSegmentsRef.current.length > 0 && 
              now - pathSegmentsRef.current[pathSegmentsRef.current.length - 1].timestamp > 3000)) {
              
              // Create new segment every 3 seconds
              const newPolyline = L.polyline([[roverPosition.lat, roverPosition.lng]], { 
                  color: '#0ea5e9', 
                  weight: 4, 
                  opacity: 1.0 
              }).addTo(mapRef.current);
              
              pathSegmentsRef.current.push({
                  polyline: newPolyline,
                  timestamp: now,
                  points: [[roverPosition.lat, roverPosition.lng]]
              });
              
              traveledPathLayerRef.current = newPolyline;
          } else {
              // Add to current segment
              traveledPathLayerRef.current.addLatLng([roverPosition.lat, roverPosition.lng]);
              if (pathSegmentsRef.current.length > 0) {
                  pathSegmentsRef.current[pathSegmentsRef.current.length - 1].points.push([roverPosition.lat, roverPosition.lng]);
              }
          }
          
          // Fade and remove old segments (older than 3 seconds)
          pathSegmentsRef.current = pathSegmentsRef.current.filter(segment => {
              const age = now - segment.timestamp;
              
              if (age > 3000) {
                  // Fade out over 1 second after 3 seconds
                  const fadeAge = age - 3000;
                  if (fadeAge < 1000) {
                      const opacity = 1.0 - (fadeAge / 1000);
                      segment.polyline.setStyle({ opacity: opacity });
                      return true;
                  } else {
                      // Remove completely after fade
                      segment.polyline.remove();
                      return false;
                  }
              }
              return true;
          });
      }
  }, [roverPosition]);

  // Keep rover in view without constantly re-fitting bounds (prevents zoom/pan oscillation)
  useEffect(() => {
    if (!mapRef.current || !roverPosition) return;
    const map = mapRef.current;
    const currentBounds = map.getBounds();
    const roverLatLng = L.latLng(roverPosition.lat, roverPosition.lng);
    // If rover drifts out of view (with a small margin), gently pan back
    if (!currentBounds.pad(-0.2).contains(roverLatLng)) {
      if (pathWaypoints.length > 0) {
        const missionBounds = L.latLngBounds(pathWaypoints.map((wp) => [wp.lat, wp.lng]));
        if (missionBounds.isValid()) {
          const combined = missionBounds.extend(roverLatLng);
          map.fitBounds(combined.pad(0.2), { animate: true, duration: 0.5, maxZoom: 22 });
        } else {
          map.panTo(roverLatLng, { animate: true, duration: 0.5 });
        }
      } else {
        map.panTo(roverLatLng, { animate: true, duration: 0.5 });
      }
    }
  }, [roverPosition, pathWaypoints]);
  
  useEffect(() => {
      if (!mapRef.current) return;
      const nextWaypoint = pathWaypoints.find((wp) => wp.id === activeWaypointIndex);
      if (roverPosition && nextWaypoint) {
          const linePoints = [[roverPosition.lat, roverPosition.lng], [nextWaypoint.lat, nextWaypoint.lng]];
          if (!headingLineRef.current) headingLineRef.current = L.polyline(linePoints, { color: '#38bdf8', weight: 2, dashArray: '5, 10' }).addTo(mapRef.current);
          else headingLineRef.current.setLatLngs(linePoints);
      } else if (headingLineRef.current) {
          headingLineRef.current.remove();
          headingLineRef.current = null;
      }
  }, [roverPosition, activeWaypointIndex, pathWaypoints]);

  const handleCenterOnUser = () => navigator.geolocation.getCurrentPosition(pos => {
    if (!mapRef.current) return;
    const latLng = { lat: pos.coords.latitude, lng: pos.coords.longitude };
    mapRef.current.panTo(latLng);
    if (userLocationMarkerRef.current) userLocationMarkerRef.current.setLatLng(latLng);
    else userLocationMarkerRef.current = L.marker(latLng).addTo(mapRef.current);
  }, () => alert('Could not get location.'));

  const handleToggleMapFullScreen = () => {
    if (!document.fullscreenElement) mapWrapperRef.current?.requestFullscreen();
    else document.exitFullscreen();
  };
  
  const handleToolToggle = (tool: Tool) => {
    setActiveTool(prev => {
        const newTool = prev === tool ? null : tool;
        setDrawnPoints([]);
        setMeasurementText(null);
        setMousePos(null);
        isDrawingRef.current = false;
        return newTool;
    });
  };

  const handleFinishDrawing = () => {
    if (drawnPoints.length > 1) {
      onNewMissionDrawn(drawnPoints);
    }
    handleCancelTool();
  };

  const handleCancelTool = () => {
    setActiveTool(null);
    setDrawnPoints([]);
    setMeasurementText(null);
    setMousePos(null);
    isDrawingRef.current = false;
  };

  const drawingTools = [
    { name: 'line', icon: LineIcon, title: 'Draw Path' },
    { name: 'rectangle', icon: RectangleIcon, title: 'Draw Rectangle' },
    { name: 'circle', icon: CircleIcon, title: 'Draw Circle' },
    { name: 'polygon', icon: PolygonIcon, title: 'Draw Polygon' },
    { name: 'hexagon', icon: HexagonIcon, title: 'Draw Hexagon' },
  ];
  
  const isToolActive = activeTool && ['line', 'rectangle', 'circle', 'polygon', 'hexagon', 'measure'].includes(activeTool);

  return (
    <div ref={mapWrapperRef} className="relative w-full flex-1 rounded-lg overflow-hidden bg-gray-700 flex flex-col">
       <DrawingInstructions activeTool={activeTool} />
       {/* North/Heading compass (updates with rover heading; placed bottom-left) */}
       <div className="absolute bottom-2 left-2 z-[1600]">
         <div className="flex items-center justify-center w-10 h-10 rounded-full bg-gray-800 bg-opacity-70 text-white border border-gray-600 shadow">
           {(() => {
             let compassHeading = 0;
             if (typeof heading === 'number' && Number.isFinite(heading)) {
               compassHeading = heading;
             } else {
               const a = prevSampleRef.current;
               const b = lastSampleRef.current;
               if (a && b) {
                 compassHeading = calculateBearing({ lat: a.lat, lng: a.lng }, { lat: b.lat, lng: b.lng });
               }
             }
             return <NorthArrowIcon className="w-16 h-16" headingDeg={compassHeading} />;
           })()}
         </div>
       </div>
       
       {isConnectedToRover && roverPosition && typeof roverPosition.lat === 'number' && (
          <div className="absolute bottom-2 left-2 z-[1000] bg-black bg-opacity-60 text-white font-mono text-xs p-2 rounded-md">
              <div>Lat: {roverPosition.lat.toFixed(7)}</div>
              <div>Lng: {roverPosition.lng.toFixed(7)}</div>
              <div className="text-green-400 text-[10px] mt-1">
                Updated: {new Date().toLocaleTimeString()}
              </div>
          </div>
       )}

       {viewMode === 'planning' && (
         <div className="absolute bottom-4 left-1/2 -translate-x-1/2 z-[1000] flex items-end gap-2">
            <div className="flex items-center gap-1 bg-gray-800 bg-opacity-80 backdrop-blur-sm rounded-lg p-1">
                {drawingTools.map(tool => {
                    const Icon = tool.icon;
                    return (
                        <button key={tool.name} onClick={() => handleToolToggle(tool.name as Tool)} className={`p-2 rounded-md ${activeTool === tool.name ? 'bg-green-600 text-white' : 'text-gray-300 hover:bg-gray-700'}`} title={tool.title}>
                            <Icon className="w-5 h-5" />
                        </button>
                    )
                })}
                <div className="border-l border-gray-600 h-6 mx-1"></div>
                <button onClick={() => handleToolToggle('measure')} className={`p-2 rounded-md ${activeTool === 'measure' ? 'bg-blue-600 text-white' : 'text-gray-300 hover:bg-gray-700'}`} title="Measure Distance">
                    <RulerIcon className="w-5 h-5" />
                </button>
            </div>
            
            {(isToolActive && drawnPoints.length > 0) && (
                <div className="flex flex-col gap-1 bg-gray-800 bg-opacity-80 backdrop-blur-sm rounded-lg p-2">
                    {['line', 'polygon'].includes(activeTool || '') && (
                        <button onClick={handleFinishDrawing} className="w-full text-xs bg-green-500 hover:bg-green-600 text-white font-bold py-1 px-4 rounded">Finish</button>
                    )}
                    <button onClick={handleCancelTool} className="w-full text-xs bg-red-500 hover:bg-red-600 text-white font-bold py-1 px-4 rounded">Cancel</button>
                </div>
            )}
            
            {activeTool === 'measure' && (
                <div className="flex flex-col items-center gap-1 bg-gray-800 bg-opacity-80 backdrop-blur-sm rounded-lg p-2">
                    {measurementText && <p className="text-xs text-center text-white font-mono whitespace-nowrap">{measurementText}</p>}
                    <button onClick={handleCancelTool} className="w-full text-xs bg-red-500 hover:bg-red-600 text-white font-bold py-1 px-4 rounded">Clear</button>
                </div>
            )}
        </div>
       )}
      
       <div className="absolute top-2 right-2 z-[1000] flex flex-col gap-2">
         <button onClick={handleToggleMapFullScreen} className="p-2 bg-gray-800 bg-opacity-70 rounded-md text-white hover:bg-opacity-90" title={isMapFullScreen ? "Exit Fullscreen" : "Enter Fullscreen"}><FullScreenToggleIcon isFullScreen={isMapFullScreen} className="w-5 h-5" /></button>
         <button onClick={handleCenterOnUser} className="p-2 bg-gray-800 bg-opacity-70 rounded-md text-white hover:bg-opacity-90" title="Center on me"><CrosshairIcon className="w-5 h-5" /></button>
         {pathWaypoints.length > 0 && (<button onClick={invalidateAndFitBounds} className="p-2 bg-gray-800 bg-opacity-70 rounded-md text-white hover:bg-opacity-90" title="Auto Zoom & Pan to mission"><ExpandIcon className="w-5 h-5" /></button>)}
       </div>
       <div ref={mapContainerRef} className="w-full flex-1" aria-label="Interactive map for setting a waypoint" />
    </div>
  );
};

export default MapView;
