import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { RoverProvider, useRover } from './context/RoverContext';
import Header from './components/Header';
import LeftSidebar from './components/LeftSidebar';
import MapView from './components/MapView';
import PlanControls from './components/plan/PlanControls';
import QGCWaypointTable from './components/plan/QGCWaypointTable';
import LiveReportView from './components/live/LiveReportView';
import MissionLogs from './components/MissionLogs';
import SetupTab from './components/setup/SetupTab';
import ServoControlTab from './components/ServoControl/ServoControlTab';
import { MissionFileInfo, Waypoint, ViewMode, RoverData } from './types';
import { usePersistentState } from './hooks/usePersistentState';
import { useMissionLogs } from './hooks/useMissionLogs';
import { calculateDistancesForMission } from './utils/geo';
import { toQGCWPL110 } from './utils/missionParser';
import { RoverTelemetry } from './types/ros';
import { ConnectionState } from './hooks/useRoverROS';
// The useTelemetry hook is no longer needed as useRoverROS provides comprehensive telemetry
// import { useTelemetry } from './hooks/useTelemetry';

const mapFixTypeToLabel = (fixType: number): string => {
  switch (fixType) {
    case 4:
      return 'RTK Fixed';
    case 3:
      return 'RTK Float';
    case 2:
      return 'DGPS';
    case 1:
      return 'GPS Fix';
    default:
      return 'No Fix';
  }
};

const toRoverData = (telemetry: RoverTelemetry, connectionState: ConnectionState): RoverData => {
  const isConnected = connectionState === 'connected';
  const position =
    Number.isFinite(telemetry.global.lat) && Number.isFinite(telemetry.global.lon)
      ? { lat: telemetry.global.lat, lng: telemetry.global.lon }
      : null;

  const currentWp = telemetry.mission.current_wp ?? 0;
  const completedWaypointIds =
    currentWp > 1 ? Array.from({ length: currentWp - 1 }, (_, idx) => idx + 1) : [];

  return {
    connected: isConnected,
    mode: telemetry.state.mode ?? 'UNKNOWN',
    status: telemetry.state.armed ? 'armed' : 'disarmed',
    position,
    latitude: position?.lat ?? undefined,
    longitude: position?.lng ?? undefined,
    altitude: telemetry.global.alt_rel,
    relative_altitude: telemetry.global.alt_rel,
    heading: 0,
    groundspeed: telemetry.global.vel,
    rtk_status: mapFixTypeToLabel(telemetry.rtk.fix_type),
    fix_type: telemetry.rtk.fix_type,
    satellites_visible: telemetry.global.satellites_visible,
    hrms: '0.000',
    vrms: '0.000',
    battery: telemetry.battery.percentage,
    voltage: telemetry.battery.voltage,
    current: telemetry.battery.current,
    cpu_load: undefined,
    drop_rate: undefined,
    rc_connected: true,
    signal_strength: isConnected ? 'Good' : 'No Link',
    imu_status: 'UNKNOWN',
    mission_progress: {
      current: telemetry.mission.current_wp,
      total: telemetry.mission.total_wp,
    },
    current_waypoint_id: telemetry.mission.current_wp,
    activeWaypointIndex:
      telemetry.mission.current_wp && telemetry.mission.current_wp > 0
        ? telemetry.mission.current_wp - 1
        : null,
    completedWaypointIds,
    distanceToNext: 0,
    lastUpdate: telemetry.lastMessageTs ?? Date.now(),
    telemetryAgeMs: telemetry.lastMessageTs ? Date.now() - telemetry.lastMessageTs : undefined,
  };
};

const AppContent: React.FC = () => {
  const { telemetry, connectionState, services } = useRover();

  const [viewMode, setViewMode] = usePersistentState<ViewMode>('app:viewMode', 'dashboard');
  const [missionWaypoints, setMissionWaypoints] = usePersistentState<Waypoint[]>(
    'app:missionWaypoints',
    [],
  );
  const [isFullScreen, setIsFullScreen] = useState(false);
  const [currentMissionFileName, setCurrentMissionFileName] = usePersistentState<string | null>(
    'app:missionFileName',
    null,
  );
  const [missionFileInfo, setMissionFileInfo] = usePersistentState<MissionFileInfo | null>(
    'app:missionFileInfo',
    null,
  );
  const [selectedWaypointIds, setSelectedWaypointIds] = usePersistentState<number[]>(
    'app:selectedWaypoints',
    [],
  );

  const {
    missionLogs,
    getActiveLogEntries,
    clearLogs,
  } = useMissionLogs();

  useEffect(() => {
    const onFullscreenChange = () => setIsFullScreen(Boolean(document.fullscreenElement));
    document.addEventListener('fullscreenchange', onFullscreenChange);
    return () => document.removeEventListener('fullscreenchange', onFullscreenChange);
  }, []);

  const isConnectedToRover = connectionState === 'connected';

  const uiRoverData: RoverData = useMemo(
    () => toRoverData(telemetry, connectionState),
    [telemetry, connectionState],
  );

  const handleToggleFullScreen = () => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen();
    } else if (document.exitFullscreen) {
      document.exitFullscreen();
    }
  };

  const handleMissionUpload = useCallback(
    (waypoints: Waypoint[], info: MissionFileInfo) => {
      setMissionWaypoints(calculateDistancesForMission(waypoints));
      setCurrentMissionFileName(info.name);
      setMissionFileInfo(info);
    },
    [setMissionWaypoints, setCurrentMissionFileName, setMissionFileInfo],
  );

  const handleMissionDownloaded = useCallback(
    (waypoints: Waypoint[]) => {
      const withDistance = calculateDistancesForMission(waypoints);
      setMissionWaypoints(withDistance);
      setCurrentMissionFileName(`Mission from Rover - ${new Date().toLocaleTimeString()}`);
      setMissionFileInfo({
        name: 'Mission from Rover',
        size: withDistance.length,
        type: 'downloaded',
        uploadedAt: new Date().toISOString(),
        waypointCount: withDistance.length,
        source: 'downloaded',
      });
    },
    [setMissionWaypoints, setCurrentMissionFileName, setMissionFileInfo],
  );

  const handleClearMission = useCallback(() => {
    setMissionWaypoints([]);
    setCurrentMissionFileName(null);
    setMissionFileInfo(null);
    setSelectedWaypointIds([]);
  }, [setMissionWaypoints, setCurrentMissionFileName, setMissionFileInfo, setSelectedWaypointIds]);

  const handleWriteToRover = useCallback(async (): Promise<boolean> => {
    if (missionWaypoints.length === 0) {
      alert('No mission loaded. Add waypoints before uploading.');
      return false;
    }
    try {
      const response = await services.uploadMission(missionWaypoints);
      if (response.success) {
        alert(`Mission upload requested (${missionWaypoints.length} waypoints).`);
        return true;
      }
      alert(response.message ?? 'Mission upload failed.');
      return false;
    } catch (error) {
      alert(error instanceof Error ? error.message : 'Mission upload failed.');
      return false;
    }
  }, [missionWaypoints, services]);

  const handleReadFromRover = useCallback(async () => {
    try {
      const response = await services.downloadMission();
      const fetched = Array.isArray((response as any).waypoints) ? (response as any).waypoints : [];
      if (response.success && fetched.length > 0) {
        handleMissionDownloaded(fetched);
        alert(`Downloaded ${fetched.length} waypoints from rover.`);
      } else if (response.success) {
        alert('Rover mission is empty.');
      } else {
        alert(response.message ?? 'Failed to download mission from rover.');
      }
    } catch (error) {
      alert(error instanceof Error ? error.message : 'Failed to download mission from rover.');
    }
  }, [services, handleMissionDownloaded]);

  const handleMapClick = () => {
    // placeholder for future map click behaviour
  };

  const handleDeleteWaypoint = useCallback(
    (id: number) => {
      setMissionWaypoints((prev) => calculateDistancesForMission(prev.filter((wp) => wp.id !== id)));
      setSelectedWaypointIds((prev) => prev.filter((selectedId) => selectedId !== id));
    },
    [setMissionWaypoints, setSelectedWaypointIds],
  );

  const handleUpdateWaypoint = useCallback(
    (id: number, newValues: Partial<Omit<Waypoint, 'id'>>) => {
      setMissionWaypoints((prev) => prev.map((wp) => (wp.id === id ? { ...wp, ...newValues } : wp)));
    },
    [setMissionWaypoints],
  );

  const handleUpdateWaypointPosition = useCallback(
    (waypointId: number, newPosition: { lat: number; lng: number }) => {
      setMissionWaypoints((prev) =>
        calculateDistancesForMission(
          prev.map((wp) => (wp.id === waypointId ? { ...wp, ...newPosition } : wp)),
        ),
      );
    },
    [setMissionWaypoints],
  );

  const handleNewMissionDrawn = useCallback(
    (points: { lat: number; lng: number }[]) => {
      const newWaypoints: Waypoint[] = points.map((p, index) => ({
        id: index + 1,
        command: 'WAYPOINT',
        lat: p.lat,
        lng: p.lng,
        alt: 50,
        frame: 3,
        current: index === 0 ? 1 : 0,
        autocontinue: 1,
      }));
      handleMissionUpload(newWaypoints, {
        name: `Drawn Mission - ${new Date().toLocaleTimeString()}`,
        size: newWaypoints.length,
        type: 'drawn',
        uploadedAt: new Date().toISOString(),
        waypointCount: newWaypoints.length,
        source: 'drawn',
      });
    },
    [handleMissionUpload],
  );

  const handleExportMission = useCallback(() => {
    if (missionWaypoints.length === 0) {
      alert('No mission to export.');
      return;
    }
    const blob = new Blob([toQGCWPL110(missionWaypoints)], { type: 'text/plain;charset=utf-8' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = 'mission.waypoints';
    link.click();
    URL.revokeObjectURL(link.href);
  }, [missionWaypoints]);

  const handleWaypointSelectionChange = useCallback(
    (id: number, isSelected: boolean) => {
      setSelectedWaypointIds((prev) =>
        isSelected ? [...prev, id] : prev.filter((selectedId) => selectedId !== id),
      );
    },
    [setSelectedWaypointIds],
  );

  const commonMapProps = {
    missionWaypoints,
    onMapClick: handleMapClick,
    roverPosition: uiRoverData.position,
    activeWaypointIndex: uiRoverData.activeWaypointIndex,
    heading: uiRoverData.heading,
    viewMode,
    isFullScreen,
    onNewMissionDrawn: handleNewMissionDrawn,
    isConnectedToRover,
    onUpdateWaypointPosition: handleUpdateWaypointPosition,
  };

  const renderMainContent = () => {
    switch (viewMode) {
      case 'servo':
        return (
          <main className="flex-1 flex p-4 overflow-hidden">
            <ServoControlTab />
          </main>
        );
      case 'planning':
        return (
          <main className="flex-1 flex p-4 gap-4 overflow-hidden relative">
            <div className="flex-1 flex flex-col gap-4">
              <MapView {...commonMapProps} />
              <div className="flex-[0_0_240px] overflow-hidden">
                <QGCWaypointTable
                  waypoints={missionWaypoints}
                  onDelete={handleDeleteWaypoint}
                  onUpdate={handleUpdateWaypoint}
                  activeWaypointIndex={uiRoverData.activeWaypointIndex}
                  selectedWaypointIds={selectedWaypointIds}
                  onWaypointSelectionChange={handleWaypointSelectionChange}
                />
              </div>
            </div>
            <aside className="w-80 max-w-xs grid grid-rows-2 gap-4">
              <PlanControls
                missionWaypoints={missionWaypoints}
                onUpload={handleMissionUpload}
                onExport={handleExportMission}
                onUploadInitiated={() => {}}
                onWriteToRover={handleWriteToRover}
                onReadFromRover={handleReadFromRover}
                missionFileInfo={missionFileInfo}
                onClearMission={handleClearMission}
                isConnected={isConnectedToRover}
                uploadProgress={0}
              />
            </aside>
          </main>
        );
      case 'live':
        return (
          <LiveReportView
            missionWaypoints={missionWaypoints}
            liveRoverData={uiRoverData}
            missionName={currentMissionFileName}
            isConnected={isConnectedToRover}
          />
        );
      case 'setup':
        return (
          <main className="flex-1 flex p-4 overflow-hidden">
            <SetupTab />
          </main>
        );
      case 'dashboard':
      default:
        return (
          <main className="flex-1 flex p-4 gap-4 overflow-hidden">
            <LeftSidebar
              missionLogs={missionLogs}
            />
            <div className="flex-1 flex flex-col gap-4">
              <MapView {...commonMapProps} />
              <div className="flex-[0_0_240px] overflow-hidden">
                <MissionLogs
                  logEntries={getActiveLogEntries()}
                  onDownload={() => {
                    const entries = getActiveLogEntries();
                    if (!entries.length) {
                      alert('No mission logs available to download yet.');
                      return;
                    }
                    const header = [
                      'Timestamp',
                      'Latitude',
                      'Longitude',
                      'Waypoint',
                      'Status',
                      'Servo Action',
                      'Remark',
                    ];
                    const rows = entries.map((entry) => {
                      const safeEvent = entry.event.replace(/"/g, '""');
                      const lat = entry.lat != null ? entry.lat.toFixed(7) : '';
                      const lng = entry.lng != null ? entry.lng.toFixed(7) : '';
                      const waypoint = entry.waypointId ?? '';
                      const status = entry.status ?? '';
                      const servo = entry.servoAction ?? '';
                      return [
                        entry.timestamp,
                        lat,
                        lng,
                        waypoint,
                        status,
                        servo,
                        `"${safeEvent}"`,
                      ].join(',');
                    });
                    const csvContent = [header.join(','), ...rows].join('\n');
                    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
                    const url = URL.createObjectURL(blob);
                    const link = document.createElement('a');
                    link.href = url;
                    link.download = `mission-log-${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
                    document.body.appendChild(link);
                    link.click();
                    document.body.removeChild(link);
                    URL.revokeObjectURL(url);
                  }}
                  onClear={() => {
                    if (missionLogs.length === 0) {
                      return;
                    }
                    const confirmClear = window.confirm(
                      'This will clear the current mission logs. Continue?',
                    );
                    if (confirmClear) {
                      clearLogs();
                    }
                  }}
                />
              </div>
            </div>
          </main>
        );
    }
  };

  return (
    <div className="text-white min-h-screen flex flex-col font-sans bg-slate-800 transition-all duration-300">
      <Header
        viewMode={viewMode}
        setViewMode={setViewMode}
        isFullScreen={isFullScreen}
        onToggleFullScreen={handleToggleFullScreen}
      />
      {renderMainContent()}
    </div>
  );
};

const App: React.FC = () => (
  <RoverProvider>
    <AppContent />
  </RoverProvider>
);

export default App;
