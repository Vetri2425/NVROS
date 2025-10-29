
import React from 'react';
import { Waypoint, RoverData } from '../../types';
import MapView from '../MapView';
import WaypointStatusList from './WaypointStatusList';
import LiveControls from './LiveControls';
import LiveStatusbar from './LiveStatusbar';

type LiveReportViewProps = {
  missionWaypoints: Waypoint[];
  liveRoverData: RoverData;
  missionName: string | null;
  isConnected: boolean;
};

const LiveReportView: React.FC<LiveReportViewProps> = ({
  missionWaypoints,
  liveRoverData,
  missionName,
  isConnected,
}) => {
  const activeWaypointId =
    liveRoverData.activeWaypointIndex !== null && liveRoverData.activeWaypointIndex !== undefined
      ? liveRoverData.activeWaypointIndex + 1
      : null;

  const currentWaypointSeq = liveRoverData.mission_progress?.current || liveRoverData.activeWaypointIndex || 0;

  return (
    <div className="flex-1 flex flex-col p-4 gap-4 overflow-hidden">
      {/* Top section with 3 panels */}
      <div className="flex-1 flex gap-4 overflow-hidden">
        {/* Left Panel: Waypoint List */}
        <aside className="w-[320px] flex-shrink-0 bg-[#111827] rounded-lg overflow-hidden">
          <WaypointStatusList
            waypoints={missionWaypoints}
            activeWaypointId={activeWaypointId}
            completedWaypointIds={liveRoverData.completedWaypointIds}
          />
        </aside>

        {/* Center Panel: Map View */}
        <main className="flex-1 flex flex-col">
          <MapView
            missionWaypoints={missionWaypoints}
            onMapClick={() => {}}
            roverPosition={liveRoverData.position}
            activeWaypointIndex={liveRoverData.activeWaypointIndex}
            heading={liveRoverData.heading}
            viewMode="live"
            isFullScreen={false} // Full screen is handled by the browser
            onNewMissionDrawn={() => {}}
            isConnectedToRover={false} // In live view, we just display data
            onUpdateWaypointPosition={() => {}}
          />
        </main>

        {/* Right Panel: Controls */}
        <aside className="w-[240px] flex-shrink-0">
          <LiveControls 
            isConnected={isConnected}
            currentWaypoint={currentWaypointSeq}
          />
        </aside>
      </div>

      {/* Bottom Panel: Status Bar */}
      <footer className="h-[180px] flex-shrink-0">
        <LiveStatusbar 
            missionName={missionName}
            waypoints={missionWaypoints}
            liveRoverData={liveRoverData}
        />
      </footer>
    </div>
  );
};

export default LiveReportView;
