
import React from 'react';
import { Waypoint, RoverData } from '../../types';
import { BatteryIcon } from '../icons/BatteryIcon';
import { RtkIcon } from '../icons/RtkIcon';
import { ImuIcon } from '../icons/ImuIcon';

type LiveStatusbarProps = {
  missionName: string | null;
  waypoints: Waypoint[];
  liveRoverData: RoverData;
};

const StatusItem: React.FC<{ label: string; value: string | number; unit?: string; status?: string }> = ({ label, value, unit, status }) => {
    let statusClass = '';
    if (status === 'OK 81%' || status === 'ALIGNED' || status === 'RTK Fixed') statusClass = 'bg-green-600 text-white';
    else if (status) statusClass = 'bg-blue-500 text-white';

    return (
        <div className="flex justify-between items-center text-sm">
            <div className="flex items-center gap-2 text-gray-300">
                {label === 'Battery' && <BatteryIcon level={typeof value === 'number' ? value : undefined} className="w-5 h-5" />}
                {label === 'GPS' && <RtkIcon className="w-5 h-5 text-blue-400" />}
                {label === 'IMU' && <ImuIcon className="w-5 h-5 text-yellow-400" />}
                <span>{label}</span>
            </div>
            <div className="font-mono flex items-center gap-2">
                {value}
                {unit && <span className="text-gray-400">{unit}</span>}
                {status && <span className={`text-xs font-bold px-2 py-0.5 rounded-full ${statusClass}`}>{status}</span>}
            </div>
        </div>
    );
};

const InfoItem: React.FC<{ label: string; children: React.ReactNode }> = ({ label, children }) => (
    <div className="text-center">
        <p className="text-sm text-gray-400">{label}</p>
        <div className="mt-1">{children}</div>
    </div>
);

const LiveStatusbar: React.FC<LiveStatusbarProps> = ({ missionName, waypoints, liveRoverData }) => {
  const { completedWaypointIds = [], activeWaypointIndex, distanceToNext, mission_progress } = liveRoverData;
  
  // Current waypoint is the active one being navigated to
  const currentWaypointSeq = mission_progress?.current || 0;
  const activeWaypointId = currentWaypointSeq > 0 ? currentWaypointSeq : null;
  
  // Last completed waypoint is the one before the current
  const lastCompletedSeq = currentWaypointSeq > 1 ? currentWaypointSeq - 1 : null;
  
  // Find waypoints by sequence number (which maps to wp.id)
  const currentWp = waypoints.find(wp => wp.id === activeWaypointId);
  const lastCompletedWp = waypoints.find(wp => wp.id === lastCompletedSeq);
  
  // Next waypoint is the one after current
  const nextWaypointSeq = currentWaypointSeq > 0 ? currentWaypointSeq + 1 : null;
  const nextWp = waypoints.find(wp => wp.id === nextWaypointSeq);

  // Calculate delta elevation between last completed and the one before it
  let deltaElevation = 0;
  if (lastCompletedWp) {
      const lastCompletedIndex = waypoints.findIndex(wp => wp.id === lastCompletedWp.id);
      if (lastCompletedIndex > 0) {
        deltaElevation = lastCompletedWp.alt - waypoints[lastCompletedIndex - 1].alt;
      }
  }

  return (
    <div className="h-full grid grid-cols-3 gap-4 text-white">
      {/* Panel 1: Status */}
      <div className="bg-[#111827] rounded-lg p-3 flex flex-col">
        <h3 className="text-center font-bold text-gray-400 border-b border-gray-700 pb-2 mb-3">Status</h3>
        <div className="space-y-3">
            <StatusItem 
              label="Battery" 
              value={liveRoverData.voltage ? `${liveRoverData.voltage.toFixed(2)}` : '---'} 
              unit="v" 
              status={liveRoverData.battery > 0 ? `${liveRoverData.battery}%` : undefined} 
            />
            <StatusItem label="GPS" value="" status={liveRoverData.rtk_status} />
            <StatusItem label="HRMS" value={liveRoverData.hrms || '0.000'} unit="m" />
            <StatusItem label="VRMS" value={liveRoverData.vrms || '0.000'} unit="m" />
            <StatusItem label="IMU" value="" status={liveRoverData.imu_status} />
            <StatusItem 
              label="Satellites" 
              value={liveRoverData.satellites_visible || 0} 
              status={liveRoverData.satellites_visible && liveRoverData.satellites_visible >= 10 ? 'Good' : undefined}
            />
        </div>
      </div>

      {/* Panel 2: Mission Grid */}
      <div className="bg-[#111827] rounded-lg p-3 flex flex-col justify-center items-center text-center">
        <h3 className="text-xl font-bold text-gray-300 truncate w-full" title={missionName || 'No Mission'}>{missionName || 'No Mission Loaded'}</h3>
        <p className="text-gray-400">{waypoints.length} points in current mission</p>
        <div className="mt-3 space-y-1">
          <p className="text-sm text-gray-400">Distance to Next Waypoint</p>
          <p className="text-4xl font-bold text-orange-400">
            {distanceToNext > 0 ? distanceToNext.toFixed(1) : '---'}
            <span className="text-2xl text-gray-400 ml-2">m</span>
          </p>
        </div>
      </div>

      {/* Panel 3: Info */}
      <div className="bg-[#111827] rounded-lg p-3 flex items-center justify-around">
        <InfoItem label="Marked Point">
            <p className="text-2xl font-bold bg-gray-600 px-4 py-1 rounded-md">
              {lastCompletedWp ? `p${lastCompletedWp.id}` : '---'}
            </p>
            <p className="text-sm text-gray-400 mt-1">
              {lastCompletedWp ? `(${lastCompletedWp.lat.toFixed(6)}, ${lastCompletedWp.lng.toFixed(6)})` : 'No waypoint marked yet'}
            </p>
        </InfoItem>
        <InfoItem label="Delta Elevation">
             <p className={`text-3xl font-bold p-2 rounded-md ${deltaElevation > 0 ? 'text-green-400' : deltaElevation < 0 ? 'text-red-400' : 'text-gray-400'}`}>
                {deltaElevation !== 0 ? (
                  <>
                    {deltaElevation >= 0 ? '+' : ''}{deltaElevation.toFixed(2)} m {deltaElevation >= 0 ? '↑' : '↓'}
                  </>
                ) : '---'}
             </p>
        </InfoItem>
        <InfoItem label="Next Point">
            <p className="text-2xl font-bold bg-orange-500 px-4 py-1 rounded-md">
              {nextWp ? `p${nextWp.id}` : currentWp ? `p${currentWp.id}` : '---'}
            </p>
            <p className="text-sm text-gray-400 mt-1">
              {nextWp ? `(${nextWp.lat.toFixed(6)}, ${nextWp.lng.toFixed(6)})` : currentWp ? `(${currentWp.lat.toFixed(6)}, ${currentWp.lng.toFixed(6)})` : 'No next waypoint'}
            </p>
        </InfoItem>
      </div>
    </div>
  );
};

export default LiveStatusbar;
