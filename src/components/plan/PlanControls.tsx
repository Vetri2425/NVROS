import React, { useRef, useState } from 'react';
import CircleTool from '../../tools/CircleTool';
import PolygonTool from '../../tools/PolygonTool';
import SurveyGridTool from '../../tools/SurveyGridTool';
import SplineWaypointTool from '../../tools/SplineWaypointTool';
import { ChevronDownIcon } from '../icons/ChevronDownIcon';
import { MissionFileInfo, Waypoint } from '../../types';
import { parseMissionFile, ParsedWaypoint } from '../../utils/missionParser';

// 🔵 ADDED

type PlanControlsProps = {
  onUpload: (waypoints: Waypoint[], info: MissionFileInfo) => void;
  onExport: () => void;
  onUploadInitiated: () => void;
  missionWaypoints: Waypoint[];

  onWriteToRover: () => Promise<boolean>;
  onReadFromRover: () => Promise<void>;
  isConnected: boolean;
  uploadProgress: number;
  missionFileInfo: MissionFileInfo | null;
  onClearMission: () => void;
};

const PlanControls: React.FC<PlanControlsProps> = ({
  onUpload,
  onExport,
  onUploadInitiated,
  missionWaypoints,
  onWriteToRover,
  onReadFromRover,
  isConnected,
  uploadProgress,
  missionFileInfo,
  onClearMission,
}) => {
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [isWriting, setIsWriting] = useState(false);
  const [lastWriteStatus, setLastWriteStatus] = useState<null | 'success' | 'error'>(null);
  const [isReading, setIsReading] = useState(false);
  const [lastReadStatus, setLastReadStatus] = useState<null | 'success' | 'error' | 'empty'>(null);
  const [readErrorMessage, setReadErrorMessage] = useState<string | null>(null);
  const [currentFile, setCurrentFile] = useState<MissionFileInfo | null>(null);
  const [showCircleTool, setShowCircleTool] = useState(false);
  const [showPolygonTool, setShowPolygonTool] = useState(false);
  const [showSurveyGridTool, setShowSurveyGridTool] = useState(false);
  const [showSplineTool, setShowSplineTool] = useState(false);

  React.useEffect(() => {
    if (missionFileInfo) {
      setCurrentFile(missionFileInfo);
    } else {
      setCurrentFile(null);
    }
  }, [missionFileInfo]);


  const validateFile = (file: File): boolean => {
    const validExtensions = ['waypoint', 'csv', 'dxf', 'json'];
    const ext = file.name.split('.').pop()?.toLowerCase() || '';
    if (!validExtensions.includes(ext)) {
      alert(`Unsupported file format: .${ext}\nAllowed: ${validExtensions.join(', ')}`);
      return false;
    }
    if (file.size === 0) {
      alert('File is empty. Please upload a valid mission file.');
      return false;
    }
    return true;
  };

  const processFiles = async (files: FileList | null) => {
    if (files && files.length > 0) {
      const file = files[0];
      if (!validateFile(file)) return;

      const ext = file.name.split('.').pop()?.toLowerCase() || '';

      try {
        const parsedWaypoints: ParsedWaypoint[] = await parseMissionFile(file);
        const mission: Waypoint[] = parsedWaypoints.map((wp, index) => ({
          ...wp,
          id: index + 1,
          command: wp.command || 'WAYPOINT',
        }));
        const info: MissionFileInfo = {
          name: file.name,
          size: file.size,
          type: file.type || ext || 'unknown',
          uploadedAt: new Date().toISOString(),
          waypointCount: mission.length,
          source: 'file',
        };
        setCurrentFile(info);
        onUpload(mission, info);
        alert(`✅ Successfully loaded ${mission.length} waypoints from ${file.name}`);
      } catch (error) {
        console.error('Error parsing mission file:', error);
        alert(`❌ Failed to parse ${file.name}: ${(error as Error).message}`);
      }
    }
  };

  const handleUploadClick = () => {
    onUploadInitiated();
    fileInputRef.current?.click();
  };

  const handleWriteToRover = async () => {
    if (!isConnected) {
      alert('Rover not connected.');
      return;
    }
    if (!missionWaypoints.length) {
      alert('No waypoints to upload.');
      return;
    }
    setIsWriting(true);
    try {
      const ok = await onWriteToRover();
      setLastWriteStatus(ok ? 'success' : 'error');
    } catch (error) {
      console.error('PlanControls.handleWriteToRover error', error);
      setLastWriteStatus('error');
    } finally {
      setIsWriting(false);
      setTimeout(() => setLastWriteStatus(null), 3000);
    }
  };

  const handleReadFromRover = async () => {
    if (!isConnected) {
      setReadErrorMessage('Rover not connected. Please connect to the rover first.');
      setLastReadStatus('error');
      setTimeout(() => {
        setReadErrorMessage(null);
        setLastReadStatus(null);
      }, 5000);
      return;
    }

    setIsReading(true);
    setReadErrorMessage(null);
    setLastReadStatus(null);

    try {
      await onReadFromRover();
      
      // Check if we actually got waypoints
      // Note: This check happens after onReadFromRover updates the mission
      // We'll add a small delay to let state update
      setTimeout(() => {
        if (missionWaypoints.length === 0) {
          setLastReadStatus('empty');
          setReadErrorMessage('Rover mission is empty. No waypoints to download.');
        } else {
          setLastReadStatus('success');
          setReadErrorMessage(null);
        }
        
        // Clear status after 5 seconds
        setTimeout(() => {
          setLastReadStatus(null);
          setReadErrorMessage(null);
        }, 5000);
      }, 100);
      
    } catch (error) {
      const errorMsg = error instanceof Error ? error.message : 'Failed to download mission from rover';
      setReadErrorMessage(errorMsg);
      setLastReadStatus('error');
      console.error('Mission download error:', error);
      
      // Clear error after 5 seconds
      setTimeout(() => {
        setReadErrorMessage(null);
        setLastReadStatus(null);
      }, 5000);
    } finally {
      setIsReading(false);
    }
  };

  const homeLocation = missionWaypoints.length > 0 ? missionWaypoints[0] : null;

  // Servo control logic removed from Plan tab

  return (
    <>
      <div className="bg-slate-800 p-4 rounded-lg">
        {/* Hidden file input */}
        <input
          ref={fileInputRef}
          type="file"
          className="hidden"
          onChange={(e) => processFiles(e.target.files)}
          accept=".waypoint,.csv,.dxf"
        />

        <div className="space-y-3">
          {/* Grid picker (unchanged) */}
          <div className="flex items-center justify-between text-white">
            <span className="text-sm">Grid</span>
            <div className="flex items-center space-x-1 text-sm">
              <select className="bg-slate-700 text-white px-2 py-1 rounded text-xs">
                <option>GoogleHybridMap</option>
                <option>OpenStreetMap</option>
                <option>GoogleSatellite</option>
              </select>
              <ChevronDownIcon className="w-4 h-4" />
            </div>
          </div>

          <div className="text-xs text-green-400">Status: loaded tiles</div>

          {/* Mission file actions */}
          <div className="space-y-2">
            <button
              onClick={handleUploadClick}
              className="w-full bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded-md transition-all duration-200 text-sm font-medium"
            >
              📁 Upload Mission
            </button>

            <button
              onClick={onExport}
              disabled={missionWaypoints.length === 0}
              className={`w-full px-4 py-2 rounded-md transition-all duration-200 text-sm font-medium ${
                missionWaypoints.length > 0
                  ? 'bg-blue-600 hover:bg-blue-700 text-white'
                  : 'bg-gray-500 text-gray-300 cursor-not-allowed'
              }`}
            >
              💾 Export Mission
            </button>

            <button
              onClick={onClearMission}
              disabled={missionWaypoints.length === 0}
              className={`w-full px-4 py-2 rounded-md transition-all duration-200 text-sm font-medium ${
                missionWaypoints.length > 0
                  ? 'bg-red-600 hover:bg-red-700 text-white'
                  : 'bg-gray-500 text-gray-300 cursor-not-allowed'
              }`}
            >
              🗑️ Remove Mission
            </button>

            {currentFile && (
              <div className="bg-slate-700 p-2 mt-3 rounded text-xs text-slate-300">
                <div>
                  📄 <strong>{currentFile.name}</strong>
                </div>
                <div>Type: {currentFile.type || 'Unknown'}</div>
                <div>Size: {(currentFile.size / 1024).toFixed(1)} KB</div>
                <div>Waypoints: {missionWaypoints.length}</div>
              </div>
            )}

            {/* Auto tools */}
            <div className="border-t border-slate-600 pt-3 mt-4">
              <div className="text-xs text-slate-300 mb-2">Auto Waypoint Tools</div>
              <div className="space-y-2">
                <button
                  onClick={() => setShowSurveyGridTool(true)}
                  className="w-full bg-green-700 hover:bg-green-800 text-white py-2 rounded-md text-sm font-medium flex items-center justify-center gap-2"
                >
                  <span>📐</span>
                  <span>Survey Grid</span>
                  <span className="text-xs bg-green-900 px-1.5 py-0.5 rounded">MP Style</span>
                </button>
                <button
                  onClick={() => setShowPolygonTool(true)}
                  className="w-full bg-purple-700 hover:bg-purple-800 text-white py-2 rounded-md text-sm font-medium"
                >
                  🗺️ Generate Polygon Mission
                </button>
                <button
                  onClick={() => setShowCircleTool(true)}
                  className="w-full bg-sky-700 hover:bg-sky-800 text-white py-2 rounded-md text-sm font-medium"
                >
                  🌀 Generate Circle
                </button>
                <button
                  onClick={() => setShowSplineTool(true)}
                  disabled={missionWaypoints.length === 0}
                  className="w-full bg-indigo-700 hover:bg-indigo-800 disabled:opacity-50 disabled:cursor-not-allowed text-white py-2 rounded-md text-sm font-medium"
                  title="Convert waypoints to smooth spline paths"
                >
                  〰️ Convert to Spline
                </button>
              </div>
            </div>

            {/* Servo Control moved to App level */}

            {/* Rover IO */}
            <div className="border-t border-slate-600 pt-3 mt-4">
              <div className="text-xs text-slate-300 mb-2">Rover Communication</div>

              <button
                onClick={handleReadFromRover}
                disabled={!isConnected || isReading}
                className={`w-full px-4 py-2 rounded-md transition-all duration-200 text-sm font-medium mb-2 ${
                  isConnected && !isReading
                    ? lastReadStatus === 'success'
                      ? 'bg-green-600 hover:bg-green-700 text-white'
                      : lastReadStatus === 'error'
                      ? 'bg-red-600 hover:bg-red-700 text-white'
                      : lastReadStatus === 'empty'
                      ? 'bg-yellow-600 hover:bg-yellow-700 text-white'
                      : 'bg-purple-600 hover:bg-purple-700 text-white'
                    : 'bg-gray-500 text-gray-300 cursor-not-allowed'
                }`}
              >
                {isReading 
                  ? '⏳ Reading...' 
                  : lastReadStatus === 'success'
                  ? '✅ Downloaded'
                  : lastReadStatus === 'error'
                  ? '❌ Failed'
                  : lastReadStatus === 'empty'
                  ? '⚠️ Empty Mission'
                  : '📥 Read from Rover'}
              </button>

              {/* Error/Success Message Display */}
              {readErrorMessage && (
                <div className={`mb-2 p-2 rounded text-xs ${
                  lastReadStatus === 'error' 
                    ? 'bg-red-900/50 text-red-200 border border-red-700'
                    : lastReadStatus === 'empty'
                    ? 'bg-yellow-900/50 text-yellow-200 border border-yellow-700'
                    : 'bg-blue-900/50 text-blue-200 border border-blue-700'
                }`}>
                  <div className="font-semibold mb-1">
                    {lastReadStatus === 'error' ? '⚠️ Download Failed' : 'ℹ️ Info'}
                  </div>
                  <div>{readErrorMessage}</div>
                </div>
              )}

              <button
                onClick={handleWriteToRover}
                disabled={!isConnected || missionWaypoints.length === 0 || isWriting}
                className={`w-full px-4 py-2 rounded-md transition-all duration-200 text-sm font-medium ${
                  isConnected && missionWaypoints.length > 0 && !isWriting
                    ? 'bg-orange-600 hover:bg-orange-700 text-white'
                    : 'bg-gray-500 text-gray-300 cursor-not-allowed'
                }`}
              >
                {isWriting
                  ? `Uploading mission (${missionWaypoints.length} waypoints)`
                  : lastWriteStatus === 'success'
                  ? '✅ Uploaded'
                  : lastWriteStatus === 'error'
                  ? '❌ Upload failed'
                  : '📤 Write to Rover'}
              </button>

              {uploadProgress > 0 && (
                <div className="w-full bg-gray-700 rounded-full h-2.5 mt-2">
                  <div
                    className="bg-green-600 h-2.5 rounded-full"
                    style={{ width: `${uploadProgress}%` }}
                  ></div>
                </div>
              )}

              <div
                className={`text-xs mt-2 px-2 py-1 rounded text-center ${
                  isConnected ? 'text-green-300 bg-green-900/30' : 'text-red-300 bg-red-900/30'
                }`}
              >
                {isConnected ? '✅ Rover Connected' : '❌ Rover Disconnected'}
              </div>
            </div>
          </div>

          {/* Home Info */}
          <div className="bg-slate-700 p-3 rounded-md mt-3">
            <div className="text-sm font-medium text-white mb-2">Home Location</div>
            <div className="grid grid-cols-2 gap-2 text-xs">
              <div>
                <span className="text-slate-300">Lat:</span>
                <div className="text-white font-mono">
                  {homeLocation ? homeLocation.lat.toFixed(8) : 'N/A'}
                </div>
              </div>
              <div>
                <span className="text-slate-300">Long:</span>
                <div className="text-white font-mono">
                  {homeLocation ? homeLocation.lng.toFixed(8) : 'N/A'}
                </div>
              </div>
              <div className="col-span-2">
                <span className="text-slate-300">ASL:</span>
                <div className="text-white font-mono">
                  {homeLocation ? homeLocation.alt.toFixed(2) : 'N/A'}
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      {showCircleTool && (
        <CircleTool
        onGenerate={(wps) =>
          onUpload(wps, {
            name: 'Circle Mission',
            size: 0,
            type: 'generated',
            uploadedAt: new Date().toISOString(),
            waypointCount: wps.length,
            source: 'generated',
          })
        }
        onClose={() => setShowCircleTool(false)}
      />
      )}
      {showPolygonTool && (
        <PolygonTool
        onGenerate={(wps) =>
          onUpload(wps, {
            name: 'Polygon Mission',
            size: 0,
            type: 'generated',
            uploadedAt: new Date().toISOString(),
            waypointCount: wps.length,
            source: 'generated',
          })
        }
        onClose={() => setShowPolygonTool(false)}
      />
      )}
      {showSurveyGridTool && (
        <SurveyGridTool
        onGenerate={(wps) =>
          onUpload(wps, {
            name: 'Survey Grid Mission',
            size: 0,
            type: 'generated',
            uploadedAt: new Date().toISOString(),
            waypointCount: wps.length,
            source: 'generated',
          })
        }
        onClose={() => setShowSurveyGridTool(false)}
      />
      )}
      {showSplineTool && (
        <SplineWaypointTool
        currentWaypoints={missionWaypoints}
        onConvert={(wps) =>
          onUpload(wps, {
            name: currentFile?.name.replace('.waypoints', '_spline.waypoints') || 'Spline Mission',
            size: 0,
            type: 'generated',
            uploadedAt: new Date().toISOString(),
            waypointCount: wps.length,
            source: 'generated',
          })
        }
        onClose={() => setShowSplineTool(false)}
      />
      )}
    </>
  );
};

export default PlanControls;





