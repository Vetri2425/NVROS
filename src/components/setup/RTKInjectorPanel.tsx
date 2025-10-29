import React, { useState, useEffect } from 'react';
import { useRover } from '../../context/RoverContext';

interface RTKConfig {
  casterAddress: string;
  port: string;
  mountpoint: string;
  username: string;
  password: string;
}

const RTKInjectorPanel: React.FC = () => {
  const {
    services: { injectRTK, stopRTK, getRTKStatus },
  } = useRover();

  const [config, setConfig] = useState<RTKConfig>({
    casterAddress: '',
    port: '2101',
    mountpoint: '',
    username: '',
    password: '',
  });

  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isStreamRunning, setIsStreamRunning] = useState(false);
  const [totalBytes, setTotalBytes] = useState(0);
  const [feedback, setFeedback] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  // Check RTK status on mount and periodically
  useEffect(() => {
    const checkStatus = async () => {
      try {
        const status = await getRTKStatus();
        if (status.success) {
          setIsStreamRunning(status.running || false);
          setTotalBytes(status.total_bytes || 0);
        }
      } catch (err) {
        console.error('Failed to get RTK status:', err);
      }
    };

    checkStatus();
    const interval = setInterval(checkStatus, 3000); // Check every 3 seconds

    return () => clearInterval(interval);
  }, [getRTKStatus]);

  const handleInputChange = (field: keyof RTKConfig) => (
    e: React.ChangeEvent<HTMLInputElement>
  ) => {
    setConfig((prev) => ({
      ...prev,
      [field]: e.target.value,
    }));
  };

  const handleStartStream = async () => {
    // Validate required fields
    const requiredFields: (keyof RTKConfig)[] = [
      'casterAddress',
      'port',
      'mountpoint',
      'username',
      'password',
    ];
    
    const missingFields = requiredFields.filter(field => !config[field].trim());
    if (missingFields.length > 0) {
      setError(`Missing required fields: ${missingFields.join(', ')}`);
      return;
    }

    setIsSubmitting(true);
    setFeedback(null);
    setError(null);

    try {
      const ntripUrl = `rtcm://${config.username}:${config.password}@${config.casterAddress}:${config.port}/${config.mountpoint}`;
      const response = await injectRTK(ntripUrl.trim());
      
      if (response.success) {
        setFeedback(response.message ?? 'RTK stream started successfully.');
        setIsStreamRunning(true);
      } else {
        setError(response.message ?? 'Failed to start RTK stream.');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to start RTK stream.');
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleStopStream = async () => {
    setIsSubmitting(true);
    setFeedback(null);
    setError(null);

    try {
      const response = await stopRTK();
      
      if (response.success) {
        setFeedback(response.message ?? 'RTK stream stopped successfully.');
        setIsStreamRunning(false);
      } else {
        setError(response.message ?? 'Failed to stop RTK stream.');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to stop RTK stream.');
    } finally {
      setIsSubmitting(false);
    }
  };

  const inputClasses = "w-full px-3 py-2 bg-slate-800 border border-slate-600 rounded-md text-white focus:ring-2 focus:ring-indigo-500 focus:border-transparent transition-all outline-none placeholder-slate-500 disabled:opacity-50 disabled:cursor-not-allowed";
  const labelClasses = "block text-sm font-medium text-slate-300";

  return (
    <div className="bg-slate-900 rounded-xl shadow-lg p-6 space-y-6">
      <header>
        <div className="flex items-center justify-between">
          <h2 className="text-xl font-semibold text-white flex items-center gap-2">
            🛰️ RTK Correction Stream
          </h2>
          <div className="flex items-center gap-2">
            {isStreamRunning && (
              <span className="flex items-center gap-2 text-green-400 text-sm font-medium">
                <span className="w-2 h-2 bg-green-400 rounded-full animate-pulse"></span>
                Stream Active
              </span>
            )}
          </div>
        </div>
        <p className="text-sm text-slate-400 mt-1">
          Configure and control RTK correction stream for high-precision positioning.
        </p>
      </header>

      {/* Stream Status */}
      {isStreamRunning && (
        <div className="bg-slate-800/50 border border-slate-700 rounded-lg p-4">
          <div className="flex items-center justify-between text-sm">
            <span className="text-slate-300">Data received:</span>
            <span className="text-green-400 font-mono font-semibold">
              {(totalBytes / 1024).toFixed(2)} KB
            </span>
          </div>
        </div>
      )}

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {/* Caster Address */}
        <div className="space-y-1">
          <label htmlFor="casterAddress" className={labelClasses}>
            Caster Address
          </label>
          <input
            id="casterAddress"
            type="text"
            value={config.casterAddress}
            onChange={handleInputChange('casterAddress')}
            placeholder="rtk.example.com"
            disabled={isStreamRunning}
            className={inputClasses}
          />
        </div>

        {/* Port */}
        <div className="space-y-1">
          <label htmlFor="port" className={labelClasses}>
            Port
          </label>
          <input
            id="port"
            type="text"
            value={config.port}
            onChange={handleInputChange('port')}
            placeholder="2101"
            disabled={isStreamRunning}
            className={inputClasses}
          />
        </div>

        {/* Mountpoint */}
        <div className="space-y-1">
          <label htmlFor="mountpoint" className={labelClasses}>
            Mountpoint
          </label>
          <input
            id="mountpoint"
            type="text"
            value={config.mountpoint}
            onChange={handleInputChange('mountpoint')}
            placeholder="RTCM3"
            disabled={isStreamRunning}
            className={inputClasses}
          />
        </div>

        {/* Username */}
        <div className="space-y-1">
          <label htmlFor="username" className={labelClasses}>
            Username
          </label>
          <input
            id="username"
            type="text"
            value={config.username}
            onChange={handleInputChange('username')}
            placeholder="Enter username"
            disabled={isStreamRunning}
            className={inputClasses}
          />
        </div>

        {/* Password */}
        <div className="space-y-1 md:col-span-2">
          <label htmlFor="password" className={labelClasses}>
            Password
          </label>
          <input
            id="password"
            type="password"
            value={config.password}
            onChange={handleInputChange('password')}
            placeholder="Enter password"
            disabled={isStreamRunning}
            className={inputClasses}
          />
        </div>
      </div>

      {/* Action Buttons */}
      <div className="flex flex-col gap-4">
        <div className="grid grid-cols-2 gap-3">
          <button
            onClick={handleStartStream}
            disabled={isSubmitting || isStreamRunning}
            className="px-4 py-2 bg-green-600 hover:bg-green-500 disabled:bg-green-800 
                     text-white font-semibold rounded-md transition-colors 
                     disabled:opacity-60 disabled:cursor-not-allowed flex items-center justify-center gap-2"
          >
            {isSubmitting ? (
              <>
                <span className="animate-spin">⏳</span> 
                Starting...
              </>
            ) : (
              <>
                <span>▶️</span>
                Start Stream
              </>
            )}
          </button>

          <button
            onClick={handleStopStream}
            disabled={isSubmitting || !isStreamRunning}
            className="px-4 py-2 bg-red-600 hover:bg-red-500 disabled:bg-red-800 
                     text-white font-semibold rounded-md transition-colors 
                     disabled:opacity-60 disabled:cursor-not-allowed flex items-center justify-center gap-2"
          >
            {isSubmitting ? (
              <>
                <span className="animate-spin">⏳</span> 
                Stopping...
              </>
            ) : (
              <>
                <span>⏹️</span>
                Stop Stream
              </>
            )}
          </button>
        </div>

        {/* Status Messages */}
        {feedback && (
          <div className="px-4 py-2 bg-green-900/50 border border-green-500 rounded-md">
            <p className="text-sm text-green-300">{feedback}</p>
          </div>
        )}
        {error && (
          <div className="px-4 py-2 bg-red-900/50 border border-red-500 rounded-md">
            <p className="text-sm text-red-300">{error}</p>
          </div>
        )}
      </div>
    </div>
  );
};

export default RTKInjectorPanel;