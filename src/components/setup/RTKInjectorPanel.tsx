import React, { useState } from 'react';
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
    services: { injectRTK },
  } = useRover();

  const [config, setConfig] = useState<RTKConfig>({
    casterAddress: '',
    port: '2101',
    mountpoint: '',
    username: '',
    password: '',
  });

  const [isSubmitting, setIsSubmitting] = useState(false);
  const [feedback, setFeedback] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const handleInputChange = (field: keyof RTKConfig) => (
    e: React.ChangeEvent<HTMLInputElement>
  ) => {
    setConfig((prev) => ({
      ...prev,
      [field]: e.target.value,
    }));
  };

  const handleInject = async () => {
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
      const ntripUrl = `ntrip://${config.username}:${config.password}@${config.casterAddress}:${config.port}/${config.mountpoint}`;
      const response = await injectRTK(ntripUrl.trim());
      
      if (response.success) {
        setFeedback(response.message ?? 'RTK injection started successfully.');
      } else {
        setError(response.message ?? 'Failed to start RTK injection.');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to start RTK injection.');
    } finally {
      setIsSubmitting(false);
    }
  };

  const inputClasses = "w-full px-3 py-2 bg-slate-800 border border-slate-600 rounded-md text-white focus:ring-2 focus:ring-indigo-500 focus:border-transparent transition-all outline-none placeholder-slate-500";
  const labelClasses = "block text-sm font-medium text-slate-300";

  return (
    <div className="bg-slate-900 rounded-xl shadow-lg p-6 space-y-6">
      <header>
        <h2 className="text-xl font-semibold text-white flex items-center gap-2">
          🛰️ RTK Correction Setup
        </h2>
        <p className="text-sm text-slate-400 mt-1">
          Configure RTK correction stream parameters to enable high-precision positioning.
        </p>
      </header>

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
            className={inputClasses}
          />
        </div>
      </div>

      {/* Action Buttons */}
      <div className="flex flex-col gap-4">
        <button
          onClick={handleInject}
          disabled={isSubmitting}
          className="w-full px-4 py-2 bg-indigo-600 hover:bg-indigo-500 disabled:bg-indigo-800 
                   text-white font-semibold rounded-md transition-colors 
                   disabled:opacity-60 disabled:cursor-not-allowed flex items-center justify-center gap-2"
        >
          {isSubmitting ? (
            <>
              <span className="animate-spin">⏳</span> 
              Connecting...
            </>
          ) : (
            'Start RTK Injection'
          )}
        </button>

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