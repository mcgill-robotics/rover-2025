'use client';

import { useEffect } from 'react';
import SpeedometerCluster from './Speedometer/SpeedometerCluster';
import MotorDiagnosticsTabs from './MotorPanel/MotorDiagnosticTabs';
import { 
  useDriveDiagnostics, 
  useDriveSpeeds, 
  useDriveStatus,
  useIsROSConnected,
  useROSError,
  useAutoConnect 
} from '@/store';

const DriveInfo: React.FC = () => {
  const { connect, reconnect } = useAutoConnect();
  const diagnostics = useDriveDiagnostics();
  const speeds = useDriveSpeeds();
  const status = useDriveStatus();
  const isConnected = useIsROSConnected();
  const error = useROSError();

  useEffect(() => {
    if (!isConnected && !error) {
      connect();
    }
  }, [connect, isConnected, error]);

  const liveMotorStats = diagnostics ? [
    { name: 'RF', speed: speeds?.RF || 0, pos: 0, volt: diagnostics.RF.voltage, curr: diagnostics.RF.current, temp: diagnostics.RF.temperature, alert: status?.RF ? undefined : 'Disconnected' },
    { name: 'RB', speed: speeds?.RB || 0, pos: 0, volt: diagnostics.RB.voltage, curr: diagnostics.RB.current, temp: diagnostics.RB.temperature, alert: status?.RB ? undefined : 'Disconnected' },
    { name: 'LB', speed: speeds?.LB || 0, pos: 0, volt: diagnostics.LB.voltage, curr: diagnostics.LB.current, temp: diagnostics.LB.temperature, alert: status?.LB ? undefined : 'Disconnected' },
    { name: 'LF', speed: speeds?.LF || 0, pos: 0, volt: diagnostics.LF.voltage, curr: diagnostics.LF.current, temp: diagnostics.LF.temperature, alert: status?.LF ? undefined : 'Disconnected' }
  ] : null;

  // Show overlay when we have no usable data OR an explicit error from the store
  const hasBlockingError = !!error || !liveMotorStats;
  const message = error
    ? error
    : 'No diagnostics data available. Please check your ROS connection.';

  return (
    <div className="relative flex flex-col w-full h-full p-2 items-center text-white gap-2 overflow-hidden">
      {/* Connection status */}
      <div className="w-full flex justify-between items-center text-xs">
        <div className={`flex items-center gap-2 ${isConnected ? 'text-green-400' : 'text-red-400'}`}>
          <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-400' : 'bg-red-400'}`} />
          {isConnected ? 'ROS Connected' : 'ROS Disconnected'}
        </div>
        {error && (
          <button
            onClick={reconnect}
            className="px-2 py-1 bg-red-600 hover:bg-red-700 rounded text-xs"
          >
            Reconnect
          </button>
        )}
      </div>

      {/* Main content (always rendered) */}
      <SpeedometerCluster />
      <div className="flex-1 min-h-0 w-full">
        {liveMotorStats && (
          <MotorDiagnosticsTabs motorStats={liveMotorStats} />
        )}
      </div>

      {/* ERROR OVERLAY */}
      {hasBlockingError && (
        <div className="absolute inset-0 z-50 grid place-items-center bg-gray-800/10 backdrop-blur-sm">
          <div className="max-w-md w-[90%] text-center bg-red-600 text-white rounded-xl shadow-2xl border border-white/20 px-5 py-4">
            <div className="text-sm font-semibold mb-1">Connection Error</div>
            <div className="text-[13px] opacity-95">{message}</div>
            <div className="mt-3 flex items-center justify-center gap-2">
              <button
                onClick={reconnect}
                className="px-3 py-2 rounded-lg bg-white/15 hover:bg-white/25 text-white text-xs"
              >
                Try Reconnect
              </button>
              <button
                onClick={connect}
                className="px-3 py-2 rounded-lg bg-black/30 hover:bg-black/40 text-white text-xs"
              >
                Connect
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default DriveInfo;
