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
  // Connect to ROS data store
  const { connect, reconnect } = useAutoConnect();
  const diagnostics = useDriveDiagnostics();
  const speeds = useDriveSpeeds();
  const status = useDriveStatus();
  const isConnected = useIsROSConnected();
  const error = useROSError();

  // Auto-connect on component mount
  useEffect(() => {
    if (!isConnected && !error) {
      connect();
    }
  }, [connect, isConnected, error]);

  // Convert ROS data to component format
  const liveMotorStats = diagnostics ? [
    {
      name: 'RF',
      speed: speeds?.RF || 0,
      pos: 0, // Position not available in current ROS data
      volt: diagnostics.RF.voltage,
      curr: diagnostics.RF.current,
      temp: diagnostics.RF.temperature,
      alert: status?.RF ? undefined : 'Disconnected'
    },
    {
      name: 'RB',
      speed: speeds?.RB || 0,
      pos: 0,
      volt: diagnostics.RB.voltage,
      curr: diagnostics.RB.current,
      temp: diagnostics.RB.temperature,
      alert: status?.RB ? undefined : 'Disconnected'
    },
    {
      name: 'LB',
      speed: speeds?.LB || 0,
      pos: 0,
      volt: diagnostics.LB.voltage,
      curr: diagnostics.LB.current,
      temp: diagnostics.LB.temperature,
      alert: status?.LB ? undefined : 'Disconnected'
    },
    {
      name: 'LF',
      speed: speeds?.LF || 0,
      pos: 0,
      volt: diagnostics.LF.voltage,
      curr: diagnostics.LF.current,
      temp: diagnostics.LF.temperature,
      alert: status?.LF ? undefined : 'Disconnected'
    }
  ] : null;

  if (!liveMotorStats) {
    return (
      <div className="flex items-center justify-center w-full h-full text-red-500 text-sm">
        ⚠️ No diagnostics data available. Please check your ROS connection.
      </div>
    );
  }

  return (
    <div className="flex flex-col w-full h-full p-2 items-center text-white gap-2 overflow-hidden">
      {/* Connection status indicator */}
      <div className="w-full flex justify-between items-center text-xs">
        <div className={`flex items-center gap-2 ${isConnected ? 'text-green-400' : 'text-red-400'}`}>
          <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-400' : 'bg-red-400'}`} />
          {isConnected ? 'ROS Connected' : 'ROS Disconnected'}
        </div>
        {error && (
          <button 
            onClick={reconnect}
            className="px-2 py-1 bg-blue-600 hover:bg-blue-700 rounded text-xs"
          >
            Reconnect
          </button>
        )}
      </div>

      {/* Speedometer section */}
      <SpeedometerCluster />

      {/* MotorDiagnostics fills */}
      <div className="flex-1 min-h-0">
        <MotorDiagnosticsTabs
          motorStats={liveMotorStats}
        />
      </div>
    </div>
  );
};

export default DriveInfo;
