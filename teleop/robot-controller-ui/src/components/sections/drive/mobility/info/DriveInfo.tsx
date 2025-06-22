'use client';

import { motorStats } from './MotorPanel/motorData';
import SpeedometerCluster from './Speedometer/SpeedometerCluster';
import MotorDiagnosticsTabs from './MotorPanel/MotorDiagnosticTabs';

const DriveInfo: React.FC = () => {

  return (
    <div className="flex flex-col w-full h-full p-2 items-centertext-white gap-2 overflow-hidden">
      {/* Speedometer section */}
      <SpeedometerCluster />

      {/* MotorDiagnostics fills */}
      <div className="flex-1 min-h-0">
        <MotorDiagnosticsTabs
          motorStats={motorStats}
        />
      </div>
    </div>
  );
};

export default DriveInfo;