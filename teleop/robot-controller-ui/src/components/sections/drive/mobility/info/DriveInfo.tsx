'use client';

import { useState } from 'react';
import { motorStats } from './MotorPanel/motorData';
import SpeedometerCluster from './Speedometer/SpeedometerCluster';
import MotorDiagnosticsTabs from './MotorPanel/MotorDiagnosticTabs';

const DriveInfo: React.FC = () => {
  const [activeTab, setActiveTab] = useState<'overview' | 'details' | 'charts'>('overview');

  return (
    <div className="flex flex-col items-center w-full h-full p-2 overflow-hidden text-white">
      {/* Speedometer section */}
      <SpeedometerCluster />

      {/* Combined Motor Diagnostics section with tab switcher */}
      <MotorDiagnosticsTabs
        activeTab={activeTab}
        setActiveTab={setActiveTab}
        motorStats={motorStats}/>
    </div>
  );
};

export default DriveInfo;