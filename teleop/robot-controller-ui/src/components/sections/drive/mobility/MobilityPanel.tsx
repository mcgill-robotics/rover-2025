'use client';

import { useState } from 'react';
import DriveInfo from './info/DriveInfo';
import DriveControl from './control/DriveControl';

type Mode = 'info' | 'control';

const TABS: { key: Mode; label: string }[] = [
  { key: 'info', label: 'Information' },
  { key: 'control', label: 'Controls' },
];

const MobilityPanel: React.FC = () => {
  const [mode, setMode] = useState<Mode>('info');

  const renderContent = () => {
    switch (mode) {
      case 'info':
        return <DriveInfo />;
      case 'control':
        return <DriveControl />;
      default:
        return null;
    }
  };

  return (
    <div className="h-full flex flex-col p-2">
      {/* Toggle Header */}
      <div className="w-full flex justify-center items-center mb-4">
        <div className="w-[300px] flex justify-between items-center bg-[#26272e] rounded-full px-2 py-1 shadow-md">
          {TABS.map(({ key, label }) => (
            <div
              key={key}
              onClick={() => setMode(key)}
              className={`flex-1 text-center font-bold py-2 px-3 rounded-full cursor-pointer transition-all duration-150
                ${mode === key
                  ? 'bg-gradient-to-t from-[#1d1d1d] to-[#131313] text-[#d63f3f]'
                  : 'text-gray-400 hover:text-white'
                }`}
            >
              {label}
            </div>
          ))}
        </div>
      </div>

      {/* Mode Content */}
      <div className="flex-1">
        {renderContent()}
      </div>
    </div>
  );
};

export default MobilityPanel;
