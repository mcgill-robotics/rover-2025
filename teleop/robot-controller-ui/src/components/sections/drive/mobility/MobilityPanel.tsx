'use client';

import { useState } from 'react';
import DriveInfo from './info/DriveInfo';
import DriveControl from './control/DriveControl';
import GPSPanel from './navigation/GPSPanel';

const MobilityPanel: React.FC = () => {
  const [mode, setMode] = useState<'info' | 'control' | 'gps'>('info');

  return (
    <div className="h-full flex flex-col p-2">
      {/* Toggle Header */}
      <div className="w-full flex justify-center items-center mb-4">
        <div className="w-[300px] flex justify-between items-center bg-[#26272e] rounded-full px-2 py-1 shadow-md">
          {['info', 'control', 'gps'].map((type) => (
            <div
              key={type}
              onClick={() => setMode(type as 'info' | 'control' | 'gps')}
              className={`flex-1 text-center font-bold py-2 px-3 rounded-full cursor-pointer transition-all duration-150
                ${mode === type
                  ? 'bg-gradient-to-t from-[#1d1d1d] to-[#131313] text-[#d63f3f]'
                  : 'text-gray-400 hover:text-white'
                }`}
            >
              {type === 'info' && 'Information'}
              {type === 'control' && 'Controls'}
              {type === 'gps' && 'GPS'}
            </div>
          ))}
        </div>
      </div>

      {/* Mode Content */}
      {mode === 'info' && <DriveInfo />}
      {mode === 'control' && <DriveControl/>}
      {mode === 'gps' && <GPSPanel />}
    </div>
  );
};

export default MobilityPanel;
