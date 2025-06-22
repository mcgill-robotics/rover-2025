// components/DriveInfo/SpeedometerCluster.tsx

import React from 'react';
import Speedometer from './Speedometer';

const SpeedometerCluster: React.FC = () => {
  return (
    <div className="w-full max-w-[600px] flex flex-col items-center mb-2">

      <div className="w-[220px] h-[167px] flex justify-center items-center relative mb-[-50px]">
        <Speedometer initialSpeed={80} />
      </div>

      <div className="relative w-full h-[160px] flex justify-center items-center">
        {[
          { label: 'FL', className: 'top-[18px] left-[5px]' },
          { label: 'FR', className: 'top-[18px] right-[5px]' },
          { label: 'BL', className: 'top-[52px] left-[102px]' },
          { label: 'BR', className: 'top-[52px] right-[102px]' },
        ].map(({ label, className }) => (
          <div
            key={label}
            className={`absolute ${className} w-[96px] h-[74px] flex justify-center items-center`}
          >
            <Speedometer initialSpeed={80} />
            <span className="absolute bottom-[-20px] text-base font-bold">{label}</span>
          </div>
        ))}
      </div>
    </div>
  );
};

export default SpeedometerCluster;
