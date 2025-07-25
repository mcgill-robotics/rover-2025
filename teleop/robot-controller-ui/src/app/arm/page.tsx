'use client';

import { ArmInfo } from '@/components/sections/arm/info';
import { ArmView } from '@/components/sections/arm/view';
import { ArmControl } from '@/components/sections/arm/control';

function Arm() {
  return (
    <div className="p-1 min-h-[100dvh] h-full">
      <div className="w-full h-[90%] flex flex-row gap-1">
        <div className="flex-1">
          <ArmInfo />
        </div>
        <div className="flex-[2]">
          <ArmView />
        </div>
        <div className="flex-1">
          <ArmControl />
        </div>
      </div>
    </div>
  );
}

export default Arm;
