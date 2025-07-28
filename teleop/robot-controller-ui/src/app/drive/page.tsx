'use client';

import { CameraView } from '@/components/sections/drive/camera';
import { MobilityPanel } from '@/components/sections/drive/mobility';

const Drive: React.FC = () => {

  return (
    <div className="w-full h-[95dvh] p-1">
      <div className="w-full h-full flex gap-1">
        <div className="w-[70%]">
          <CameraView />
        </div>
        <div className="w-[30%]">
          <MobilityPanel />
        </div>
      </div>
    </div>
  );
};

export default Drive;
