// DriveControl.tsx
'use client';

import React from 'react';
import Headlight from './Headlight/Headlight';
import Knob from './MaxSpeedKnob/MaxSpeedKnob';
import CameraManager from './CameraManager';
import Card from '@/components/ui/Card';
import { Gauge, SunMedium } from 'lucide-react';

const DriveControl: React.FC = () => {
  return (
    <div className="w-full h-full grid grid-cols-2 grid-rows-[40%_60%] gap-0">
      {/* Top-left: Maximum speed */}
      <Card
        title="Maximum speed"
        icon={<Gauge className="w-5 h-5" />}
        className="rounded-none md:rounded-tl-2xl h-full"
        center                                    
      >
        <Knob />
      </Card>

      {/* Top-right: Brightness */}
      <Card
        title="Brightness"
        icon={<SunMedium className="w-5 h-5" />}
        className="rounded-none md:rounded-tr-2xl h-full"
        center                                     
      >
        <Headlight />
      </Card>

      {/* Bottom row: Camera Manager across both columns */}
      <div className="col-span-2 row-span-1">
        <CameraManager />
      </div>
    </div>
  );
};

export default DriveControl;
