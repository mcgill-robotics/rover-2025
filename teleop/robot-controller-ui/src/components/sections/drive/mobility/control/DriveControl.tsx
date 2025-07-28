'use client';

import Headlight from './Headlight/Headlight';
import Knob from './MaxSpeedKnob/MaxSpeedKnob';

const DriveControl: React.FC = () => {

  return (
    <div className="flex flex-col w-full h-full p-2">
      <div className="flex flex-row w-full">
        {/* Knob Section */}
        <div className="w-1/2 h-[30%] flex flex-col items-center">
          <h2 className="text-xl font-bold text-center mb-10">Maximum speed</h2>
          <Knob />
        </div>

        {/* Headlight Section */}
        <div className="w-1/2 h-[75%] flex flex-col items-center">
          <h2 className="text-xl font-bold text-center mb-5">Brightness</h2>
          <Headlight />
        </div>
      </div>
    </div>
  );
};

export default DriveControl;
