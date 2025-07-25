'use client';

import CoordinateDisplay from './CoordinateDisplay';
import JointSelector from './JointSelector';
import SpeedControl from './SpeedControl';
import GraphPanel from './GraphPanel';

const ArmInfo: React.FC = () => {
  return (
    <div className="w-full h-full flex flex-col">
      <CoordinateDisplay />

      <div className="flex flex-row flex-1 gap-[7.5%] mr-[2.5%] mb-[2.5%]">
        <div className="flex-[2]">
          <h1 className="text-xl font-bold my-1 ml-0">Joint Selector</h1>
          <JointSelector />
        </div>

        <div className="flex-[3]">
          <h1 className="text-xl font-bold my-1 ml-0">Speed Control</h1>
          <SpeedControl />
        </div>
      </div>

      <GraphPanel />
    </div>
  );
};

export default ArmInfo;
