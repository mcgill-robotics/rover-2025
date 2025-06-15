'use client';

import Speedometer from './Speedometer/Speedometer';

const DriveInfo: React.FC = () => {
  return (
    <div className="flex flex-col justify-center items-center p-2 h-full w-full">
      {/* Speedometer section */}
      <div className="w-[400px] h-[30%] relative flex flex-col items-center mb-[17.5%]">
        <h2 className="text-xl font-bold text-center mb-1">Rover Speed</h2>

        {/* Main Speedometer */}
        <div className="w-[220px] h-[167px] flex justify-center items-center mb-[-50px] relative">
          <Speedometer initialSpeed={80} />
        </div>

        {/* Four Small Speedometers */}
        <div className="relative w-full h-[160px] flex justify-center items-center">
          <div className="absolute top-[18px] left-[5px] w-[96px] h-[74px] flex justify-center items-center">
            <Speedometer initialSpeed={80} />
            <span className="absolute bottom-[-20px] text-base font-bold">FL</span>
          </div>
          <div className="absolute top-[18px] right-[5px] w-[96px] h-[74px] flex justify-center items-center">
            <Speedometer initialSpeed={80} />
            <span className="absolute bottom-[-20px] text-base font-bold">FR</span>
          </div>
          <div className="absolute top-[52px] left-[102px] w-[96px] h-[74px] flex justify-center items-center">
            <Speedometer initialSpeed={80} />
            <span className="absolute bottom-[-20px] text-base font-bold">BL</span>
          </div>
          <div className="absolute top-[52px] right-[102px] w-[96px] h-[74px] flex justify-center items-center">
            <Speedometer initialSpeed={80} />
            <span className="absolute bottom-[-20px] text-base font-bold">BR</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default DriveInfo;
