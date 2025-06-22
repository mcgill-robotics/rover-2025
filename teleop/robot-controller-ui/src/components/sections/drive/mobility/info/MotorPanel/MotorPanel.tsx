import React from 'react';

interface MotorPanelProps {
  name: string;
  speed: number;
  pos: number;
  volt: number;
  curr: number;
  temp: number;
  alert?: string;
}

const MotorPanel: React.FC<MotorPanelProps> = ({
  name, speed, pos, volt, curr, temp, alert
}) => {
  return (
    <div className="bg-[#1c1c24] text-white rounded-lg p-3 border border-white/10 shadow-sm text-sm flex flex-col justify-between h-full">
      {/* Header: Name + Status */}
      <div className="flex justify-between items-center mb-2">
        <h3 className="font-semibold text-base">{name}</h3>
        <div className="flex gap-1 items-center">
          <span className="w-2.5 h-2.5 bg-green-500 rounded-full" />
          {/* Optionally add ✔/✖ icons here */}
        </div>
      </div>

      {/* Stats Grid */}
      <div className="grid grid-cols-2 gap-x-4 gap-y-1 mb-2">
        <span>Speed: {speed}</span>
        <span>Pos: {pos}°</span>
        <span>Volt: {volt}V</span>
        <span>Curr: {curr}A</span>
        <span>Temp: {temp}°C</span>
      </div>

      {/* Ping + Alert */}
      <div className="flex items-center justify-between mt-auto">
        <button className="text-xs px-2 py-1 bg-white/10 rounded hover:bg-white/20 transition">Ping</button>
        {alert && (
          <span className="text-xs font-semibold text-red-500 bg-red-500/10 px-2 py-0.5 rounded">
            ⚠ {alert}
          </span>
        )}
      </div>
    </div>
  );
};

export default MotorPanel;
