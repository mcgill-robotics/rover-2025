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
    <div className="bg-[#1c1c24] text-white rounded-xl p-3 border border-white/10 shadow-md">
      <div className="flex justify-between items-center mb-1">
        <h3 className="font-bold">{name}</h3>
        <span className="w-3 h-3 bg-green-500 rounded-full" />
      </div>
      <p className="text-sm">Speed: {speed}</p>
      <p className="text-sm">Pos: {pos}°</p>
      <p className="text-sm">Volt: {volt}V</p>
      <p className="text-sm">Curr: {curr}A</p>
      <p className="text-sm">Temp: {temp}°C</p>
      <button className="mt-1 text-xs px-2 py-1 bg-white/10 rounded">Ping</button>
      {alert && (
        <div className="mt-2 text-xs text-red-500 font-semibold">
          ⚠ {alert}
        </div>
      )}
    </div>
  );
};

export default MotorPanel;
