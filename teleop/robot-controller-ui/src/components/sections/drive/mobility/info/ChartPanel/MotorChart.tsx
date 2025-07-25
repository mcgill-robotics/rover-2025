import React from 'react';
import {
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer
} from 'recharts';

interface DataPoint {
  timestamp: number;
  voltage: number;
  current: number;
  temp: number;
}

interface Props {
  motor: {
    name: string;
    data: DataPoint[];
  };
}

const MotorChart: React.FC<Props> = ({ motor }) => {
  return (
    <div className="bg-[#1c1c24] rounded-md border border-white/10 h-full flex flex-col p-2">
      <h3 className="text-white mb-2 font-semibold text-sm">{motor.name}</h3>
      <div className="flex-1 min-h-0">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={motor.data} margin={{ top: 10, right: 20, bottom: 10, left: 5 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="#444" />
            <XAxis
              dataKey="timestamp"
              tickFormatter={(t) => `${Math.round((t % 1000000) / 1000)}s`}
              stroke="#aaa"
            />
            <YAxis stroke="#aaa" tickMargin={4} width={30} />
            <Tooltip />
            {/* Legend removed */}
            <Line type="monotone" dataKey="voltage" stroke="#facc15" dot={false} name="Voltage (V)" />
            <Line type="monotone" dataKey="current" stroke="#38bdf8" dot={false} name="Current (A)" />
            <Line type="monotone" dataKey="temp" stroke="#ef4444" dot={false} name="Temp (°C)" />
          </LineChart>
        </ResponsiveContainer>
      </div>

      {/* Custom Legend */}
      <div className="flex justify-center gap-4 text-xs text-white/60 mt-1">
        <div className="flex items-center gap-1">
          <span className="w-3 h-1 bg-yellow-400 inline-block" />
          <span>Voltage (V)</span>
        </div>
        <div className="flex items-center gap-1">
          <span className="w-3 h-1 bg-sky-400 inline-block" />
          <span>Current (A)</span>
        </div>
        <div className="flex items-center gap-1">
          <span className="w-3 h-1 bg-red-500 inline-block" />
          <span>Temp (°C)</span>
        </div>
      </div>
    </div>
  );
};


export default MotorChart;
