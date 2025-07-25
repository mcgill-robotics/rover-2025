'use client';

import React, { useEffect, useState } from 'react';
import MotorChart from './MotorChart';

const MOTOR_NAMES = ['FL', 'FR', 'BL', 'BR'];

interface DataPoint {
  timestamp: number;
  voltage: number;
  current: number;
  temp: number;
}

interface MotorData {
  name: string;
  data: DataPoint[];
}

const generateMockData = (): DataPoint => {
  return {
    timestamp: Date.now(),
    voltage: +(12 + Math.random() * 2).toFixed(2),
    current: +(1 + Math.random() * 3).toFixed(2),
    temp: +(30 + Math.random() * 15).toFixed(1),
  };
};

const ChartPanel: React.FC = () => {
  const [motorData, setMotorData] = useState<MotorData[]>(
    MOTOR_NAMES.map((name) => ({
      name,
      data: [],
    }))
  );

  const [activeMotor, setActiveMotor] = useState<string>('FL');

  useEffect(() => {
    const interval = setInterval(() => {
      setMotorData((prev) =>
        prev.map((motor) => {
          const newData = [...motor.data, generateMockData()].slice(-30); // keep last 30
          return { ...motor, data: newData };
        })
      );
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  const currentMotor = motorData.find((m) => m.name === activeMotor);

  return (
    <div className="w-full h-full flex flex-col">
      {/* Header */}
      <div className="flex justify-between items-center mb-2">
        <h3 className="text-white font-semibold text-sm">Motor: {activeMotor}</h3>
        <div className="flex gap-2">
          {MOTOR_NAMES.map((name) => (
            <button
              key={name}
              onClick={() => setActiveMotor(name)}
              className={`px-3 py-1 rounded-full text-sm font-medium transition-colors
                ${activeMotor === name
                  ? 'bg-white text-black'
                  : 'bg-white/10 text-white hover:bg-white/20'}`}
            >
              {name}
            </button>
          ))}
        </div>
      </div>

      {/* Chart area fills the rest */}
      <div className="flex-1 min-h-0">
        {currentMotor && <MotorChart motor={currentMotor} />}
      </div>
    </div>
  );
};

export default ChartPanel;
