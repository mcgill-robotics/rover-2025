'use client';

import { useState } from 'react';

const SpeedControl: React.FC = () => {
  const speeds = [0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0];
  const [speed, setSpeed] = useState(1.5);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newValue = parseFloat(e.target.value);
    const closestSpeed = speeds.reduce((prev, curr) =>
      Math.abs(curr - newValue) < Math.abs(prev - newValue) ? curr : prev
    );
    setSpeed(closestSpeed);
  };

  return (
    <div className="flex flex-col items-center gap-[10px] h-full relative">
      <input
        type="range"
        min={Math.min(...speeds)}
        max={Math.max(...speeds)}
        step="0.01"
        value={speed}
        onChange={handleChange}
        className="mt-[5%] h-32"
        style={{ writingMode: 'vertical-lr', direction: 'rtl' }}
      />
      <div className="h-[20%] font-bold text-center">Speed: {speed}x</div>
    </div>
  );
};

export default SpeedControl;
