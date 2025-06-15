import { useState } from 'react';
import './Headlight.css';

const Headlight: React.FC = () => {
  const [brightness, setBrightness] = useState<number>(100);

  const handleBrightnessChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(event.target.value);
    setBrightness(value);
  };

  return (
    <div className="w-full h-full flex flex-col items-center gap-2 brightness-container">
      <div className="relative w-[200px] h-[200px] flex items-end justify-center overflow-hidden pizza-segment-container">
        <div
          className="pizza-segment"
          style={{
            background: `linear-gradient(to bottom, rgba(255, 255, 255, 0) 0%, rgba(255, 255, 255, ${brightness / 100}) 100%)`
          }}
        />
      </div>
      <input
        type="range"
        min="0"
        max="100"
        value={brightness}
        onChange={handleBrightnessChange}
        className="w-4/5 mt-2 range-input"
      />
      <span className="text-sm">{brightness}%</span>
    </div>
  );
};

export default Headlight;
