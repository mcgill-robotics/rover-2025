import { useState } from "react";
import "./styles/Headlight.css";

const Headlight: React.FC = () => {
  const [brightness, setBrightness] = useState<number>(100);

  const handleBrightnessChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(event.target.value);
    setBrightness(value);
  };

  return (
    <div className="brightness-container">
      <div className="pizza-segment-container">
        <div
          className="pizza-segment"
          style={{
            background: `linear-gradient(to bottom, rgba(255, 255, 255, 0) 0%, rgba(255, 255, 255, ${brightness / 100}) 100%)`
          }}
        ></div>
      </div>
      <input
        type="range"
        min="0"
        max="100"
        value={brightness}
        onChange={handleBrightnessChange}
      />
      <span>{brightness}%</span>
      
    </div>
  );
};

export default Headlight;