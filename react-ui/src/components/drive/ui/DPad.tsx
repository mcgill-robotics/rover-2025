import React, { useState, useEffect } from 'react';
import './styles/DPad.css';

interface DPadProps {
  inputStream: string; // "up", "down", "left", "right"
}

const DPad: React.FC<DPadProps> = ({ inputStream }) => {
  const [activeButton, setActiveButton] = useState<string | null>(null);

  useEffect(() => {
    setActiveButton(inputStream); // Update active button on new input
  }, [inputStream]);

  return (
    <svg
      width="100%"
      height="100%"
      viewBox="0 0 100 100"
      xmlns="http://www.w3.org/2000/svg"
    >
      {/* Top Button */}
      <g transform="rotate(0, 50, 50)">
        <polygon 
          points="40,15 60,15 50,25" 
          fill={activeButton === "up" ? "black" : "lightgrey"} 
          transform="translate(0, 20)" 
        />
        <rect 
          x="40" 
          y="13" 
          width="20" 
          height="25" 
          rx="10" 
          ry="5" 
          fill={activeButton === "up" ? "black" : "lightgrey"} 
        />
      </g>

      {/* Bottom Button */}
      <g transform="rotate(180, 50, 50)">
        <polygon 
          points="40,15 60,15 50,25" 
          fill={activeButton === "down" ? "black" : "lightgrey"} 
          transform="translate(0, 20)" 
        />
        <rect 
          x="40" 
          y="13" 
          width="20" 
          height="25" 
          rx="10" 
          ry="5" 
          fill={activeButton === "down" ? "black" : "lightgrey"} 
        />
      </g>

      {/* Left Button */}
      <g transform="rotate(-90, 50, 50)">
        <polygon 
          points="40,15 60,15 50,25" 
          fill={activeButton === "left" ? "black" : "lightgrey"} 
          transform="translate(0, 20)" 
        />
        <rect 
          x="40" 
          y="13" 
          width="20" 
          height="25" 
          rx="10" 
          ry="5" 
          fill={activeButton === "left" ? "black" : "lightgrey"} 
        />
      </g>

      {/* Right Button */}
      <g transform="rotate(90, 50, 50)">
        <polygon 
          points="40,15 60,15 50,25" 
          fill={activeButton === "right" ? "black" : "lightgrey"} 
          transform="translate(0, 20)" 
        />
        <rect 
          x="40" 
          y="13" 
          width="20" 
          height="25" 
          rx="10" 
          ry="5" 
          fill={activeButton === "right" ? "black" : "lightgrey"} 
        />
      </g>
    </svg>
  );
};

export default DPad;