'use client';

import React, { useState, useEffect } from 'react';

interface DPadProps {
  inputStream: string; // "up", "down", "left", "right"
}

const DPad: React.FC<DPadProps> = ({ inputStream }) => {
  const [activeButton, setActiveButton] = useState<string | null>(null);

  useEffect(() => {
    setActiveButton(inputStream);
  }, [inputStream]);

  const getFill = (dir: string) => (activeButton === dir ? 'black' : 'lightgrey');

  return (
    <div className="w-full h-full">
      <svg
        width="100%"
        height="100%"
        viewBox="0 0 100 100"
        xmlns="http://www.w3.org/2000/svg"
        className="dpad"
      >
        {/* Top */}
        <g transform="rotate(0, 50, 50)">
          <polygon
            className="arrow"
            points="40,15 60,15 50,25"
            fill={getFill('up')}
            transform="translate(0, 20)"
          />
          <rect
            x="40"
            y="13"
            width="20"
            height="25"
            rx="10"
            ry="5"
            fill={getFill('up')}
          />
        </g>

        {/* Bottom */}
        <g transform="rotate(180, 50, 50)">
          <polygon
            className="arrow"
            points="40,15 60,15 50,25"
            fill={getFill('down')}
            transform="translate(0, 20)"
          />
          <rect
            x="40"
            y="13"
            width="20"
            height="25"
            rx="10"
            ry="5"
            fill={getFill('down')}
          />
        </g>

        {/* Left */}
        <g transform="rotate(-90, 50, 50)">
          <polygon
            className="arrow"
            points="40,15 60,15 50,25"
            fill={getFill('left')}
            transform="translate(0, 20)"
          />
          <rect
            x="40"
            y="13"
            width="20"
            height="25"
            rx="10"
            ry="5"
            fill={getFill('left')}
          />
        </g>

        {/* Right */}
        <g transform="rotate(90, 50, 50)">
          <polygon
            className="arrow"
            points="40,15 60,15 50,25"
            fill={getFill('right')}
            transform="translate(0, 20)"
          />
          <rect
            x="40"
            y="13"
            width="20"
            height="25"
            rx="10"
            ry="5"
            fill={getFill('right')}
          />
        </g>
      </svg>
    </div>
  );
};

export default DPad;
