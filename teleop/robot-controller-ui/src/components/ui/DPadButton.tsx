import React from 'react';

interface DPadButtonProps {
  direction: 'up' | 'down' | 'left' | 'right';
  active: boolean;
}

const rotationMap = {
  up: 0,
  down: 180,
  left: -90,
  right: 90,
};

const DPadButton: React.FC<DPadButtonProps> = ({ direction, active }) => {
  const fill = active ? 'black' : 'lightgrey';
  const rotation = rotationMap[direction];

  return (
    <g transform={`rotate(${rotation}, 50, 50)`}>
      <polygon
        points="40,15 60,15 50,25"
        fill={fill}
        transform="translate(0, 20)"
      />
      <rect
        x="40"
        y="13"
        width="20"
        height="25"
        rx="10"
        ry="5"
        fill={fill}
      />
    </g>
  );
};

export default DPadButton;
