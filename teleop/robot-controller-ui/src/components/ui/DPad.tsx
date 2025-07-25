import React from 'react';
import DPadButton from './DPadButton';

interface DPadProps {
  activeDirection: 'up' | 'down' | 'left' | 'right' | null;
}

const DPad: React.FC<DPadProps> = ({ activeDirection }) => {
  return (
    <div className="w-full h-full">
      <svg
        width="100%"
        height="100%"
        viewBox="0 0 100 100"
        xmlns="http://www.w3.org/2000/svg"
      >
        {['up', 'down', 'left', 'right'].map((dir) => (
          <DPadButton
            key={dir}
            direction={dir as 'up' | 'down' | 'left' | 'right'}
            active={activeDirection === dir}
          />
        ))}
      </svg>
    </div>
  );
};

export default DPad;
