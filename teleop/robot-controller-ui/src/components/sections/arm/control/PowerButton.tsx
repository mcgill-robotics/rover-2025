'use client';

import React, { useState } from 'react';

interface PowerButtonProps {
  onClick?: () => void;
  size?: 'small' | 'medium' | 'large';
  isActive?: boolean;
  label?: string;
}

const sizeMap = {
  small: 'w-[60px] h-[60px] text-[8px]',
  medium: 'w-[100px] h-[100px] text-[12px]',
  large: 'w-[150px] h-[150px] text-[16px] mt-2',
};

const PowerButton: React.FC<PowerButtonProps> = ({
  onClick,
  size = 'medium',
  isActive = false,
  label = 'POWER'
}) => {
  const [active, setActive] = useState(isActive);

  const handleClick = () => {
    setActive(!active);
    onClick?.();
  };

  const sizeClass = sizeMap[size];

  return (
    <div className="w-full flex-1 flex justify-center items-center my-6">
      <button
        onClick={handleClick}
        aria-label="Power button"
        className={`relative rounded-full border-none cursor-pointer flex items-center justify-center overflow-hidden transition-all duration-300 ease-in-out shadow-[0_4px_8px_rgba(0,0,0,0.5),inset_0_2px_3px_rgba(255,255,255,0.1)] active:scale-95 group ${
          sizeClass
        } ${active ? 'bg-[#333] shadow-[0_2px_4px_rgba(0,0,0,0.5),inset_0_1px_2px_rgba(0,0,0,0.8)]' : 'bg-[#2a2a2a]'}`}
      >
        {/* Outer glow ring */}
        <div className="absolute inset-0 rounded-full z-10 shadow-[inset_0_0_0_2px_rgba(255,255,255,0.05)]"></div>

        {/* Inner background */}
        <div className={`absolute inset-[5%] rounded-full z-0 ${
          active
            ? 'bg-[linear-gradient(145deg,#222222,#2a2a2a)]'
            : 'bg-[linear-gradient(145deg,#333333,#222222)]'
        }`}></div>

        {/* Content */}
        <div className="relative z-20 flex flex-col items-center justify-center w-full h-full">
          <svg
            className={`mb-1 transition-colors duration-300 ${
              active ? 'text-[#d63f3f]' : 'text-[#666] group-hover:text-[#999] group-active:text-[#f08c8c]'
            }`}
            xmlns="http://www.w3.org/2000/svg"
            width="32" height="32" viewBox="0 0 24 24"
            fill="none" stroke="currentColor" strokeWidth="2.5"
            strokeLinecap="round" strokeLinejoin="round"
          >
            <line x1="12" y1="2" x2="12" y2="12" />
            <path d="M17.657 6.343a8 8 0 1 1-11.314 0" />
          </svg>
          <div className={`uppercase tracking-wider font-bold transition-colors duration-300 ${
            active ? 'text-[#d63f3f] group-hover:text-[#f08c8c]' : 'text-[#666] group-hover:text-[#999]'
          }`}>
            {label}
          </div>
        </div>
      </button>
    </div>
  );
};

export default PowerButton;
