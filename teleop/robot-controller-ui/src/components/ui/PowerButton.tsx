'use client';

import React, { useState } from 'react';
import { cn } from '@/lib/utils';

interface PowerButtonProps {
  onClick?: () => void;
  size?: 'small' | 'medium' | 'large';
  isActive?: boolean;
  label?: string;
}

const sizeClasses = {
  small: 'w-[60px] h-[60px]',
  medium: 'w-[100px] h-[100px]',
  large: 'w-[150px] h-[150px]',
};

const labelSizeClasses = {
  small: 'text-[8px]',
  medium: 'text-[12px]',
  large: 'text-[16px] mt-2',
};

const PowerButton: React.FC<PowerButtonProps> = ({
  onClick,
  size = 'medium',
  isActive = false,
  label = 'POWER',
}) => {
  const [active, setActive] = useState(isActive);

  const handleClick = () => {
    setActive(!active);
    if (onClick) onClick();
  };

  return (
    <div className="w-full flex-1 flex justify-center items-center my-6">
      <button
        onClick={handleClick}
        aria-label="Power button"
        className={cn(
          'relative rounded-full border-none cursor-pointer flex items-center justify-center p-0 transition-all overflow-hidden',
          'shadow-[0_4px_8px_rgba(0,0,0,0.5),inset_0_2px_3px_rgba(255,255,255,0.1)]',
          sizeClasses[size],
          active
            ? 'bg-[#333] shadow-[0_2px_4px_rgba(0,0,0,0.5),inset_0_1px_2px_rgba(0,0,0,0.8)]'
            : 'bg-[#2a2a2a]'
        )}
        style={{
          transform: active ? 'scale(0.95)' : undefined,
        }}
      >
        {/* Outer ring */}
        <div className="absolute inset-0 rounded-full z-20 shadow-[inset_0_0_0_2px_rgba(255,255,255,0.05)]" />

        {/* Gradient inner background */}
        <div
          className={cn(
            'absolute top-[5%] left-[5%] right-[5%] bottom-[5%] rounded-full z-10',
            active
              ? 'bg-[linear-gradient(145deg,#222222,#2a2a2a)]'
              : 'bg-[linear-gradient(145deg,#333333,#222222)]'
          )}
        />

        {/* Button content */}
        <div className="relative flex flex-col items-center justify-center z-30 w-full h-full">
          <svg
            className={cn(
              'mb-1 transition-colors',
              active
                ? 'text-[#d63f3f] hover:text-[#f08c8c]'
                : 'text-[#666] hover:text-[#999]'
            )}
            xmlns="http://www.w3.org/2000/svg"
            width="32"
            height="32"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2.5"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <line x1="12" y1="2" x2="12" y2="12" />
            <path d="M17.657 6.343a8 8 0 1 1-11.314 0" />
          </svg>
          <div
            className={cn(
              'font-bold uppercase tracking-wider transition-colors',
              labelSizeClasses[size],
              active
                ? 'text-[#d63f3f] hover:text-[#f08c8c]'
                : 'text-[#666] hover:text-[#999]'
            )}
          >
            {label}
          </div>
        </div>
      </button>
    </div>
  );
};

export default PowerButton;