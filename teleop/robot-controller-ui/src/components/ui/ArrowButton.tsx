'use client';

import React from 'react';
import { cn } from '@/lib/utils';
import ArrowLeftIcon from '@/components/icons/ArrowLeftIcon';
import ArrowRightIcon from '@/components/icons/ArrowRightIcon';

interface ArrowButtonProps {
  direction: 'left' | 'right';
  onClick: () => void;
  className?: string;
  icon?: React.ReactNode;
}

const ArrowButton: React.FC<ArrowButtonProps> = ({
  direction,
  onClick,
  className = '',
  icon,
}) => {
  const positionClasses =
    direction === 'left'
      ? 'left-4'
      : 'right-4';

  return (
    <button
      onClick={onClick}
      className={cn(
        'absolute top-1/2 -translate-y-1/2 w-12 h-12 text-2xl rounded-full bg-black/50 text-white hover:bg-black/70 z-10 transition flex items-center justify-center',
        positionClasses,
        className
      )}
      aria-label={`${direction} arrow button`}
    >
      {icon ?? (direction === 'left' ? <ArrowLeftIcon /> : <ArrowRightIcon />)}
    </button>
  );
};

export default ArrowButton;
