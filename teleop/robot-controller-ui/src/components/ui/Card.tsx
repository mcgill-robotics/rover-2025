// DriveControlCard.tsx
'use client';

import React from 'react';

interface ControlCardProps {
  title: string;
  icon?: React.ReactNode;
  className?: string;
  actions?: React.ReactNode;
  children: React.ReactNode;
  subtitle?: string;
  center?: boolean;
  bodyClassName?: string;
}

const Card: React.FC<ControlCardProps> = ({
  title,
  icon,
  className = '',
  actions,
  children,
  subtitle,
  center = false,
  bodyClassName = '',
}) => {
  return (
    <div
      className={`h-full w-full rounded-2xl border border-white/10 bg-neutral-900/60 backdrop-blur
        shadow-[0_10px_30px_-10px_rgba(0,0,0,0.6)] flex flex-col ${className}`}
    >
      <div className="flex items-center justify-between px-4 py-3 border-b border-white/10">
        <div className="flex items-center gap-2 min-w-0">
          {icon && <span className="shrink-0 text-red-400">{icon}</span>}
          <div className="min-w-0">
            <h3 className="text-sm font-semibold text-zinc-100 truncate">{title}</h3>
            {subtitle && <p className="text-[11px] text-zinc-400 truncate">{subtitle}</p>}
          </div>
        </div>
        {actions && <div className="flex items-center gap-1">{actions}</div>}
      </div>

      <div
        className={`flex-1 overflow-hidden p-3 sm:p-4 ${
          center ? 'flex items-center justify-center' : ''
        } ${bodyClassName}`}
      >
        {children}
      </div>
    </div>
  );
};

export default Card;
