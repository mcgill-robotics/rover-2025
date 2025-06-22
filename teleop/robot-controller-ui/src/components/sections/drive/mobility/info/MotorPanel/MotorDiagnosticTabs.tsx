'use client';

import React, { useState } from 'react';
import MotorPanel from './MotorPanel';
import { BarChart3, PanelLeft } from 'lucide-react'; // or any icon set
import ChartPanel from '../ChartPanel/ChartPanel';

interface Props {
  motorStats: {
    name: string;
    speed: number;
    pos: number;
    volt: number;
    curr: number;
    temp: number;
    alert?: string;
  }[];
}

const MotorDiagnosticsTabs: React.FC<Props> = ({ motorStats }) => {
  const [view, setView] = useState<'overview' | 'charts'>('overview');
  const MOTOR_PANEL_HEIGHT = 'h-[360px]'; 

  return (
    <div className="w-full rounded-xl shadow-md bg-[#18181b] text-white p-2">
      {/* Header */}
      <div className="flex justify-between items-center mb-2 px-1">
        <h2 className="text-sm font-semibold">Motor Diagnostics</h2>
        <button
          onClick={() => setView(view === 'charts' ? 'overview' : 'charts')}
          className="p-1 hover:text-white text-white/60 transition"
          title={view === 'charts' ? 'Show Overview' : 'Show Charts'}
        >
          {view === 'charts' ? <PanelLeft className="w-5 h-5" /> : <BarChart3 className="w-5 h-5" />}
        </button>
      </div>

      {/* View Container (same height) */}
      <div className={`${MOTOR_PANEL_HEIGHT} w-full`}>
        {view === 'overview' ? (
          <div className="grid grid-cols-2 gap-3 h-full">
            {motorStats.map((motor) => (
              <MotorPanel key={motor.name} {...motor} />
            ))}
          </div>
        ) : (
          <div className="h-full">
            <ChartPanel />
          </div>
        )}
      </div>
    </div>
  );
};

export default MotorDiagnosticsTabs;
