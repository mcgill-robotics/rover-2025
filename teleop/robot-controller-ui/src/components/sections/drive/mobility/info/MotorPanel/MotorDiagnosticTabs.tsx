import React from 'react';
import MotorPanel from './MotorPanel';

interface Props {
  activeTab: 'overview' | 'details' | 'charts';
  setActiveTab: (tab: 'overview' | 'details' | 'charts') => void;
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

const MotorDiagnosticsTabs: React.FC<Props> = ({ activeTab, setActiveTab, motorStats }) => {
  const tabs: ('overview' | 'details' | 'charts')[] = ['overview', 'details', 'charts'];

  return (
    <div className="w-full max-w-[600px] bg-[#18181b] rounded-xl shadow-md">
      <div className="flex flex-col items-center">

        {/* Tab switcher (compact full-width style) */}
        <div className="w-full flex">
          {tabs.map((tab) => (
            <button
              key={tab}
              className={`flex-1 py-2 text-sm font-medium border-b-2 transition-all
                ${activeTab === tab
                  ? 'bg-[#2a2a33] text-white border-white'
                  : 'bg-[#18181b] text-white/60 border-transparent hover:text-white'}`}
              onClick={() => setActiveTab(tab)}
            >
              {tab.charAt(0).toUpperCase() + tab.slice(1)}
            </button>
          ))}
        </div>

        {/* Tab content */}
        <div className="w-full p-4">
          {activeTab === 'overview' && (
            <div className="grid grid-cols-2 gap-4 w-full">
              {motorStats.map((motor) => (
                <MotorPanel key={motor.name} {...motor} />
              ))}
            </div>
          )}
          {activeTab === 'details' && (
            <div className="text-center text-white/60 text-sm py-10">[Details View Placeholder]</div>
          )}
          {activeTab === 'charts' && (
            <div className="text-center text-white/60 text-sm py-10">[Charts View Placeholder]</div>
          )}
        </div>
      </div>
    </div>
  );
};

export default MotorDiagnosticsTabs;
