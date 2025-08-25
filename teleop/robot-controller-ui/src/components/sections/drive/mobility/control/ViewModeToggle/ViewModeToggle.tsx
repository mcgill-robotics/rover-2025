'use client';

import React from 'react';
import { Monitor, Grid3X3 } from 'lucide-react';

interface ViewModeToggleProps {
  viewMode: 'single' | 'multi';
  onViewModeChange: (mode: 'single' | 'multi') => void;
}

const ViewModeToggle: React.FC<ViewModeToggleProps> = ({
  viewMode,
  onViewModeChange,
}) => {
  return (
    <div className="w-full h-full flex flex-col">
      <div className="flex items-center space-x-2 mb-4">
        <Monitor className="w-5 h-5 text-blue-600" />
        <h3 className="text-lg font-semibold">View Mode</h3>
      </div>

      <div className="flex-1 flex flex-col justify-center">
        <div className="bg-gray-100 rounded-lg p-1 border border-gray-200">
          <div className="flex">
            <button
              onClick={() => onViewModeChange('single')}
              className={`flex-1 flex items-center justify-center space-x-2 py-3 px-4 rounded-md text-sm font-medium transition-all duration-200 ${
                viewMode === 'single'
                  ? 'bg-white text-blue-600 shadow-sm border border-gray-200'
                  : 'text-gray-600 hover:text-gray-800 hover:bg-gray-50'
              }`}
            >
              <Monitor className="w-4 h-4" />
              <span>Single</span>
            </button>
            <button
              onClick={() => onViewModeChange('multi')}
              className={`flex-1 flex items-center justify-center space-x-2 py-3 px-4 rounded-md text-sm font-medium transition-all duration-200 ${
                viewMode === 'multi'
                  ? 'bg-white text-blue-600 shadow-sm border border-gray-200'
                  : 'text-gray-600 hover:text-gray-800 hover:bg-gray-50'
              }`}
            >
              <Grid3X3 className="w-4 h-4" />
              <span>Multi</span>
            </button>
          </div>
        </div>

        <div className="mt-4 text-center">
          <p className="text-xs text-gray-500">
            {viewMode === 'single' 
              ? 'View one camera at a time with navigation controls'
              : 'View multiple cameras simultaneously in a grid layout'
            }
          </p>
        </div>
      </div>
    </div>
  );
};

export default ViewModeToggle;
