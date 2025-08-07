"use client";

import React from "react";

interface ViewModeControlsProps {
  viewMode: 'single' | 'multi';
  onViewModeChange: (mode: 'single' | 'multi') => void;
  onToggleCameraManager: () => void;
  showCameraManager: boolean;
  activeCameraCount: number;
  overallFPS: number;
  bitrateKbps: string;
}

const ViewModeControls: React.FC<ViewModeControlsProps> = ({
  viewMode,
  onViewModeChange,
  onToggleCameraManager,
  showCameraManager,
  activeCameraCount,
  overallFPS,
  bitrateKbps,
}) => {
  return (
    <div className="absolute top-4 left-4 right-4 flex justify-between items-center z-30">
      {/* Left side - View Mode Controls */}
      <div className="flex items-center gap-2">
        <button
          onClick={() => onViewModeChange('single')}
          className={`px-3 py-1 rounded-lg text-sm font-medium transition-colors ${
            viewMode === 'single'
              ? 'bg-blue-600 text-white'
              : 'bg-gray-700/50 text-gray-300 hover:bg-gray-600/50'
          }`}
        >
          Single
        </button>
        <button
          onClick={() => onViewModeChange('multi')}
          className={`px-3 py-1 rounded-lg text-sm font-medium transition-colors ${
            viewMode === 'multi'
              ? 'bg-blue-600 text-white'
              : 'bg-gray-700/50 text-gray-300 hover:bg-gray-600/50'
          }`}
        >
          Multi
        </button>
      </div>

      {/* Right side - Camera Manager Button */}
      <button
        onClick={onToggleCameraManager}
        className={`px-4 py-2 rounded-lg text-sm font-medium transition-colors ${
          showCameraManager
            ? 'bg-red-600 hover:bg-red-700 text-white'
            : 'bg-green-600 hover:bg-green-700 text-white'
        }`}
      >
        {showCameraManager ? 'Close Manager' : 'Camera Manager'}
      </button>

      {/* Multi-view Stats (only show in multi mode) */}
      {viewMode === 'multi' && activeCameraCount > 0 && (
        <div className="absolute top-12 right-4 text-white text-sm bg-gray-800/80 backdrop-blur-md px-4 py-2 rounded-lg text-right border border-gray-600/50">
          <div className="font-medium text-red-300 mb-1">Multi-View Stats</div>
          <div>Cameras: {activeCameraCount}</div>
          <div>Total FPS: {overallFPS.toFixed(1)}</div>
          <div>Bitrate: {bitrateKbps !== "0" ? `${bitrateKbps} kbps` : "N/A"}</div>
        </div>
      )}
    </div>
  );
};

export default ViewModeControls; 