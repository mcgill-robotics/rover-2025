"use client";

import React from "react";
import { CameraInfo } from "@/hooks/useMultiCameraStream";

interface CameraSlot {
  camera: CameraInfo | null;
  isActive: boolean;
}

interface CameraGridProps {
  cameraSlots: CameraSlot[];
  onRemoveCamera: (slotIndex: number) => void;
  registerCanvas: (cameraId: string, canvasRef: HTMLCanvasElement) => void;
  streamStates: Record<string, any>;
}

const CameraGrid: React.FC<CameraGridProps> = ({
  cameraSlots,
  onRemoveCamera,
  registerCanvas,
  streamStates,
}) => {
  const getGridLayout = (cameraCount: number) => {
    switch (cameraCount) {
      case 1:
        return "grid-cols-1 grid-rows-1";
      case 2:
        return "grid-cols-2 grid-rows-1";
      case 3:
        return "grid-cols-2 grid-rows-2";
      case 4:
        return "grid-cols-2 grid-rows-2";
      default:
        return "grid-cols-2 grid-rows-2";
    }
  };

  const activeCameras = cameraSlots.filter(slot => slot.camera && slot.isActive);
  const gridLayout = getGridLayout(activeCameras.length);

  return (
    <div className={`grid ${gridLayout} gap-2 h-full`}>
      {cameraSlots.map((slot, index) => {
        if (!slot.camera || !slot.isActive) return null;

        const streamState = streamStates[slot.camera.camera_id];
        const isStreaming = streamState?.isReceivingFrames || false;

        return (
          <div
            key={`${slot.camera.camera_id}-${index}`}
            className="relative bg-black rounded-lg overflow-hidden border border-gray-600/50"
          >
            {/* Camera Canvas */}
            <canvas
              ref={(canvas) => {
                if (canvas) {
                  registerCanvas(slot.camera!.camera_id, canvas);
                }
              }}
              className="w-full h-full object-contain"
              style={{ imageRendering: 'pixelated' }}
            />

            {/* Camera Info Overlay */}
            <div className="absolute top-2 left-2 bg-black/70 backdrop-blur-sm text-white text-xs px-2 py-1 rounded">
              <div className="font-medium">{slot.camera.name}</div>
              <div className="text-gray-300">{slot.camera.camera_id}</div>
            </div>

            {/* Status Indicator */}
            <div className="absolute top-2 right-2">
              <div className={`w-3 h-3 rounded-full ${
                isStreaming ? 'bg-green-400' : 'bg-yellow-400'
              }`}></div>
            </div>

            {/* Remove Button */}
            <button
              onClick={() => onRemoveCamera(index)}
              className="absolute bottom-2 right-2 bg-red-600 hover:bg-red-700 text-white w-6 h-6 rounded-full text-xs flex items-center justify-center transition-colors"
            >
              ✕
            </button>

            {/* Stream Stats */}
            {isStreaming && streamState && (
              <div className="absolute bottom-2 left-2 bg-black/70 backdrop-blur-sm text-white text-xs px-2 py-1 rounded">
                <div>FPS: {streamState.fps?.toFixed(1) || 'N/A'}</div>
                <div>Latency: {streamState.latency?.toFixed(0) || 'N/A'}ms</div>
              </div>
            )}
          </div>
        );
      })}
    </div>
  );
};

export default CameraGrid; 