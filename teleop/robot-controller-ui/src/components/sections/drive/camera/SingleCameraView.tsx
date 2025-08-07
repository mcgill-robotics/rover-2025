"use client";

import React from "react";
import { CameraInfo } from "@/hooks/useMultiCameraStream";
import DPad from "./DPadController";
import ArrowButton from "@/components/ui/ArrowButton";

interface SingleCameraViewProps {
  selectedCamera: CameraInfo | null;
  registerCanvas: (cameraId: string, canvasRef: HTMLCanvasElement) => void;
  streamStates: Record<string, any>;
  onNext: () => void;
  onPrevious: () => void;
  hasMultipleCameras: boolean;
}

const SingleCameraView: React.FC<SingleCameraViewProps> = ({
  selectedCamera,
  registerCanvas,
  streamStates,
  onNext,
  onPrevious,
  hasMultipleCameras,
}) => {
  if (!selectedCamera) {
    return (
      <div className="flex items-center justify-center h-full bg-gray-900 text-gray-400">
        <div className="text-center">
          <div className="text-6xl mb-4">📷</div>
          <div className="text-xl font-medium mb-2">No Camera Selected</div>
          <div className="text-sm">Use the camera manager to start a camera</div>
        </div>
      </div>
    );
  }

  const streamState = streamStates[selectedCamera.camera_id];
  const isStreaming = streamState?.isReceivingFrames || false;

  return (
    <div className="relative h-full bg-black rounded-lg overflow-hidden border border-gray-600/50">
      {/* Camera Canvas */}
      <canvas
        ref={(canvas) => {
          if (canvas) {
            registerCanvas(selectedCamera.camera_id, canvas);
          }
        }}
        className="w-full h-full object-contain"
        style={{ imageRendering: 'pixelated' }}
      />

      {/* Camera Info Overlay */}
      <div className="absolute top-4 left-4 bg-black/70 backdrop-blur-sm text-white px-3 py-2 rounded-lg">
        <div className="font-medium text-sm">{selectedCamera.name}</div>
        <div className="text-xs text-gray-300 font-mono">{selectedCamera.camera_id}</div>
        {isStreaming && streamState && (
          <div className="text-xs text-gray-300 mt-1">
            <div>FPS: {streamState.fps?.toFixed(1) || 'N/A'}</div>
            <div>Latency: {streamState.latency?.toFixed(0) || 'N/A'}ms</div>
          </div>
        )}
      </div>

      {/* Status Indicator */}
      <div className="absolute top-4 right-4">
        <div className={`w-4 h-4 rounded-full ${
          isStreaming ? 'bg-green-400' : 'bg-yellow-400'
        }`}></div>
      </div>

      {/* Navigation Controls */}
      {hasMultipleCameras && (
        <>
          <ArrowButton
            direction="left"
            onClick={onPrevious}
            className="absolute left-4 top-1/2 transform -translate-y-1/2"
          />
          <ArrowButton
            direction="right"
            onClick={onNext}
            className="absolute right-4 top-1/2 transform -translate-y-1/2"
          />
        </>
      )}

      {/* DPad Controller (if applicable) */}
      {selectedCamera.name.includes("USB 2.0 Camera") && (
        <div className="absolute bottom-4 right-4">
          <DPad cameraId={selectedCamera.camera_id} />
        </div>
      )}

      {/* Stream Stats */}
      {isStreaming && streamState && (
        <div className="absolute bottom-4 left-4 bg-black/70 backdrop-blur-sm text-white text-sm px-3 py-2 rounded-lg">
          <div className="font-medium mb-1">Stream Stats</div>
          <div className="text-xs space-y-1">
            <div>FPS: {streamState.fps?.toFixed(1) || 'N/A'}</div>
            <div>Latency: {streamState.latency?.toFixed(0) || 'N/A'}ms</div>
            <div>Frame Count: {streamState.frameCount || 0}</div>
            <div>Resolution: {streamState.width || 0}x{streamState.height || 0}</div>
          </div>
        </div>
      )}
    </div>
  );
};

export default SingleCameraView; 