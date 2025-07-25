"use client";

import React, { useEffect, useState } from "react";
import { useMultiCameraStream, CameraInfo } from "@/hooks/useMultiCameraStream";
import { useBandwidthStats } from "@/hooks/useBandwidthStats";
import { CAMERA_CONFIG } from "@/config/camera";

import DPad from "./DPadController";
import PowerButton from "@/components/ui/PowerButton";
import ArrowButton from "@/components/ui/ArrowButton";

type ViewMode = 'single' | 'multi';

interface CameraSlot {
  camera: CameraInfo | null;
  isActive: boolean;
}

const MultiCameraView: React.FC = () => {
  const [viewMode, setViewMode] = useState<ViewMode>('single');
  const [selectedCameraIndex, setSelectedCameraIndex] = useState(0);
  const [multiCameraSlots, setMultiCameraSlots] = useState<CameraSlot[]>([
    { camera: null, isActive: false },
    { camera: null, isActive: false },
    { camera: null, isActive: false },
    { camera: null, isActive: false }
  ]);
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [availableCameraIndex, setAvailableCameraIndex] = useState(0);

  const {
    cameras,
    streamStates,
    isLoading,
    error,
    fetchCameras,
    connectCamera,
    disconnectCamera,
    registerCanvas,
  } = useMultiCameraStream({
    backendUrl: CAMERA_CONFIG.BACKEND.WEBSOCKET_URL,
  });

  const { bitrateKbps, pingMs } = useBandwidthStats(
    Object.values(streamStates).some(state => state.isReceivingFrames)
  );

  // Get currently selected camera for single mode
  const selectedCamera = cameras[selectedCameraIndex] || null;

  // Get active cameras in multi mode
  const activeCameras = multiCameraSlots.filter(slot => slot.camera && slot.isActive);

  // Handle single camera navigation
  const handleNext = () => {
    if (cameras.length === 0) return;
    const nextIndex = (selectedCameraIndex + 1) % cameras.length;
    setSelectedCameraIndex(nextIndex);
  };

  const handlePrevious = () => {
    if (cameras.length === 0) return;
    const prevIndex = (selectedCameraIndex - 1 + cameras.length) % cameras.length;
    setSelectedCameraIndex(prevIndex);
  };

  // Handle multi-camera slot management
  const addCameraToSlot = (camera: CameraInfo) => {
    const emptySlotIndex = multiCameraSlots.findIndex(slot => !slot.camera);
    if (emptySlotIndex === -1) return; // No empty slots

    const newSlots = [...multiCameraSlots];
    newSlots[emptySlotIndex] = { camera, isActive: true };
    setMultiCameraSlots(newSlots);
    connectCamera(camera.camera_id);
  };

  const removeCameraFromSlot = (slotIndex: number) => {
    const slot = multiCameraSlots[slotIndex];
    if (slot.camera) {
      disconnectCamera(slot.camera.camera_id);
      const newSlots = [...multiCameraSlots];
      newSlots[slotIndex] = { camera: null, isActive: false };
      setMultiCameraSlots(newSlots);
    }
  };

  // Handle arrow navigation in multi mode
  const handleMultiModeNext = () => {
    if (cameras.length === 0) return;
    const nextIndex = (availableCameraIndex + 1) % cameras.length;
    setAvailableCameraIndex(nextIndex);
  };

  const handleMultiModePrevious = () => {
    if (cameras.length === 0) return;
    const prevIndex = (availableCameraIndex - 1 + cameras.length) % cameras.length;
    setAvailableCameraIndex(prevIndex);
  };

  const addCurrentAvailableCamera = () => {
    const camera = cameras[availableCameraIndex];
    if (camera && !multiCameraSlots.some(slot => slot.camera?.camera_id === camera.camera_id)) {
      addCameraToSlot(camera);
    }
  };

  // Start/stop streaming
  const handleStartStop = () => {
    if (viewMode === 'single') {
      if (selectedCamera) {
        const streamState = streamStates[selectedCamera.camera_id];
        if (streamState?.isConnected) {
          disconnectCamera(selectedCamera.camera_id);
        } else {
          connectCamera(selectedCamera.camera_id);
        }
      }
    } else {
      // Multi mode - toggle all active cameras
      const hasActiveStreams = activeCameras.some(slot => 
        streamStates[slot.camera!.camera_id]?.isConnected
      );
      
      if (hasActiveStreams) {
        // Stop all
        activeCameras.forEach(slot => {
          if (slot.camera) {
            disconnectCamera(slot.camera.camera_id);
          }
        });
      } else {
        // Start all
        activeCameras.forEach(slot => {
          if (slot.camera) {
            connectCamera(slot.camera.camera_id);
          }
        });
      }
    }
  };

  // Get grid layout class based on number of active cameras
  const getGridLayout = (activeCameraCount: number) => {
    switch (activeCameraCount) {
      case 1: return "grid-cols-1 grid-rows-1";
      case 2: return "grid-cols-2 grid-rows-1";
      case 3: return "grid-cols-2 grid-rows-2";
      case 4: return "grid-cols-2 grid-rows-2";
      default: return "grid-cols-1 grid-rows-1";
    }
  };

  // Calculate individual camera size for 3-camera layout
  const getCameraSize = (index: number, total: number) => {
    if (total === 3) {
      return index === 0 ? "col-span-2" : "col-span-1";
    }
    return "";
  };

  // Check if streaming
  const isStreaming = viewMode === 'single' 
    ? selectedCamera && streamStates[selectedCamera.camera_id]?.isConnected
    : activeCameras.some(slot => slot.camera && streamStates[slot.camera.camera_id]?.isConnected);

  // Get overall FPS for display
  const overallFPS = viewMode === 'single' && selectedCamera
    ? streamStates[selectedCamera.camera_id]?.fps || 0
    : activeCameras.reduce((sum, slot) => {
        if (slot.camera) {
          return sum + (streamStates[slot.camera.camera_id]?.fps || 0);
        }
        return sum;
      }, 0);

  // Check if any camera is live
  const isLive = viewMode === 'single' && selectedCamera
    ? streamStates[selectedCamera.camera_id]?.isReceivingFrames || false
    : activeCameras.some(slot => 
        slot.camera && streamStates[slot.camera.camera_id]?.isReceivingFrames
      );

  // Refresh cameras periodically
  useEffect(() => {
    const interval = setInterval(fetchCameras, 10000); // Refresh every 10 seconds
    return () => clearInterval(interval);
  }, [fetchCameras]);

  return (
    <div className="relative w-full h-screen bg-black flex flex-col">
      {/* Top Controls */}
      <div className="absolute top-4 left-1/2 transform -translate-x-1/2 z-30 flex items-center gap-4">
        {/* View Mode Toggle */}
        <button
          onClick={() => setViewMode(viewMode === 'single' ? 'multi' : 'single')}
          className="bg-white/10 hover:bg-white/20 text-white px-4 py-2 rounded-lg backdrop-blur-md transition-colors"
        >
          {viewMode === 'single' ? 'Multi View' : 'Single View'}
        </button>

        {/* Camera Dropdown (Multi Mode) */}
        {viewMode === 'multi' && (
          <div className="relative">
            <button
              onClick={() => setIsDropdownOpen(!isDropdownOpen)}
              className="bg-white/10 hover:bg-white/20 text-white px-4 py-2 rounded-lg backdrop-blur-md transition-colors min-w-[200px] text-left"
            >
              {cameras[availableCameraIndex]?.name || 'No cameras available'}
              <span className="float-right">▼</span>
            </button>
            
            {isDropdownOpen && (
              <div className="absolute top-full mt-2 left-0 bg-black/90 backdrop-blur-md rounded-lg border border-white/20 min-w-[200px] max-h-60 overflow-y-auto z-40">
                {cameras.map((camera, index) => {
                  const isAlreadySelected = multiCameraSlots.some(slot => 
                    slot.camera?.camera_id === camera.camera_id
                  );
                  
                  return (
                    <button
                      key={camera.camera_id}
                      onClick={() => {
                        setAvailableCameraIndex(index);
                        setIsDropdownOpen(false);
                        if (!isAlreadySelected) {
                          addCameraToSlot(camera);
                        }
                      }}
                      disabled={isAlreadySelected}
                      className={`w-full text-left px-4 py-2 hover:bg-white/10 transition-colors ${
                        isAlreadySelected ? 'text-gray-500 cursor-not-allowed' : 'text-white'
                      } ${index === availableCameraIndex ? 'bg-white/10' : ''}`}
                    >
                      {camera.name}
                      {isAlreadySelected && <span className="float-right">✓</span>}
                    </button>
                  );
                })}
              </div>
            )}
          </div>
        )}

        {/* Add Camera Button (Multi Mode) */}
        {viewMode === 'multi' && cameras[availableCameraIndex] && (
          <button
            onClick={addCurrentAvailableCamera}
            disabled={multiCameraSlots.every(slot => slot.camera) || 
                     multiCameraSlots.some(slot => slot.camera?.camera_id === cameras[availableCameraIndex]?.camera_id)}
            className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 disabled:cursor-not-allowed text-white px-4 py-2 rounded-lg transition-colors"
          >
            Add Camera
          </button>
        )}
      </div>

      {/* Main Content Area */}
      {viewMode === 'single' ? (
        /* Single Camera View */
        selectedCamera && isStreaming ? (
          <div className="relative w-full h-full overflow-hidden">
            <canvas
              ref={(canvas) => registerCanvas(selectedCamera.camera_id, canvas)}
              className="w-full h-full object-cover"
            />

            <div className="absolute top-4 left-4 text-white text-base bg-black/50 px-4 py-2 rounded-lg">
              <strong>{selectedCamera.name}</strong><br />
              <span
                className={`inline-block w-2.5 h-2.5 rounded-full mr-2 ${
                  isLive ? "bg-lime-400" : "bg-red-500"
                }`}
              ></span>
              Live
            </div>

            <div className="absolute top-4 right-4 text-white text-sm bg-black/50 px-4 py-2 rounded-lg text-right">
              FPS: {overallFPS}
              <br />
              Bitrate: {bitrateKbps !== "0" ? `${bitrateKbps} kbps` : "N/A"}
              <br />
              Ping: {pingMs !== null ? `${pingMs} ms` : "N/A"}
            </div>

            <ArrowButton direction="left" onClick={handlePrevious} />
            <ArrowButton direction="right" onClick={handleNext} />

            {selectedCamera.name.includes("USB 2.0 Camera") && (
              <div className="absolute bottom-4 right-8 w-[100px] h-[100px]">
                <DPad inputStream="up" />
              </div>
            )}
          </div>
        ) : (
          <div className="absolute inset-0 flex flex-col justify-center items-center bg-black/60 text-white text-center z-10">
            <p className="bg-white/10 px-6 py-4 rounded-xl font-medium backdrop-blur-md">
              {selectedCamera ? (
                <>
                  No stream yet.
                  <br />
                  Click <strong>Start</strong> to begin.
                </>
              ) : (
                <>
                  {isLoading ? 'Loading cameras...' : 'No cameras available'}
                  {error && <><br />Error: {error}</>}
                </>
              )}
            </p>
          </div>
        )
      ) : (
        /* Multi Camera View */
        <div className="relative w-full h-full p-4">
          {activeCameras.length > 0 ? (
            <div className={`grid ${getGridLayout(activeCameras.length)} gap-4 h-full`}>
              {multiCameraSlots.map((slot, index) => {
                if (!slot.camera || !slot.isActive) return null;
                
                const streamState = streamStates[slot.camera.camera_id];
                const isConnected = streamState?.isConnected || false;
                const isReceiving = streamState?.isReceivingFrames || false;
                const fps = streamState?.fps || 0;
                
                return (
                  <div
                    key={slot.camera.camera_id}
                    className={`relative bg-gray-900 rounded-lg overflow-hidden ${getCameraSize(index, activeCameras.length)}`}
                  >
                    {isConnected ? (
                      <canvas
                        ref={(canvas) => registerCanvas(slot.camera!.camera_id, canvas)}
                        className="w-full h-full object-cover"
                      />
                    ) : (
                      <div className="w-full h-full flex items-center justify-center text-white">
                        <div className="text-center">
                          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-white mx-auto mb-2"></div>
                          <p>Connecting...</p>
                        </div>
                      </div>
                    )}

                    {/* Camera Info Overlay */}
                    <div className="absolute top-2 left-2 text-white text-sm bg-black/50 px-2 py-1 rounded">
                      <div className="font-medium">{slot.camera.name}</div>
                      <div className="flex items-center gap-2">
                        <span
                          className={`inline-block w-2 h-2 rounded-full ${
                            isReceiving ? "bg-lime-400" : "bg-red-500"
                          }`}
                        ></span>
                        <span>FPS: {fps}</span>
                      </div>
                    </div>

                    {/* Remove Camera Button */}
                    <button
                      onClick={() => removeCameraFromSlot(index)}
                      className="absolute top-2 right-2 bg-red-600 hover:bg-red-700 text-white w-6 h-6 rounded-full flex items-center justify-center text-sm transition-colors"
                    >
                      ×
                    </button>

                    {/* Waiting State */}
                    {!isReceiving && isConnected && (
                      <div className="absolute inset-0 flex items-center justify-center bg-black/50 text-white">
                        <p>Waiting for camera...</p>
                      </div>
                    )}
                  </div>
                );
              })}
            </div>
          ) : (
            <div className="flex items-center justify-center h-full text-white text-center">
              <div className="bg-white/10 px-6 py-4 rounded-xl backdrop-blur-md">
                <p className="font-medium mb-2">No cameras selected</p>
                <p className="text-sm opacity-75">
                  Use the dropdown above to add cameras to the grid
                </p>
              </div>
            </div>
          )}

          {/* Arrow Navigation for Multi Mode */}
          {viewMode === 'multi' && (
            <>
              <ArrowButton direction="left" onClick={handleMultiModePrevious} />
              <ArrowButton direction="right" onClick={handleMultiModeNext} />
            </>
          )}
        </div>
      )}

      {/* Global Stats (Multi Mode) */}
      {viewMode === 'multi' && activeCameras.length > 0 && (
        <div className="absolute top-4 right-4 text-white text-sm bg-black/50 px-4 py-2 rounded-lg text-right">
          Cameras: {activeCameras.length}
          <br />
          Total FPS: {overallFPS.toFixed(1)}
          <br />
          Bitrate: {bitrateKbps !== "0" ? `${bitrateKbps} kbps` : "N/A"}
        </div>
      )}

      {/* Power Button */}
      <div className="absolute bottom-2 left-1/2 -translate-x-1/2 z-20">
        <PowerButton
          onClick={handleStartStop}
          size="small"
          isActive={!!isStreaming}
          label="Start"
        />
      </div>

      {/* Refresh Button */}
      <div className="absolute bottom-2 right-4 z-20">
        <button
          onClick={fetchCameras}
          disabled={isLoading}
          className="bg-white/10 hover:bg-white/20 disabled:opacity-50 text-white p-2 rounded-lg backdrop-blur-md transition-colors"
        >
          {isLoading ? '⟳' : '↻'}
        </button>
      </div>
    </div>
  );
};

export default MultiCameraView;
