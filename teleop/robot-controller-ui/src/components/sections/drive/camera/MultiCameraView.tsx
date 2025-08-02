"use client";

import React, { useEffect, useState } from "react";
import { useMultiCameraStream, CameraInfo } from "@/hooks/useMultiCameraStream";
import { useBandwidthStats } from "@/hooks/useBandwidthStats";
import { CAMERA_CONFIG } from "@/config/camera";

import DPad from "./DPadController";
import PowerButton from "@/components/ui/PowerButton";
import ArrowButton from "@/components/ui/ArrowButton";

interface AvailableCamera {
  device_id: string;
  host: string;
  last_heartbeat: number;
  status: string;
}

interface CameraManagementState {
  availableCameras: AvailableCamera[];
  jetsonDevices: string[];
  loading: boolean;
  error: string | null;
}

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
  const [showCameraManager, setShowCameraManager] = useState(false);
  
  // Camera management state
  const [cameraManagement, setCameraManagement] = useState<CameraManagementState>({
    availableCameras: [],
    jetsonDevices: [],
    loading: false,
    error: null,
  });

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

  // Camera management functions
  const fetchAvailableCameras = async () => {
    setCameraManagement(prev => ({ ...prev, loading: true, error: null }));
    
    try {
      const backendUrl = CAMERA_CONFIG.BACKEND.WEBSOCKET_URL.replace('ws://', 'http://');
      const response = await fetch(`${backendUrl}/api/cameras/available`);
      
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      
      const data = await response.json();
      setCameraManagement(prev => ({
        ...prev,
        availableCameras: data.available_cameras || [],
        jetsonDevices: data.jetson_devices || [],
        loading: false,
      }));
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to fetch available cameras';
      setCameraManagement(prev => ({ ...prev, error: errorMessage, loading: false }));
      console.error('Failed to fetch available cameras:', err);
    }
  };

  const startCamera = async (cameraId: string) => {
    setCameraManagement(prev => ({ ...prev, loading: true, error: null }));
    
    try {
      const backendUrl = CAMERA_CONFIG.BACKEND.WEBSOCKET_URL.replace('ws://', 'http://');
      const response = await fetch(`${backendUrl}/api/cameras/${cameraId}/start`, {
        method: 'POST',
      });
      
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      
      const data = await response.json();
      if (!data.success) {
        throw new Error(data.error || 'Failed to start camera');
      }
      
      console.log(`Camera ${cameraId} start command sent successfully`);
      
      // Refresh cameras after a short delay
      setTimeout(() => {
        fetchCameras();
      }, 2000);
      
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : `Failed to start camera ${cameraId}`;
      setCameraManagement(prev => ({ ...prev, error: errorMessage }));
      console.error(`Failed to start camera ${cameraId}:`, err);
    } finally {
      setCameraManagement(prev => ({ ...prev, loading: false }));
    }
  };

  const stopCamera = async (cameraId: string) => {
    setCameraManagement(prev => ({ ...prev, loading: true, error: null }));
    
    try {
      const backendUrl = CAMERA_CONFIG.BACKEND.WEBSOCKET_URL.replace('ws://', 'http://');
      const response = await fetch(`${backendUrl}/api/cameras/${cameraId}/stop`, {
        method: 'POST',
      });
      
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      
      const data = await response.json();
      if (!data.success) {
        throw new Error(data.error || 'Failed to stop camera');
      }
      
      console.log(`Camera ${cameraId} stop command sent successfully`);
      
      // Refresh cameras after a short delay
      setTimeout(() => {
        fetchCameras();
      }, 1000);
      
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : `Failed to stop camera ${cameraId}`;
      setCameraManagement(prev => ({ ...prev, error: errorMessage }));
      console.error(`Failed to stop camera ${cameraId}:`, err);
    } finally {
      setCameraManagement(prev => ({ ...prev, loading: false }));
    }
  };

  // Generate potential camera IDs based on connected Jetson devices
  const generatePotentialCameras = () => {
    const potentialCameras: string[] = [];
    
    cameraManagement.jetsonDevices.forEach(deviceId => {
      // Assume each device can have up to 3 cameras (cam00, cam01, cam02)
      for (let i = 0; i < 3; i++) {
        potentialCameras.push(`${deviceId}-cam${i.toString().padStart(2, '0')}`);
      }
    });
    
    return potentialCameras;
  };

  // Refresh cameras periodically
  useEffect(() => {
    const interval = setInterval(() => {
      fetchCameras();
      fetchAvailableCameras();
    }, 10000); // Refresh every 10 seconds
    return () => clearInterval(interval);
  }, [fetchCameras]);

  // Initial fetch of available cameras
  useEffect(() => {
    fetchAvailableCameras();
  }, []);

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

        {/* Camera Manager Toggle */}
        <button
          onClick={() => setShowCameraManager(!showCameraManager)}
          className="bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded-lg backdrop-blur-md transition-colors"
        >
          {showCameraManager ? 'Hide Manager' : 'Manage Cameras'}
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

      {/* Camera Management Panel */}
      {showCameraManager && (
        <div className="absolute top-20 left-4 right-4 bg-black/90 backdrop-blur-md rounded-lg border border-white/20 p-4 z-40 max-h-[60vh] overflow-y-auto">
          <div className="text-white">
            <h3 className="text-lg font-bold mb-4">Camera Management</h3>
            
            {cameraManagement.error && (
              <div className="bg-red-600/20 border border-red-600 text-red-200 p-3 rounded-lg mb-4">
                {cameraManagement.error}
              </div>
            )}
            
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
              {/* Connected Devices */}
              <div className="bg-white/5 rounded-lg p-3">
                <h4 className="font-semibold mb-2">Connected Devices</h4>
                {cameraManagement.availableCameras.length > 0 ? (
                  <div className="space-y-2">
                    {cameraManagement.availableCameras.map(camera => (
                      <div key={camera.device_id} className="flex items-center justify-between text-sm">
                        <div>
                          <div className="font-medium">{camera.device_id}</div>
                          <div className="text-gray-400">({camera.host})</div>
                        </div>
                        <span className={`px-2 py-1 rounded text-xs ${
                          camera.status === 'connected' 
                            ? 'bg-green-600 text-green-100' 
                            : 'bg-red-600 text-red-100'
                        }`}>
                          {camera.status}
                        </span>
                      </div>
                    ))}
                  </div>
                ) : (
                  <p className="text-gray-400 text-sm">No Jetson devices connected</p>
                )}
              </div>

              {/* Available Cameras */}
              <div className="bg-white/5 rounded-lg p-3">
                <h4 className="font-semibold mb-2">Available Cameras</h4>
                {generatePotentialCameras().length > 0 ? (
                  <div className="space-y-2">
                    {generatePotentialCameras().map(cameraId => {
                      const isActive = cameras.some(cam => cam.camera_id === cameraId);
                      return (
                        <div key={cameraId} className="flex items-center justify-between text-sm">
                          <span className="font-mono">{cameraId}</span>
                          <div className="flex gap-1">
                            {isActive ? (
                              <button
                                className="bg-red-600 hover:bg-red-700 text-white px-2 py-1 rounded text-xs transition-colors"
                                onClick={() => stopCamera(cameraId)}
                                disabled={cameraManagement.loading}
                              >
                                Stop
                              </button>
                            ) : (
                              <button
                                className="bg-green-600 hover:bg-green-700 text-white px-2 py-1 rounded text-xs transition-colors"
                                onClick={() => startCamera(cameraId)}
                                disabled={cameraManagement.loading}
                              >
                                Start
                              </button>
                            )}
                            {isActive && (
                              <button
                                className="bg-blue-600 hover:bg-blue-700 text-white px-2 py-1 rounded text-xs transition-colors"
                                onClick={() => {
                                  if (viewMode === 'single') {
                                    const cameraIndex = cameras.findIndex(cam => cam.camera_id === cameraId);
                                    if (cameraIndex !== -1) {
                                      setSelectedCameraIndex(cameraIndex);
                                    }
                                  } else {
                                    const camera = cameras.find(cam => cam.camera_id === cameraId);
                                    if (camera) {
                                      addCameraToSlot(camera);
                                    }
                                  }
                                  setShowCameraManager(false);
                                }}
                              >
                                View
                              </button>
                            )}
                          </div>
                        </div>
                      );
                    })}
                  </div>
                ) : (
                  <p className="text-gray-400 text-sm">No cameras available</p>
                )}
              </div>

              {/* Currently Active Cameras */}
              <div className="bg-white/5 rounded-lg p-3">
                <h4 className="font-semibold mb-2">Active Cameras</h4>
                {cameras.length > 0 ? (
                  <div className="space-y-2">
                    {cameras.map(camera => (
                      <div key={camera.camera_id} className="text-sm">
                        <div className="flex items-center justify-between">
                          <div>
                            <div className="font-medium">{camera.name}</div>
                            <div className="text-gray-400 font-mono text-xs">({camera.camera_id})</div>
                          </div>
                          <div className="flex gap-1">
                            <button
                              className="bg-red-600 hover:bg-red-700 text-white px-2 py-1 rounded text-xs transition-colors"
                              onClick={() => stopCamera(camera.camera_id)}
                              disabled={cameraManagement.loading}
                            >
                              Stop
                            </button>
                            <button
                              className="bg-blue-600 hover:bg-blue-700 text-white px-2 py-1 rounded text-xs transition-colors"
                              onClick={() => {
                                if (viewMode === 'single') {
                                  const cameraIndex = cameras.findIndex(cam => cam.camera_id === camera.camera_id);
                                  if (cameraIndex !== -1) {
                                    setSelectedCameraIndex(cameraIndex);
                                  }
                                } else {
                                  addCameraToSlot(camera);
                                }
                                setShowCameraManager(false);
                              }}
                            >
                              View
                            </button>
                          </div>
                        </div>
                        {streamStates[camera.camera_id] && (
                          <div className="text-xs text-gray-400 mt-1">
                            <span className={`inline-block w-2 h-2 rounded-full mr-1 ${
                              streamStates[camera.camera_id].isReceivingFrames ? 'bg-green-400' : 'bg-red-400'
                            }`}></span>
                            {streamStates[camera.camera_id].isReceivingFrames ? 'Live' : 'No signal'} • 
                            FPS: {streamStates[camera.camera_id].fps || 0}
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                ) : (
                  <p className="text-gray-400 text-sm">No active cameras</p>
                )}
              </div>
            </div>
            
            {cameraManagement.loading && (
              <div className="mt-4 text-center">
                <div className="inline-flex items-center gap-2 text-blue-400">
                  <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-blue-400"></div>
                  Processing camera command...
                </div>
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default MultiCameraView;
