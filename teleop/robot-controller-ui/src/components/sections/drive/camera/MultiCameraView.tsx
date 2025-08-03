"use client";

import React, { useEffect, useState } from "react";
import { useMultiCameraStream, CameraInfo } from "@/hooks/useMultiCameraStream";
import { useBandwidthStats } from "@/hooks/useBandwidthStats";
import { CAMERA_CONFIG } from "@/config/camera";

import DPad from "./DPadController";
import ArrowButton from "@/components/ui/ArrowButton";

interface AvailableCamera {
  camera_id: string;
  device_id: string;
  name: string;
  device_path: string;
  is_active: boolean;
  rtp_port: number | null;
  host: string;
  last_heartbeat: number;
  status: string;
}

interface CameraManagementState {
  availableCameras: AvailableCamera[];
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
  const [multiCameraSlots, setMultiCameraSlots] = useState<CameraSlot[]>([]);
  const [showCameraManager, setShowCameraManager] = useState(false);
  
  // Camera management state
  const [cameraManagement, setCameraManagement] = useState<CameraManagementState>({
    availableCameras: [],
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

  // Get all available cameras from the API
  const allAvailableCameras = cameraManagement.availableCameras;
  
  // Get currently selected camera for single mode (from all available cameras)
  const selectedAvailableCamera = allAvailableCameras[selectedCameraIndex] || null;
  const selectedCamera = selectedAvailableCamera ? cameras.find(cam => cam.camera_id === selectedAvailableCamera.camera_id) : null;

  // Get active cameras in multi mode (dynamic slots)
  const activeCameras = multiCameraSlots.filter(slot => slot.camera && slot.isActive);

  // Handle single camera navigation - cycle through ALL available cameras
  const handleNext = async () => {
    if (allAvailableCameras.length === 0) return;
    const nextIndex = (selectedCameraIndex + 1) % allAvailableCameras.length;
    setSelectedCameraIndex(nextIndex);
    
    // Auto-start camera if it's not active
    const nextCamera = allAvailableCameras[nextIndex];
    if (nextCamera && !nextCamera.is_active) {
      await startCamera(nextCamera.camera_id);
    }
  };

  const handlePrevious = async () => {
    if (allAvailableCameras.length === 0) return;
    const prevIndex = (selectedCameraIndex - 1 + allAvailableCameras.length) % allAvailableCameras.length;
    setSelectedCameraIndex(prevIndex);
    
    // Auto-start camera if it's not active
    const prevCamera = allAvailableCameras[prevIndex];
    if (prevCamera && !prevCamera.is_active) {
      await startCamera(prevCamera.camera_id);
    }
  };

  // Handle multi-camera slot management (dynamic)
  const addCameraToSlot = (camera: CameraInfo) => {
    // Check if camera is already in a slot
    const isAlreadyInSlot = multiCameraSlots.some(slot => 
      slot.camera?.camera_id === camera.camera_id
    );
    if (isAlreadyInSlot) return;

    // Add camera to a new slot
    const newSlot: CameraSlot = { camera, isActive: true };
    setMultiCameraSlots(prev => [...prev, newSlot]);
    connectCamera(camera.camera_id);
  };

  const removeCameraFromSlot = (slotIndex: number) => {
    const slot = multiCameraSlots[slotIndex];
    if (slot.camera) {
      disconnectCamera(slot.camera.camera_id);
      // Remove the slot entirely (dynamic)
      setMultiCameraSlots(prev => prev.filter((_, index) => index !== slotIndex));
    }
  };

  // Get grid layout class based on number of cameras
  const getGridLayout = (cameraCount: number) => {
    switch (cameraCount) {
      case 1: return "grid-cols-1 grid-rows-1";
      case 2: return "grid-cols-2 grid-rows-1";
      case 3: return "grid-cols-2 grid-rows-2";
      case 4: return "grid-cols-2 grid-rows-2";
      case 5: return "grid-cols-3 grid-rows-2";
      case 6: return "grid-cols-3 grid-rows-2";
      default: return "grid-cols-3 grid-rows-3";
    }
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

  // Refresh cameras periodically - more frequent to prevent disappearing
  useEffect(() => {
    const interval = setInterval(() => {
      fetchCameras();
      fetchAvailableCameras();
    }, 5000); // Refresh every 5 seconds instead of 10
    return () => clearInterval(interval);
  }, [fetchCameras]);

  // Initial fetch of available cameras
  useEffect(() => {
    fetchAvailableCameras();
    // Also fetch immediately after a short delay to ensure we get the data
    const timeout = setTimeout(() => {
      fetchAvailableCameras();
    }, 1000);
    return () => clearTimeout(timeout);
  }, []);

  // Refresh available cameras when the camera manager is opened
  useEffect(() => {
    if (showCameraManager) {
      fetchAvailableCameras();
    }
  }, [showCameraManager]);

  return (
    <div className="relative w-full h-screen bg-black flex flex-col">
      {/* Top Controls */}
      <div className="absolute top-4 left-1/2 transform -translate-x-1/2 z-30 flex items-center gap-4">
        {/* Camera Manager Toggle */}
        <button
          onClick={() => setShowCameraManager(!showCameraManager)}
          className="bg-green-600/50 hover:bg-green-600 text-white px-4 py-2 rounded-lg backdrop-blur-md transition-all duration-300"
        >
          {showCameraManager ? 'Hide Manager' : 'Manage Cameras'}
        </button>
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
              <div className="flex items-center gap-2">
                <span
                  className={`inline-block w-2.5 h-2.5 rounded-full ${
                    isLive ? "bg-lime-400" : "bg-red-500"
                  }`}
                ></span>
                <span>Live</span>
                <span className="text-sm opacity-75">
                  ({selectedCameraIndex + 1} of {allAvailableCameras.length})
                </span>
              </div>
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
            <div className="bg-white/10 px-6 py-4 rounded-xl font-medium backdrop-blur-md">
              {selectedCamera ? (
                <>
                  <p className="mb-2">No stream yet.</p>
                  <p className="text-sm opacity-75">
                    Open <strong>Camera Manager</strong> to start cameras
                  </p>
                </>
              ) : (
                <>
                  <p className="mb-2">
                    {isLoading ? 'Loading cameras...' : 'No cameras available'}
                  </p>
                  <p className="text-sm opacity-75">
                    Click <strong>Manage Cameras</strong> to get started
                  </p>
                  {error && <p className="text-red-400 text-xs mt-2">Error: {error}</p>}
                </>
              )}
            </div>
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
                    className="relative bg-gray-900 rounded-lg overflow-hidden"
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

                    {/* Remove Camera Button - Improved visibility */}
                    <button
                      onClick={() => removeCameraFromSlot(index)}
                      className="absolute top-2 right-2 bg-red-600 hover:bg-red-700 text-white w-8 h-8 rounded-full flex items-center justify-center text-lg font-bold transition-colors shadow-lg border-2 border-white/20 hover:border-white/40 z-10"
                      title={`Remove ${slot.camera.name}`}
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
                <p className="font-medium mb-2">No cameras in multi-view</p>
                <p className="text-sm opacity-75">
                  Open <strong>Camera Manager</strong> to start and add cameras
                </p>
              </div>
            </div>
          )}
        </div>
      )}

      {/* View Mode Toggle - Bottom Left */}
      <div className="absolute bottom-4 left-4 z-30">
        <div className="bg-gray-800/90 backdrop-blur-lg rounded-xl p-1.5 border border-gray-600/50 shadow-2xl">
          <div className="flex items-center gap-1">
            <button
              onClick={() => setViewMode('single')}
              className={`relative px-5 py-2.5 rounded-lg text-sm font-semibold transition-all duration-300 ${
                viewMode === 'single'
                  ? 'bg-gradient-to-r from-red-500 to-red-600 text-white shadow-lg shadow-red-500/25 scale-105'
                  : 'text-gray-400 hover:text-white hover:bg-gray-700/50 hover:scale-102'
              }`}
            >
              <span className="relative z-10">Single</span>
              {viewMode === 'single' && (
                <div className="absolute inset-0 bg-gradient-to-r from-red-400 to-red-500 rounded-lg blur-sm opacity-50"></div>
              )}
            </button>
            <button
              onClick={() => setViewMode('multi')}
              className={`relative px-5 py-2.5 rounded-lg text-sm font-semibold transition-all duration-300 ${
                viewMode === 'multi'
                  ? 'bg-gradient-to-r from-red-500 to-red-600 text-white shadow-lg shadow-red-500/25 scale-105'
                  : 'text-gray-400 hover:text-white hover:bg-gray-700/50 hover:scale-102'
              }`}
            >
              <span className="relative z-10">Multi</span>
              {viewMode === 'multi' && (
                <div className="absolute inset-0 bg-gradient-to-r from-red-400 to-red-500 rounded-lg blur-sm opacity-50"></div>
              )}
            </button>
          </div>
        </div>
      </div>

      {/* Global Stats (Multi Mode) - Top Right */}
      {viewMode === 'multi' && activeCameras.length > 0 && (
        <div className="absolute top-4 right-4 text-white text-sm bg-gray-800/80 backdrop-blur-md px-4 py-2 rounded-lg text-right border border-gray-600/50">
          <div className="font-medium text-red-300 mb-1">Multi-View Stats</div>
          <div>Cameras: {activeCameras.length}</div>
          <div>Total FPS: {overallFPS.toFixed(1)}</div>
          <div>Bitrate: {bitrateKbps !== "0" ? `${bitrateKbps} kbps` : "N/A"}</div>
        </div>
      )}


      {/* Camera Management Panel */}
      {showCameraManager && (
        <div className="absolute top-20 left-4 right-4 bg-gray-800/95 backdrop-blur-md rounded-lg border border-gray-600/50 p-4 z-40 max-h-[60vh] overflow-y-auto shadow-2xl">
          <div className="text-white">
            <h3 className="text-lg font-bold mb-4 text-gray-100">Camera Management</h3>
            
            {cameraManagement.error && (
              <div className="bg-red-600/20 border border-red-500/50 text-red-200 p-3 rounded-lg mb-4">
                {cameraManagement.error}
              </div>
            )}
            
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              {/* Connected Devices */}
              <div className="bg-gray-700/50 rounded-lg p-3 border border-gray-600/30">
                <h4 className="font-semibold mb-2 text-gray-200">Connected Devices</h4>
                {cameraManagement.availableCameras.length > 0 ? (
                  <div className="space-y-2">
                    {/* Group cameras by device_id to avoid duplicate keys */}
                    {Array.from(new Set(cameraManagement.availableCameras.map(cam => cam.device_id))).map(deviceId => {
                      const deviceCameras = cameraManagement.availableCameras.filter(cam => cam.device_id === deviceId);
                      const firstCamera = deviceCameras[0];
                      return (
                        <div key={deviceId} className="flex items-center justify-between text-sm">
                          <div>
                            <div className="font-medium text-gray-200">{deviceId}</div>
                            <div className="text-gray-400">({firstCamera.host}) - {deviceCameras.length} cameras</div>
                          </div>
                          <span className={`px-2 py-1 rounded text-xs ${
                            firstCamera.status === 'connected' 
                              ? 'bg-green-600 text-green-100' 
                              : 'bg-red-600 text-red-100'
                          }`}>
                            {firstCamera.status}
                          </span>
                        </div>
                      );
                    })}
                  </div>
                ) : (
                  <p className="text-gray-400 text-sm">No Jetson devices connected</p>
                )}
              </div>

              {/* All Cameras (Combined Available + Active) */}
              <div className="bg-gray-700/50 rounded-lg p-3 border border-gray-600/30">
                <h4 className="font-semibold mb-2 text-gray-200">All Cameras</h4>
                {allAvailableCameras.length > 0 ? (
                  <div className="space-y-2">
                    {allAvailableCameras.map(availableCamera => {
                      const isActive = availableCamera.is_active;
                      const isAlreadyInSlot = multiCameraSlots.some(slot => 
                        slot.camera?.camera_id === availableCamera.camera_id
                      );
                      const streamState = streamStates[availableCamera.camera_id];
                      const isStreaming = streamState?.isReceivingFrames || false;
                      const fps = streamState?.fps || 0;
                      
                      return (
                        <div key={availableCamera.camera_id} className="flex items-center justify-between text-sm">
                          <div>
                            <div className="font-medium text-gray-200">{availableCamera.name}</div>
                            <div className="font-mono text-xs text-gray-400">{availableCamera.camera_id}</div>
                            {isActive && (
                              <div className="text-xs text-gray-400 mt-1">
                                <span className={`inline-block w-2 h-2 rounded-full mr-1 ${
                                  isStreaming ? 'bg-green-400' : 'bg-yellow-400'
                                }`}></span>
                                {isStreaming ? 'Live' : 'Active'} • FPS: {fps}
                              </div>
                            )}
                          </div>
                          <div className="flex gap-1">
                            {isActive ? (
                              <>
                                <button
                                  className="bg-red-600 hover:bg-red-700 text-white px-2 py-1 rounded text-xs transition-colors"
                                  onClick={() => stopCamera(availableCamera.camera_id)}
                                  disabled={cameraManagement.loading}
                                >
                                  Stop
                                </button>
                                <button
                                  className="bg-blue-600 hover:bg-blue-700 text-white px-2 py-1 rounded text-xs transition-colors"
                                  onClick={() => {
                                    if (viewMode === 'single') {
                                      const cameraIndex = allAvailableCameras.findIndex(cam => cam.camera_id === availableCamera.camera_id);
                                      if (cameraIndex !== -1) {
                                        setSelectedCameraIndex(cameraIndex);
                                      }
                                    } else {
                                      const camera = cameras.find(cam => cam.camera_id === availableCamera.camera_id);
                                      if (camera && !isAlreadyInSlot) {
                                        addCameraToSlot(camera);
                                      }
                                    }
                                    setShowCameraManager(false);
                                  }}
                                  disabled={viewMode === 'multi' && isAlreadyInSlot}
                                >
                                  {viewMode === 'multi' && isAlreadyInSlot ? 'In Use' : 'View'}
                                </button>
                              </>
                            ) : (
                              <button
                                className="bg-green-600 hover:bg-green-700 text-white px-2 py-1 rounded text-xs transition-colors"
                                onClick={() => startCamera(availableCamera.camera_id)}
                                disabled={cameraManagement.loading}
                              >
                                Start
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
