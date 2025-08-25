"use client";

import React, { useEffect, useState } from "react";
import { useMultiCameraStream } from "@/hooks/useMultiCameraStream";
import { useBandwidthStats } from "@/hooks/useBandwidthStats";
import { CAMERA_CONFIG } from "@/config/camera";
import { useCameraStore } from "@/store";

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
  fps?: number;
  resolution?: [number, number];
  backend_is_active?: boolean;
}

const MultiCameraView: React.FC = () => {
  const { 
    viewMode, 
    showCameraManager, 
    selectedCameraIndex, 
    multiCameraSlots,
    setSelectedCameraIndex 
  } = useCameraStore();

  // Camera management state - now handled in control tab
  const [cameraManagement, setCameraManagement] = useState<{
    availableCameras: AvailableCamera[];
    loading: boolean;
    error: string | null;
  }>({
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

  // Get active cameras in multi mode (from store slots)
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

  // Camera management functions (simplified - just for starting/stopping)
  const startCamera = async (cameraId: string) => {
    setCameraManagement(prev => ({ ...prev, loading: true }));
    try {
      const response = await fetch(`http://localhost:8000/cameras/${cameraId}/start`, {
        method: 'POST',
      });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      await fetchAvailableCameras(); // Refresh the list
    } catch (err) {
      setCameraManagement(prev => ({ 
        ...prev, 
        error: err instanceof Error ? err.message : 'Failed to start camera' 
      }));
      console.error('Error starting camera:', err);
    } finally {
      setCameraManagement(prev => ({ ...prev, loading: false }));
    }
  };

  const fetchAvailableCameras = async () => {
    setCameraManagement(prev => ({ ...prev, loading: true, error: null }));
    try {
      const response = await fetch('http://localhost:8000/cameras');
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setCameraManagement(prev => ({ 
        ...prev, 
        availableCameras: data.cameras || [] 
      }));
    } catch (err) {
      setCameraManagement(prev => ({ 
        ...prev, 
        error: err instanceof Error ? err.message : 'Failed to fetch cameras' 
      }));
      console.error('Error fetching cameras:', err);
    } finally {
      setCameraManagement(prev => ({ ...prev, loading: false }));
    }
  };

  // Calculate overall FPS for multi-view
  const overallFPS = activeCameras.reduce((total, slot) => {
    const streamState = slot.camera ? streamStates[slot.camera.camera_id] : null;
    return total + (streamState?.fps || 0);
  }, 0);

  // Check if selected camera is streaming
  const isStreaming = selectedCamera ? streamStates[selectedCamera.camera_id]?.isReceivingFrames || false : false;
  const isLive = selectedCamera ? streamStates[selectedCamera.camera_id]?.isConnected || false : false;

  // Get grid layout for multi-view
  const getGridLayout = (cameraCount: number) => {
    if (cameraCount <= 1) return 'grid-cols-1 grid-rows-1';
    if (cameraCount <= 2) return 'grid-cols-2 grid-rows-1';
    if (cameraCount <= 4) return 'grid-cols-2 grid-rows-2';
    if (cameraCount <= 6) return 'grid-cols-3 grid-rows-2';
    if (cameraCount <= 9) return 'grid-cols-3 grid-rows-3';
    return 'grid-cols-4 grid-rows-3';
  };

  // Refresh cameras periodically
  useEffect(() => {
    const interval = setInterval(() => {
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

  return (
    <div className="relative w-full h-screen bg-black flex flex-col">
      {/* Top Controls - Removed camera manager button (moved to control tab) */}

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
              FPS: {streamStates[selectedCamera.camera_id]?.fps || 0}
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
                    Use <strong>Camera Manager</strong> in control tab to start cameras
                  </p>
                </>
              ) : (
                <>
                  <p className="mb-2">
                    {isLoading ? 'Loading cameras...' : 'No cameras available'}
                  </p>
                  <p className="text-sm opacity-75">
                    Use <strong>Camera Manager</strong> in control tab to get started
                  </p>
                  {error && <p className="text-red-400 text-xs mt-2">Error: {error}</p>}
                </>
              )}
            </div>
          </div>
        )
      ) : (
        /* Multi Camera View */
        <div className="relative w-full h-full overflow-hidden">
          {activeCameras.length > 0 ? (
            <div className={`grid ${getGridLayout(activeCameras.length)} gap-2 p-2 h-full`}>
              {activeCameras.map((slot, index) => {
                const camera = slot.camera;
                if (!camera) return null;

                const streamState = streamStates[camera.camera_id];
                const isReceiving = streamState?.isReceivingFrames || false;
                const isConnected = streamState?.isConnected || false;

                return (
                  <div key={`${camera.camera_id}-${index}`} className="relative bg-gray-900 rounded-lg overflow-hidden">
                    <canvas
                      ref={(canvas) => registerCanvas(camera.camera_id, canvas)}
                      className="w-full h-full object-cover"
                    />

                    {/* Camera Info Overlay */}
                    <div className="absolute top-2 left-2 text-white text-xs bg-black/50 px-2 py-1 rounded">
                      <div className="font-medium">{camera.name}</div>
                      <div className="flex items-center gap-1">
                        <span
                          className={`inline-block w-2 h-2 rounded-full ${
                            isReceiving ? "bg-green-400" : "bg-red-500"
                          }`}
                        ></span>
                        <span>{isReceiving ? "Live" : "Offline"}</span>
                        {isReceiving && streamState?.fps && (
                          <span>• {streamState.fps.toFixed(1)} FPS</span>
                        )}
                      </div>
                    </div>

                    {/* Remove Button */}
                    <button
                      onClick={() => {
                        // Remove from slot (handled by store)
                        // This would need to be implemented in the store
                      }}
                      className="absolute top-2 right-2 w-6 h-6 bg-red-600 hover:bg-red-700 text-white rounded-full flex items-center justify-center text-xs transition-colors"
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
                  Use <strong>Camera Manager</strong> in control tab to start and add cameras
                </p>
              </div>
            </div>
          )}
        </div>
      )}

      {/* View Mode Toggle - Removed (moved to control tab) */}

      {/* Global Stats (Multi Mode) - Top Right */}
      {viewMode === 'multi' && activeCameras.length > 0 && (
        <div className="absolute top-4 right-4 text-white text-sm bg-gray-800/80 backdrop-blur-md px-4 py-2 rounded-lg text-right border border-gray-600/50">
          <div className="font-medium text-red-300 mb-1">Multi-View Stats</div>
          <div>Cameras: {activeCameras.length}</div>
          <div>Total FPS: {overallFPS.toFixed(1)}</div>
          <div>Bitrate: {bitrateKbps !== "0" ? `${bitrateKbps} kbps` : "N/A"}</div>
        </div>
      )}

      {/* Camera Management Panel - Simplified, only shows when toggled from control tab */}
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
                            deviceCameras.some(cam => cam.is_active) ? 'bg-green-600/30 text-green-300' : 'bg-gray-600/30 text-gray-400'
                          }`}>
                            {deviceCameras.some(cam => cam.is_active) ? 'Active' : 'Inactive'}
                          </span>
                        </div>
                      );
                    })}
                  </div>
                ) : (
                  <p className="text-gray-400 text-sm">No devices connected</p>
                )}
              </div>

              {/* All Cameras (Combined Available + Active) */}
              <div className="bg-gray-700/50 rounded-lg p-3 border border-gray-600/30">
                <h4 className="font-semibold mb-2 text-gray-200">All Cameras</h4>
                {cameraManagement.availableCameras.length > 0 ? (
                  <div className="space-y-2">
                    {cameraManagement.availableCameras.map(availableCamera => {
                      const isActive = availableCamera.is_active;
                      const streamState = streamStates[availableCamera.camera_id];
                      const isStreaming = streamState?.isReceivingFrames || false;
                      
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
                                {isStreaming ? 'Live' : 'Active'} • FPS: {availableCamera.fps || 0}
                              </div>
                            )}
                          </div>
                          <div className="flex gap-1">
                            {isActive ? (
                              <button
                                className="bg-blue-600 hover:bg-blue-700 text-white px-2 py-1 rounded text-xs transition-colors"
                                onClick={() => {
                                  if (viewMode === 'single') {
                                    const cameraIndex = cameraManagement.availableCameras.findIndex(cam => cam.camera_id === availableCamera.camera_id);
                                    if (cameraIndex !== -1) {
                                      setSelectedCameraIndex(cameraIndex);
                                      connectCamera(availableCamera.camera_id);
                                    }
                                  }
                                  // setShowCameraManager(false); // This line was removed from the new_code
                                }}
                              >
                                View
                              </button>
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
