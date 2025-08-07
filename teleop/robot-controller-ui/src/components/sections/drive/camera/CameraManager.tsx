"use client";

import React, { useState, useEffect } from "react";
import { CameraInfo } from "@/hooks/useMultiCameraStream";
import { CAMERA_CONFIG } from "@/config/camera";

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
  bitrate?: number;
}

interface CameraManagementState {
  availableCameras: AvailableCamera[];
  loading: boolean;
  error: string | null;
}

interface CameraManagerProps {
  isVisible: boolean;
  onClose: () => void;
  onStartCamera: (cameraId: string) => Promise<void>;
  onStopCamera: (cameraId: string) => Promise<void>;
  onConnectCamera: (cameraId: string) => void;
  onAddCameraToSlot: (camera: CameraInfo) => void;
  viewMode: 'single' | 'multi';
  cameras: CameraInfo[];
  multiCameraSlots: { camera: CameraInfo | null; isActive: boolean }[];
  streamStates: Record<string, any>;
}

const CameraManager: React.FC<CameraManagerProps> = ({
  isVisible,
  onClose,
  onStartCamera,
  onStopCamera,
  onConnectCamera,
  onAddCameraToSlot,
  viewMode,
  cameras,
  multiCameraSlots,
  streamStates,
}) => {
  const [cameraManagement, setCameraManagement] = useState<CameraManagementState>({
    availableCameras: [],
    loading: false,
    error: null,
  });

  const [cameraSettings, setCameraSettings] = useState<Record<string, { bitrate: number; fps: number }>>({});
  const [dynamicBitrates, setDynamicBitrates] = useState<Record<string, number>>({});

  // Fetch available cameras
  const fetchAvailableCameras = async () => {
    setCameraManagement(prev => ({ ...prev, loading: true, error: null }));
    
    try {
      const response = await fetch(`${CAMERA_CONFIG.BACKEND.API_BASE_URL}/api/cameras/available`);
      
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      
      const data = await response.json();
      setCameraManagement(prev => ({ 
        ...prev, 
        availableCameras: data.cameras || [], 
        loading: false 
      }));
    } catch (error) {
      console.error('Failed to fetch available cameras:', error);
      setCameraManagement(prev => ({ 
        ...prev, 
        error: `Failed to fetch cameras: ${error instanceof Error ? error.message : 'Unknown error'}`, 
        loading: false 
      }));
    }
  };

  // Dynamic bitrate update (no restart needed)
  const updateBitrateDynamic = async (cameraId: string, bitrate: number) => {
    try {
      const response = await fetch(`${CAMERA_CONFIG.BACKEND.API_BASE_URL}/api/cameras/${cameraId}/bitrate`, {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ bitrate }),
      });
      
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      
      const data = await response.json();
      if (data.success) {
        // Update local state immediately for instant feedback
        setDynamicBitrates(prev => ({
          ...prev,
          [cameraId]: bitrate
        }));
        
        // Show success feedback
        console.log(`Bitrate updated to ${bitrate} kbps for camera ${cameraId}`);
      } else {
        throw new Error(data.message || 'Failed to update bitrate');
      }
    } catch (error) {
      console.error('Failed to update bitrate dynamically:', error);
      setCameraManagement(prev => ({ 
        ...prev, 
        error: `Failed to update bitrate: ${error instanceof Error ? error.message : 'Unknown error'}` 
      }));
    }
  };

  // Update camera settings (bitrate/FPS) - legacy method with restart
  const updateCameraSettings = async (cameraId: string, settings: { bitrate?: number; fps?: number }) => {
    setCameraManagement(prev => ({ ...prev, loading: true, error: null }));
    
    try {
      const response = await fetch(`${CAMERA_CONFIG.BACKEND.API_BASE_URL}/api/cameras/${cameraId}/settings`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(settings),
      });
      
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      
      // Update local state
      setCameraSettings(prev => ({
        ...prev,
        [cameraId]: { ...prev[cameraId], ...settings }
      }));
      
      setCameraManagement(prev => ({ ...prev, loading: false }));
    } catch (error) {
      console.error('Failed to update camera settings:', error);
      setCameraManagement(prev => ({ 
        ...prev, 
        error: `Failed to update settings: ${error instanceof Error ? error.message : 'Unknown error'}`, 
        loading: false 
      }));
    }
  };

  useEffect(() => {
    if (isVisible) {
      fetchAvailableCameras();
    }
  }, [isVisible]);

  if (!isVisible) return null;

  const allAvailableCameras = cameraManagement.availableCameras;

  return (
    <div className="absolute top-20 left-4 right-4 bg-gray-800/95 backdrop-blur-md rounded-lg border border-gray-600/50 p-4 z-40 max-h-[60vh] overflow-y-auto shadow-2xl">
      <div className="text-white">
        <div className="flex justify-between items-center mb-4">
          <h3 className="text-lg font-bold text-gray-100">Camera Management</h3>
          <button
            onClick={onClose}
            className="text-gray-400 hover:text-white transition-colors"
          >
            ✕
          </button>
        </div>
        
        {cameraManagement.error && (
          <div className="bg-red-600/20 border border-red-500/50 text-red-200 p-3 rounded-lg mb-4">
            {cameraManagement.error}
          </div>
        )}
        
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
          {/* Connected Devices */}
          <div className="bg-gray-700/50 rounded-lg p-3 border border-gray-600/30">
            <h4 className="font-semibold mb-2 text-gray-200">Connected Devices</h4>
            {allAvailableCameras.length > 0 ? (
              <div className="space-y-2">
                {Array.from(new Set(allAvailableCameras.map(cam => cam.device_id))).map(deviceId => {
                  const deviceCameras = allAvailableCameras.filter(cam => cam.device_id === deviceId);
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

          {/* Camera List */}
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
                  
                  return (
                    <div key={availableCamera.camera_id} className="text-sm">
                      <div className="flex items-center justify-between mb-1">
                        <div>
                          <div className="font-medium text-gray-200">{availableCamera.name}</div>
                          <div className="font-mono text-xs text-gray-400">{availableCamera.camera_id}</div>
                        </div>
                        <div className="flex gap-1">
                          {isActive ? (
                            <>
                              <button
                                className="bg-red-600 hover:bg-red-700 text-white px-2 py-1 rounded text-xs transition-colors"
                                onClick={() => onStopCamera(availableCamera.camera_id)}
                                disabled={cameraManagement.loading}
                              >
                                Stop
                              </button>
                              <button
                                className="bg-blue-600 hover:bg-blue-700 text-white px-2 py-1 rounded text-xs transition-colors"
                                onClick={() => {
                                  if (viewMode === 'single') {
                                    onConnectCamera(availableCamera.camera_id);
                                  } else {
                                    const camera = cameras.find(cam => cam.camera_id === availableCamera.camera_id);
                                    if (camera && !isAlreadyInSlot) {
                                      onAddCameraToSlot(camera);
                                    }
                                  }
                                  onClose();
                                }}
                                disabled={viewMode === 'multi' && isAlreadyInSlot}
                              >
                                {viewMode === 'multi' && isAlreadyInSlot ? 'In Use' : 'View'}
                              </button>
                            </>
                          ) : (
                            <button
                              className="bg-green-600 hover:bg-green-700 text-white px-2 py-1 rounded text-xs transition-colors"
                              onClick={() => onStartCamera(availableCamera.camera_id)}
                              disabled={cameraManagement.loading}
                            >
                              Start
                            </button>
                          )}
                        </div>
                      </div>
                      
                      {isActive && (
                        <div className="text-xs text-gray-400 mb-2">
                          <span className={`inline-block w-2 h-2 rounded-full mr-1 ${
                            isStreaming ? 'bg-green-400' : 'bg-yellow-400'
                          }`}></span>
                          {isStreaming ? 'Live' : 'Active'} • FPS: {availableCamera.fps || 0}
                        </div>
                      )}
                    </div>
                  );
                })}
              </div>
            ) : (
              <p className="text-gray-400 text-sm">No cameras available</p>
            )}
          </div>

          {/* Dynamic Camera Settings */}
          <div className="bg-gray-700/50 rounded-lg p-3 border border-gray-600/30">
            <h4 className="font-semibold mb-2 text-gray-200">Dynamic Settings</h4>
            {allAvailableCameras.filter(cam => cam.is_active).length > 0 ? (
              <div className="space-y-4">
                {allAvailableCameras.filter(cam => cam.is_active).map(availableCamera => {
                  const currentBitrate = dynamicBitrates[availableCamera.camera_id] || availableCamera.bitrate || 512;
                  
                  return (
                    <div key={availableCamera.camera_id} className="text-sm">
                      <div className="font-medium text-gray-200 mb-3">{availableCamera.name}</div>
                      
                      {/* Dynamic Bitrate Control */}
                      <div className="mb-3">
                        <div className="flex items-center justify-between mb-1">
                          <label className="block text-xs text-gray-400">Bitrate (kbps)</label>
                          <span className="text-xs text-blue-300 font-mono">{currentBitrate}</span>
                        </div>
                        <div className="flex items-center gap-2">
                          <input
                            type="range"
                            min="128"
                            max="2048"
                            step="128"
                            value={currentBitrate}
                            onChange={(e) => {
                              const newBitrate = parseInt(e.target.value);
                              setDynamicBitrates(prev => ({
                                ...prev,
                                [availableCamera.camera_id]: newBitrate
                              }));
                            }}
                            onMouseUp={(e) => {
                              const newBitrate = parseInt((e.target as HTMLInputElement).value);
                              updateBitrateDynamic(availableCamera.camera_id, newBitrate);
                            }}
                            onKeyUp={(e) => {
                              if (e.key === 'Enter') {
                                const newBitrate = parseInt((e.target as HTMLInputElement).value);
                                updateBitrateDynamic(availableCamera.camera_id, newBitrate);
                              }
                            }}
                            className="flex-1 h-2 bg-gray-600 rounded-lg appearance-none cursor-pointer slider"
                          />
                          <div className="text-xs text-green-400 animate-pulse">⚡</div>
                        </div>
                        <div className="text-xs text-gray-500 mt-1">
                          Real-time update • No restart needed
                        </div>
                      </div>
                    </div>
                  );
                })}
              </div>
            ) : (
              <p className="text-gray-400 text-sm">No active cameras to configure</p>
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
  );
};

export default CameraManager; 