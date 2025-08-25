'use client';

import React, { useState, useEffect } from 'react';
import { Camera, Settings, Eye, EyeOff } from 'lucide-react';
import { useCameraStore } from '@/store';

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

interface CameraManagerProps {
  onCameraStart?: (cameraId: string) => void;
  onCameraStop?: (cameraId: string) => void;
}

const CameraManager: React.FC<CameraManagerProps> = ({
  onCameraStart,
  onCameraStop
}) => {
  const { showCameraManager, setShowCameraManager } = useCameraStore();
  const [availableCameras, setAvailableCameras] = useState<AvailableCamera[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const fetchAvailableCameras = async () => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetch('http://localhost:8000/cameras');
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setAvailableCameras(data.cameras || []);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch cameras');
      console.error('Error fetching cameras:', err);
    } finally {
      setLoading(false);
    }
  };

  const startCamera = async (cameraId: string) => {
    setLoading(true);
    try {
      const response = await fetch(`http://localhost:8000/cameras/${cameraId}/start`, {
        method: 'POST',
      });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      await fetchAvailableCameras(); // Refresh the list
      onCameraStart?.(cameraId);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to start camera');
      console.error('Error starting camera:', err);
    } finally {
      setLoading(false);
    }
  };

  const stopCamera = async (cameraId: string) => {
    setLoading(true);
    try {
      const response = await fetch(`http://localhost:8000/cameras/${cameraId}/stop`, {
        method: 'POST',
      });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      await fetchAvailableCameras(); // Refresh the list
      onCameraStop?.(cameraId);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to stop camera');
      console.error('Error stopping camera:', err);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchAvailableCameras();
  }, []);

  return (
    <div className="w-full h-full flex flex-col">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center space-x-2">
          <Camera className="w-5 h-5 text-blue-600" />
          <h3 className="text-lg font-semibold">Camera Manager</h3>
        </div>
        <div className="flex items-center space-x-2">
          <button
            onClick={fetchAvailableCameras}
            disabled={loading}
            className="p-1 text-gray-600 hover:text-gray-800 transition-colors"
            title="Refresh cameras"
          >
            <Settings className="w-4 h-4" />
          </button>
          <button
            onClick={() => setShowCameraManager(!showCameraManager)}
            className={`p-2 rounded-lg transition-colors ${
              showCameraManager 
                ? 'bg-red-600 hover:bg-red-700 text-white' 
                : 'bg-green-600 hover:bg-green-700 text-white'
            }`}
            title={showCameraManager ? 'Hide camera panel' : 'Show camera panel'}
          >
            {showCameraManager ? <EyeOff className="w-4 h-4" /> : <Eye className="w-4 h-4" />}
          </button>
        </div>
      </div>

      {error && (
        <div className="mb-3 p-2 bg-red-100 border border-red-300 text-red-700 text-xs rounded">
          {error}
        </div>
      )}

      <div className="flex-1 overflow-y-auto space-y-2">
        {availableCameras.length === 0 ? (
          <div className="text-center text-gray-500 text-sm py-4">
            {loading ? 'Loading cameras...' : 'No cameras available'}
          </div>
        ) : (
          availableCameras.map((camera) => (
            <div
              key={camera.camera_id}
              className="bg-gray-50 border border-gray-200 rounded-lg p-3"
            >
              <div className="flex items-center justify-between">
                <div className="flex-1 min-w-0">
                  <h4 className="font-medium text-sm text-gray-900 truncate">
                    {camera.name}
                  </h4>
                  <p className="text-xs text-gray-500 truncate">
                    {camera.device_id}
                  </p>
                  <div className="flex items-center mt-1">
                    <span
                      className={`inline-block w-2 h-2 rounded-full mr-2 ${
                        camera.is_active ? 'bg-green-400' : 'bg-gray-400'
                      }`}
                    />
                    <span className="text-xs text-gray-600">
                      {camera.is_active ? 'Active' : 'Inactive'}
                    </span>
                    {camera.is_active && camera.fps && (
                      <span className="text-xs text-gray-600 ml-2">
                        • {camera.fps} FPS
                      </span>
                    )}
                  </div>
                </div>
                <div className="ml-2">
                  {camera.is_active ? (
                    <button
                      onClick={() => stopCamera(camera.camera_id)}
                      disabled={loading}
                      className="px-3 py-1 bg-red-600 hover:bg-red-700 text-white text-xs rounded transition-colors disabled:opacity-50"
                    >
                      Stop
                    </button>
                  ) : (
                    <button
                      onClick={() => startCamera(camera.camera_id)}
                      disabled={loading}
                      className="px-3 py-1 bg-green-600 hover:bg-green-700 text-white text-xs rounded transition-colors disabled:opacity-50"
                    >
                      Start
                    </button>
                  )}
                </div>
              </div>
            </div>
          ))
        )}
      </div>

      {loading && (
        <div className="mt-2 text-center">
          <div className="inline-flex items-center gap-2 text-blue-600 text-xs">
            <div className="animate-spin rounded-full h-3 w-3 border-b-2 border-blue-600"></div>
            Processing...
          </div>
        </div>
      )}
    </div>
  );
};

export default CameraManager;
