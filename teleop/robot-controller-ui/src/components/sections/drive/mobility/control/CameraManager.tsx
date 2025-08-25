'use client';

import React, { useEffect, useState } from 'react';
import { Camera, Settings, Monitor, Grid3X3 } from 'lucide-react';
import Card from '@/components/ui/Card';
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
  const { viewMode, setViewMode } = useCameraStore(); // keep only view mode in store
  const [availableCameras, setAvailableCameras] = useState<AvailableCamera[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showSettings, setShowSettings] = useState(false);

  const fetchAvailableCameras = async () => {
    setLoading(true);
    setError(null);
    try {
      const res = await fetch('http://localhost:8000/cameras');
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const data = await res.json();
      setAvailableCameras(data.cameras || []);
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to fetch cameras');
      console.error(e);
    } finally {
      setLoading(false);
    }
  };

  const startCamera = async (cameraId: string) => {
    setLoading(true);
    try {
      const res = await fetch(`http://localhost:8000/cameras/${cameraId}/start`, { method: 'POST' });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      await fetchAvailableCameras();
      onCameraStart?.(cameraId);
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to start camera');
      console.error(e);
    } finally {
      setLoading(false);
    }
  };

  const stopCamera = async (cameraId: string) => {
    setLoading(true);
    try {
      const res = await fetch(`http://localhost:8000/cameras/${cameraId}/stop`, { method: 'POST' });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      await fetchAvailableCameras();
      onCameraStop?.(cameraId);
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to stop camera');
      console.error(e);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => { fetchAvailableCameras(); }, []);

  return (
    <Card
      title="Camera Manager"
      icon={<Camera className="w-5 h-5" />}
      actions={
        <>
          <button
            onClick={() => setShowSettings(true)}
            className="h-8 w-8 grid place-items-center rounded-lg border border-white/10 text-zinc-300 hover:text-white hover:bg-white/5 disabled:opacity-50"
            title="Refresh"
          >
            <Settings className="w-4 h-4" />
          </button>
        </>
      }
    >
      {error && (
        <div className="mb-3 rounded-lg border border-red-500/30 bg-red-500/10 px-3 py-2 text-[12px] text-red-300">
          {error}
        </div>
      )}

      <div className="h-full overflow-y-auto pr-1 space-y-2">
        {availableCameras.length === 0 ? (
          <div className="text-center text-zinc-400 text-sm py-6">
            {loading ? 'Loading cameras…' : 'No cameras available'}
          </div>
        ) : (
          availableCameras.map((cam) => (
            <div key={cam.camera_id} className="rounded-xl border border-white/10 bg-white/5 px-3 py-2">
              <div className="flex items-center justify-between gap-3">
                <div className="min-w-0">
                  <div className="flex items-center gap-2">
                    <div className={`h-2 w-2 rounded-full ${cam.is_active ? 'bg-emerald-400' : 'bg-zinc-500'}`} />
                    <h4 className="text-sm font-medium text-zinc-100 truncate">{cam.name}</h4>
                  </div>
                  <p className="text-[11px] text-zinc-400 truncate">{cam.device_id}</p>
                  <p className="text-[11px] text-zinc-500">
                    {cam.is_active ? 'Active' : 'Inactive'}
                    {cam.is_active && cam.fps ? <> • {cam.fps} FPS</> : null}
                  </p>
                </div>
                <div className="shrink-0">
                  {cam.is_active ? (
                    <button
                      onClick={() => stopCamera(cam.camera_id)}
                      disabled={loading}
                      className="px-3 py-1.5 rounded-lg text-xs bg-red-600 hover:bg-red-700 text-white disabled:opacity-50"
                    >
                      Stop
                    </button>
                  ) : (
                    <button
                      onClick={() => startCamera(cam.camera_id)}
                      disabled={loading}
                      className="px-3 py-1.5 rounded-lg text-xs bg-emerald-600 hover:bg-emerald-700 text-white disabled:opacity-50"
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
        <div className="mt-3 text-center text-xs text-red-300 flex items-center justify-center gap-2">
          <span className="h-3 w-3 rounded-full border-2 border-red-400 border-b-transparent animate-spin" />
          Processing…
        </div>
      )}

      {/* Settings Modal */}
      {showSettings && (
        <div className="fixed inset-0 z-50 grid place-items-center bg-black/60">
          <div className="w-full max-w-md rounded-2xl border border-white/10 bg-neutral-900/90 backdrop-blur p-4 shadow-2xl">
            <div className="flex items-center justify-between pb-3 border-b border-white/10">
              <div className="flex items-center gap-2 text-zinc-100">
                <Settings className="w-4 h-4 text-red-400" />
                <span className="text-sm font-semibold">Camera Settings</span>
              </div>
              <button
                onClick={() => setShowSettings(false)}
                className="h-8 w-8 grid place-items-center rounded-lg hover:bg-white/5 text-zinc-300"
                aria-label="Close"
              >
                ✕
              </button>
            </div>

            <div className="py-4 space-y-4">
              {/* View mode toggle moved here */}
              <div>
                <p className="text-xs text-zinc-400 mb-2">View Mode</p>
                <div className="grid grid-cols-2 gap-2">
                  <button
                    onClick={() => setViewMode('single')}
                    className={`flex items-center justify-center gap-2 rounded-lg px-4 py-3 text-sm transition-all
                      ${viewMode === 'single'
                        ? 'bg-neutral-900 text-red-300 border border-red-500/30 shadow-inner'
                        : 'text-zinc-300 hover:text-white hover:bg-white/10'}`}
                  >
                    <Monitor className="w-4 h-4" />
                    <span>Single</span>
                  </button>
                  <button
                    onClick={() => setViewMode('multi')}
                    className={`flex items-center justify-center gap-2 rounded-lg px-4 py-3 text-sm transition-all
                      ${viewMode === 'multi'
                        ? 'bg-neutral-900 text-red-300 border border-red-500/30 shadow-inner'
                        : 'text-zinc-300 hover:text-white hover:bg-white/10'}`}
                  >
                    <Grid3X3 className="w-4 h-4" />
                    <span>Multi</span>
                  </button>
                </div>
                <p className="mt-2 text-[11px] text-zinc-400">
                  {viewMode === 'single'
                    ? 'View one camera at a time with navigation controls.'
                    : 'View multiple cameras simultaneously in a grid.'}
                </p>
              </div>
            </div>

            <div className="pt-3 border-t border-white/10 flex justify-end gap-2">
              <button
                onClick={() => setShowSettings(false)}
                className="px-3 py-2 rounded-lg text-sm bg-white/10 hover:bg-white/15 text-zinc-200"
              >
                Close
              </button>
            </div>
          </div>
        </div>
      )}
    </Card>
  );
};

export default CameraManager;
