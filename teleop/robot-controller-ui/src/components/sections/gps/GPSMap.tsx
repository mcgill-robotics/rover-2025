'use client';

import React, { useEffect, useMemo, useRef, useState } from 'react';
import { useGPSData } from '../../../hooks/useGPSData';

interface Waypoint {
  id: string;
  latitude: number;
  longitude: number;
  name: string;
  timestamp: number;
}

interface TooltipData {
  x: number;
  y: number;
  text: string;
  type: 'current' | 'waypoint';
  id?: string;
}

type Bounds = {
  west: number;
  east: number;
  north: number;
  south: number;
};

// ==== CONFIG: your image + geobounds ====
const IMAGE_SRC = '/traversal_map.png';

// top-left:   (lat=51.47228, lon=-112.75490)  => north, west
// bottom-right(lat=51.46956, lon=-112.74859)  => south, east
const BOUNDS: Bounds = {
  west: -112.75490,
  east: -112.74859,
  north: 51.47228,
  south: 51.46956,
};

// ---- Web Mercator helpers ----
function mercY(latDeg: number) {
  const lat = (latDeg * Math.PI) / 180;
  return Math.log(Math.tan(Math.PI / 4 + lat / 2));
}
function mercLat(y: number) {
  return (2 * Math.atan(Math.exp(y)) - Math.PI / 2) * (180 / Math.PI);
}

// Map lon/lat -> image pixel using Mercator Y
function lonLatToImageXY(lon: number, lat: number, imgW: number, imgH: number, b: Bounds) {
  const yN = mercY(b.north), yS = mercY(b.south);
  const x = ((lon - b.west) / (b.east - b.west)) * imgW;
  const y = ((yN - mercY(lat)) / (yN - yS)) * imgH;
  return { x, y };
}

// Fit the image into container (letterbox) and return draw rect + scale
function fitImageIntoCanvas(imgW: number, imgH: number, canW: number, canH: number) {
  if (imgW === 0 || imgH === 0 || canW === 0 || canH === 0) {
    return { dx: 0, dy: 0, dw: 0, dh: 0, scale: 1 };
  }
  const imgAspect = imgW / imgH;
  const canAspect = canW / canH;
  let dw, dh, dx, dy, scale;
  if (imgAspect > canAspect) {
    dw = canW; dh = canW / imgAspect; dx = 0; dy = (canH - dh) / 2; scale = dw / imgW;
  } else {
    dh = canH; dw = canH * imgAspect; dy = 0; dx = (canW - dw) / 2; scale = dh / imgH;
  }
  return { dx, dy, dw, dh, scale };
}

const GPSMap: React.FC = () => {
  const { gpsData, gpsStatus, isLoading, error } = useGPSData();

  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [newWaypoint, setNewWaypoint] = useState({ latitude: '', longitude: '', name: '' });
  const [showWaypointManager, setShowWaypointManager] = useState(false);

  const [tooltip, setTooltip] = useState<TooltipData | null>(null);
  const [hoverHUD, setHoverHUD] = useState<{ label: string } | null>(null);
  const [highlightId, setHighlightId] = useState<string | 'current' | null>(null);

  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  const [dims, setDims] = useState({ w: 0, h: 0 });
  const [imgNatural, setImgNatural] = useState<{ w: number; h: number } | null>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);

  const hasGPS = useMemo(() => {
    const lat = Number(gpsData?.latitude);
    const lng = Number(gpsData?.longitude);
    return Number.isFinite(lat) && Number.isFinite(lng);
  }, [gpsData?.latitude, gpsData?.longitude]);

  // Load image once to get natural size
  useEffect(() => {
    const img = new Image();
    img.src = IMAGE_SRC;
    img.onload = () => {
      imageRef.current = img;
      setImgNatural({ w: img.naturalWidth || img.width, h: img.naturalHeight || img.height });
    };
    img.onerror = () => console.error('Failed to load overview image:', IMAGE_SRC);
  }, []);

  // Measure container and keep canvas sized
  useEffect(() => {
    const measure = () => {
      const el = containerRef.current;
      if (!el) return;
      const w = el.clientWidth || 0;
      const h = el.clientHeight || 0;
      setDims(prev => (prev.w === w && prev.h === h) ? prev : { w, h });
    };
    measure();
    const ro = new ResizeObserver(measure);
    if (containerRef.current) ro.observe(containerRef.current);
    window.addEventListener('resize', measure);
    let i = 0;
    const kick = () => { measure(); if (i++ < 10) requestAnimationFrame(kick); };
    kick();
    return () => { ro.disconnect(); window.removeEventListener('resize', measure); };
  }, []);

  // Sync canvas DPR
  useEffect(() => {
    const c = canvasRef.current;
    if (!c) return;
    const dpr = window.devicePixelRatio || 1;
    c.width = Math.max(1, Math.floor(dims.w * dpr));
    c.height = Math.max(1, Math.floor(dims.h * dpr));
    c.style.width = `${dims.w}px`;
    c.style.height = `${dims.h}px`;
    const ctx = c.getContext('2d');
    if (ctx) { ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.scale(dpr, dpr); }
    draw(); // redraw after size change
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [dims.w, dims.h, imgNatural?.w, imgNatural?.h]);

  // Project world->canvas pixels (accounting for letterboxing)
  const project = (lat: number, lon: number) => {
    if (!imgNatural) return { x: -9999, y: -9999 };
    const { w: imgW, h: imgH } = imgNatural;
    const { dx, dy, dw, dh } = fitImageIntoCanvas(imgW, imgH, dims.w, dims.h);
    const pImg = lonLatToImageXY(lon, lat, imgW, imgH, BOUNDS);
    const scaleX = dw / imgW;
    const scaleY = dh / imgH;
    return { x: dx + pImg.x * scaleX, y: dy + pImg.y * scaleY };
  };

  // Interactions (hover)
  const handleCanvasMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const container = containerRef.current;
    if (!container) return;
    const rect = container.getBoundingClientRect();
    const relX = e.clientX - rect.left;
    const relY = e.clientY - rect.top;

    if (hasGPS) {
      const p = project(Number(gpsData!.latitude), Number(gpsData!.longitude));
      if (Math.hypot(relX - p.x, relY - p.y) < 15) {
        setTooltip({
          x: relX, y: relY,
          text: `Current Position\nLat: ${Number(gpsData!.latitude).toFixed(6)}\nLng: ${Number(gpsData!.longitude).toFixed(6)}`,
          type: 'current', id: 'current'
        });
        setHoverHUD({ label: `Current • ${Number(gpsData!.latitude).toFixed(6)}, ${Number(gpsData!.longitude).toFixed(6)}` });
        setHighlightId('current');
        return;
      }
    }

    for (const wp of waypoints) {
      const p = project(Number(wp.latitude), Number(wp.longitude));
      if (Math.hypot(relX - p.x, relY - p.y) < 15) {
        setTooltip({
          x: relX, y: relY,
          text: `${wp.name}\nLat: ${Number(wp.latitude).toFixed(6)}\nLng: ${Number(wp.longitude).toFixed(6)}`,
          type: 'waypoint', id: wp.id
        });
        setHoverHUD({ label: `${wp.name} • ${Number(wp.latitude).toFixed(6)}, ${Number(wp.longitude).toFixed(6)}` });
        setHighlightId(wp.id);
        return;
      }
    }

    setTooltip(null);
    setHoverHUD(null);
    setHighlightId(null);
  };

  const handleCanvasMouseLeave = () => {
    setTooltip(null);
    setHoverHUD(null);
    setHighlightId(null);
  };

  // Drawing
  const draw = () => {
    const c = canvasRef.current;
    const img = imageRef.current;
    if (!c || !img) return;
    const ctx = c.getContext('2d');
    if (!ctx) return;

    // clear
    ctx.clearRect(0, 0, dims.w, dims.h);

    // draw image fitted
    const { dx, dy, dw, dh } = fitImageIntoCanvas(img.naturalWidth || img.width, img.naturalHeight || img.height, dims.w, dims.h);
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(img, dx, dy, dw, dh);

    // draw waypoints
    for (const wp of waypoints) {
      const pos = project(Number(wp.latitude), Number(wp.longitude));
      const isHL = highlightId === wp.id;

      ctx.beginPath();
      ctx.arc(pos.x, pos.y, isHL ? 4 : 3, 0, Math.PI * 2);
      ctx.fillStyle = isHL ? '#f87171' : '#ef4444';
      ctx.fill();

      ctx.lineWidth = isHL ? 3 : 2;
      ctx.strokeStyle = '#0b1020';
      ctx.stroke();

      ctx.fillStyle = isHL ? '#ffffff' : '#f3f4f6';
      ctx.font = `${isHL ? '700' : '500'} 12px Inter, system-ui, sans-serif`;
      ctx.textAlign = 'center';
      ctx.fillText(wp.name, pos.x, pos.y - 12);
    }

    // draw current position
    if (hasGPS) {
      const lat = Number(gpsData!.latitude);
      const lng = Number(gpsData!.longitude);
      const pos = project(lat, lng);
      const isHL = highlightId === 'current';

      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#22c55e';
      ctx.fill();

      const t = Date.now() / 700;
      const pulse = 18 + (Math.sin(t) + 1) * 6;
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, pulse, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(34,197,94,0.35)';
      ctx.lineWidth = 3;
      ctx.stroke();

      ctx.lineWidth = isHL ? 5 : 4;
      ctx.strokeStyle = '#0b1020';
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 5, 0, Math.PI * 2);
      ctx.stroke();

      ctx.fillStyle = '#e5e7eb';
      ctx.font = '700 10px Inter, system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Current', pos.x, pos.y - 20);
    }
  };

  // animation loop (pulse)
  useEffect(() => {
    let raf: number;
    const tick = () => { draw(); raf = requestAnimationFrame(tick); };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [waypoints, gpsData, highlightId, dims.w, dims.h, imgNatural?.w, imgNatural?.h]);

  if (isLoading || !imgNatural) {
    return (
      <div className="w-full h-screen flex items-center justify-center bg-gray-900">
        <div className="text-lg text-white">Loading map…</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="w-full h-screen flex items-center justify-center bg-gray-900">
        <div className="text-red-400">Error: {error}</div>
      </div>
    );
  }

  return (
    <div ref={containerRef} className="relative w-full h-screen min-h={[`400px`]} bg-gray-900 text-white overflow-hidden">
      {/* Single canvas: we draw the image + overlays onto it */}
      <canvas
        ref={canvasRef}
        className="absolute inset-0 w-full h-full block pointer-events-auto"
        onMouseMove={handleCanvasMouseMove}
        onMouseLeave={handleCanvasMouseLeave}
      />

      {tooltip && (
        <div
          className="pointer-events-none absolute z-20 bg-gray-900/95 text-white p-2 rounded-lg shadow-lg border border-gray-700 text-sm"
          style={{ left: tooltip.x + 10, top: tooltip.y - 10, transform: 'translateY(-100%)' }}
        >
          <div className="whitespace-pre-line">{tooltip.text}</div>
        </div>
      )}

      {/* HUD */}
      {hoverHUD && (
        <div className="absolute top-3 left-1/2 -translate-x-1/2 z-30 px-3 py-2 rounded-lg bg-gray-800/90 border border-gray-700 text-sm font-medium">
          {hoverHUD.label}
        </div>
      )}

      {/* Controls */}
      <div className="absolute top-3 left-3 z-30 flex items-center gap-2">
        <button
          onClick={() => setShowWaypointManager(s => !s)}
          className="bg-gray-800 hover:bg-gray-700 text-white px-3 py-2 rounded-lg border border-gray-700"
        >
          {showWaypointManager ? 'Hide' : 'Show'} Waypoints
        </button>
      </div>

      {/* GPS Status */}
      <div className="absolute top-3 right-3 z-30 bg-gray-800/90 p-3 rounded-lg border border-gray-700">
        <div className="grid grid-cols-2 gap-x-6 gap-y-2 text-xs md:text-sm">
          <div><span className="text-gray-400">Lat</span> <span className="font-semibold">{hasGPS ? Number(gpsData!.latitude).toFixed(6) : 'N/A'}</span></div>
          <div><span className="text-gray-400">Lng</span> <span className="font-semibold">{hasGPS ? Number(gpsData!.longitude).toFixed(6) : 'N/A'}</span></div>
          <div><span className="text-gray-400">Heading</span> <span className="font-semibold">{Number.isFinite(Number(gpsData?.heading)) ? Number(gpsData!.heading).toFixed(1) : 'N/A'}°</span></div>
          <div><span className="text-gray-400">Accuracy</span> <span className="font-semibold">{Number.isFinite(Number(gpsData?.accuracy)) ? Number(gpsData!.accuracy).toFixed(1) : 'N/A'}m</span></div>
          <div><span className="text-gray-400">Sats</span> <span className="font-semibold">{gpsStatus?.satellites ?? 'N/A'}</span></div>
          <div>
            <span className="text-gray-400">Fix</span>{' '}
            <span className={`font-semibold ${gpsStatus?.has_fix ? 'text-green-400' : 'text-red-400'}`}>
              {gpsStatus?.has_fix ? 'Good' : 'Poor'}
            </span>
          </div>
        </div>
      </div>

      {/* Waypoint Manager */}
      {showWaypointManager && (
        <div className="absolute bottom-3 left-3 z-30 w-80 max-h-[60vh] bg-gray-800/95 border border-gray-700 rounded-lg p-4 overflow-hidden">
          <div className="flex justify-between items-center mb-3">
            <h3 className="font-semibold">Waypoints ({waypoints.length})</h3>
            <button onClick={() => setShowWaypointManager(false)} className="text-gray-400 hover:text-white">×</button>
          </div>
          {waypoints.length === 0 ? (
            <div className="text-gray-400 text-center py-6">No waypoints yet</div>
          ) : (
            <div className="space-y-2 overflow-y-auto pr-1" style={{ maxHeight: '40vh' }}>
              {waypoints.map((wp) => (
                <div
                  key={wp.id}
                  className={`p-3 rounded-lg border ${highlightId === wp.id ? 'border-blue-400 bg-gray-700/80' : 'border-gray-600 bg-gray-700/60'}`}
                  onMouseEnter={() => {
                    setHighlightId(wp.id);
                    setHoverHUD({ label: `${wp.name} • ${Number(wp.latitude).toFixed(6)}, ${Number(wp.longitude).toFixed(6)}` });
                  }}
                  onMouseLeave={() => {
                    setHighlightId(null);
                    setHoverHUD(null);
                  }}
                >
                  <div className="flex justify-between items-start mb-1">
                    <div className="font-medium">{wp.name}</div>
                    <button
                      onClick={() => setWaypoints(prev => prev.filter(p => p.id !== wp.id))}
                      className="text-red-400 hover:text-red-300 ml-2"
                    >
                      ×
                    </button>
                  </div>
                  <div className="text-sm text-gray-300">
                    <div>Lat: {Number(wp.latitude).toFixed(6)}</div>
                    <div>Lng: {Number(wp.longitude).toFixed(6)}</div>
                    <div className="text-xs text-gray-400 mt-1">Added: {new Date(wp.timestamp).toLocaleString()}</div>
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      )}

      {/* Add waypoint form */}
      <div className="absolute bottom-3 right-3 z-30 bg-gray-800/95 p-3 rounded-lg border border-gray-700 w-[300px]">
        <h3 className="font-semibold mb-2">Add Waypoint</h3>
        <div className="grid grid-cols-1 gap-2">
          <input
            type="number" step="any" placeholder="Latitude"
            value={newWaypoint.latitude}
            onChange={(e) => setNewWaypoint({ ...newWaypoint, latitude: e.target.value })}
            className="px-3 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500"
          />
          <input
            type="number" step="any" placeholder="Longitude"
            value={newWaypoint.longitude}
            onChange={(e) => setNewWaypoint({ ...newWaypoint, longitude: e.target.value })}
            className="px-3 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500"
          />
          <input
            type="text" placeholder="Name (optional)"
            value={newWaypoint.name}
            onChange={(e) => setNewWaypoint({ ...newWaypoint, name: e.target.value })}
            className="px-3 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500"
          />
          <button
            onClick={() => {
              if (!newWaypoint.latitude || !newWaypoint.longitude) return;
              const lat = parseFloat(newWaypoint.latitude);
              const lon = parseFloat(newWaypoint.longitude);
              if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
              const wp: Waypoint = {
                id: Date.now().toString(),
                latitude: lat,
                longitude: lon,
                name: newWaypoint.name || `WP ${waypoints.length + 1}`,
                timestamp: Date.now(),
              };
              setWaypoints(prev => [...prev, wp]);
              setNewWaypoint({ latitude: '', longitude: '', name: '' });
            }}
            disabled={!newWaypoint.latitude || !newWaypoint.longitude}
            className="bg-green-600 hover:bg-green-700 disabled:bg-gray-600 text-white px-3 py-2 rounded-lg font-medium"
          >
            Add
          </button>
        </div>
      </div>
    </div>
  );
};

export default GPSMap;
