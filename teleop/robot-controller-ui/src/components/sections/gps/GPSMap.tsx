'use client';

import React, {
  useState,
  useEffect,
  useMemo,
  useRef,
  useLayoutEffect,
} from 'react';
import { useGPSData } from '../../../hooks/useGPSData';

interface Waypoint {
  id: string;
  latitude: number;
  longitude: number;
  name: string;
  timestamp: number;
}

interface TooltipData {
  x: number; // container-relative
  y: number; // container-relative
  text: string;
  type: 'current' | 'waypoint';
  id?: string;
}

const metersPerDegLat = 110540; // approx
const metersPerDegLng = (lat: number) =>
  111320 * Math.cos((lat * Math.PI) / 180);

const GPSMap: React.FC = () => {
  const { gpsData, gpsStatus, isLoading, error } = useGPSData();

  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [newWaypoint, setNewWaypoint] = useState({
    latitude: '',
    longitude: '',
    name: '',
  });
  const [showWaypointManager, setShowWaypointManager] = useState(false);

  const [tooltip, setTooltip] = useState<TooltipData | null>(null);
  const [hoverHUD, setHoverHUD] = useState<{ label: string } | null>(null);
  const [highlightId, setHighlightId] = useState<string | 'current' | null>(
    null
  );

  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [dims, setDims] = useState({ w: 0, h: 0 });

  const defaultCenter = { lat: 45.5048, lng: -73.5772 };

  const hasGPS = useMemo(() => {
    const lat = Number(gpsData?.latitude);
    const lng = Number(gpsData?.longitude);
    return Number.isFinite(lat) && Number.isFinite(lng);
  }, [gpsData?.latitude, gpsData?.longitude]);

  // Measure BEFORE first paint, and retry briefly until size is non-zero
  useLayoutEffect(() => {
    let tries = 0;
    const MAX_TRIES = 20;
    let timeoutId: number | null = null;

    const measure = () => {
      const el = containerRef.current;
      if (!el) return;
      const w = el.clientWidth || el.offsetWidth || 0;
      const h = el.clientHeight || el.offsetHeight || 0;
      setDims((prev) => (prev.w === w && prev.h === h ? prev : { w, h }));

      if ((w === 0 || h === 0) && tries < MAX_TRIES) {
        tries++;
        timeoutId = window.setTimeout(measure, 50);
      }
    };

    measure();
    const ro = new ResizeObserver(measure);
    if (containerRef.current) ro.observe(containerRef.current);
    window.addEventListener('resize', measure);
    const raf = requestAnimationFrame(measure);

    return () => {
      ro.disconnect();
      window.removeEventListener('resize', measure);
      cancelAnimationFrame(raf);
      if (timeoutId) clearTimeout(timeoutId);
    };
  }, []);

  // Whenever dims change, update intrinsic canvas bitmap + DPR scale
  useEffect(() => {
    const c = canvasRef.current;
    if (!c || !dims.w || !dims.h) return;
    const dpr = window.devicePixelRatio || 1;

    c.width = Math.max(1, Math.floor(dims.w * dpr));
    c.height = Math.max(1, Math.floor(dims.h * dpr));

    c.style.width = `${dims.w}px`;
    c.style.height = `${dims.h}px`;

    const ctx = c.getContext('2d');
    if (ctx) {
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.scale(dpr, dpr);
    }
  }, [dims.w, dims.h]);

  // Points list (for fit)
  const points = useMemo(() => {
    const list: Array<{
      lat: number;
      lng: number;
      type: 'current' | 'wp';
      id?: string;
      name?: string;
    }> = [];
    if (hasGPS) {
      list.push({
        lat: Number(gpsData!.latitude),
        lng: Number(gpsData!.longitude),
        type: 'current',
        id: 'current',
      });
    }
    for (const wp of waypoints) {
      list.push({
        lat: Number(wp.latitude),
        lng: Number(wp.longitude),
        type: 'wp',
        id: wp.id,
        name: wp.name,
      });
    }
    return list;
  }, [hasGPS, gpsData, waypoints]);

  // Fit all points (or default) to canvas
  const fit = useMemo(() => {
    if (!dims.w || !dims.h) {
      return {
        centerLat: defaultCenter.lat,
        centerLng: defaultCenter.lng,
        pxPerMeter: 1,
        padding: 60,
      };
    }

    const pts = points.length
      ? points
      : [{ lat: defaultCenter.lat, lng: defaultCenter.lng, type: 'wp' as const }];

    const latMin = Math.min(...pts.map((p) => p.lat));
    const latMax = Math.max(...pts.map((p) => p.lat));
    const lngMin = Math.min(...pts.map((p) => p.lng));
    const lngMax = Math.max(...pts.map((p) => p.lng));

    let centerLat = (latMin + latMax) / 2;
    let centerLng = (lngMin + lngMax) / 2;

    if (hasGPS) {
      centerLat = Number(gpsData!.latitude);
      centerLng = Number(gpsData!.longitude);
    }

    const mPerDegX = metersPerDegLng(centerLat || defaultCenter.lat);
    const spanXMeters = Math.max((lngMax - lngMin) * mPerDegX, 200);
    const spanYMeters = Math.max((latMax - latMin) * metersPerDegLat, 200);

    const padding = 60;
    const usableW = Math.max(1, dims.w - padding * 2);
    const usableH = Math.max(1, dims.h - padding * 2);

    const pxPerMeter = Math.max(
      0.0001,
      Math.min(usableW / spanXMeters, usableH / spanYMeters)
    );

    return { centerLat, centerLng, pxPerMeter, padding };
  }, [points, dims.w, dims.h, hasGPS, gpsData]);

  const project = (lat: number, lng: number) => {
    const dLat = lat - fit.centerLat;
    const dLng = lng - fit.centerLng;
    const dxMeters = dLng * metersPerDegLng(fit.centerLat || defaultCenter.lat);
    const dyMeters = -dLat * metersPerDegLat;
    return {
      x: dims.w / 2 + dxMeters * fit.pxPerMeter,
      y: dims.h / 2 + dyMeters * fit.pxPerMeter,
    };
  };

  // Interactions (hover/tooltip)
  const handleCanvasMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;

    const canvasRect = canvas.getBoundingClientRect();
    const containerRect = container.getBoundingClientRect();
    const x = e.clientX - canvasRect.left;
    const y = e.clientY - canvasRect.top;
    const relX = e.clientX - containerRect.left;
    const relY = e.clientY - containerRect.top;

    if (hasGPS) {
      const p = project(Number(gpsData!.latitude), Number(gpsData!.longitude));
      if (Math.hypot(x - p.x, y - p.y) < 15) {
        setTooltip({
          x: relX,
          y: relY,
          text: `Current Position\nLat: ${Number(gpsData!.latitude).toFixed(
            6
          )}\nLng: ${Number(gpsData!.longitude).toFixed(6)}`,
          type: 'current',
          id: 'current',
        });
        setHoverHUD({
          label: `Current • ${Number(gpsData!.latitude).toFixed(
            6
          )}, ${Number(gpsData!.longitude).toFixed(6)}`,
        });
        setHighlightId('current');
        return;
      }
    }

    for (const wp of waypoints) {
      const p = project(Number(wp.latitude), Number(wp.longitude));
      if (Math.hypot(x - p.x, y - p.y) < 15) {
        setTooltip({
          x: relX,
          y: relY,
          text: `${wp.name}\nLat: ${Number(wp.latitude).toFixed(
            6
          )}\nLng: ${Number(wp.longitude).toFixed(6)}`,
          type: 'waypoint',
          id: wp.id,
        });
        setHoverHUD({
          label: `${wp.name} • ${Number(wp.latitude).toFixed(
            6
          )}, ${Number(wp.longitude).toFixed(6)}`,
        });
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

  const handleCanvasClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!e.altKey) return; // require Alt+Click to add
    const rect = (e.target as HTMLCanvasElement).getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    // inverse of project()
    const dxMeters = (x - dims.w / 2) / fit.pxPerMeter;
    const dyMeters = (y - dims.h / 2) / fit.pxPerMeter;
    const lat = fit.centerLat + -dyMeters / metersPerDegLat;
    const lng =
      fit.centerLng + dxMeters / metersPerDegLng(fit.centerLat || defaultCenter.lat);

    const wp: Waypoint = {
      id: Date.now().toString(),
      latitude: lat,
      longitude: lng,
      name: `WP ${waypoints.length + 1}`,
      timestamp: Date.now(),
    };
    setWaypoints((prev) => [...prev, wp]);
  };

  // Drawing
  const draw = () => {
    const canvas = canvasRef.current;
    if (!canvas || !dims.w || !dims.h) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // background
    ctx.clearRect(0, 0, dims.w, dims.h);
    ctx.fillStyle = '#111827';
    ctx.fillRect(0, 0, dims.w, dims.h);

    // grid (~100m)
    const gridMeters = 100;
    const gridPx = gridMeters * fit.pxPerMeter;
    if (gridPx >= 20) {
      ctx.strokeStyle = '#374151';
      ctx.lineWidth = 1;

      const startX = ((-dims.w / 2) % gridPx) + dims.w / 2;
      for (let x = startX; x <= dims.w; x += gridPx) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, dims.h);
        ctx.stroke();
      }
      const startY = ((-dims.h / 2) % gridPx) + dims.h / 2;
      for (let y = startY; y <= dims.h; y += gridPx) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(dims.w, y);
        ctx.stroke();
      }
    }

    // waypoints
    for (const wp of waypoints) {
      const pos = project(Number(wp.latitude), Number(wp.longitude));
      const isHL = highlightId === wp.id;

      ctx.beginPath();
      ctx.arc(pos.x, pos.y, isHL ? 8 : 6, 0, Math.PI * 2);
      ctx.fillStyle = isHL ? '#f87171' : '#ef4444';
      ctx.fill();

      ctx.lineWidth = isHL ? 3 : 2;
      ctx.strokeStyle = '#1f2937';
      ctx.stroke();

      ctx.fillStyle = isHL ? '#ffffff' : '#f3f4f6';
      ctx.font = `${isHL ? '700' : '500'} 12px Inter, system-ui, sans-serif`;
      ctx.textAlign = 'center';
      ctx.fillText(wp.name, pos.x, pos.y - 12);
    }

    // current position
    if (hasGPS) {
      const lat = Number(gpsData!.latitude);
      const lng = Number(gpsData!.longitude);
      const pos = project(lat, lng);
      const isHL = highlightId === 'current';

      // marker
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 14, 0, Math.PI * 2);
      ctx.fillStyle = '#22c55e';
      ctx.fill();

      // pulse ring
      const t = Date.now() / 700;
      const pulse = 18 + (Math.sin(t) + 1) * 6;
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, pulse, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(34,197,94,0.35)';
      ctx.lineWidth = 3;
      ctx.stroke();

      // outline
      ctx.lineWidth = isHL ? 5 : 4;
      ctx.strokeStyle = '#0b1020';
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 14, 0, Math.PI * 2);
      ctx.stroke();

      // label
      ctx.fillStyle = '#e5e7eb';
      ctx.font = '700 15px Inter, system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Current', pos.x, pos.y - 20);
    }
  };

  // Animation loop
  useEffect(() => {
    let raf: number;
    const tick = () => {
      draw();
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [dims, waypoints, gpsData, highlightId, fit]);

  // Form add manually
  const addWaypointByForm = () => {
    if (!newWaypoint.latitude || !newWaypoint.longitude) return;
    const lat = parseFloat(newWaypoint.latitude);
    const lng = parseFloat(newWaypoint.longitude);
    if (!Number.isFinite(lat) || !Number.isFinite(lng)) return;
    const wp: Waypoint = {
      id: Date.now().toString(),
      latitude: lat,
      longitude: lng,
      name: newWaypoint.name || `WP ${waypoints.length + 1}`,
      timestamp: Date.now(),
    };
    setWaypoints((prev) => [...prev, wp]);
    setNewWaypoint({ latitude: '', longitude: '', name: '' });
  };

  if (isLoading) {
    return (
      <div className="w-full h-[100dvh] flex items-center justify-center bg-gray-900">
        <div className="text-lg text-white">Loading GPS data...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="w-full h-[100dvh] flex items-center justify-center bg-gray-900">
        <div className="text-red-400">Error: {error}</div>
      </div>
    );
  }

  return (
    <div className="relative w-full h-[100dvh] min-h-[400px] bg-gray-900 text-white overflow-hidden">
      {/* Map/Canvas */}
      <div ref={containerRef} className="absolute inset-0">
        <canvas
          ref={canvasRef}
          className="w-full h-full block cursor-crosshair"
          onMouseMove={handleCanvasMouseMove}
          onMouseLeave={handleCanvasMouseLeave}
          onClick={handleCanvasClick}
        />
        {tooltip && (
          <div
            className="pointer-events-none absolute z-20 bg-gray-900/95 text-white p-2 rounded-lg shadow-lg border border-gray-700 text-sm"
            style={{
              left: tooltip.x + 10,
              top: tooltip.y - 10,
              transform: 'translateY(-100%)',
            }}
          >
            <div className="whitespace-pre-line">{tooltip.text}</div>
          </div>
        )}
      </div>

      {/* Top Hover HUD */}
      {hoverHUD && (
        <div className="absolute top-3 left-1/2 -translate-x-1/2 z-30 px-3 py-2 rounded-lg bg-gray-800/90 border border-gray-700 text-sm font-medium">
          {hoverHUD.label}
        </div>
      )}

      {/* Controls */}
      <div className="absolute top-3 left-3 z-30 flex items-center gap-2">
        <button
          onClick={() => setShowWaypointManager((s) => !s)}
          className="bg-gray-800 hover:bg-gray-700 text-white px-3 py-2 rounded-lg border border-gray-700"
        >
          {showWaypointManager ? 'Hide' : 'Show'} Waypoints
        </button>
        <span className="text-xs text-gray-400 px-2 py-1 bg-gray-800/70 rounded border border-gray-700">
          Alt+Click map to add waypoint
        </span>
      </div>

      {/* GPS Status */}
      <div className="absolute top-3 right-3 z-30 bg-gray-800/90 p-3 rounded-lg border border-gray-700">
        <div className="grid grid-cols-2 gap-x-6 gap-y-2 text-xs md:text-sm">
          <div>
            <span className="text-gray-400">Lat</span>{' '}
            <span className="font-semibold">
              {hasGPS ? Number(gpsData!.latitude).toFixed(6) : 'N/A'}
            </span>
          </div>
          <div>
            <span className="text-gray-400">Lng</span>{' '}
            <span className="font-semibold">
              {hasGPS ? Number(gpsData!.longitude).toFixed(6) : 'N/A'}
            </span>
          </div>
          <div>
            <span className="text-gray-400">Heading</span>{' '}
            <span className="font-semibold">
              {Number.isFinite(Number(gpsData?.heading))
                ? Number(gpsData!.heading).toFixed(1)
                : 'N/A'}
              °
            </span>
          </div>
          <div>
            <span className="text-gray-400">Accuracy</span>{' '}
            <span className="font-semibold">
              {Number.isFinite(Number(gpsData?.accuracy))
                ? Number(gpsData!.accuracy).toFixed(1)
                : 'N/A'}
              m
            </span>
          </div>
          <div>
            <span className="text-gray-400">Sats</span>{' '}
            <span className="font-semibold">
              {gpsStatus?.satellites ?? 'N/A'}
            </span>
          </div>
          <div>
            <span className="text-gray-400">Fix</span>{' '}
            <span
              className={`font-semibold ${
                gpsStatus?.has_fix ? 'text-green-400' : 'text-red-400'
              }`}
            >
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
            <button
              onClick={() => setShowWaypointManager(false)}
              className="text-gray-400 hover:text-white"
            >
              ×
            </button>
          </div>
          {waypoints.length === 0 ? (
            <div className="text-gray-400 text-center py-6">
              No waypoints yet
            </div>
          ) : (
            <div className="space-y-2 overflow-y-auto pr-1" style={{ maxHeight: '40vh' }}>
              {waypoints.map((wp) => (
                <div
                  key={wp.id}
                  className={`p-3 rounded-lg border ${
                    highlightId === wp.id
                      ? 'border-blue-400 bg-gray-700/80'
                      : 'border-gray-600 bg-gray-700/60'
                  }`}
                  onMouseEnter={() => {
                    setHighlightId(wp.id);
                    setHoverHUD({
                      label: `${wp.name} • ${Number(wp.latitude).toFixed(
                        6
                      )}, ${Number(wp.longitude).toFixed(6)}`,
                    });
                  }}
                  onMouseLeave={() => {
                    setHighlightId(null);
                    setHoverHUD(null);
                  }}
                >
                  <div className="flex justify-between items-start mb-1">
                    <div className="font-medium">{wp.name}</div>
                    <button
                      onClick={() =>
                        setWaypoints((prev) => prev.filter((p) => p.id !== wp.id))
                      }
                      className="text-red-400 hover:text-red-300 ml-2"
                    >
                      ×
                    </button>
                  </div>
                  <div className="text-sm text-gray-300">
                    <div>Lat: {Number(wp.latitude).toFixed(6)}</div>
                    <div>Lng: {Number(wp.longitude).toFixed(6)}</div>
                    <div className="text-xs text-gray-400 mt-1">
                      Added: {new Date(wp.timestamp).toLocaleString()}
                    </div>
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
            type="number"
            step="any"
            placeholder="Latitude"
            value={newWaypoint.latitude}
            onChange={(e) =>
              setNewWaypoint({ ...newWaypoint, latitude: e.target.value })
            }
            className="px-3 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500"
          />
          <input
            type="number"
            step="any"
            placeholder="Longitude"
            value={newWaypoint.longitude}
            onChange={(e) =>
              setNewWaypoint({ ...newWaypoint, longitude: e.target.value })
            }
            className="px-3 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500"
          />
          <input
            type="text"
            placeholder="Name (optional)"
            value={newWaypoint.name}
            onChange={(e) =>
              setNewWaypoint({ ...newWaypoint, name: e.target.value })
            }
            className="px-3 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500"
          />
          <button
            onClick={addWaypointByForm}
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
