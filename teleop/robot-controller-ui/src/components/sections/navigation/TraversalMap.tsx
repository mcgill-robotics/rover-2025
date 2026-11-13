'use client';

import React, { useEffect, useMemo, useRef, useState } from 'react';
import { GPSData, Waypoint } from '@/types/navigation';
import { WAYPOINT_COLORS } from '@/config/navigation';

interface TraversalMapProps {
  gpsData: GPSData | null;
  waypoints: Waypoint[];
  selectedWaypoint: Waypoint | null;
  onWaypointSelect: (waypoint: Waypoint | null) => void;
  onAddWaypoint: (lat: number, lng: number) => void;
  onMapLoad: (loaded: boolean) => void;
}

interface TooltipData {
  x: number;
  y: number;
  text: string;
}

type Bounds = {
  west: number;
  east: number;
  north: number;
  south: number;
};

const IMAGE_SRC = '/traversal_map.png';

const BOUNDS: Bounds = {
  west: -112.75490,
  east: -112.74859,
  north: 51.47228,
  south: 51.46956,
};

function mercY(latDeg: number) {
  const lat = (latDeg * Math.PI) / 180;
  return Math.log(Math.tan(Math.PI / 4 + lat / 2));
}
function mercLat(y: number) {
  return (2 * Math.atan(Math.exp(y)) - Math.PI / 2) * (180 / Math.PI);
}

function lonLatToImageXY(lon: number, lat: number, imgW: number, imgH: number, b: Bounds) {
  const yN = mercY(b.north), yS = mercY(b.south);
  const x = ((lon - b.west) / (b.east - b.west)) * imgW;
  const y = ((yN - mercY(lat)) / (yN - yS)) * imgH;
  return { x, y };
}

function imageXYToLonLat(x: number, y: number, imgW: number, imgH: number, b: Bounds) {
  const yN = mercY(b.north), yS = mercY(b.south);
  const lon = b.west + (x / imgW) * (b.east - b.west);
  const latMerc = yN - (y / imgH) * (yN - yS);
  const lat = mercLat(latMerc);
  return { lon, lat };
}

function fitImageIntoCanvas(imgW: number, imgH: number, canW: number, canH: number) {
  if (imgW === 0 || imgH === 0 || canW === 0 || canH === 0) {
    return { dx: 0, dy: 0, dw: 0, dh: 0 };
  }
  const imgAspect = imgW / imgH;
  const canAspect = canW / canH;
  let dw, dh, dx, dy;
  if (imgAspect > canAspect) {
    dw = canW; dh = canW / imgAspect; dx = 0; dy = (canH - dh) / 2;
  } else {
    dh = canH; dw = canH * imgAspect; dy = 0; dx = (canW - dw) / 2;
  }
  return { dx, dy, dw, dh };
}

export const TraversalMap: React.FC<TraversalMapProps> = ({
  gpsData,
  waypoints,
  selectedWaypoint,
  onWaypointSelect,
  onAddWaypoint,
  onMapLoad,
}) => {
  const [tooltip, setTooltip] = useState<TooltipData | null>(null);
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

  useEffect(() => {
    const img = new Image();
    img.src = IMAGE_SRC;
    img.onload = () => {
      imageRef.current = img;
      setImgNatural({ w: img.naturalWidth || img.width, h: img.naturalHeight || img.height });
      onMapLoad(true);
    };
    img.onerror = () => {
      console.error('Failed to load overview image:', IMAGE_SRC);
      onMapLoad(false);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

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
    return () => { ro.disconnect(); window.removeEventListener('resize', measure); };
  }, [imgNatural]);

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
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [dims.w, dims.h, imgNatural?.w, imgNatural?.h]);

  const project = (lat: number, lon: number) => {
    if (!imgNatural) return { x: -9999, y: -9999 };
    const { w: imgW, h: imgH } = imgNatural;
    const { dx, dy, dw, dh } = fitImageIntoCanvas(imgW, imgH, dims.w, dims.h);
    const pImg = lonLatToImageXY(lon, lat, imgW, imgH, BOUNDS);
    const scaleX = dw / imgW;
    const scaleY = dh / imgH;
    return { x: dx + pImg.x * scaleX, y: dy + pImg.y * scaleY };
  };

  const unproject = (relX: number, relY: number) => {
    if (!imgNatural) return null;
    const { w: imgW, h: imgH } = imgNatural;
    const { dx, dy, dw, dh } = fitImageIntoCanvas(imgW, imgH, dims.w, dims.h);
    const scaleX = dw / imgW;
    const scaleY = dh / imgH;
    const imgX = (relX - dx) / scaleX;
    const imgY = (relY - dy) / scaleY;
    return imageXYToLonLat(imgX, imgY, imgW, imgH, BOUNDS);
  };

  const handleCanvasMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const container = containerRef.current;
    if (!container) return;
    const rect = container.getBoundingClientRect();
    const relX = e.clientX - rect.left;
    const relY = e.clientY - rect.top;

    if (hasGPS) {
      const p = project(Number(gpsData!.latitude), Number(gpsData!.longitude));
      if (Math.hypot(relX - p.x, relY - p.y) < 15) {
        setTooltip({ x: relX, y: relY, text: `Current Position\nLat: ${Number(gpsData!.latitude).toFixed(6)}\nLng: ${Number(gpsData!.longitude).toFixed(6)}` });
        setHighlightId('current');
        return;
      }
    }

    for (const wp of waypoints) {
      const p = project(Number(wp.latitude), Number(wp.longitude));
      if (Math.hypot(relX - p.x, relY - p.y) < 15) {
        setTooltip({ x: relX, y: relY, text: `${wp.name}\nLat: ${Number(wp.latitude).toFixed(6)}\nLng: ${Number(wp.longitude).toFixed(6)}` });
        setHighlightId(wp.id);
        return;
      }
    }

    setTooltip(null);
    setHighlightId(null);
  };

  const handleCanvasMouseLeave = () => {
    setTooltip(null);
    setHighlightId(null);
  };

  const handleCanvasClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const container = containerRef.current;
    if (!container || !imgNatural) return;
    const rect = container.getBoundingClientRect();
    const relX = e.clientX - rect.left;
    const relY = e.clientY - rect.top;

    for (const wp of waypoints) {
      const p = project(Number(wp.latitude), Number(wp.longitude));
      if (Math.hypot(relX - p.x, relY - p.y) < 15) {
        onWaypointSelect(wp);
        return;
      }
    }

    const { w: imgW, h: imgH } = imgNatural;
    const { dx, dy, dw, dh } = fitImageIntoCanvas(imgW, imgH, dims.w, dims.h);
    if (relX < dx || relX > dx + dw || relY < dy || relY > dy + dh) {
      return;
    }

    const result = unproject(relX, relY);
    if (result) {
      onAddWaypoint(result.lat, result.lon);
    }
  };

  const draw = () => {
    const c = canvasRef.current;
    const img = imageRef.current;
    if (!c || !img) return;
    const ctx = c.getContext('2d');
    if (!ctx) return;

    ctx.clearRect(0, 0, dims.w, dims.h);

    const { dx, dy, dw, dh } = fitImageIntoCanvas(img.naturalWidth || img.width, img.naturalHeight || img.height, dims.w, dims.h);
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(img, dx, dy, dw, dh);

    for (const wp of waypoints) {
      const pos = project(Number(wp.latitude), Number(wp.longitude));
      const isHL = highlightId === wp.id;
      const isSelected = selectedWaypoint?.id === wp.id;

      ctx.beginPath();
      ctx.arc(pos.x, pos.y, isHL ? 4 : 3, 0, Math.PI * 2);
      ctx.fillStyle = isSelected ? WAYPOINT_COLORS.SELECTED : WAYPOINT_COLORS.DEFAULT;
      ctx.fill();

      ctx.lineWidth = isHL ? 3 : 2;
      ctx.strokeStyle = '#0b1020';
      ctx.stroke();

      ctx.fillStyle = isHL ? '#ffffff' : '#f3f4f6';
      ctx.font = `${isHL ? '700' : '500'} 12px Inter, system-ui, sans-serif`;
      ctx.textAlign = 'center';
      ctx.fillText(wp.name, pos.x, pos.y - 12);
    }

    if (hasGPS) {
      const lat = Number(gpsData!.latitude);
      const lng = Number(gpsData!.longitude);
      const pos = project(lat, lng);

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

      ctx.lineWidth = highlightId === 'current' ? 5 : 4;
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

  useEffect(() => {
    let raf: number;
    const tick = () => { draw(); raf = requestAnimationFrame(tick); };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [waypoints, selectedWaypoint, gpsData, highlightId, dims.w, dims.h, imgNatural?.w, imgNatural?.h]);

  if (!imgNatural) {
    return (
      <div className="relative w-full h-full flex items-center justify-center bg-gray-900">
        <div className="text-lg text-white">Loading traversal map…</div>
      </div>
    );
  }

  return (
    <div ref={containerRef} className="relative w-full h-full bg-gray-900 text-white overflow-hidden">
      <canvas
        ref={canvasRef}
        className="absolute inset-0 w-full h-full block pointer-events-auto cursor-crosshair"
        onMouseMove={handleCanvasMouseMove}
        onMouseLeave={handleCanvasMouseLeave}
        onClick={handleCanvasClick}
      />
      {tooltip && (
        <div
          className="pointer-events-none absolute z-20 bg-gray-900/95 text-white p-2 rounded-lg shadow-lg border border-gray-700 text-sm"
          style={{ left: tooltip.x + 10, top: tooltip.y - 10, transform: 'translateY(-100%)' }}
        >
          <div className="whitespace-pre-line">{tooltip.text}</div>
        </div>
      )}
      <div className="absolute top-4 left-4 bg-white bg-opacity-90 rounded-lg p-3 text-sm text-gray-700">
        <p>Click on the map to add a waypoint</p>
        <p>Click an existing pin to select it</p>
      </div>
    </div>
  );
};

export default TraversalMap;