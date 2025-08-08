'use client';

import React, { useState, useEffect, useRef } from 'react';
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
}

const GPSMap: React.FC = () => {
  const { gpsData, gpsStatus, isLoading, error } = useGPSData();
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [newWaypoint, setNewWaypoint] = useState({ latitude: '', longitude: '', name: '' });
  const [showAddForm, setShowAddForm] = useState(false);
  const [showWaypointManager, setShowWaypointManager] = useState(false);
  const [tooltip, setTooltip] = useState<TooltipData | null>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  // Default center (McGill University coordinates)
  const defaultCenter = { lat: 45.5048, lng: -73.5772 };
  const mapSize = 400;
  const zoom = 15;

  const addWaypoint = () => {
    if (!newWaypoint.latitude || !newWaypoint.longitude) return;
    
    const waypoint: Waypoint = {
      id: Date.now().toString(),
      latitude: parseFloat(newWaypoint.latitude),
      longitude: parseFloat(newWaypoint.longitude),
      name: newWaypoint.name || `Waypoint ${waypoints.length + 1}`,
      timestamp: Date.now()
    };
    
    setWaypoints([...waypoints, waypoint]);
    setNewWaypoint({ latitude: '', longitude: '', name: '' });
    setShowAddForm(false);
  };

  const removeWaypoint = (id: string) => {
    setWaypoints(waypoints.filter(wp => wp.id !== id));
  };

  const latLngToPixel = (lat: number, lng: number, centerLat: number, centerLng: number) => {
    const latDiff = lat - centerLat;
    const lngDiff = lng - centerLng;
    
    // Convert to pixels (approximate conversion)
    const x = (lngDiff * 111320 * Math.cos(centerLat * Math.PI / 180) * zoom) + mapSize / 2;
    const y = (-latDiff * 110540 * zoom) + mapSize / 2;
    
    return { x, y };
  };

  const pixelToLatLng = (x: number, y: number, centerLat: number, centerLng: number) => {
    const lngDiff = (x - mapSize / 2) / (111320 * Math.cos(centerLat * Math.PI / 180) * zoom);
    const latDiff = -(y - mapSize / 2) / (110540 * zoom);
    
    return {
      lat: centerLat + latDiff,
      lng: centerLng + lngDiff
    };
  };

  const handleCanvasMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    const centerLat = gpsData?.latitude || defaultCenter.lat;
    const centerLng = gpsData?.longitude || defaultCenter.lng;
    
    // Check if mouse is near current position
    if (gpsData && gpsData.latitude !== 0 && gpsData.longitude !== 0) {
      const currentPos = latLngToPixel(gpsData.latitude, gpsData.longitude, centerLat, centerLng);
      const distance = Math.sqrt((x - currentPos.x) ** 2 + (y - currentPos.y) ** 2);
      
      if (distance < 15) {
        setTooltip({
          x: e.clientX,
          y: e.clientY,
          text: `Current Position\nLat: ${gpsData.latitude.toFixed(6)}\nLng: ${gpsData.longitude.toFixed(6)}`,
          type: 'current'
        });
        return;
      }
    }
    
    // Check if mouse is near any waypoint
    for (const waypoint of waypoints) {
      const waypointPos = latLngToPixel(waypoint.latitude, waypoint.longitude, centerLat, centerLng);
      const distance = Math.sqrt((x - waypointPos.x) ** 2 + (y - waypointPos.y) ** 2);
      
      if (distance < 15) {
        setTooltip({
          x: e.clientX,
          y: e.clientY,
          text: `${waypoint.name}\nLat: ${waypoint.latitude.toFixed(6)}\nLng: ${waypoint.longitude.toFixed(6)}`,
          type: 'waypoint'
        });
        return;
      }
    }
    
    setTooltip(null);
  };

  const handleCanvasMouseLeave = () => {
    setTooltip(null);
  };

  const drawMap = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.clearRect(0, 0, mapSize, mapSize);

    // Draw background grid with dark theme colors
    ctx.strokeStyle = '#374151';
    ctx.lineWidth = 1;
    
    // Vertical lines
    for (let i = 0; i <= mapSize; i += 50) {
      ctx.beginPath();
      ctx.moveTo(i, 0);
      ctx.lineTo(i, mapSize);
      ctx.stroke();
    }
    
    // Horizontal lines
    for (let i = 0; i <= mapSize; i += 50) {
      ctx.beginPath();
      ctx.moveTo(0, i);
      ctx.lineTo(mapSize, i);
      ctx.stroke();
    }

    // Determine center point
    const centerLat = gpsData?.latitude || defaultCenter.lat;
    const centerLng = gpsData?.longitude || defaultCenter.lng;

    // Draw waypoints
    waypoints.forEach(waypoint => {
      const pos = latLngToPixel(waypoint.latitude, waypoint.longitude, centerLat, centerLng);
      
      // Draw waypoint circle with dark theme colors
      ctx.fillStyle = '#ef4444';
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 6, 0, 2 * Math.PI);
      ctx.fill();
      
      // Draw waypoint border
      ctx.strokeStyle = '#1f2937';
      ctx.lineWidth = 2;
      ctx.stroke();
      
      // Draw waypoint label
      ctx.fillStyle = '#f3f4f6';
      ctx.font = '12px Inter, system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(waypoint.name, pos.x, pos.y - 10);
    });

    // Draw current position
    if (gpsData && gpsData.latitude !== 0 && gpsData.longitude !== 0) {
      const pos = latLngToPixel(gpsData.latitude, gpsData.longitude, centerLat, centerLng);
      
      // Draw position circle with dark theme colors
      ctx.fillStyle = '#10b981';
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 8, 0, 2 * Math.PI);
      ctx.fill();
      
      // Draw position border
      ctx.strokeStyle = '#1f2937';
      ctx.lineWidth = 3;
      ctx.stroke();
      
      // Draw heading indicator
      if (gpsData.heading !== 0) {
        const headingRad = (gpsData.heading * Math.PI) / 180;
        const arrowLength = 20;
        const arrowEndX = pos.x + Math.cos(headingRad) * arrowLength;
        const arrowEndY = pos.y - Math.sin(headingRad) * arrowLength;
        
        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(pos.x, pos.y);
        ctx.lineTo(arrowEndX, arrowEndY);
        ctx.stroke();
        
        // Draw arrowhead
        ctx.fillStyle = '#3b82f6';
        ctx.beginPath();
        ctx.moveTo(arrowEndX, arrowEndY);
        ctx.lineTo(arrowEndX - 5 * Math.cos(headingRad - Math.PI/6), arrowEndY + 5 * Math.sin(headingRad - Math.PI/6));
        ctx.lineTo(arrowEndX - 5 * Math.cos(headingRad + Math.PI/6), arrowEndY + 5 * Math.sin(headingRad + Math.PI/6));
        ctx.closePath();
        ctx.fill();
      }
      
      // Draw position label
      ctx.fillStyle = '#f3f4f6';
      ctx.font = '14px Inter, system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Current Position', pos.x, pos.y - 15);
    }
  };

  useEffect(() => {
    drawMap();
  }, [gpsData, waypoints]);

  if (isLoading) {
    return (
      <div className="w-full h-full flex items-center justify-center bg-gray-900">
        <div className="text-lg text-white">Loading GPS data...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="w-full h-full flex items-center justify-center bg-gray-900">
        <div className="text-red-400">Error: {error}</div>
      </div>
    );
  }

  return (
    <div className="w-full h-full flex flex-col bg-gray-900 text-white p-4">
      {/* Header */}
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-xl font-bold text-white">GPS Navigation</h2>
        <div className="flex gap-2">
          <button
            onClick={() => setShowWaypointManager(!showWaypointManager)}
            className="bg-gray-700 hover:bg-gray-600 text-white px-4 py-2 rounded-lg transition-colors"
          >
            {showWaypointManager ? 'Hide' : 'Show'} Waypoints
          </button>
          <button
            onClick={() => setShowAddForm(!showAddForm)}
            className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition-colors"
          >
            {showAddForm ? 'Cancel' : 'Add Waypoint'}
          </button>
        </div>
      </div>

      {/* GPS Status */}
      <div className="bg-gray-800 p-4 rounded-lg mb-4 border border-gray-700">
        <div className="grid grid-cols-2 md:grid-cols-3 gap-4 text-sm">
          <div className="flex flex-col">
            <span className="text-gray-400 font-medium">Latitude</span>
            <span className="text-white font-semibold">{gpsData?.latitude?.toFixed(6) || 'N/A'}</span>
          </div>
          <div className="flex flex-col">
            <span className="text-gray-400 font-medium">Longitude</span>
            <span className="text-white font-semibold">{gpsData?.longitude?.toFixed(6) || 'N/A'}</span>
          </div>
          <div className="flex flex-col">
            <span className="text-gray-400 font-medium">Heading</span>
            <span className="text-white font-semibold">{gpsData?.heading?.toFixed(1) || 'N/A'}°</span>
          </div>
          <div className="flex flex-col">
            <span className="text-gray-400 font-medium">Accuracy</span>
            <span className="text-white font-semibold">{gpsData?.accuracy?.toFixed(1) || 'N/A'}m</span>
          </div>
          <div className="flex flex-col">
            <span className="text-gray-400 font-medium">Satellites</span>
            <span className="text-white font-semibold">{gpsStatus?.satellites || 'N/A'}</span>
          </div>
          <div className="flex flex-col">
            <span className="text-gray-400 font-medium">Fix Quality</span>
            <span className={`font-semibold ${gpsStatus?.has_fix ? 'text-green-400' : 'text-red-400'}`}>
              {gpsStatus?.has_fix ? 'Good' : 'Poor'}
            </span>
          </div>
        </div>
      </div>

      {/* Add Waypoint Form */}
      {showAddForm && (
        <div className="bg-gray-800 p-4 rounded-lg mb-4 border border-gray-700">
          <h3 className="font-semibold mb-3 text-white">Add New Waypoint</h3>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-3">
            <input
              type="number"
              step="any"
              placeholder="Latitude"
              value={newWaypoint.latitude}
              onChange={(e) => setNewWaypoint({...newWaypoint, latitude: e.target.value})}
              className="px-3 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            />
            <input
              type="number"
              step="any"
              placeholder="Longitude"
              value={newWaypoint.longitude}
              onChange={(e) => setNewWaypoint({...newWaypoint, longitude: e.target.value})}
              className="px-3 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            />
            <input
              type="text"
              placeholder="Name (optional)"
              value={newWaypoint.name}
              onChange={(e) => setNewWaypoint({...newWaypoint, name: e.target.value})}
              className="px-3 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            />
          </div>
          <button
            onClick={addWaypoint}
            disabled={!newWaypoint.latitude || !newWaypoint.longitude}
            className="mt-3 bg-green-600 hover:bg-green-700 disabled:bg-gray-600 text-white px-4 py-2 rounded-lg font-medium transition-colors"
          >
            Add Waypoint
          </button>
        </div>
      )}

      {/* Main Content Area */}
      <div className="flex-1 flex gap-4 min-h-0">
        {/* Map Canvas */}
        <div className="flex-1 bg-gray-800 border border-gray-700 rounded-lg overflow-hidden relative">
          <canvas
            ref={canvasRef}
            width={mapSize}
            height={mapSize}
            className="w-full h-full cursor-crosshair"
            onMouseMove={handleCanvasMouseMove}
            onMouseLeave={handleCanvasMouseLeave}
          />
          
          {/* Tooltip */}
          {tooltip && (
            <div
              className="absolute z-10 bg-gray-900 text-white p-2 rounded-lg shadow-lg border border-gray-700 text-sm"
              style={{
                left: tooltip.x + 10,
                top: tooltip.y - 10,
                transform: 'translateY(-100%)'
              }}
            >
              <div className="whitespace-pre-line">{tooltip.text}</div>
            </div>
          )}
        </div>

        {/* Waypoint Manager Sidebar */}
        {showWaypointManager && (
          <div className="w-80 bg-gray-800 border border-gray-700 rounded-lg p-4 overflow-hidden">
            <div className="flex justify-between items-center mb-4">
              <h3 className="font-semibold text-white">Waypoints ({waypoints.length})</h3>
              <button
                onClick={() => setShowWaypointManager(false)}
                className="text-gray-400 hover:text-white"
              >
                ×
              </button>
            </div>
            
            {waypoints.length === 0 ? (
              <div className="text-gray-400 text-center py-8">
                No waypoints added yet
              </div>
            ) : (
              <div className="space-y-2 max-h-96 overflow-y-auto">
                {waypoints.map(waypoint => (
                  <div key={waypoint.id} className="bg-gray-700 p-3 rounded-lg border border-gray-600">
                    <div className="flex justify-between items-start mb-2">
                      <div className="font-medium text-white">{waypoint.name}</div>
                      <button
                        onClick={() => removeWaypoint(waypoint.id)}
                        className="text-red-400 hover:text-red-300 ml-2"
                      >
                        ×
                      </button>
                    </div>
                    <div className="text-sm text-gray-400">
                      <div>Lat: {waypoint.latitude.toFixed(6)}</div>
                      <div>Lng: {waypoint.longitude.toFixed(6)}</div>
                      <div className="text-xs text-gray-500 mt-1">
                        Added: {new Date(waypoint.timestamp).toLocaleString()}
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default GPSMap;
