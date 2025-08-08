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

const GPSMap: React.FC = () => {
  const { gpsData, gpsStatus, isLoading, error } = useGPSData();
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [newWaypoint, setNewWaypoint] = useState({ latitude: '', longitude: '', name: '' });
  const [showAddForm, setShowAddForm] = useState(false);
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

  const drawMap = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.clearRect(0, 0, mapSize, mapSize);

    // Draw background grid
    ctx.strokeStyle = '#e0e0e0';
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
      
      // Draw waypoint circle
      ctx.fillStyle = '#ff6b6b';
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 6, 0, 2 * Math.PI);
      ctx.fill();
      
      // Draw waypoint border
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.stroke();
      
      // Draw waypoint label
      ctx.fillStyle = '#333333';
      ctx.font = '12px Arial';
      ctx.textAlign = 'center';
      ctx.fillText(waypoint.name, pos.x, pos.y - 10);
    });

    // Draw current position
    if (gpsData && gpsData.latitude !== 0 && gpsData.longitude !== 0) {
      const pos = latLngToPixel(gpsData.latitude, gpsData.longitude, centerLat, centerLng);
      
      // Draw position circle
      ctx.fillStyle = '#4CAF50';
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 8, 0, 2 * Math.PI);
      ctx.fill();
      
      // Draw position border
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 3;
      ctx.stroke();
      
      // Draw heading indicator
      if (gpsData.heading !== 0) {
        const headingRad = (gpsData.heading * Math.PI) / 180;
        const arrowLength = 20;
        const arrowEndX = pos.x + Math.cos(headingRad) * arrowLength;
        const arrowEndY = pos.y - Math.sin(headingRad) * arrowLength;
        
        ctx.strokeStyle = '#2196F3';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(pos.x, pos.y);
        ctx.lineTo(arrowEndX, arrowEndY);
        ctx.stroke();
        
        // Draw arrowhead
        ctx.fillStyle = '#2196F3';
        ctx.beginPath();
        ctx.moveTo(arrowEndX, arrowEndY);
        ctx.lineTo(arrowEndX - 5 * Math.cos(headingRad - Math.PI/6), arrowEndY + 5 * Math.sin(headingRad - Math.PI/6));
        ctx.lineTo(arrowEndX - 5 * Math.cos(headingRad + Math.PI/6), arrowEndY + 5 * Math.sin(headingRad + Math.PI/6));
        ctx.closePath();
        ctx.fill();
      }
      
      // Draw position label
      ctx.fillStyle = '#333333';
      ctx.font = '14px Arial';
      ctx.textAlign = 'center';
      ctx.fillText('Current Position', pos.x, pos.y - 15);
    }
  };

  useEffect(() => {
    drawMap();
  }, [gpsData, waypoints]);

  if (isLoading) {
    return (
      <div className="w-full h-full flex items-center justify-center">
        <div className="text-lg">Loading GPS data...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="w-full h-full flex items-center justify-center">
        <div className="text-red-500">Error: {error}</div>
      </div>
    );
  }

  return (
    <div className="w-full h-full flex flex-col p-4">
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-xl font-bold">GPS Navigation</h2>
        <button
          onClick={() => setShowAddForm(!showAddForm)}
          className="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded"
        >
          {showAddForm ? 'Cancel' : 'Add Waypoint'}
        </button>
      </div>

      {/* GPS Status */}
      <div className="bg-gray-100 p-3 rounded mb-4">
        <div className="grid grid-cols-2 gap-4 text-sm">
          <div>
            <span className="font-semibold">Latitude:</span> {gpsData?.latitude?.toFixed(6) || 'N/A'}
          </div>
          <div>
            <span className="font-semibold">Longitude:</span> {gpsData?.longitude?.toFixed(6) || 'N/A'}
          </div>
          <div>
            <span className="font-semibold">Heading:</span> {gpsData?.heading?.toFixed(1) || 'N/A'}°
          </div>
          <div>
            <span className="font-semibold">Accuracy:</span> {gpsData?.accuracy?.toFixed(1) || 'N/A'}m
          </div>
          <div>
            <span className="font-semibold">Satellites:</span> {gpsStatus?.satellites || 'N/A'}
          </div>
          <div>
            <span className="font-semibold">Fix Quality:</span> {gpsStatus?.has_fix ? 'Good' : 'Poor'}
          </div>
        </div>
      </div>

      {/* Add Waypoint Form */}
      {showAddForm && (
        <div className="bg-blue-50 p-4 rounded mb-4">
          <h3 className="font-semibold mb-2">Add New Waypoint</h3>
          <div className="grid grid-cols-3 gap-2">
            <input
              type="number"
              step="any"
              placeholder="Latitude"
              value={newWaypoint.latitude}
              onChange={(e) => setNewWaypoint({...newWaypoint, latitude: e.target.value})}
              className="px-3 py-2 border rounded"
            />
            <input
              type="number"
              step="any"
              placeholder="Longitude"
              value={newWaypoint.longitude}
              onChange={(e) => setNewWaypoint({...newWaypoint, longitude: e.target.value})}
              className="px-3 py-2 border rounded"
            />
            <input
              type="text"
              placeholder="Name (optional)"
              value={newWaypoint.name}
              onChange={(e) => setNewWaypoint({...newWaypoint, name: e.target.value})}
              className="px-3 py-2 border rounded"
            />
          </div>
          <button
            onClick={addWaypoint}
            disabled={!newWaypoint.latitude || !newWaypoint.longitude}
            className="mt-2 bg-green-500 hover:bg-green-600 disabled:bg-gray-300 text-white px-4 py-2 rounded"
          >
            Add Waypoint
          </button>
        </div>
      )}

      {/* Map Canvas */}
      <div className="flex-1 bg-white border rounded-lg overflow-hidden">
        <canvas
          ref={canvasRef}
          width={mapSize}
          height={mapSize}
          className="w-full h-full"
        />
      </div>

      {/* Waypoints List */}
      {waypoints.length > 0 && (
        <div className="mt-4">
          <h3 className="font-semibold mb-2">Waypoints ({waypoints.length})</h3>
          <div className="max-h-32 overflow-y-auto">
            {waypoints.map(waypoint => (
              <div key={waypoint.id} className="flex justify-between items-center bg-gray-50 p-2 rounded mb-1">
                <div>
                  <span className="font-medium">{waypoint.name}</span>
                  <span className="text-sm text-gray-600 ml-2">
                    {waypoint.latitude.toFixed(6)}, {waypoint.longitude.toFixed(6)}
                  </span>
                </div>
                <button
                  onClick={() => removeWaypoint(waypoint.id)}
                  className="text-red-500 hover:text-red-700"
                >
                  ×
                </button>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default GPSMap;
