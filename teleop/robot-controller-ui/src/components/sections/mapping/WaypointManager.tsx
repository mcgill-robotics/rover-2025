'use client';

import React, { useState } from 'react';
import { MapPin, Trash2, Navigation, Edit3, Plus, Target } from 'lucide-react';

interface GPSData {
  latitude: number;
  longitude: number;
  heading: number;
  accuracy: number;
  timestamp: number;
}

interface Waypoint {
  id: string;
  latitude: number;
  longitude: number;
  name: string;
  description?: string;
}

interface WaypointManagerProps {
  waypoints: Waypoint[];
  selectedWaypoint: Waypoint | null;
  onWaypointSelect: (waypoint: Waypoint | null) => void;
  onDeleteWaypoint: (id: string) => void;
  gpsData: GPSData | null;
}

const WaypointManager: React.FC<WaypointManagerProps> = ({
  waypoints,
  selectedWaypoint,
  onWaypointSelect,
  onDeleteWaypoint,
  gpsData
}) => {
  const [editingWaypoint, setEditingWaypoint] = useState<Waypoint | null>(null);
  const [editName, setEditName] = useState('');
  const [editDescription, setEditDescription] = useState('');

  const calculateDistance = (lat1: number, lon1: number, lat2: number, lon2: number) => {
    const R = 6371e3; // Earth's radius in meters
    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;
    const Δφ = (lat2 - lat1) * Math.PI / 180;
    const Δλ = (lon2 - lon1) * Math.PI / 180;

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c;
  };

  const calculateBearing = (lat1: number, lon1: number, lat2: number, lon2: number) => {
    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;
    const Δλ = (lon2 - lon1) * Math.PI / 180;

    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) -
              Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
    
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
  };

  const handleEditWaypoint = (waypoint: Waypoint) => {
    setEditingWaypoint(waypoint);
    setEditName(waypoint.name);
    setEditDescription(waypoint.description || '');
  };

  const handleSaveEdit = () => {
    if (editingWaypoint) {
      const updatedWaypoint = {
        ...editingWaypoint,
        name: editName,
        description: editDescription
      };
      
      // Update the waypoint in the list
      const updatedWaypoints = waypoints.map(wp => 
        wp.id === editingWaypoint.id ? updatedWaypoint : wp
      );
      
      // This would need to be handled by the parent component
      // For now, we'll just close the edit mode
      setEditingWaypoint(null);
      setEditName('');
      setEditDescription('');
    }
  };

  const handleCancelEdit = () => {
    setEditingWaypoint(null);
    setEditName('');
    setEditDescription('');
  };

  const getClosestWaypoint = () => {
    if (!gpsData || waypoints.length === 0) return null;
    
    return waypoints.reduce((closest, waypoint) => {
      const distance = calculateDistance(
        gpsData.latitude, gpsData.longitude,
        waypoint.latitude, waypoint.longitude
      );
      
      if (!closest || distance < closest.distance) {
        return { waypoint, distance };
      }
      return closest;
    }, null as { waypoint: Waypoint; distance: number } | null);
  };

  const closestWaypoint = getClosestWaypoint();

  return (
    <div className="space-y-4">
      {/* Header */}
      <div className="flex items-center justify-between">
        <h2 className="text-lg font-semibold text-gray-900">Waypoints</h2>
        <span className="text-sm text-gray-500">{waypoints.length} total</span>
      </div>

      {/* Closest Waypoint Info */}
      {closestWaypoint && gpsData && (
        <div className="bg-blue-50 border border-blue-200 rounded-lg p-3">
          <div className="flex items-center space-x-2 mb-2">
            <Target className="w-4 h-4 text-blue-600" />
            <span className="text-sm font-medium text-blue-900">Closest Waypoint</span>
          </div>
          <div className="text-sm space-y-1">
            <div className="font-medium">{closestWaypoint.waypoint.name}</div>
            <div className="text-gray-600">
              Distance: {(closestWaypoint.distance / 1000).toFixed(2)} km
            </div>
            <div className="text-gray-600">
              Bearing: {calculateBearing(
                gpsData.latitude, gpsData.longitude,
                closestWaypoint.waypoint.latitude, closestWaypoint.waypoint.longitude
              ).toFixed(0)}°
            </div>
          </div>
        </div>
      )}

      {/* Waypoints List */}
      <div className="space-y-2 max-h-96 overflow-y-auto">
        {waypoints.length === 0 ? (
          <div className="text-center py-8 text-gray-500">
            <MapPin className="w-8 h-8 mx-auto mb-2 opacity-50" />
            <p>No waypoints yet</p>
            <p className="text-sm">Click on the map to add waypoints</p>
          </div>
        ) : (
          waypoints.map((waypoint) => {
            const isSelected = selectedWaypoint?.id === waypoint.id;
            const isEditing = editingWaypoint?.id === waypoint.id;
            const distance = gpsData ? calculateDistance(
              gpsData.latitude, gpsData.longitude,
              waypoint.latitude, waypoint.longitude
            ) : null;

            return (
              <div
                key={waypoint.id}
                className={`border rounded-lg p-3 cursor-pointer transition-colors ${
                  isSelected 
                    ? 'border-blue-500 bg-blue-50' 
                    : 'border-gray-200 hover:border-gray-300'
                }`}
                onClick={() => onWaypointSelect(waypoint)}
              >
                {isEditing ? (
                  <div className="space-y-2">
                    <input
                      type="text"
                      value={editName}
                      onChange={(e) => setEditName(e.target.value)}
                      className="w-full px-2 py-1 text-sm border border-gray-300 rounded"
                      placeholder="Waypoint name"
                    />
                    <textarea
                      value={editDescription}
                      onChange={(e) => setEditDescription(e.target.value)}
                      className="w-full px-2 py-1 text-sm border border-gray-300 rounded"
                      placeholder="Description (optional)"
                      rows={2}
                    />
                    <div className="flex space-x-2">
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          handleSaveEdit();
                        }}
                        className="px-2 py-1 text-xs bg-green-600 text-white rounded hover:bg-green-700"
                      >
                        Save
                      </button>
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          handleCancelEdit();
                        }}
                        className="px-2 py-1 text-xs bg-gray-600 text-white rounded hover:bg-gray-700"
                      >
                        Cancel
                      </button>
                    </div>
                  </div>
                ) : (
                  <div>
                    <div className="flex items-start justify-between">
                      <div className="flex-1">
                        <div className="flex items-center space-x-2">
                          <MapPin className="w-4 h-4 text-gray-500" />
                          <h3 className="font-medium text-gray-900">{waypoint.name}</h3>
                        </div>
                        {waypoint.description && (
                          <p className="text-sm text-gray-600 mt-1">{waypoint.description}</p>
                        )}
                        <div className="text-xs text-gray-500 mt-2 space-y-1">
                          <div>Lat: {waypoint.latitude.toFixed(6)}</div>
                          <div>Lon: {waypoint.longitude.toFixed(6)}</div>
                          {distance && (
                            <div>Distance: {(distance / 1000).toFixed(2)} km</div>
                          )}
                        </div>
                      </div>
                      <div className="flex space-x-1">
                        <button
                          onClick={(e) => {
                            e.stopPropagation();
                            handleEditWaypoint(waypoint);
                          }}
                          className="p-1 text-gray-400 hover:text-gray-600"
                        >
                          <Edit3 className="w-3 h-3" />
                        </button>
                        <button
                          onClick={(e) => {
                            e.stopPropagation();
                            onDeleteWaypoint(waypoint.id);
                          }}
                          className="p-1 text-gray-400 hover:text-red-600"
                        >
                          <Trash2 className="w-3 h-3" />
                        </button>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            );
          })
        )}
      </div>

      {/* Navigation Controls */}
      {selectedWaypoint && gpsData && (
        <div className="bg-gray-50 border border-gray-200 rounded-lg p-3">
          <div className="flex items-center space-x-2 mb-2">
            <Navigation className="w-4 h-4 text-gray-600" />
            <span className="text-sm font-medium text-gray-900">Navigation</span>
          </div>
          <div className="text-sm space-y-1">
            <div>Target: {selectedWaypoint.name}</div>
            <div>
              Distance: {(calculateDistance(
                gpsData.latitude, gpsData.longitude,
                selectedWaypoint.latitude, selectedWaypoint.longitude
              ) / 1000).toFixed(2)} km
            </div>
            <div>
              Bearing: {calculateBearing(
                gpsData.latitude, gpsData.longitude,
                selectedWaypoint.latitude, selectedWaypoint.longitude
              ).toFixed(0)}°
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default WaypointManager; 