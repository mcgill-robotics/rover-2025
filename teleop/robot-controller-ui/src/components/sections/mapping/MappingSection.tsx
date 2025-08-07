'use client';

import React, { useState, useEffect } from 'react';
import { MapPin, Navigation, Target, Layers, Download, Upload } from 'lucide-react';
import OfflineMap from './OfflineMap';
import GPSDisplay from './GPSDisplay';
import WaypointManager from './WaypointManager';
import { useGPSData } from '@/hooks/useGPSData';

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

const MappingSection: React.FC = () => {
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [selectedWaypoint, setSelectedWaypoint] = useState<Waypoint | null>(null);
  const [mapStyle, setMapStyle] = useState<'satellite' | 'street' | 'terrain'>('street');
  const [isMapLoaded, setIsMapLoaded] = useState(false);

  // Use real GPS data from the GPS service
  const { gpsData, gpsStatus, isLoading: gpsLoading, error: gpsError } = useGPSData();

  const handleAddWaypoint = (lat: number, lng: number) => {
    const newWaypoint: Waypoint = {
      id: `wp_${Date.now()}`,
      latitude: lat,
      longitude: lng,
      name: `Waypoint ${waypoints.length + 1}`,
      description: `Added at ${new Date().toLocaleTimeString()}`
    };
    setWaypoints([...waypoints, newWaypoint]);
  };

  const handleDeleteWaypoint = (id: string) => {
    setWaypoints(waypoints.filter(wp => wp.id !== id));
    if (selectedWaypoint?.id === id) {
      setSelectedWaypoint(null);
    }
  };

  const handleExportWaypoints = () => {
    const dataStr = JSON.stringify(waypoints, null, 2);
    const dataBlob = new Blob([dataStr], { type: 'application/json' });
    const url = URL.createObjectURL(dataBlob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `waypoints_${new Date().toISOString().split('T')[0]}.json`;
    link.click();
    URL.revokeObjectURL(url);
  };

  const handleImportWaypoints = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (file) {
      const reader = new FileReader();
      reader.onload = (e) => {
        try {
          const importedWaypoints = JSON.parse(e.target?.result as string);
          setWaypoints(importedWaypoints);
        } catch (error) {
          console.error('Error importing waypoints:', error);
          alert('Error importing waypoints. Please check the file format.');
        }
      };
      reader.readAsText(file);
    }
  };

  return (
    <div className="flex flex-col h-full bg-gray-50">
      {/* Header */}
      <div className="bg-white border-b border-gray-200 p-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-3">
            <MapPin className="w-6 h-6 text-blue-600" />
            <h1 className="text-2xl font-bold text-gray-900">Navigation</h1>
          </div>
          <div className="flex items-center space-x-2">
            <button
              onClick={handleExportWaypoints}
              className="flex items-center space-x-2 px-3 py-2 text-sm bg-green-600 text-white rounded-lg hover:bg-green-700 transition-colors"
            >
              <Download className="w-4 h-4" />
              <span>Export</span>
            </button>
            <label className="flex items-center space-x-2 px-3 py-2 text-sm bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors cursor-pointer">
              <Upload className="w-4 h-4" />
              <span>Import</span>
              <input
                type="file"
                accept=".json"
                onChange={handleImportWaypoints}
                className="hidden"
              />
            </label>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 flex">
        {/* Map Container */}
        <div className="flex-1 relative">
          <OfflineMap
            gpsData={gpsData}
            waypoints={waypoints}
            selectedWaypoint={selectedWaypoint}
            onWaypointSelect={setSelectedWaypoint}
            onAddWaypoint={handleAddWaypoint}
            mapStyle={mapStyle}
            onMapLoad={setIsMapLoaded}
          />
          
          {/* Map Controls Overlay */}
          <div className="absolute top-4 right-4 bg-white rounded-lg shadow-lg p-2">
            <div className="flex flex-col space-y-2">
              <button
                onClick={() => setMapStyle('street')}
                className={`px-3 py-2 text-sm rounded ${mapStyle === 'street' ? 'bg-blue-600 text-white' : 'bg-gray-100 text-gray-700 hover:bg-gray-200'}`}
              >
                Street
              </button>
              <button
                onClick={() => setMapStyle('satellite')}
                className={`px-3 py-2 text-sm rounded ${mapStyle === 'satellite' ? 'bg-blue-600 text-white' : 'bg-gray-100 text-gray-700 hover:bg-gray-200'}`}
              >
                Satellite
              </button>
              <button
                onClick={() => setMapStyle('terrain')}
                className={`px-3 py-2 text-sm rounded ${mapStyle === 'terrain' ? 'bg-blue-600 text-white' : 'bg-gray-100 text-gray-700 hover:bg-gray-200'}`}
              >
                Terrain
              </button>
            </div>
          </div>

          {/* GPS Status Indicator */}
          <div className="absolute bottom-4 left-4 bg-white rounded-lg shadow-lg p-3">
            <GPSDisplay gpsData={gpsData} isMapLoaded={isMapLoaded} />
          </div>
        </div>

        {/* Sidebar */}
        <div className="w-80 bg-white border-l border-gray-200 p-4 overflow-y-auto">
          <WaypointManager
            waypoints={waypoints}
            selectedWaypoint={selectedWaypoint}
            onWaypointSelect={setSelectedWaypoint}
            onDeleteWaypoint={handleDeleteWaypoint}
            gpsData={gpsData}
          />
        </div>
      </div>
    </div>
  );
};

export default MappingSection; 