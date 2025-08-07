'use client';

import React from 'react';
import { Navigation, Satellite, Clock, Target } from 'lucide-react';
import { GPSData } from '@/types/navigation';
import { formatCoordinate, formatHeading, getGPSStatus } from '@/utils/navigation';

interface GPSDisplayProps {
  gpsData: GPSData | null;
  isMapLoaded: boolean;
  isLoading: boolean;
  error: Error | null;
}

const GPSDisplay: React.FC<GPSDisplayProps> = ({
  gpsData,
  isMapLoaded,
  isLoading,
  error
}) => {
  const gpsStatus = getGPSStatus(gpsData);

  if (error) {
    return (
      <div className="text-red-500 text-sm">
        <Satellite className="w-4 h-4 mb-1" />
        <span>GPS Error: {error.message}</span>
      </div>
    );
  }

  if (isLoading) {
    return (
      <div className="text-blue-500 text-sm animate-pulse">
        <Satellite className="w-4 h-4 mb-1" />
        <span>Loading GPS data...</span>
      </div>
    );
  }

  return (
    <div className="space-y-3">
      {/* GPS Status */}
      <div className="flex items-center space-x-2">
        <Satellite className={`w-4 h-4 ${gpsStatus.color}`} />
        <span className={`text-sm font-medium ${gpsStatus.color}`}>
          {gpsStatus.status}
        </span>
      </div>

      {/* Coordinates */}
      {gpsData && (
        <div className="space-y-2">
          <div className="flex items-center space-x-2">
            <Target className="w-4 h-4 text-gray-500" />
            <div className="text-xs space-y-1">
              <div className="font-mono">
                Lat: {formatCoordinate(gpsData.latitude, true)}
              </div>
              <div className="font-mono">
                Lon: {formatCoordinate(gpsData.longitude, false)}
              </div>
            </div>
          </div>

          {/* Heading */}
          <div className="flex items-center space-x-2">
            <Navigation className="w-4 h-4 text-gray-500" />
            <span className="text-xs font-mono">
              {formatHeading(gpsData.heading)}
            </span>
          </div>

          {/* Accuracy */}
          <div className="text-xs text-gray-600">
            Accuracy: ±{gpsData.accuracy.toFixed(1)}m
          </div>

          {/* Timestamp */}
          <div className="flex items-center space-x-2">
            <Clock className="w-3 h-3 text-gray-500" />
            <span className="text-xs text-gray-600">
              {new Date(gpsData.timestamp).toLocaleTimeString()}
            </span>
          </div>
        </div>
      )}

      {/* Map Status */}
      <div className="flex items-center space-x-2 pt-2 border-t border-gray-200">
        <div className={`w-2 h-2 rounded-full ${isMapLoaded ? 'bg-green-500' : 'bg-yellow-500'}`} />
        <span className="text-xs text-gray-600">
          {isMapLoaded ? 'Map Loaded' : 'Loading Map...'}
        </span>
      </div>
    </div>
  );
};

export default GPSDisplay;