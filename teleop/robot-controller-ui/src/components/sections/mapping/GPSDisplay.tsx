'use client';

import React from 'react';
import { Navigation, Satellite, Clock, Target } from 'lucide-react';

interface GPSData {
  latitude: number;
  longitude: number;
  heading: number;
  accuracy: number;
  timestamp: number;
}

interface GPSDisplayProps {
  gpsData: GPSData | null;
  isMapLoaded: boolean;
}

const GPSDisplay: React.FC<GPSDisplayProps> = ({ gpsData, isMapLoaded }) => {
  const formatCoordinate = (coord: number, isLatitude: boolean) => {
    const direction = isLatitude 
      ? (coord >= 0 ? 'N' : 'S')
      : (coord >= 0 ? 'E' : 'W');
    
    const absCoord = Math.abs(coord);
    const degrees = Math.floor(absCoord);
    const minutes = (absCoord - degrees) * 60;
    
    return `${degrees}° ${minutes.toFixed(4)}' ${direction}`;
  };

  const formatHeading = (heading: number) => {
    const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
    const index = Math.round(heading / 45) % 8;
    return `${Math.round(heading)}° ${directions[index]}`;
  };

  const getGPSStatus = () => {
    if (!gpsData) return { status: 'No GPS', color: 'text-red-500', icon: 'disconnected' };
    if (gpsData.accuracy > 20) return { status: 'Poor Signal', color: 'text-yellow-500', icon: 'weak' };
    if (gpsData.accuracy > 10) return { status: 'Fair Signal', color: 'text-orange-500', icon: 'fair' };
    return { status: 'Good Signal', color: 'text-green-500', icon: 'good' };
  };

  const gpsStatus = getGPSStatus();

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