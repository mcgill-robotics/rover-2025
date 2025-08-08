import { GPSData, Waypoint, GPSStatus } from '@/types/navigation';
import { GPS_THRESHOLDS } from '@/config/navigation';

/**
 * Calculate distance between two points using the Haversine formula.
 */
export const calculateDistance = (lat1: number, lon1: number, lat2: number, lon2: number): number => {
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

/**
 * Calculate bearing between two points.
 */
export const calculateBearing = (lat1: number, lon1: number, lat2: number, lon2: number): number => {
  const φ1 = lat1 * Math.PI / 180;
  const φ2 = lat2 * Math.PI / 180;
  const Δλ = (lon2 - lon1) * Math.PI / 180;

  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) -
            Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  
  return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
};

/**
 * Format coordinate as degrees and minutes.
 */
export const formatCoordinate = (coord: number, isLatitude: boolean): string => {
  const direction = isLatitude 
    ? (coord >= 0 ? 'N' : 'S')
    : (coord >= 0 ? 'E' : 'W');
  
  const absCoord = Math.abs(coord);
  const degrees = Math.floor(absCoord);
  const minutes = (absCoord - degrees) * 60;
  
  return `${degrees}° ${minutes.toFixed(4)}' ${direction}`;
};

/**
 * Format heading with cardinal direction.
 */
export const formatHeading = (heading: number): string => {
  const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  const index = Math.round(heading / 45) % 8;
  return `${Math.round(heading)}° ${directions[index]}`;
};

/**
 * Get GPS status based on accuracy.
 */
export const getGPSStatus = (gpsData: GPSData | null): GPSStatus => {
  if (!gpsData) {
    return { 
      status: 'No GPS',
      color: 'text-red-500',
      icon: 'disconnected'
    };
  }

  if (gpsData.accuracy > GPS_THRESHOLDS.POOR_ACCURACY) {
    return {
      status: 'Poor Signal',
      color: 'text-yellow-500',
      icon: 'weak'
    };
  }

  if (gpsData.accuracy > GPS_THRESHOLDS.FAIR_ACCURACY) {
    return {
      status: 'Fair Signal',
      color: 'text-orange-500',
      icon: 'fair'
    };
  }

  return {
    status: 'Good Signal',
    color: 'text-green-500',
    icon: 'good'
  };
};

/**
 * Find closest waypoint to current position.
 */
export const findClosestWaypoint = (
  gpsData: GPSData | null,
  waypoints: Waypoint[]
): { waypoint: Waypoint; distance: number } | null => {
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

