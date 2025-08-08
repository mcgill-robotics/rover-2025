export interface GPSData {
  latitude: number;
  longitude: number;
  heading: number;
  accuracy: number;
  timestamp: number;
}

export interface Waypoint {
  id: string;
  latitude: number;
  longitude: number;
  name: string;
  description?: string;
}

export interface MapConfig {
  defaultCenter: [number, number];  // [longitude, latitude]
  defaultZoom: number;
  minZoom: number;
  maxZoom: number;
  tileServerUrl: string;
}

export type MapStyle = 'satellite' | 'street' | 'terrain';

export interface MapStyleConfig {
  version: number;
  sources: {
    [key: string]: {
      type: string;
      tiles: string[];
      tileSize: number;
      attribution: string;
    };
  };
  layers: {
    id: string;
    type: string;
    source: string;
    minzoom: number;
    maxzoom: number;
  }[];
}

export interface GPSStatus {
  status: 'No GPS' | 'Poor Signal' | 'Fair Signal' | 'Good Signal';
  color: string;
  icon: 'disconnected' | 'weak' | 'fair' | 'good';
}

export interface WaypointEdit {
  id: string;
  name: string;
  description: string;
}
