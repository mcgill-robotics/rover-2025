import { MapConfig, MapStyle, MapStyleConfig } from '@/types/navigation';

export const MAP_CONFIG: MapConfig = {
  defaultCenter: [-73.5772, 45.5048], // McGill University coordinates
  defaultZoom: 15,
  minZoom: 10,
  maxZoom: 22,
  tileServerUrl: process.env.NODE_ENV === 'production'
    ? 'http://localhost:8080' // TileServer-GL URL
    : 'https://tiles.stadiamaps.com' // Online tiles for development
};

export const getMapStyle = (style: MapStyle): MapStyleConfig => {
  const baseUrl = MAP_CONFIG.tileServerUrl;

  const styles: Record<MapStyle, MapStyleConfig> = {
    street: {
      version: 8,
      sources: {
        'osm': {
          type: 'raster',
          tiles: [`${baseUrl}/styles/osm-bright/{z}/{x}/{y}.png`],
          tileSize: 256,
          attribution: '© OpenStreetMap contributors'
        }
      },
      layers: [{
        id: 'osm',
        type: 'raster',
        source: 'osm',
        minzoom: MAP_CONFIG.minZoom,
        maxzoom: MAP_CONFIG.maxZoom
      }]
    },
    satellite: {
      version: 8,
      sources: {
        'satellite': {
          type: 'raster',
          tiles: [`${baseUrl}/styles/satellite/{z}/{x}/{y}.png`],
          tileSize: 256,
          attribution: '© OpenStreetMap contributors'
        }
      },
      layers: [{
        id: 'satellite',
        type: 'raster',
        source: 'satellite',
        minzoom: MAP_CONFIG.minZoom,
        maxzoom: MAP_CONFIG.maxZoom
      }]
    },
    terrain: {
      version: 8,
      sources: {
        'terrain': {
          type: 'raster',
          tiles: [`${baseUrl}/styles/terrain/{z}/{x}/{y}.png`],
          tileSize: 256,
          attribution: '© OpenStreetMap contributors'
        }
      },
      layers: [{
        id: 'terrain',
        type: 'raster',
        source: 'terrain',
        minzoom: MAP_CONFIG.minZoom,
        maxzoom: MAP_CONFIG.maxZoom
      }]
    }
  };

  return styles[style];
};

export const GPS_THRESHOLDS = {
  POOR_ACCURACY: 20, // meters
  FAIR_ACCURACY: 10  // meters
};

export const WAYPOINT_COLORS = {
  DEFAULT: '#10b981',  // Green
  SELECTED: '#ef4444', // Red
  CLOSEST: '#3b82f6'   // Blue
};

export const MAP_UPDATE_INTERVAL = 1000; // ms
export const GPS_UPDATE_INTERVAL = 1000; // ms
