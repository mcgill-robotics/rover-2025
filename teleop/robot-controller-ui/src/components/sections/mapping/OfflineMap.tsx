'use client';

import React, { useEffect, useRef, useState } from 'react';
import maplibregl from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';

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

interface OfflineMapProps {
  gpsData: GPSData | null;
  waypoints: Waypoint[];
  selectedWaypoint: Waypoint | null;
  onWaypointSelect: (waypoint: Waypoint | null) => void;
  onAddWaypoint: (lat: number, lng: number) => void;
  mapStyle: 'satellite' | 'street' | 'terrain';
  onMapLoad: (loaded: boolean) => void;
}

const OfflineMap: React.FC<OfflineMapProps> = ({
  gpsData,
  waypoints,
  selectedWaypoint,
  onWaypointSelect,
  onAddWaypoint,
  mapStyle,
  onMapLoad
}) => {
  const mapContainer = useRef<HTMLDivElement>(null);
  const map = useRef<maplibregl.Map | null>(null);
  const roverMarker = useRef<maplibregl.Marker | null>(null);
  const waypointMarkers = useRef<{ [key: string]: maplibregl.Marker }>({});
  const accuracyCircle = useRef<maplibregl.Circle | null>(null);

  // Offline tile server configuration
  const getMapStyle = (style: string) => {
    // For development, we'll use online tiles. In production, replace with your TileServer-GL URL
    const baseUrl = process.env.NODE_ENV === 'production' 
      ? 'http://localhost:8080' // Your TileServer-GL URL
      : 'https://tiles.stadiamaps.com'; // Fallback to online tiles for development

    const styles = {
      street: {
        version: 8,
        sources: {
          'osm': {
            type: 'raster',
            tiles: [
              `${baseUrl}/styles/osm-bright/{z}/{x}/{y}.png`
            ],
            tileSize: 256,
            attribution: '© OpenStreetMap contributors'
          }
        },
        layers: [
          {
            id: 'osm',
            type: 'raster',
            source: 'osm',
            minzoom: 0,
            maxzoom: 22
          }
        ]
      },
      satellite: {
        version: 8,
        sources: {
          'satellite': {
            type: 'raster',
            tiles: [
              `${baseUrl}/styles/satellite/{z}/{x}/{y}.png`
            ],
            tileSize: 256,
            attribution: '© OpenStreetMap contributors'
          }
        },
        layers: [
          {
            id: 'satellite',
            type: 'raster',
            source: 'satellite',
            minzoom: 0,
            maxzoom: 22
          }
        ]
      },
      terrain: {
        version: 8,
        sources: {
          'terrain': {
            type: 'raster',
            tiles: [
              `${baseUrl}/styles/terrain/{z}/{x}/{y}.png`
            ],
            tileSize: 256,
            attribution: '© OpenStreetMap contributors'
          }
        },
        layers: [
          {
            id: 'terrain',
            type: 'raster',
            source: 'terrain',
            minzoom: 0,
            maxzoom: 22
          }
        ]
      }
    };

    return styles[style as keyof typeof styles] || styles.street;
  };

  // Initialize map
  useEffect(() => {
    if (!mapContainer.current || map.current) return;

    map.current = new maplibregl.Map({
      container: mapContainer.current,
      style: getMapStyle(mapStyle),
      center: [-73.5772, 45.5048], // McGill University coordinates
      zoom: 15,
      attributionControl: true
    });

    map.current.on('load', () => {
      onMapLoad(true);
      
      // Add navigation controls
      map.current?.addControl(new maplibregl.NavigationControl(), 'top-left');
      
      // Add fullscreen control
      map.current?.addControl(new maplibregl.FullscreenControl(), 'top-right');

      // Add click handler for adding waypoints
      map.current?.on('click', (e) => {
        const { lng, lat } = e.lngLat;
        onAddWaypoint(lat, lng);
      });

      // Create rover marker
      const roverEl = document.createElement('div');
      roverEl.className = 'rover-marker';
      roverEl.innerHTML = `
        <div style="
          width: 20px;
          height: 20px;
          background: #3b82f6;
          border: 3px solid white;
          border-radius: 50%;
          box-shadow: 0 2px 4px rgba(0,0,0,0.3);
          position: relative;
        ">
          <div style="
            position: absolute;
            top: 50%;
            left: 50%;
            width: 2px;
            height: 12px;
            background: white;
            transform: translate(-50%, -50%) rotate(0deg);
            transform-origin: center bottom;
          "></div>
        </div>
      `;

      roverMarker.current = new maplibregl.Marker({
        element: roverEl,
        anchor: 'center'
      }).setLngLat([-73.5772, 45.5048]).addTo(map.current);

      // Create accuracy circle
      accuracyCircle.current = new maplibregl.Circle({
        'circle-radius': 10,
        'circle-color': '#3b82f6',
        'circle-opacity': 0.2,
        'circle-stroke-width': 1,
        'circle-stroke-color': '#3b82f6'
      }).addTo(map.current);
    });

    return () => {
      if (map.current) {
        map.current.remove();
        map.current = null;
      }
    };
  }, []);

  // Update map style
  useEffect(() => {
    if (map.current) {
      map.current.setStyle(getMapStyle(mapStyle));
    }
  }, [mapStyle]);

  // Update rover position
  useEffect(() => {
    if (gpsData && roverMarker.current && map.current) {
      const { longitude, latitude, heading } = gpsData;
      
      // Update rover marker position
      roverMarker.current.setLngLat([longitude, latitude]);
      
      // Update rover heading
      const roverEl = roverMarker.current.getElement();
      const headingIndicator = roverEl?.querySelector('div > div') as HTMLElement;
      if (headingIndicator) {
        headingIndicator.style.transform = `translate(-50%, -50%) rotate(${heading}deg)`;
      }

      // Update accuracy circle
      if (accuracyCircle.current) {
        accuracyCircle.current.setCenter([longitude, latitude]);
        accuracyCircle.current.setRadius(gpsData.accuracy);
      }

      // Center map on rover if it's the first GPS fix
      if (!map.current.isMoving()) {
        map.current.flyTo({
          center: [longitude, latitude],
          zoom: 16,
          duration: 1000
        });
      }
    }
  }, [gpsData]);

  // Update waypoint markers
  useEffect(() => {
    // Remove old markers
    Object.values(waypointMarkers.current).forEach(marker => {
      marker.remove();
    });
    waypointMarkers.current = {};

    // Add new markers
    waypoints.forEach(waypoint => {
      const waypointEl = document.createElement('div');
      waypointEl.className = 'waypoint-marker';
      waypointEl.innerHTML = `
        <div style="
          width: 16px;
          height: 16px;
          background: ${selectedWaypoint?.id === waypoint.id ? '#ef4444' : '#10b981'};
          border: 2px solid white;
          border-radius: 50%;
          box-shadow: 0 2px 4px rgba(0,0,0,0.3);
          cursor: pointer;
        "></div>
      `;

      waypointEl.addEventListener('click', () => {
        onWaypointSelect(waypoint);
      });

      const marker = new maplibregl.Marker({
        element: waypointEl,
        anchor: 'center'
      })
        .setLngLat([waypoint.longitude, waypoint.latitude])
        .addTo(map.current!);

      waypointMarkers.current[waypoint.id] = marker;
    });
  }, [waypoints, selectedWaypoint, onWaypointSelect]);

  return (
    <div className="relative w-full h-full">
      <div ref={mapContainer} className="w-full h-full" />
      
      {/* Map Instructions */}
      <div className="absolute top-4 left-4 bg-white bg-opacity-90 rounded-lg p-3 text-sm text-gray-700">
        <p>Click on the map to add waypoints</p>
        <p>Blue dot shows rover position</p>
        <p>Green dots are waypoints</p>
      </div>
    </div>
  );
};

export default OfflineMap; 