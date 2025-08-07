'use client';

import React, { useEffect, useRef, useCallback } from 'react';
import maplibregl from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';
import { GPSData, Waypoint, MapStyle } from '@/types/navigation';
import { MAP_CONFIG, getMapStyle, WAYPOINT_COLORS, MAP_UPDATE_INTERVAL } from '@/config/navigation';

interface OfflineMapProps {
  gpsData: GPSData | null;
  waypoints: Waypoint[];
  selectedWaypoint: Waypoint | null;
  onWaypointSelect: (waypoint: Waypoint | null) => void;
  onAddWaypoint: (lat: number, lng: number) => void;
  mapStyle: MapStyle;
  onMapLoad: (loaded: boolean) => void;
}

export const OfflineMap: React.FC<OfflineMapProps> = ({
  gpsData,
  waypoints,
  selectedWaypoint,
  onWaypointSelect,
  onAddWaypoint,
  mapStyle,
  onMapLoad
}) => {
  // Refs
  const mapContainer = useRef<HTMLDivElement>(null);
  const map = useRef<maplibregl.Map | null>(null);
  const roverMarker = useRef<maplibregl.Marker | null>(null);
  const waypointMarkers = useRef<{ [key: string]: maplibregl.Marker }>({});
  const accuracyCircle = useRef<maplibregl.Circle | null>(null);
  const updateInterval = useRef<NodeJS.Timeout>();

  // Initialize map
  useEffect(() => {
    if (!mapContainer.current || map.current) return;

    try {
      map.current = new maplibregl.Map({
        container: mapContainer.current,
        style: getMapStyle(mapStyle),
        center: MAP_CONFIG.defaultCenter,
        zoom: MAP_CONFIG.defaultZoom,
        minZoom: MAP_CONFIG.minZoom,
        maxZoom: MAP_CONFIG.maxZoom,
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
        }).setLngLat(MAP_CONFIG.defaultCenter).addTo(map.current);

        // Create accuracy circle
        accuracyCircle.current = new maplibregl.Circle({
          'circle-radius': 10,
          'circle-color': '#3b82f6',
          'circle-opacity': 0.2,
          'circle-stroke-width': 1,
          'circle-stroke-color': '#3b82f6'
        }).addTo(map.current);
      });

      map.current.on('error', (e) => {
        console.error('Map error:', e);
        onMapLoad(false);
      });

    } catch (error) {
      console.error('Error initializing map:', error);
      onMapLoad(false);
    }

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
      try {
        map.current.setStyle(getMapStyle(mapStyle));
      } catch (error) {
        console.error('Error updating map style:', error);
      }
    }
  }, [mapStyle]);

  // Update rover position
  const updateRoverPosition = useCallback(() => {
    if (!gpsData || !roverMarker.current || !map.current) return;

    try {
      const { longitude, latitude, heading, accuracy } = gpsData;
      
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
        accuracyCircle.current.setRadius(accuracy);
      }

      // Center map on rover if it's the first GPS fix
      if (!map.current.isMoving()) {
        map.current.flyTo({
          center: [longitude, latitude],
          zoom: MAP_CONFIG.defaultZoom,
          duration: 1000
        });
      }
    } catch (error) {
      console.error('Error updating rover position:', error);
    }
  }, [gpsData]);

  // Set up periodic position updates
  useEffect(() => {
    updateRoverPosition();
    updateInterval.current = setInterval(updateRoverPosition, MAP_UPDATE_INTERVAL);

    return () => {
      if (updateInterval.current) {
        clearInterval(updateInterval.current);
      }
    };
  }, [updateRoverPosition]);

  // Update waypoint markers
  useEffect(() => {
    try {
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
            background: ${selectedWaypoint?.id === waypoint.id ? WAYPOINT_COLORS.SELECTED : WAYPOINT_COLORS.DEFAULT};
            border: 2px solid white;
            border-radius: 50%;
            box-shadow: 0 2px 4px rgba(0,0,0,0.3);
            cursor: pointer;
          "></div>
        `;

        waypointEl.addEventListener('click', (e) => {
          e.stopPropagation();
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
    } catch (error) {
      console.error('Error updating waypoint markers:', error);
    }
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