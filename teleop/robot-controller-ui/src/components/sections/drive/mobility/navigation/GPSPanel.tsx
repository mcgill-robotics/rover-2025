import { useState, useEffect, useRef } from 'react';
import 'leaflet/dist/leaflet.css';
import 'leaflet.offline';
import ROSLIB from 'roslib';
import L from 'leaflet';
import CurrentLocation from './CurrentLocation';
import './GPS.css';

declare module 'leaflet' {
    interface TileLayer {
        saveTilesForOffline?(options: {
            bounds: L.LatLngBounds;
            minZoom: number;
            maxZoom: number;
        }): Promise<void>;
    }
}

// ================================
// Define interfaces outside component
// ================================
interface Coordinates {
    latitude: number;
    longitude: number;
}

interface Waypoint extends Coordinates {
    id: number;
}

const GPS: React.FC = () => {
    const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });
    ros.on('connection', () => {
        console.log('✅ Connected to rosbridge');
    });
    ros.on('error', (error) => {
        console.error('❌ Connection error:', error);
    });
    ros.on('close', () => {
        console.warn('⚠️ Connection closed');
    });

    const mapRef = useRef<HTMLDivElement>(null);
    const mapInstance = useRef<L.Map | null>(null);
    const markersRef = useRef<L.Marker[]>([]);
    const polylineRef = useRef<L.Polyline | null>(null);

    // initial rover location
    const [currentLocation, setCurrentLocation] = useState<Coordinates>({
        latitude: 45.5049216,
        longitude: -73.56316
    });

    // initial waypoint
    const [waypoints, setWaypoints] = useState<Waypoint[]>([
        {
            id: 1,
            latitude: 45.5059216,
            longitude: -73.56316
        }
    ]);

    const [nextWaypoint, setNextWaypoint] = useState<Coordinates>({
        latitude: 45.5059216,
        longitude: -73.56316
    });

    const [inputError, setInputError] = useState<string | null>(null);

    const subscribeToTopic = (callback: (msg: any) => void) => {
        const subscriber = new ROSLIB.Topic({
            ros: ros,
            name: '/minimal_publisher',
            messageType: 'std_msgs/msg/Float64MultiArray'
        });

        console.log('Subscribing to /minimal_publisher...');

        subscriber.subscribe((message) => {
            console.log('Received:', message);
            callback(message);
        });
    };

    useEffect(() => {
        console.log('Connecting to ROS...');
        subscribeToTopic((msg) => {
            const data = msg.data;
            console.log('Received data:', data);
            setCurrentLocation({
                latitude: data[0],
                longitude: data[1]
            });
        });
    }, []);

    useEffect(() => {
        if (mapRef.current && !mapInstance.current) {
            mapInstance.current = L.map(mapRef.current).setView(
                [currentLocation.latitude, currentLocation.longitude],
                15
            );

            const offlineLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
                subdomains: ['a', 'b', 'c'],
                minZoom: 13,
                maxZoom: 19
            });

            offlineLayer.addTo(mapInstance.current);

            const saveTiles = async () => {
                const bounds = mapInstance.current?.getBounds();
                if (!bounds) return;

                try {
                    if (typeof offlineLayer.saveTilesForOffline === 'function') {
                        await offlineLayer.saveTilesForOffline({
                            bounds: bounds,
                            minZoom: 13,
                            maxZoom: 19
                        });
                        console.log('Tiles saved for offline use');
                    } else {
                        console.log('Offline saving not available yet');
                    }
                } catch (error) {
                    console.error('Error saving tiles:', error);
                }
            };

            if (navigator.onLine) {
                setTimeout(saveTiles, 1000);
            }

            window.addEventListener('online', () => {
                console.log('Back online, saving tiles...');
                setTimeout(saveTiles, 1000);
            });
            window.addEventListener('offline', () => {
                console.log('Using offline tiles');
            });

            return () => {
                window.removeEventListener('online', saveTiles);
                window.removeEventListener('offline', () => {});
                if (mapInstance.current) {
                    mapInstance.current.remove();
                    mapInstance.current = null;
                }
            };
        }
    }, []);

    useEffect(() => {
        if (!mapInstance.current) return;

        markersRef.current.forEach(marker => marker.remove());
        markersRef.current = [];
        if (polylineRef.current) {
            polylineRef.current.remove();
            polylineRef.current = null;
        }

        const roverMarker = L.marker([currentLocation.latitude, currentLocation.longitude], {
            icon: L.divIcon({
                className: 'custom-marker',
                html: '<div style="background-color: #4CAF10; width: 20px; height: 20px; border-radius: 50%; border: 2px solid #4CAF10;"></div>',
                iconSize: [20, 20],
                iconAnchor: [10, 10]
            })
        }).addTo(mapInstance.current);

        const waypointMarkers = waypoints.map((waypoint, index) => {
            const marker = L.marker([waypoint.latitude, waypoint.longitude], {
                icon: L.divIcon({
                    className: 'custom-marker',
                    html: `<div style="background-color: #FF3636; width: 20px; height: 20px; border-radius: 50%; border: 2px solid #FF3636;"></div>`,
                    iconSize: [20, 20],
                    iconAnchor: [10, 10]
                })
            });

            if (mapInstance.current) {
                marker.addTo(mapInstance.current);
              }
              
            marker.bindPopup(`Waypoint #${index + 1}: ${waypoint.latitude.toFixed(6)}, ${waypoint.longitude.toFixed(6)}`);
            return marker;
        });

        const allPoints = [
            [currentLocation.latitude, currentLocation.longitude],
            ...waypoints.map(wp => [wp.latitude, wp.longitude])
        ];

        const newPolyline = L.polyline(allPoints as L.LatLngExpression[], {
            color: '#4CAF10',
            weight: 3,
            opacity: 0.8
        }).addTo(mapInstance.current);

        markersRef.current = [roverMarker, ...waypointMarkers];
        polylineRef.current = newPolyline;

        const bounds = L.latLngBounds(allPoints as L.LatLngExpression[]);
        mapInstance.current.fitBounds(bounds, { padding: [50, 50] });
    }, [currentLocation, waypoints]);

    const handleNextWaypointChange = (
        coordinate: 'latitude' | 'longitude',
        value: string
    ) => {
        const numValue = parseFloat(value);
        if (isNaN(numValue)) {
            setInputError('Please enter a valid number');
            return;
        }
        setInputError(null);
        setNextWaypoint(prev => ({
            ...prev,
            [coordinate]: numValue
        }));
    };

    const addWaypoint = () => {
        if (isNaN(nextWaypoint.latitude) || isNaN(nextWaypoint.longitude)) {
            setInputError('Please enter valid coordinates');
            return;
        }
        setInputError(null);

        const newId = Math.max(...waypoints.map(wp => wp.id), 0) + 1;
        setWaypoints(prev => [...prev, {
            id: newId,
            latitude: nextWaypoint.latitude,
            longitude: nextWaypoint.longitude
        }]);
    };

    const deleteWaypoint = (id: number) => {
        setWaypoints(prev => prev.filter(wp => wp.id !== id));
    };

    return (
        <div className="gps-container">
            <div className="gps-header">
                <h2>GPS Navigation</h2>
            </div>

            <div className="gps-content">
                <CurrentLocation
                    currentLocation={currentLocation}
                    setCurrentLocation={setCurrentLocation}
                />

                <div className="gps-location-card waypoints">
                    <h3>Waypoints</h3>
                    <div className="waypoints-list">
                        {waypoints.map((waypoint, index) => (
                            <div key={waypoint.id} className="waypoint-item">
                                <span>Waypoint #{index + 1}: {waypoint.latitude.toFixed(6)}, {waypoint.longitude.toFixed(6)}</span>
                                <button
                                    className="delete-waypoint"
                                    onClick={() => deleteWaypoint(waypoint.id)}
                                >
                                    Delete
                                </button>
                            </div>
                        ))}
                    </div>

                    <div className="next-waypoint-inputs">
                        <h4>Next Waypoint</h4>
                        <div className="gps-coordinates">
                            <div className="gps-coordinate">
                                <span className="gps-coordinate-label">Latitude:</span>
                                <input
                                    type="number"
                                    className={`gps-coordinate-input ${inputError ? 'error' : ''}`}
                                    value={nextWaypoint.latitude}
                                    onChange={(e) => handleNextWaypointChange('latitude', e.target.value)}
                                    step="0.0001"
                                />
                            </div>
                            <div className="gps-coordinate">
                                <span className="gps-coordinate-label">Longitude:</span>
                                <input
                                    type="number"
                                    className={`gps-coordinate-input ${inputError ? 'error' : ''}`}
                                    value={nextWaypoint.longitude}
                                    onChange={(e) => handleNextWaypointChange('longitude', e.target.value)}
                                    step="0.0001"
                                />
                            </div>
                        </div>
                        {inputError && <div className="error-message">{inputError}</div>}
                        <button
                            className="add-waypoint"
                            onClick={addWaypoint}
                            disabled={!!inputError}
                        >
                            Add Waypoint
                        </button>
                    </div>
                </div>

                <div className="gps-location-card current">
                    <div className="map-container-container">
                        <div className="gps-map-container" ref={mapRef} />
                    </div>
                </div>

            </div>
        </div>
    );
};

export default GPS;
