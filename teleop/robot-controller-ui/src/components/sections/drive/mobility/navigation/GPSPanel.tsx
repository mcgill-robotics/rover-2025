import { useState, useEffect } from 'react';
import 'leaflet/dist/leaflet.css';
import ROSLIB from 'roslib';
import CurrentLocation from './CurrentLocation';
import './GPS.css';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import { MapContainer, TileLayer, Marker, Popup, Polyline } from 'react-leaflet';


interface Coordinates {
  latitude: number;
  longitude: number;
}

interface Waypoint extends Coordinates {
  id: number;
}

// Custom marker icons for current location and waypoints
const currentLocationIcon = L.divIcon({
  className: 'custom-marker',
  html: '<div style="background-color: #4CAF10; width: 20px; height: 20px; border-radius: 50%; border: 2px solid #4CAF10;"></div>',
  iconSize: [20, 20],
  iconAnchor: [10, 10]
});

const waypointIcon = L.divIcon({
  className: 'custom-marker',
  html: '<div style="background-color: #FF3636; width: 20px; height: 20px; border-radius: 50%; border: 2px solid #FF3636;"></div>',
  iconSize: [20, 20],
  iconAnchor: [10, 10]
});

const GPS: React.FC = () => {
  // ROS connection
  const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

  ros.on('connection', () => console.log('✅ Connected to rosbridge'));
  ros.on('error', (error) => console.error('❌ Connection error:', error));
  ros.on('close', () => console.warn('⚠️ Connection closed'));

  const [currentLocation, setCurrentLocation] = useState<Coordinates>({
    latitude: 45.5049216,
    longitude: -73.56316,
  });

  const [waypoints, setWaypoints] = useState<Waypoint[]>([
    { id: 1, latitude: 45.5059216, longitude: -73.56316 }
  ]);

  const [nextWaypoint, setNextWaypoint] = useState<Coordinates>({
    latitude: 45.5059216,
    longitude: -73.56316
  });

  const [inputError, setInputError] = useState<string | null>(null);

  // ROS topic subscription
  const subscribeToTopic = (callback: (msg: any) => void) => {
    const subscriber = new ROSLIB.Topic({
      ros,
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
    subscribeToTopic((msg) => {
      const data = msg.data;
      setCurrentLocation({ latitude: data[0], longitude: data[1] });
    });
  }, []);

  // Handlers
  const handleNextWaypointChange = (coordinate: 'latitude' | 'longitude', value: string) => {
    const numValue = parseFloat(value);
    if (isNaN(numValue)) {
      setInputError('Please enter a valid number');
      return;
    }
    setInputError(null);
    setNextWaypoint(prev => ({ ...prev, [coordinate]: numValue }));
  };

  const addWaypoint = () => {
    if (isNaN(nextWaypoint.latitude) || isNaN(nextWaypoint.longitude)) {
      setInputError('Please enter valid coordinates');
      return;
    }
    setInputError(null);
    const newId = Math.max(...waypoints.map(wp => wp.id), 0) + 1;
    setWaypoints(prev => [...prev, { id: newId, ...nextWaypoint }]);
  };

  const deleteWaypoint = (id: number) => {
    setWaypoints(prev => prev.filter(wp => wp.id !== id));
  };

  // Polyline path
  const allPoints = [
    [currentLocation.latitude, currentLocation.longitude],
    ...waypoints.map(wp => [wp.latitude, wp.longitude])
  ] as [number, number][];

  return (
    <div className="gps-container">
      <div className="gps-header">
        <h2>GPS Navigation</h2>
      </div>

      <div className="gps-content">
        <CurrentLocation currentLocation={currentLocation} setCurrentLocation={setCurrentLocation} />

        <div className="gps-location-card waypoints">
          <h3>Waypoints</h3>
          <div className="waypoints-list">
            {waypoints.map((waypoint, index) => (
              <div key={waypoint.id} className="waypoint-item">
                <span>
                  Waypoint #{index + 1}: {waypoint.latitude.toFixed(6)}, {waypoint.longitude.toFixed(6)}
                </span>
                <button className="delete-waypoint" onClick={() => deleteWaypoint(waypoint.id)}>
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
            <button className="add-waypoint" onClick={addWaypoint} disabled={!!inputError}>
              Add Waypoint
            </button>
          </div>
        </div>

        {/* React-Leaflet Map */}
        <div className="gps-location-card current">
          <MapContainer
            center={[currentLocation.latitude, currentLocation.longitude]}
            zoom={15}
            scrollWheelZoom={true}
            className="gps-map-container"
            style={{ height: '400px', width: '100%' }}
          >
            <TileLayer
              attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
              url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            />

            {/* Current Location Marker */}
            <Marker
              position={[currentLocation.latitude, currentLocation.longitude]}
              icon={currentLocationIcon}
            >
              <Popup>Current Location</Popup>
            </Marker>

            {/* Waypoints */}
            {waypoints.map((wp, index) => (
              <Marker
                key={wp.id}
                position={[wp.latitude, wp.longitude]}
                icon={waypointIcon}
              >
                <Popup>Waypoint #{index + 1}</Popup>
              </Marker>
            ))}

            {/* Path */}
            <Polyline positions={allPoints} color="#4CAF10" weight={3} opacity={0.8} />
          </MapContainer>
        </div>
      </div>
    </div>
  );
};

export default GPS;
