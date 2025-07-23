import 'leaflet/dist/leaflet.css';
import L from 'leaflet'; // Leaflet library for map
import { useEffect, useState, useRef } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMap } from 'react-leaflet';

// Type definition for the GPS props
type GpsProps = {
  initialPosition?: [number, number];
  zoom?: number;
};

const LocateUser: React.FC<{ onLocate: (pos: [number, number]) => void }> = ({ onLocate }) => {
  const map = useMap();

  useEffect(() => {
    map.locate({ setView: true, maxZoom: 16 });
    map.on('locationfound', (e: L.LocationEvent) => {
      onLocate([e.latitude, e.longitude]);
    });
    return () => map.off('locationfound');
  }, [map, onLocate]);

  return null;
};

const Gps: React.FC<GpsProps> = ({ initialPosition = [51.505, -0.09], zoom = 13 }) => {
  const [position, setPosition] = useState<[number, number]>(initialPosition);
  const [targets, setTargets] = useState<[number, number][]>([]);
  const [isClient, setIsClient] = useState(false);
  const mapRef = useRef<L.Map | null>(null);

  useEffect(() => {
    setIsClient(true);  // Set `isClient` to true once mounted
  }, []);

  const addTarget = () => {
    const lat = parseFloat(prompt('Enter latitude:') || '');
    const lng = parseFloat(prompt('Enter longitude:') || '');
    if (!isNaN(lat) && !isNaN(lng)) {
      const newTarget: [number, number] = [lat, lng];
      setTargets(prev => [...prev, newTarget]);
      mapRef.current?.flyTo(newTarget, mapRef.current.getZoom());
    }
  };

  const removeTarget = (index: number) => {
    setTargets(prev => prev.filter((_, i) => i !== index));
  };

  // Default Icon (Leaflet's built-in default icon)
  const getDefaultIcon = () => {
    return new L.Icon({
      iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',  // Leaflet's default marker image URL
      iconSize: [25, 41], // Size of the icon
      iconAnchor: [12, 41], // Anchor point for the icon
      popupAnchor: [1, -34], // Popup position relative to the icon
    });
  };

  if (!isClient) return null;  // Prevent rendering map on the server

  return (
    <>
      <button onClick={addTarget} style={styles.addButton}>Add Target</button>
      <h2 style={styles.header}>GPS Tracker</h2>
      <p style={styles.positionText}>
        Current Position: {position[0].toFixed(4)}, {position[1].toFixed(4)}
      </p>

      {/* Target List */}
      <div style={styles.targetListContainer}>
        <h3 style={styles.targetListHeader}>Targets:</h3>
        <ul style={styles.targetList}>
          {targets.map((tgt, i) => (
            <li key={i} style={styles.targetItem}>
              <span style={{ marginRight: '10px' }}>
                Target {i + 1}: ({tgt[0].toFixed(4)}, {tgt[1].toFixed(4)})
              </span>
              <button onClick={() => removeTarget(i)} style={styles.removeButton}>Remove</button>
            </li>
          ))}
        </ul>
      </div>

      {/* Render Map only on the client-side */}
      <MapContainer
        center={position}
        zoom={zoom}
        style={{ height: '400px', width: '100%' }}
        whenCreated={mapInstance => { mapRef.current = mapInstance; }}
      >
        <TileLayer
          attribution='&copy; <a href="https://osm.org/copyright">OpenStreetMap</a>'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <Marker position={position} icon={getDefaultIcon()}>
          <Popup>Your current location</Popup>
        </Marker>
        {targets.map((tgt, i) => (
          <Marker
            key={i}
            position={tgt}
            icon={getDefaultIcon()}  // Apply the default icon to the target markers
          >
            <Popup>
              Target {i + 1}<br />
              ({tgt[0].toFixed(4)}, {tgt[1].toFixed(4)})
              <br />
              <button onClick={() => removeTarget(i)} style={styles.removeButton}>Remove</button>
            </Popup>
          </Marker>
        ))}
        <LocateUser onLocate={setPosition} />
      </MapContainer>
    </>
  );
};

// Simple Styles for UI
const styles = {
  addButton: {
    margin: '10px',
    padding: '10px',
    backgroundColor: '#4CAF50',
    color: 'white',
    border: 'none',
    cursor: 'pointer',
    borderRadius: '5px',
  },
  header: {
    color: '#333',
    textAlign: 'center',
  },
  positionText: {
    fontSize: '16px',
    color: '#666',
  },
  targetListContainer: {
    marginTop: '20px',
    padding: '10px',
    backgroundColor: '#f4f4f4',
    borderRadius: '5px',
  },
  targetListHeader: {
    fontSize: '18px',
    fontWeight: 'bold',
  },
  targetList: {
    listStyle: 'none',
    paddingLeft: '0',
  },
  targetItem: {
    padding: '5px',
    fontSize: '14px',
  },
  removeButton: {
    backgroundColor: '#FF6347',
    color: 'white',
    padding: '5px 10px',
    border: 'none',
    cursor: 'pointer',
    borderRadius: '5px',
  },
};

export default Gps;

