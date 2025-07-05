import React from 'react';

type CurrentLocationType = {
  latitude: number;
  longitude: number;
};

type LocationProps = {
  currentLocation: CurrentLocationType;
  setCurrentLocation: React.Dispatch<React.SetStateAction<CurrentLocationType>>;
};

const CurrentLocation = ({ currentLocation, setCurrentLocation }: LocationProps) => {
    
  const handleCoordinateChange = (
    coordinate: 'latitude' | 'longitude',
    value: string
  ) => {
    const numValue = parseFloat(value);
    if (isNaN(numValue)) return;

    setCurrentLocation(prev => ({
      ...prev,
      [coordinate]: numValue
    }));
  };

  return (
    <div className="gps-location-card current">
      <h3>Current Location</h3>
      <div className="gps-coordinates">
        <div className="gps-coordinate">
          <span className="gps-coordinate-label">Latitude:</span>
          <input
            type="number"
            className="gps-coordinate-input"
            value={currentLocation.latitude}
            onChange={(e) => handleCoordinateChange('latitude', e.target.value)}
            step="0.0001"
          />
        </div>
        <div className="gps-coordinate">
          <span className="gps-coordinate-label">Longitude:</span>
          <input
            type="number"
            className="gps-coordinate-input"
            value={currentLocation.longitude}
            onChange={(e) => handleCoordinateChange('longitude', e.target.value)}
            step="0.0001"
          />
        </div>
      </div>
    </div>
  );
};

export default CurrentLocation;
