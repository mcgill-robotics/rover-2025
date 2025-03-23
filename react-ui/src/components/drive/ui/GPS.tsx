import './styles/GPS.css';
import { useState, useMemo } from 'react';

interface Coordinates {
    latitude: number;
    longitude: number;
}

interface MapPoint {
    x: number;
    y: number;
}

const GPS: React.FC = () => {
    const [currentLocation, setCurrentLocation] = useState<Coordinates>({
        latitude: 37.7749,
        longitude: -122.4194
    });

    const [goalLocation, setGoalLocation] = useState<Coordinates>({
        latitude: 37.7833,
        longitude: -122.4167
    });

    const handleCoordinateChange = (
        location: 'current' | 'goal',
        coordinate: 'latitude' | 'longitude',
        value: string
    ) => {
        const numValue = parseFloat(value);
        if (isNaN(numValue)) return;

        if (location === 'current') {
            setCurrentLocation(prev => ({
                ...prev,
                [coordinate]: numValue
            }));
        } else {
            setGoalLocation(prev => ({
                ...prev,
                [coordinate]: numValue
            }));
        }
    };

    // Calculate map points based on coordinates
    const mapPoints = useMemo(() => {
        // Define the map boundaries (should be adjusted based on your coordinate range)
        const mapBounds = {
            minLat: Math.min(currentLocation.latitude, goalLocation.latitude) - 0.01,
            maxLat: Math.max(currentLocation.latitude, goalLocation.latitude) + 0.01,
            minLon: Math.min(currentLocation.longitude, goalLocation.longitude) - 0.01,
            maxLon: Math.max(currentLocation.longitude, goalLocation.longitude) + 0.01,
        };

        // Convert coordinates to map pixels
        const coordToPixel = (coord: Coordinates): MapPoint => {
            const x = ((coord.longitude - mapBounds.minLon) / (mapBounds.maxLon - mapBounds.minLon)) * 100;
            const y = ((coord.latitude - mapBounds.minLat) / (mapBounds.maxLat - mapBounds.minLat)) * 100;
            return { x, y };
        };

        return {
            rover: coordToPixel(currentLocation),
            goal: coordToPixel(goalLocation)
        };
    }, [currentLocation, goalLocation]);

    // Calculate the line angle between points
    const lineAngle = useMemo(() => {
        const dx = mapPoints.goal.x - mapPoints.rover.x;
        const dy = mapPoints.goal.y - mapPoints.rover.y;
        return Math.atan2(dy, dx) * (180 / Math.PI);
    }, [mapPoints]);

    // Calculate the line length
    const lineLength = useMemo(() => {
        const dx = mapPoints.goal.x - mapPoints.rover.x;
        const dy = mapPoints.goal.y - mapPoints.rover.y;
        return Math.sqrt(dx * dx + dy * dy);
    }, [mapPoints]);

    return (
        <div className="gps-container">
            <div className="gps-header">
                <h2>GPS Navigation</h2>
            </div>
            
            <div className="gps-content">
                <div className="gps-location-card current">
                    <h3>Current Location</h3>
                    <div className="gps-coordinates">
                        <div className="gps-coordinate">
                            <span className="gps-coordinate-label">Latitude:</span>
                            <input
                                type="number"
                                className="gps-coordinate-input"
                                value={currentLocation.latitude}
                                onChange={(e) => handleCoordinateChange('current', 'latitude', e.target.value)}
                                step="0.001"
                            />
                        </div>
                        <div className="gps-coordinate">
                            <span className="gps-coordinate-label">Longitude:</span>
                            <input
                                type="number"
                                className="gps-coordinate-input"
                                value={currentLocation.longitude}
                                onChange={(e) => handleCoordinateChange('current', 'longitude', e.target.value)}
                                step="0.001"
                            />
                        </div>
                    </div>
                </div>

                <div className="gps-location-card goal">
                    <h3>Goal Location</h3>
                    <div className="gps-coordinates">
                        <div className="gps-coordinate">
                            <span className="gps-coordinate-label">Latitude:</span>
                            <input
                                type="number"
                                className="gps-coordinate-input"
                                value={goalLocation.latitude}
                                onChange={(e) => handleCoordinateChange('goal', 'latitude', e.target.value)}
                                step="0.001"
                            />
                        </div>
                        <div className="gps-coordinate">
                            <span className="gps-coordinate-label">Longitude:</span>
                            <input
                                type="number"
                                className="gps-coordinate-input"
                                value={goalLocation.longitude}
                                onChange={(e) => handleCoordinateChange('goal', 'longitude', e.target.value)}
                                step="0.001"
                            />
                        </div>
                    </div>
                </div>

                <div className="gps-distance-text">
                    Distance to goal: 2.5 km
                </div>

                <div className="gps-map-container">
                    <div className="gps-map">
                        <div 
                            className="gps-map-point rover" 
                            title="Rover Location"
                            style={{ left: `${mapPoints.rover.x}%`, top: `${mapPoints.rover.y}%` }}
                        ></div>
                        <div 
                            className="gps-map-point goal" 
                            title="Goal Location"
                            style={{ left: `${mapPoints.goal.x}%`, top: `${mapPoints.goal.y}%` }}
                        ></div>
                        <div 
                            className="gps-map-line"
                            style={{
                                left: `${mapPoints.rover.x}%`,
                                top: `${mapPoints.rover.y}%`,
                                width: `${lineLength}%`,
                                transform: `rotate(${lineAngle}deg)`
                            }}
                        >
                            <div className="gps-map-arrow"></div>
                        </div>
                        <div className="gps-map-legend">
                            <div className="gps-map-legend-item">
                                <div className="gps-map-legend-color rover"></div>
                                <span>Rover Location</span>
                            </div>
                            <div className="gps-map-legend-item">
                                <div className="gps-map-legend-color goal"></div>
                                <span>Goal Location</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default GPS;