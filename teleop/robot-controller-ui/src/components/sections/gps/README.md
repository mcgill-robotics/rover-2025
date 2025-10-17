# Simplified GPS Component

This is a simplified GPS component that displays the current GPS position and allows manual waypoint management without requiring the complex map tile system.

## Features

- **Real-time GPS Display**: Shows current latitude, longitude, heading, and accuracy
- **GPS Status**: Displays satellite count, fix quality, and connection status
- **Visual Map**: Simple canvas-based map showing current position as a green dot
- **Heading Indicator**: Blue arrow showing the current heading direction
- **Manual Waypoints**: Add custom waypoints by entering latitude/longitude coordinates
- **Waypoint Management**: View, name, and remove waypoints
- **Responsive Design**: Works on different screen sizes

## Usage

### As a Page
Navigate to `/gps` in the application to access the full GPS interface.

### As a Component
```tsx
import { GPSMap } from '@/components/sections/gps';

function MyComponent() {
  return (
    <div className="w-full h-full">
      <GPSMap />
    </div>
  );
}
```

## GPS Data Source

The component uses the existing GPS API service (`services/gps/gps_api.py`) which provides:

- **GPS Data Endpoint**: `GET /api/gps/data`
- **GPS Status Endpoint**: `GET /api/gps/status`
- **Real-time Updates**: Polls every second for position updates

## API Response Format

### GPS Data
```json
{
  "latitude": 45.5048,
  "longitude": -73.5772,
  "heading": 180.5,
  "accuracy": 2.5,
  "timestamp": 1640995200000
}
```

### GPS Status
```json
{
  "has_fix": true,
  "fix_quality": 1,
  "satellites": 8,
  "accuracy": 2.5,
  "last_update": 1640995200000
}
```

## Waypoint Management

### Adding Waypoints
1. Click "Add Waypoint" button
2. Enter latitude and longitude coordinates
3. Optionally provide a name for the waypoint
4. Click "Add Waypoint" to save

### Waypoint Features
- **Visual Display**: Red dots on the map
- **Named Labels**: Custom names displayed above waypoints
- **Coordinate Display**: Shows exact lat/lng in the waypoints list
- **Easy Removal**: Click the × button to delete waypoints

## Map Display

The component uses a simple HTML5 Canvas to render:
- **Grid Background**: Reference grid for orientation
- **Current Position**: Green dot with white border
- **Heading Arrow**: Blue arrow showing direction of travel
- **Waypoints**: Red dots with labels
- **Responsive Scaling**: Adapts to container size

## Dependencies

- **React**: For component lifecycle and state management
- **useGPSData Hook**: For GPS data fetching and real-time updates
- **Tailwind CSS**: For styling and responsive design

## Configuration

The component automatically uses the GPS API URL from environment variables:
- `NEXT_PUBLIC_GPS_API_URL`: Custom GPS API URL (defaults to `http://localhost:5001/api`)

## Error Handling

- **Loading State**: Shows "Loading GPS data..." while fetching
- **Error Display**: Shows error messages if GPS service is unavailable
- **Graceful Degradation**: Continues to work even if some GPS data is missing

## Benefits Over Complex Map System

1. **No External Dependencies**: No need for map tiles or external map services
2. **Faster Loading**: Immediate display without downloading map data
3. **Offline Capable**: Works without internet connection
4. **Simpler Setup**: No Docker containers or tile servers required
5. **Lightweight**: Minimal resource usage
6. **Real-time Focus**: Optimized for showing current position and waypoints
