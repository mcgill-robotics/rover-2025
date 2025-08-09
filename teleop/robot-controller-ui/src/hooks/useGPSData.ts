import { useState, useEffect, useCallback } from 'react';

interface GPSData {
  latitude: number;
  longitude: number;
  heading: number;
  accuracy: number;
  timestamp: number;
  fix_quality?: number;
  satellites?: number;
}

interface GPSStatus {
  has_fix: boolean;
  fix_quality: number;
  satellites: number;
  accuracy: number;
  last_update: number;
}

interface UseGPSDataReturn {
  gpsData: GPSData | null;
  gpsStatus: GPSStatus | null;
  isLoading: boolean;
  error: string | null;
  refetch: () => void;
}

const GPS_API_BASE = process.env.NEXT_PUBLIC_GPS_API_URL || 'http://localhost:5001/api';

export const useGPSData = (): UseGPSDataReturn => {
  const [gpsData, setGpsData] = useState<GPSData | null>(null);
  const [gpsStatus, setGpsStatus] = useState<GPSStatus | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const fetchGPSData = useCallback(async () => {
    try {
      const response = await fetch(`${GPS_API_BASE}/gps/data`);
      if (!response.ok) {
        throw new Error(`GPS API error: ${response.status}`);
      }
      const data = await response.json();
      
      if (data.error) {
        throw new Error(data.error);
      }
      
      setGpsData(data);
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch GPS data');
      console.error('GPS data fetch error:', err);
    }
  }, []);

  const fetchGPSStatus = useCallback(async () => {
    try {
      const response = await fetch(`${GPS_API_BASE}/gps/status`);
      if (!response.ok) {
        throw new Error(`GPS API error: ${response.status}`);
      }
      const data = await response.json();
      
      if (data.error) {
        throw new Error(data.error);
      }
      
      setGpsStatus(data);
    } catch (err) {
      console.error('GPS status fetch error:', err);
    }
  }, []);

  const refetch = useCallback(() => {
    setIsLoading(true);
    Promise.all([fetchGPSData(), fetchGPSStatus()]).finally(() => {
      setIsLoading(false);
    });
  }, [fetchGPSData, fetchGPSStatus]);

  // Initial fetch
  useEffect(() => {
    refetch();
  }, [refetch]);

  // Set up polling for GPS data
  useEffect(() => {
    const interval = setInterval(() => {
      fetchGPSData();
    }, 1000); // Update every second

    return () => clearInterval(interval);
  }, [fetchGPSData]);

  // Set up polling for GPS status (less frequent)
  useEffect(() => {
    const interval = setInterval(() => {
      fetchGPSStatus();
    }, 5000); // Update every 5 seconds

    return () => clearInterval(interval);
  }, [fetchGPSStatus]);

  return {
    gpsData,
    gpsStatus,
    isLoading,
    error,
    refetch
  };
};

// Hook for real-time GPS streaming (alternative to polling)
export const useGPSStream = (): UseGPSDataReturn => {
  const [gpsData, setGpsData] = useState<GPSData | null>(null);
  const [gpsStatus, setGpsStatus] = useState<GPSStatus | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let eventSource: EventSource | null = null;

    const connectToStream = () => {
      try {
        eventSource = new EventSource(`${GPS_API_BASE}/gps/stream`);
        
        eventSource.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            
            if (data.error) {
              setError(data.error);
            } else {
              setGpsData(data);
              setError(null);
            }
          } catch (err) {
            console.error('Error parsing GPS stream data:', err);
          }
        };

        eventSource.onerror = (event) => {
          console.error('GPS stream error:', event);
          setError('GPS stream connection failed');
          eventSource?.close();
          
          // Attempt to reconnect after 5 seconds
          setTimeout(connectToStream, 5000);
        };

        eventSource.onopen = () => {
          setIsLoading(false);
          setError(null);
        };

      } catch (err) {
        setError('Failed to connect to GPS stream');
        setIsLoading(false);
      }
    };

    connectToStream();

    return () => {
      if (eventSource) {
        eventSource.close();
      }
    };
  }, []);

  const refetch = useCallback(() => {
    // For streaming, refetch is not applicable
    // The stream will automatically provide updates
  }, []);

  return {
    gpsData,
    gpsStatus,
    isLoading,
    error,
    refetch
  };
}; 