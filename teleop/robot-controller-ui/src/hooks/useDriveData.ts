import { useState, useEffect, useCallback, useRef } from 'react';

export interface DriveMotorData {
  voltage: number;
  current: number;
  state: number;
  temperature: number;
}

export interface DriveMotorsData {
  RF: DriveMotorData;
  RB: DriveMotorData;
  LB: DriveMotorData;
  LF: DriveMotorData;
}

export interface DriveSpeedsData {
  RF: number;
  RB: number;
  LB: number;
  LF: number;
}

export interface DriveStatusData {
  RF: boolean;
  RB: boolean;
  LB: boolean;
  LF: boolean;
}

export interface DriveConnectionStatus {
  diagnostics_connected: boolean;
  speeds_connected: boolean;
  status_connected: boolean;
  service_available: boolean;
  last_update: {
    diagnostics: number;
    speeds: number;
    status: number;
  };
}

export interface DriveSummaryData {
  connection_status: DriveConnectionStatus;
  data_available: {
    diagnostics: boolean;
    speeds: boolean;
    status: boolean;
  };
  latest_diagnostics?: {
    motors: DriveMotorsData;
    timestamp: number;
  };
  latest_speeds?: {
    speeds: DriveSpeedsData;
    timestamp: number;
  };
  latest_status?: {
    status: DriveStatusData;
    timestamp: number;
  };
}

export interface UseDriveDataReturn {
  // Data
  diagnostics: DriveMotorsData | null;
  speeds: DriveSpeedsData | null;
  status: DriveStatusData | null;
  summary: DriveSummaryData | null;
  
  // Connection state
  isConnected: boolean;
  isLoading: boolean;
  error: string | null;
  
  // Methods
  reconnect: () => void;
  fetchLatest: () => Promise<void>;
}

const ROS_API_BASE = `http://${typeof window !== 'undefined' ? window.location.hostname : 'localhost'}:8082/api`;
const WS_URL = `ws://${typeof window !== 'undefined' ? window.location.hostname : 'localhost'}:8082/ws`;

export function useDriveData(): UseDriveDataReturn {
  const [diagnostics, setDiagnostics] = useState<DriveMotorsData | null>(null);
  const [speeds, setSpeeds] = useState<DriveSpeedsData | null>(null);
  const [status, setStatus] = useState<DriveStatusData | null>(null);
  const [summary, setSummary] = useState<DriveSummaryData | null>(null);
  
  const [isConnected, setIsConnected] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const reconnectAttempts = useRef(0);
  const maxReconnectAttempts = 5;
  
  const fetchData = useCallback(async (endpoint: string) => {
    try {
      const response = await fetch(`${ROS_API_BASE}${endpoint}`);
      const result = await response.json();
      
      if (result.success) {
        return result.data;
      } else {
        throw new Error(result.error || 'Unknown error');
      }
    } catch (err) {
      console.error(`Error fetching ${endpoint}:`, err);
      throw err;
    }
  }, []);
  
  const fetchLatest = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    
    try {
      const [diagnosticsData, speedsData, statusData, summaryData] = await Promise.allSettled([
        fetchData('/drive/diagnostics'),
        fetchData('/drive/speeds'),
        fetchData('/drive/status'),
        fetchData('/drive/summary')
      ]);
      
      if (diagnosticsData.status === 'fulfilled') {
        setDiagnostics(diagnosticsData.value.motors);
      }
      
      if (speedsData.status === 'fulfilled') {
        setSpeeds(speedsData.value.speeds);
      }
      
      if (statusData.status === 'fulfilled') {
        setStatus(statusData.value.status);
      }
      
      if (summaryData.status === 'fulfilled') {
        setSummary(summaryData.value);
      }
      
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch data');
    } finally {
      setIsLoading(false);
    }
  }, [fetchData]);
  
  const connectWebSocket = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      return;
    }
    
    try {
      const ws = new WebSocket(WS_URL);
      wsRef.current = ws;
      
      ws.onopen = () => {
        console.log('WebSocket connected to ROS Manager');
        setIsConnected(true);
        setError(null);
        reconnectAttempts.current = 0;
        
        // Send subscription message
        ws.send(JSON.stringify({
          type: 'subscribe',
          topics: ['drive_diagnostics', 'drive_speeds', 'drive_status']
        }));
      };
      
      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          
          switch (message.type) {
            case 'initial_data':
              if (message.data) {
                setSummary(message.data);
                if (message.data.latest_diagnostics) {
                  setDiagnostics(message.data.latest_diagnostics.motors);
                }
                if (message.data.latest_speeds) {
                  setSpeeds(message.data.latest_speeds.speeds);
                }
                if (message.data.latest_status) {
                  setStatus(message.data.latest_status.status);
                }
              }
              break;
              
            case 'data_update':
              if (message.data) {
                // Update based on data type
                if (message.data.motors) {
                  setDiagnostics(message.data.motors);
                }
                if (message.data.speeds) {
                  setSpeeds(message.data.speeds);
                }
                if (message.data.status) {
                  setStatus(message.data.status);
                }
              }
              break;
              
            case 'pong':
              // Handle ping/pong for connection health
              break;
              
            default:
              console.log('Unknown WebSocket message type:', message.type);
          }
        } catch (err) {
          console.error('Error parsing WebSocket message:', err);
        }
      };
      
      ws.onclose = (event) => {
        console.log('WebSocket disconnected:', event.code, event.reason);
        setIsConnected(false);
        
        // Attempt to reconnect if not a clean close
        if (event.code !== 1000 && reconnectAttempts.current < maxReconnectAttempts) {
          const delay = Math.min(1000 * Math.pow(2, reconnectAttempts.current), 30000);
          reconnectAttempts.current++;
          
          console.log(`Attempting to reconnect in ${delay}ms (attempt ${reconnectAttempts.current})`);
          
          reconnectTimeoutRef.current = setTimeout(() => {
            connectWebSocket();
          }, delay);
        }
      };
      
      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        setError('WebSocket connection error');
      };
      
    } catch (err) {
      console.error('Failed to create WebSocket connection:', err);
      setError('Failed to connect to ROS Manager');
    }
  }, []);
  
  const reconnect = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
    }
    
    if (wsRef.current) {
      wsRef.current.close();
    }
    
    reconnectAttempts.current = 0;
    connectWebSocket();
  }, [connectWebSocket]);
  
  // Initialize connection and fetch initial data
  useEffect(() => {
    fetchLatest();
    connectWebSocket();
    
    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, [fetchLatest, connectWebSocket]);
  
  // Periodic health check
  useEffect(() => {
    const healthCheck = setInterval(() => {
      if (wsRef.current?.readyState === WebSocket.OPEN) {
        wsRef.current.send(JSON.stringify({ type: 'ping' }));
      }
    }, 30000); // Every 30 seconds
    
    return () => clearInterval(healthCheck);
  }, []);
  
  return {
    diagnostics,
    speeds,
    status,
    summary,
    isConnected,
    isLoading,
    error,
    reconnect,
    fetchLatest
  };
}
