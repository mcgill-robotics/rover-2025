import { create } from 'zustand';
import { devtools, subscribeWithSelector } from 'zustand/middleware';
import {
  DriveMotorsData,
  DriveSpeedsData,
  DriveStatusData,
  DriveSummaryData,
  DriveConnectionStatus,
  ArmData,
  ConnectionState,
  WebSocketMessage,
  WebSocketRequest
} from './types';

// Store state interface
interface ROSStore {
  // Drive system data
  drive: {
    diagnostics: DriveMotorsData | null;
    speeds: DriveSpeedsData | null;
    status: DriveStatusData | null;
    connectionStatus: DriveConnectionStatus | null;
    summary: DriveSummaryData | null;
    lastUpdated: number;
  };

  // Future: Arm system data
  arm: {
    data: ArmData | null;
    lastUpdated: number;
  };

  // Connection management
  connection: ConnectionState;

  // Actions
  actions: {
    // Connection actions
    connect: () => void;
    disconnect: () => void;
    reconnect: () => void;
    setConnectionState: (state: Partial<ConnectionState>) => void;
    
    // Drive actions
    updateDriveData: (type: 'diagnostics' | 'speeds' | 'status' | 'summary', data: unknown) => void;
    setDriveConnectionStatus: (status: DriveConnectionStatus) => void;
    
    // Future: Arm actions
    updateArmData: (data: ArmData) => void;
    
    // WebSocket actions
    sendMessage: (message: WebSocketRequest) => void;
    handleMessage: (message: WebSocketMessage) => void;
  };
}

// Configuration
const WS_URL = `ws://${typeof window !== 'undefined' ? window.location.hostname : 'localhost'}:8082/ws`;
const MAX_RECONNECT_ATTEMPTS = 5;
const RECONNECT_DELAYS = [1000, 2000, 4000, 8000, 16000]; // Exponential backoff

// Create the store
export const useROSStore = create<ROSStore>()(
  devtools(
    subscribeWithSelector((set, get) => ({
      // Initial state
      drive: {
        diagnostics: null,
        speeds: null,
        status: null,
        connectionStatus: null,
        summary: null,
        lastUpdated: 0,
      },

      arm: {
        data: null,
        lastUpdated: 0,
      },

      connection: {
        isConnected: false,
        isLoading: false,
        error: null,
        reconnectAttempts: 0,
        lastConnected: null,
        websocket: null,
      },

      // Actions
      actions: {
        connect: () => {
          const state = get();
          
          // Don't connect if already connected or connecting
          if (state.connection.isConnected || state.connection.isLoading) {
            return;
          }

          set((state) => ({
            connection: {
              ...state.connection,
              isLoading: true,
              error: null,
            },
          }));

          try {
            const ws = new WebSocket(WS_URL);
            
            ws.onopen = () => {
              console.log('ROS WebSocket connected');
              set((state) => ({
                connection: {
                  ...state.connection,
                  isConnected: true,
                  isLoading: false,
                  error: null,
                  reconnectAttempts: 0,
                  lastConnected: Date.now(),
                  websocket: ws,
                },
              }));

              // Subscribe to drive topics
              get().actions.sendMessage({
                type: 'subscribe',
                topics: ['drive_diagnostics', 'drive_speeds', 'drive_status'],
                timestamp: Date.now(),
              });
            };

            ws.onmessage = (event) => {
              try {
                const message: WebSocketMessage = JSON.parse(event.data);
                get().actions.handleMessage(message);
              } catch (error) {
                console.error('Error parsing WebSocket message:', error);
              }
            };

            ws.onclose = (event) => {
              console.log('ROS WebSocket disconnected:', event.code, event.reason);
              
              set((state) => ({
                connection: {
                  ...state.connection,
                  isConnected: false,
                  isLoading: false,
                  websocket: null,
                },
              }));

              // Auto-reconnect if not a clean close
              if (event.code !== 1000) {
                const currentState = get();
                if (currentState.connection.reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                  const delay = RECONNECT_DELAYS[Math.min(currentState.connection.reconnectAttempts, RECONNECT_DELAYS.length - 1)];
                  
                  console.log(`Attempting to reconnect in ${delay}ms (attempt ${currentState.connection.reconnectAttempts + 1})`);
                  
                  setTimeout(() => {
                    set((state) => ({
                      connection: {
                        ...state.connection,
                        reconnectAttempts: state.connection.reconnectAttempts + 1,
                      },
                    }));
                    get().actions.connect();
                  }, delay);
                }
              }
            };

            ws.onerror = (error) => {
              console.error('ROS WebSocket error:', error);
              set((state) => ({
                connection: {
                  ...state.connection,
                  error: 'WebSocket connection error',
                  isLoading: false,
                },
              }));
            };

          } catch (error) {
            console.error('Failed to create WebSocket connection:', error);
            set((state) => ({
              connection: {
                ...state.connection,
                error: 'Failed to connect to ROS Manager',
                isLoading: false,
              },
            }));
          }
        },

        disconnect: () => {
          const state = get();
          if (state.connection.websocket) {
            state.connection.websocket.close(1000, 'User disconnected');
          }
          
          set((state) => ({
            connection: {
              ...state.connection,
              isConnected: false,
              isLoading: false,
              websocket: null,
              reconnectAttempts: 0,
            },
          }));
        },

        reconnect: () => {
          get().actions.disconnect();
          setTimeout(() => {
            set((state) => ({
              connection: {
                ...state.connection,
                reconnectAttempts: 0,
              },
            }));
            get().actions.connect();
          }, 1000);
        },

        setConnectionState: (newState) => {
          set((state) => ({
            connection: {
              ...state.connection,
              ...newState,
            },
          }));
        },

        updateDriveData: (type, data) => {
          const timestamp = Date.now();
          
          set((state) => {
            const newDriveState = { ...state.drive };
            
            switch (type) {
              case 'diagnostics':
                newDriveState.diagnostics = data as DriveMotorsData;
                break;
              case 'speeds':
                newDriveState.speeds = data as DriveSpeedsData;
                break;
              case 'status':
                newDriveState.status = data as DriveStatusData;
                break;
              case 'summary':
                const summaryData = data as DriveSummaryData;
                newDriveState.summary = summaryData;
                
                // Update individual data from summary if available
                if (summaryData.latest_diagnostics) {
                  newDriveState.diagnostics = summaryData.latest_diagnostics.motors;
                }
                if (summaryData.latest_speeds) {
                  newDriveState.speeds = summaryData.latest_speeds.speeds;
                }
                if (summaryData.latest_status) {
                  newDriveState.status = summaryData.latest_status.status;
                }
                if (summaryData.connection_status) {
                  newDriveState.connectionStatus = summaryData.connection_status;
                }
                break;
            }
            
            newDriveState.lastUpdated = timestamp;
            
            return {
              drive: newDriveState,
            };
          });
        },

        setDriveConnectionStatus: (status) => {
          set((state) => ({
            drive: {
              ...state.drive,
              connectionStatus: status,
            },
          }));
        },

        updateArmData: (data) => {
          set(() => ({
            arm: {
              data,
              lastUpdated: Date.now(),
            },
          }));
        },

        sendMessage: (message) => {
          const state = get();
          if (state.connection.websocket && state.connection.isConnected) {
            try {
              state.connection.websocket.send(JSON.stringify(message));
            } catch (error) {
              console.error('Error sending WebSocket message:', error);
            }
          }
        },

        handleMessage: (message) => {
          switch (message.type) {
            case 'initial_data':
              if (message.data) {
                get().actions.updateDriveData('summary', message.data);
              }
              break;

            case 'data_update':
              if (message.data) {
                // Determine data type and update accordingly
                if ('motors' in message.data) {
                  get().actions.updateDriveData('diagnostics', message.data.motors);
                }
                if ('speeds' in message.data) {
                  get().actions.updateDriveData('speeds', message.data.speeds);
                }
                if ('status' in message.data) {
                  get().actions.updateDriveData('status', message.data.status);
                }
              }
              break;

            case 'pong':
              // Handle ping/pong for connection health
              break;

            case 'subscription_ack':
              console.log('Subscription acknowledged:', message.data);
              break;

            case 'error':
              console.error('WebSocket error message:', message.error);
              set((state) => ({
                connection: {
                  ...state.connection,
                  error: message.error || 'Unknown WebSocket error',
                },
              }));
              break;

            default:
              console.log('Unknown WebSocket message type:', message.type);
          }
        },
      },
    })),
    {
      name: 'ros-store',
      partialize: (state: ROSStore) => ({
        // Only persist connection settings, not live data
        connection: {
          reconnectAttempts: state.connection.reconnectAttempts,
        },
      }),
    }
  )
);

// Selectors for easy access to specific data
export const useDriveData = () => useROSStore((state) => state.drive);
export const useArmData = () => useROSStore((state) => state.arm);
export const useConnectionState = () => useROSStore((state) => state.connection);
export const useROSActions = () => useROSStore((state) => state.actions);

// Specific drive data selectors
export const useDriveDiagnostics = () => useROSStore((state) => state.drive.diagnostics);
export const useDriveSpeeds = () => useROSStore((state) => state.drive.speeds);
export const useDriveStatus = () => useROSStore((state) => state.drive.status);
export const useDriveConnectionStatus = () => useROSStore((state) => state.drive.connectionStatus);

// Connection helpers
export const useIsROSConnected = () => useROSStore((state) => state.connection.isConnected);
export const useROSError = () => useROSStore((state) => state.connection.error);

// Auto-connect hook
export const useAutoConnect = () => {
  const actions = useROSActions();
  const isConnected = useIsROSConnected();
  
  // Auto-connect on mount
  if (typeof window !== 'undefined' && !isConnected) {
    setTimeout(() => actions.connect(), 100);
  }
  
  return { connect: actions.connect, disconnect: actions.disconnect, reconnect: actions.reconnect };
};
