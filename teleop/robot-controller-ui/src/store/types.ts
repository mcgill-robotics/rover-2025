// Type definitions for the ROS data store

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

// Future: Arm data types
export interface ArmJointData {
  position: number;
  velocity: number;
  effort: number;
  temperature: number;
}

export interface ArmData {
  joints: {
    [jointName: string]: ArmJointData;
  };
  end_effector: {
    x: number;
    y: number;
    z: number;
    roll: number;
    pitch: number;
    yaw: number;
  };
  timestamp: number;
}

// Connection management types
export interface ConnectionState {
  isConnected: boolean;
  isLoading: boolean;
  error: string | null;
  reconnectAttempts: number;
  lastConnected: number | null;
  websocket: WebSocket | null;
}

// WebSocket message types
export interface WebSocketMessage {
  type: 'initial_data' | 'data_update' | 'pong' | 'subscription_ack' | 'error';
  data?: DriveSummaryData | DriveMotorsData | DriveSpeedsData | DriveStatusData | Record<string, unknown>;
  timestamp?: number;
  error?: string;
}

export interface WebSocketRequest {
  type: 'ping' | 'subscribe' | 'unsubscribe';
  topics?: string[];
  timestamp?: number;
}
