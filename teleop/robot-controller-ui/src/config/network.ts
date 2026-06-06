export const BASE_STATION_IP = 
typeof window !== "undefined" ? window.location.hostname : "localhost";
export const JETSON_IP = "192.168.1.69"; // Jetson IP

// Re-export camera configuration for easy access
export { CAMERA_CONFIG, MULTI_CAMERA_BACKEND, JETSON_DEVICES } from "./camera";
