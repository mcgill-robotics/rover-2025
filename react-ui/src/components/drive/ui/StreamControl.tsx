import React, { useEffect, useState } from "react";
import axios from "axios";
import "./styles/StreamControl.css";

interface StreamControlProps {
  onStart: (deviceId: number, index: number) => void;
  onStop: (index: number) => void;
}

const PlayIcon: React.FC = () => (
  <svg className="neon-icon" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
    <polygon points="7,6 16,12 7,18" strokeWidth="1.5" transform="translate(2, 0)"/>
  </svg>
);

const SquareIcon: React.FC = () => (
  <svg className="neon-icon" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
    <rect x="6" y="6" width="12" height="12" strokeWidth="1.5" />
  </svg>
);

const StreamControl: React.FC<StreamControlProps> = ({ onStart, onStop }) => {
  const [devices, setDevices] = useState<string[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [cameraConnections, setCameraConnections] = useState<(string | null)[]>([null, null, null, null]);
  const [selectedDevice, setSelectedDevice] = useState<string | null>(null);

  // Fetch Devices
  useEffect(() => {
    const fetchDevices = async () => {
      try {
        const response = await axios.get<{ devices: string[] }>("http://localhost:8081/video-devices");
        const filteredDevices = response.data.devices.filter((device) => {
          const match = device.match(/\/dev\/video(\d+)/);
          return match ? parseInt(match[1], 10) % 2 === 0 : false; // Keep only even devices
        });
        setDevices(filteredDevices);
        // setDevices(["/dev/video2", "/dev/video4", "/dev/video6", "/dev/video8", "/dev/video10"])`// Extra device examples for UI
        
        const pc = new RTCPeerConnection();  // You need to initialize this properly based on your app setup

        // Wait for stats (you can periodically call this)
        const stats = await pc.getStats();
        stats.forEach(report => {
          if (report.type === 'inbound-rtp' && report.hasOwnProperty('bitrate')) {
            console.log('Bitrate:', report.bitrate);  // Log bitrate to console
          }
        });

      } catch (error) {
        console.error("Error fetching video devices:", error);
        setError("Failed to fetch video devices");
      } finally {
        setLoading(false);
      }
    };

    fetchDevices();
  }, []);

  // Start Camera
  const handleStart = (device: string, cameraIndex: number) => {
    const deviceId = parseInt(device.match(/\d+/)?.[0] ?? "0", 10);
    onStart(deviceId, cameraIndex);

    // Update connections and available devices
    const updatedConnections = [...cameraConnections];
    updatedConnections[cameraIndex] = device;
    setCameraConnections(updatedConnections);
    setDevices((prev) => prev.filter((d) => d !== device));
    setSelectedDevice(null);
  };

  // Stop Camera
  const handleStop = (cameraIndex: number) => {
    onStop(cameraIndex);

    // Restore disconnected device
    const updatedConnections = [...cameraConnections];
    const disconnectedDevice = updatedConnections[cameraIndex];
    updatedConnections[cameraIndex] = null;
    setCameraConnections(updatedConnections);
    if (disconnectedDevice) {
      setDevices((prev) => [...prev, disconnectedDevice]);
    }
  };

  return (
    <div className="stream-controls-container">
      {/* Device List */}
      <div className="devices-list-box">
        {loading ? (
          <p>Loading devices...</p>
        ) : error ? (
          <div className="list-box-error">
            <p className="error">{error}</p>
          </div>
        ) : devices.length > 0 ? (
          devices.map((device, index) => (
            <div
              key={index}
              className={`device-item ${device === selectedDevice ? "selected" : ""}`}
              onClick={() => setSelectedDevice(device)}
            >
              {device}
            </div>
          ))
        ) : (
          <div className="list-box-text">
            <p>No video devices found.</p>
          </div>
        )}
      </div>

      {/* Camera Controls */}
      <div className="camera-btn-container">
        {cameraConnections.map((connection, cameraIndex) => (
          <div className="camera-btn" key={cameraIndex}>
            <p className="camera-label">
              {cameraIndex + 1 === 4 ? 'Pan Tilt' : `Camera ${cameraIndex + 1}`}
            </p>
            {connection ? (
              <div className="stop-btn-container">
                <button
                  className="stop-btn"
                  onClick={() => handleStop(cameraIndex)}
                  title="Stop Camera"
                >
                  <SquareIcon />
                </button>

                <p className="device-name">{connection}</p>
              </div>
            ) : (
              <div className="start-btn-container">
                <button
                  className={`start-btn ${!selectedDevice ? "disabled" : ""}`}
                  onClick={() => selectedDevice && handleStart(selectedDevice, cameraIndex)}
                  disabled={!selectedDevice}
                  title={selectedDevice ? "Start Camera" : "Select Camera"}
                >
                  <PlayIcon />
                  <p className="device-name">{" "}</p>
                </button>
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
}

export default StreamControl;