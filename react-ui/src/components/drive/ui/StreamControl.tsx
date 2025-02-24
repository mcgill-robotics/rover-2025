import React, { useEffect, useState } from "react";
import axios from "axios";
import "./styles/StreamControl.css";

interface StreamControlProps {
  onStart: (deviceId: number, index: number) => void;
  onStop: (index: number) => void;
}

const StreamControl: React.FC<StreamControlProps> = ({ onStart, onStop }) => {
  const [devices, setDevices] = useState<string[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [cameraConnections, setCameraConnections] = useState<(string | null)[]>([null, null, null, null]);
  const [selectedDevice, setSelectedDevice] = useState<string | null>(null);

  useEffect(() => {
    const fetchDevices = async () => {
      try {
        const response = await axios.get<{ devices: string[] }>("http://localhost:8081/video-devices");

        // Filter out devices where the number is odd
        const filteredDevices = response.data.devices.filter((device) => {
          const match = device.match(/\/dev\/video(\d+)/);
          if (match) {
            const deviceNumber = parseInt(match[1], 10);
            return deviceNumber % 2 === 0; // Only keep devices with even numbers
          }
          return false;
        });

        setDevices(filteredDevices);
      } catch (error) {
        console.error("Error fetching video devices:", error);
        setError("Failed to fetch video devices");
      } finally {
        setLoading(false);
      }
    };

    fetchDevices();
  }, []);

  const handleStart = (device: string, cameraIndex: number) => {
    onStart(parseInt(device.match(/\d+/)?.[0] ?? "0", 10), cameraIndex);

    const updatedConnections = [...cameraConnections];
    updatedConnections[cameraIndex] = device;
    setCameraConnections(updatedConnections);

    setDevices((prevDevices) => {
      const newDevices = prevDevices.filter((d) => d !== device);
      setSelectedDevice(null); // Unselect the device
      return newDevices;
    });
  };

  const handleStop = (cameraIndex: number) => {
    onStop(cameraIndex);

    const updatedConnections = [...cameraConnections];
    updatedConnections[cameraIndex] = null;
    setCameraConnections(updatedConnections);

    const deviceToReAdd = cameraConnections[cameraIndex];
    if (deviceToReAdd) {
      setDevices((prevDevices) => [...prevDevices, deviceToReAdd]);
    }
  };

  const handleDeviceClick = (device: string) => {
    setSelectedDevice(device);
  };

  return (
    <div className="stream-controls-container">
      {loading ? (
        <p>Loading devices...</p>
      ) : error ? (
        <div className="list-box-error">
          <p className="error">{error}</p>
        </div>
      ) : devices.length > 0 ? (
        <div className="devices-list-box">
          {devices.map((device, index) => (
            <div
              key={index}
              className={`device-item ${device === selectedDevice ? "selected" : ""}`}
              onClick={() => handleDeviceClick(device)} // Handle click
            >
              {device}
            </div>
          ))}
        </div>
      ) : (
        <div className="devices-list-box">
          <div className="list-box-text">
            <p>No video devices found.</p>
          </div>
        </div>
      )}

      <div className="camera-btn-container">
        {[...Array(4)].map((_, cameraIndex) => (
          <div className="camera-btn" key={cameraIndex}>
            {cameraConnections[cameraIndex] === null ? (
              <div className="start-btn-container">
                <button
                  className="start-btn"
                  onClick={() => {
                    if (!selectedDevice) {
                      console.error("No device selected!");
                      return;
                    }
                    handleStart(selectedDevice, cameraIndex);
                  }}
                >
                  Start Camera {cameraIndex + 1}
                </button>
              </div>
            ) : (
              <div className="stop-btn-container">
                <p>Camera {cameraIndex + 1} connected to: {cameraConnections[cameraIndex]}</p>
                <button
                  className="stop-btn"
                  onClick={() => handleStop(cameraIndex)}
                >
                  Stop Camera {cameraIndex + 1}
                </button>
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
};

export default StreamControl;