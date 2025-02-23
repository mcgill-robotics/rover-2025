import React, { useState } from "react";
import VideoFeed from "./ui/Camera";
import DPad from "./ui/DPad";
import "./styles/CameraView.css";

interface CameraViewProps {
  streams: (MediaStream | null)[];
}

const CameraView: React.FC<CameraViewProps> = ({ streams }) => {
  const cameras = [
    { id: 1, name: "Camera 1" },
    { id: 2, name: "Camera 2" },
    { id: 3, name: "Camera 3" },
    { id: 4, name: "Pan Tilt" },
  ];

  const [activeCamera, setActiveCamera] = useState<number | null>(null);
  const [hoveredCameraId, setHoveredCameraId] = useState<number | null>(null);

  // Switch to a full view of the selected camera
  const handleSwitchCamera = (cameraId: number) => {
    setActiveCamera(cameraId);
  };

  // Return to the grid view
  const handleBackToGrid = () => {
    setActiveCamera(null);
  };

  return (
    <div className="camera-view">
      {/* Camera Selection Buttons */}
      <div className="camera-switch-buttons">
        {activeCamera !== null && (
          <button className="camera-button back-to-grid" onClick={handleBackToGrid}>
            <svg
              className="back-arrow"
              xmlns="http://www.w3.org/2000/svg"
              width="16"
              height="16"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <path d="M15 18l-6-6 6-6" />
            </svg>
            Full View
          </button>
        )}

        {cameras.map((camera, index) => (
          <div className="camera-button-container" key={camera.id}>
            <button
              onClick={() => handleSwitchCamera(camera.id)}
              onMouseEnter={() => setHoveredCameraId(camera.id)}
              onMouseLeave={() => setHoveredCameraId(null)}
              className={`camera-button ${activeCamera === camera.id ? "active-button" : ""}`}
            >
              <div className={`camera-connection ${streams[index] ? "connected" : "disconnected"}`} />
              {camera.name}
            </button>
          </div>
        ))}
      </div>

      {/* Camera Display Section */}
      {activeCamera === null ? (
        // Grid View (All 4 Cameras)
        <div className="camera-grid">
          {cameras.map((camera, index) => (
            <div
              key={camera.id}
              className="camera-card"
              onClick={() => handleSwitchCamera(camera.id)}
              onMouseEnter={() => setHoveredCameraId(camera.id)}
              onMouseLeave={() => setHoveredCameraId(null)}
            >
              <div className={`camera-placeholder ${hoveredCameraId === camera.id ? "hover-highlight" : ""}`}>
                <h2 className="camera-title">{camera.name}</h2>

                {/* Display individual camera streams */}
                <VideoFeed stream={streams[index]} />

                {camera.id === 4 && (
                  <div className="dpad">
                    <DPad inputStream="down" />
                  </div>
                )}
              </div>
            </div>
          ))}
        </div>
      ) : (
        // Full View (Single Camera)
        <div className="camera-card" onClick={handleBackToGrid}>
          <div className="camera-placeholder">
            <h2 className="camera-title">
              {cameras.find((camera) => camera.id === activeCamera)?.name}
            </h2>

            {/* Display the selected camera stream */}
            <VideoFeed stream={streams[activeCamera - 1]} />

            {activeCamera === 4 && (
              <div className="dpad">
                <DPad inputStream="up" />
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default CameraView;