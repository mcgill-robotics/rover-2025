import React, { useState } from "react";
import Camera from './ui/Camera'
import "./styles/CameraView.css";

const CameraView: React.FC = () => {
  const cameras = [
    { id: 1, name: "Camera 1" },
    { id: 2, name: "Camera 2" },
    { id: 3, name: "Camera 3" },
    { id: 4, name: "Pan Tilt Camera" },
  ];

  const [activeCamera, setActiveCamera] = useState<number | null>(null);
  const [hoveredCameraId, setHoveredCameraId] = useState<number | null>(null);

  const handleSwitchCamera = (cameraId: number) => {
    setActiveCamera(cameraId);
  };

  const handleBackToGrid = () => {
    setActiveCamera(null);
  };

  return (
    <div className="camera-view">
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
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <path d="M15 18l-6-6 6-6" />
            </svg>
            Full View
          </button>
        )}
        {cameras.map((camera) => (
          <div className="camera-button-container" key={camera.id}>
            <button
              onClick={() => handleSwitchCamera(camera.id)}
              onMouseEnter={() => setHoveredCameraId(camera.id)}
              onMouseLeave={() => setHoveredCameraId(null)}
              className={`camera-button ${activeCamera === camera.id ? "active-button" : ""}`}
            >
              <div className={`camera-connection ${false ? 'connected' : 'disconnected'}`} /> {/* TODO: Add function to track camera connection*/}
              {camera.name}
            </button>
          </div>
        ))}
      </div>

      {activeCamera === null ? (
        <div className="camera-grid">
          {cameras.map((camera) => (
            <div
              key={camera.id}
              className="camera-card"
              onClick={() => handleSwitchCamera(camera.id)}
              onMouseEnter={() => setHoveredCameraId(camera.id)}
              onMouseLeave={() => setHoveredCameraId(null)}
            >
              <div className={`camera-placeholder ${hoveredCameraId === camera.id ? "hover-highlight" : ""}`}>
                <h2 className="camera-title">{camera.name}</h2>
                <Camera />
              </div>
            </div>
          ))}
        </div>
      ) : (
        <div className="camera-card" onClick={handleBackToGrid}>
          <div className="camera-placeholder">
            <h2 className="camera-title">
              {cameras.find((camera) => camera.id === activeCamera)?.name}
              <Camera />
            </h2>
          </div>
        </div>
      )}
    </div>
  );
};

export default CameraView;