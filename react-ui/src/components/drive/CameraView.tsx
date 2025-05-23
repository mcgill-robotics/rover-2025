import React, { useState } from "react";
import MJPEGFeed from "./ui/Camera";
import DPad from "./ui/DPad";
import "./styles/CameraView.css";

const CameraView: React.FC = () => {
  const cameras = [
    { id: 1, name: "Camera 1" },
    { id: 2, name: "Camera 2" },
    { id: 3, name: "Camera 3" },
    { id: 4, name: "Pan Tilt" },
  ];

  const [currentIndex, setCurrentIndex] = useState(0);

  const handleNext = () => {
    setCurrentIndex((prev) => (prev + 1) % cameras.length);
  };

  const handlePrevious = () => {
    setCurrentIndex((prev) => (prev - 1 + cameras.length) % cameras.length);
  };

  const currentCamera = cameras[currentIndex];

  return (
    <div className="camera-view" style={{ width: "100%", maxWidth: "960px", margin: "0 auto", padding: "1rem" }}>
      <div className="camera-header" style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "1rem" }}>
        <button onClick={handlePrevious} className="nav-button">⟵ Previous</button>
        <h2 style={{ textAlign: "center" }}>{currentCamera.name}</h2>
        <button onClick={handleNext} className="nav-button">Next ⟶</button>
      </div>

      <div className="camera-card" style={{ borderRadius: "12px", overflow: "hidden", boxShadow: "0 4px 20px rgba(0,0,0,0.1)" }}>
        <MJPEGFeed url="http://localhost:8080/" />

        {currentCamera.id === 4 && (
          <div className="dpad" style={{ marginTop: "1rem" }}>
            <DPad inputStream="up" />
          </div>
        )}
      </div>
    </div>
  );
};

export default CameraView;
