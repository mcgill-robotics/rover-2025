"use client";

import React, { useEffect, useState } from "react";
import WebRTCPlayer from "./ui/WebRTCPlayer";
import DPad from "./ui/DPad";
import axios from "axios";
import "./styles/CameraView.css";

type CameraInfo = {
  name: string;
  path: string; // e.g., "/dev/video2"
};

const CameraView: React.FC = () => {
  const [cameras, setCameras] = useState<CameraInfo[]>([]);
  const [currentIndex, setCurrentIndex] = useState(0);

  useEffect(() => {
    const fetchCameras = async () => {
      const res = await axios.get("http://localhost:8081/video-devices");
      const flattened = res.data.devices.flatMap((d: any) =>
        d.devices.map((path: string) => ({
          name: d.name,
          path,
        }))
      );
      setCameras(flattened);
    };

    fetchCameras();
  }, []);

  const handleNext = () => {
    setCurrentIndex((prev) => (prev + 1) % cameras.length);
  };

  const handlePrevious = () => {
    setCurrentIndex((prev) => (prev - 1 + cameras.length) % cameras.length);
  };

  const currentCamera = cameras[currentIndex];

  return (
    <div
      className="camera-view"
      style={{ width: "100%", maxWidth: "960px", margin: "0 auto", padding: "1rem" }}
    >
      <div
        className="camera-header"
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          marginBottom: "1rem",
        }}
      >
        <button onClick={handlePrevious} className="nav-button">
          ⟵ Previous
        </button>
        <h2 style={{ textAlign: "center" }}>{currentCamera?.name ?? "Loading..."}</h2>
        <button onClick={handleNext} className="nav-button">
          Next ⟶
        </button>
      </div>

      <div
        className="camera-card"
        style={{
          borderRadius: "12px",
          overflow: "hidden",
          boxShadow: "0 4px 20px rgba(0,0,0,0.1)",
        }}
      >
        {currentCamera?.name === "Pan Tilt" ? (
          <>
            <WebRTCPlayer devicePath={currentCamera.path} />
            <div className="dpad" style={{ marginTop: "1rem" }}>
              <DPad inputStream="up" />
            </div>
          </>
        ) : currentCamera ? (
          <WebRTCPlayer devicePath={currentCamera.path} />
        ) : (
          <p>Loading stream...</p>
        )}
      </div>
    </div>
  );
};

export default CameraView;
