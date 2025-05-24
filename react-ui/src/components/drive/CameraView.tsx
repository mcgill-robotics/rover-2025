// CameraView.tsx (Accurate FPS via currentTime)
"use client";

import React, { useEffect, useRef, useState } from "react";
import WebRTCPlayer from "./ui/WebRTCPlayer";
import DPad from "./ui/DPad";
import axios from "axios";
import "./styles/CameraView.css";

const targetCameraNames = [
  "USB 2.0 Camera",
  "Logitech HD Webcam",
  "Integrated Camera",
  "CC HD webcam            "
];

interface CameraInfo {
  name: string;
  path: string;
}

const CameraView: React.FC = () => {
  const [cameras, setCameras] = useState<CameraInfo[]>([]);
  const [currentIndex, setCurrentIndex] = useState(0);
  const [isStreaming, setIsStreaming] = useState(false);
  const [frameTimestamps, setFrameTimestamps] = useState<number[]>([]);
  const [lastFrameTime, setLastFrameTime] = useState<number | null>(null);
  const [resolution, setResolution] = useState<string>("N/A");
  const videoRef = useRef<HTMLVideoElement>(null);
  const lastSeenTimeRef = useRef<number>(0);

  useEffect(() => {
    const fetchCameraPaths = async () => {
      try {
        const res = await axios.get("http://localhost:8081/video-devices");
        const deviceMap: CameraInfo[] = [];

        res.data.devices.forEach((entry: { name: string; devices: string[] }) => {
          for (const targetName of targetCameraNames) {
            if (entry.name.includes(targetName)) {
              deviceMap.push({ name: targetName, path: entry.devices[0] });
            }
          }
        });

        setCameras(deviceMap);
      } catch (error) {
        console.error("Failed to fetch camera devices:", error);
      }
    };

    fetchCameraPaths();
  }, []);

  useEffect(() => {
    let animationId: number;

    const monitor = () => {
      if (videoRef.current && videoRef.current.readyState >= 2) {
        const currentTime = Date.now();
        const videoTime = videoRef.current.currentTime;

        if (videoTime !== lastSeenTimeRef.current) {
          lastSeenTimeRef.current = videoTime;
          setFrameTimestamps((prev) => {
            const updated = [...prev, currentTime].filter((t) => currentTime - t <= 2000);
            return updated;
          });
          setLastFrameTime(currentTime);
          setResolution(`${videoRef.current.videoWidth}x${videoRef.current.videoHeight}`);
        }
      }

      if (lastFrameTime && Date.now() - lastFrameTime > 3000) {
        console.warn("Stream stalled. Restarting...");
        setIsStreaming(false);
        setTimeout(() => setIsStreaming(true), 500);
      }

      animationId = requestAnimationFrame(monitor);
    };

    if (isStreaming) {
      monitor();
    }

    return () => cancelAnimationFrame(animationId);
  }, [isStreaming, lastFrameTime]);

  const handleNext = () => {
    setCurrentIndex((prev) => (prev + 1) % cameras.length);
    resetStats();
  };

  const handlePrevious = () => {
    setCurrentIndex((prev) => (prev - 1 + cameras.length) % cameras.length);
    resetStats();
  };

  const handleStart = () => {
    setIsStreaming(true);
    resetStats();
  };

  const handleStop = () => setIsStreaming(false);

  const resetStats = () => {
    setFrameTimestamps([]);
    setLastFrameTime(null);
    setResolution("N/A");
    lastSeenTimeRef.current = 0;
  };

  const currentCamera = cameras[currentIndex];
  const fps = frameTimestamps.length > 1
    ? ((frameTimestamps.length - 1) / ((frameTimestamps.at(-1)! - frameTimestamps[0]) / 1000)).toFixed(1)
    : "0.0";
  const isLive = lastFrameTime && Date.now() - lastFrameTime < 2000;

  return (
    <div className="camera-view" style={{ width: "100%", maxWidth: "960px", margin: "0 auto", padding: "1rem" }}>
      <div className="camera-header" style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "1rem" }}>
        <button onClick={handlePrevious} className="nav-button">⟵ Previous</button>
        <h2 style={{ textAlign: "center" }}>{currentCamera?.name ?? "Loading..."}</h2>
        <button onClick={handleNext} className="nav-button">Next ⟶</button>
      </div>

      <div className="camera-card" style={{ borderRadius: "12px", overflow: "hidden", boxShadow: "0 4px 20px rgba(0,0,0,0.1)" }}>
        {currentCamera && isStreaming ? (
          <>
            <WebRTCPlayer devicePath={currentCamera.path} forwardedRef={videoRef} />
            {currentCamera.name.includes("Pan Tilt") && (
              <div className="dpad" style={{ marginTop: "1rem" }}>
                <DPad inputStream="up" />
              </div>
            )}
          </>
        ) : (
          <p>Click Start to begin stream...</p>
        )}
      </div>

      <div style={{ marginTop: "1rem", textAlign: "center" }}>
        <button onClick={handleStart} disabled={isStreaming} style={{
          marginRight: "1rem",
          padding: "0.5rem 1.5rem",
          backgroundColor: "#4CAF50",
          color: "white",
          border: "none",
          borderRadius: "6px",
          cursor: "pointer"
        }}>Start</button>
        <button onClick={handleStop} disabled={!isStreaming} style={{
          padding: "0.5rem 1.5rem",
          backgroundColor: "#f44336",
          color: "white",
          border: "none",
          borderRadius: "6px",
          cursor: "pointer"
        }}>Stop</button>
        <div style={{ marginTop: "0.75rem" }}>
          <span style={{
            display: "inline-block",
            width: "10px",
            height: "10px",
            borderRadius: "50%",
            backgroundColor: isLive ? "green" : "red",
            marginRight: "0.5rem"
          }}></span>
          Live Status • FPS: {fps} • Resolution: {resolution} • Total Frames: {frameTimestamps.length}
        </div>
      </div>
    </div>
  );
};

export default CameraView;