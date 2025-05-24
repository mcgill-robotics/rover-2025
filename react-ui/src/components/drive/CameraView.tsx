// CameraView.tsx (Accurate FPS via currentTime)
"use client";

import React, { useEffect, useRef, useState } from "react";
import WebRTCPlayer from "./ui/WebRTCPlayer";
import DPad from "./ui/DPad";
import PowerButton from "../arm/ui/PowerButton";
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
  const [rtt, setRtt] = useState<number | null>(null);
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

  useEffect(() => {
    let intervalId: NodeJS.Timeout;

    const fetchStats = async () => {
      try {
        const res = await axios.get("http://localhost:8081/bandwidth-stats");
        const stats = res.data.bandwidth_stats?.[0];
        if (stats?.rtt_ms != null) {
          setRtt(stats.rtt_ms);
        }
      } catch (err) {
        console.error("Failed to fetch bandwidth stats:", err);
      }
    };

    if (isStreaming) {
      fetchStats(); // initial fetch
      intervalId = setInterval(fetchStats, 2000); // poll every 2s
    }

    return () => clearInterval(intervalId);
  }, [isStreaming]);


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
    <div className="camera-view" style={{ height: "100vh", position: "relative", backgroundColor: "#000" }}>
      {currentCamera && isStreaming ? (
        <div style={{ height: "100%", width: "100%", position: "relative", overflow: "hidden" }}>
          {/* WebRTC Video Feed */}
          <WebRTCPlayer devicePath={currentCamera.path} forwardedRef={videoRef} />

          {/* Top Left - Camera Name + Live Status */}
          <div style={{
            position: "absolute",
            top: "1rem",
            left: "1rem",
            color: "white",
            fontSize: "1.2rem",
            backgroundColor: "rgba(0, 0, 0, 0.4)",
            padding: "0.5rem 1rem",
            borderRadius: "8px"
          }}>
            <strong>{currentCamera.name}</strong><br />
            <span style={{
              display: "inline-block",
              width: "10px",
              height: "10px",
              borderRadius: "50%",
              backgroundColor: isLive ? "limegreen" : "red",
              marginRight: "0.5rem",
            }}></span>
            Live
          </div>

          {/* Top Right - Diagnostic Info */}
          <div style={{
            position: "absolute",
            top: "1rem",
            right: "1rem",
            color: "white",
            textAlign: "right",
            fontSize: "0.9rem",
            backgroundColor: "rgba(0, 0, 0, 0.4)",
            padding: "0.5rem 1rem",
            borderRadius: "8px"
          }}>
            FPS: {fps}<br />
            Res: {resolution}<br />
            Ping: {rtt ? `${rtt.toFixed(0)} ms` : "N/A"}
          </div>


          {/* Prev Button - Left */}
          <button onClick={handlePrevious} style={{
            position: "absolute",
            left: "1rem",
            top: "50%",
            transform: "translateY(-50%)",
            background: "rgba(0,0,0,0.5)",
            color: "white",
            border: "none",
            borderRadius: "50%",
            width: "50px",
            height: "50px",
            fontSize: "1.5rem",
            cursor: "pointer"
          }}>⟵</button>

          {/* Next Button - Right */}
          <button onClick={handleNext} style={{
            position: "absolute",
            right: "1rem",
            top: "50%",
            transform: "translateY(-50%)",
            background: "rgba(0,0,0,0.5)",
            color: "white",
            border: "none",
            borderRadius: "50%",
            width: "50px",
            height: "50px",
            fontSize: "1.5rem",
            cursor: "pointer"
          }}>⟶</button>

          {/* D-Pad */}
          {currentCamera.name.includes("USB 2.0 Camera") && (
            <div style={{ position: "absolute", height: "100px", width: "100px", bottom: "2rem", right: "2rem", transform: "translateX(-50%)" }}>
              <DPad inputStream="up" />
            </div>
          )}
          
        </div>
      ) : (
        <div style={{
          position: "absolute",
          inset: 0,
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
          background: "rgba(0, 0, 0, 0.6)",
          color: "white",
          fontSize: "1.2rem",
          flexDirection: "column",
          textAlign: "center",
          zIndex: 5
        }}>
          <p style={{
            background: "rgba(255,255,255,0.1)",
            padding: "0.75rem 1.5rem",
            borderRadius: "12px",
            fontWeight: "500",
            backdropFilter: "blur(8px)"
          }}>
            No stream yet.<br />Click <strong>Start</strong> to begin.
          </p>
        </div>
      )}

      {/* Power Toggle Button */}
      <div style={{
        position: "absolute",
        bottom: "-75px",
        left: "50%",
        transform: "translate(-50%, -50%)",
        zIndex: 10
      }}>
        <PowerButton
          isActive={isStreaming}
          onClick={() => {
            if (isStreaming) {
              handleStop();
            } else {
              handleStart();
            }
          }}
        />
      </div>
    </div>
  );
}
export default CameraView;