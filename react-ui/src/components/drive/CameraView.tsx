"use client";

import React, { useEffect, useState } from "react";
import WebRTCPlayer from "./ui/WebRTCPlayer";
import DPad from "./ui/DPad";
import PowerButton from "../arm/ui/PowerButton";
import axios from "axios";
import "./styles/CameraView.css";
import { useWebRTCStream } from "../../hooks/useWebRTCStreams";

const targetCameraNames = [
  "USB 2.0 Camera",
  "Logitech HD Webcam",
  "VGA USB Camera",
  "CC HD webcam            "
];

interface CameraInfo {
  name: string;
  path: string;
}

const CameraView: React.FC = () => {
  const [cameras, setCameras] = useState<CameraInfo[]>([]);
  const [currentIndex, setCurrentIndex] = useState(0);
  const [bitrate_kbps, setBitrate] = useState<string>("N/A");
  const [ping_ms, setPing] = useState<number | null>(null);

  const currentCamera = cameras[currentIndex];
  const {
    isStreaming,
    startStream,
    stopStream,
    videoKey,
    videoRef,
    fps,
    isLive,
  } = useWebRTCStream({ devicePath: currentCamera?.path || "" });

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

  // Automatically reconnect stream when camera changes
  useEffect(() => {
    if (!currentCamera?.path) return;

    if (isStreaming) {
      stopStream();
      setTimeout(() => {
        startStream();
      }, 100);
    }
  }, [currentCamera?.path]);

  useEffect(() => {
    let intervalId: NodeJS.Timeout;

    const fetchStats = async () => {
      try {
        const res = await axios.get("http://localhost:8081/bandwidth-stats");
        const stats = res.data.bandwidth_stats?.[0];
        if (stats?.rtt_ms != null) setBitrate(stats.bitrate_kbps);
        if (res.data?.ping_ms != null) setPing(res.data.ping_ms);
      } catch (err) {
        console.error("Failed to fetch bandwidth stats:", err);
      }
    };

    if (isStreaming) {
      fetchStats();
      intervalId = setInterval(fetchStats, 2000);
    }

    return () => clearInterval(intervalId);
  }, [isStreaming]);

  const handleNext = () => {
    setCurrentIndex((prev) => (prev + 1) % cameras.length);
  };

  const handlePrevious = () => {
    setCurrentIndex((prev) => (prev - 1 + cameras.length) % cameras.length);
  };

  return (
    <div className="camera-view" style={{ height: "100vh", position: "relative", backgroundColor: "#000" }}>
      {currentCamera && isStreaming ? (
        <div style={{ height: "100%", width: "100%", position: "relative", overflow: "hidden" }}>
          <WebRTCPlayer key={videoKey} devicePath={currentCamera.path} forwardedRef={videoRef} />

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
            Bitrate: {bitrate_kbps ? `${bitrate_kbps} kbps` : "N/A"}<br />
            Ping: {ping_ms ? `${ping_ms} ms` : "N/A"}
          </div>

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
              stopStream();
            } else {
              startStream();
            }
          }}
        />
      </div>
    </div>
  );
};

export default CameraView;
