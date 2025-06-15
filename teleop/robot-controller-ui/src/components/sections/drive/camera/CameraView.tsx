"use client";

import React, { useEffect, useState } from "react";
import WebRTCPlayer from "./WebRTCPlayer";
import DPad from "./DPad";
// import PowerButton from "../arm/ui/PowerButton";
import axios from "axios";
import { useWebRTCStream } from "@/hooks/useWebRTCStreams";
import { CAMERAIP } from "@/config/config"

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
        const res = await axios.get(`http://${CAMERAIP}:8081/video-devices`);
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
  }, [currentCamera?.path, isStreaming, startStream, stopStream]);

  useEffect(() => {
    let intervalId: NodeJS.Timeout;

    const fetchStats = async () => {
      try {
        const res = await axios.get(`http://${CAMERAIP}:8081/bandwidth-stats`);
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
    <div className="relative w-full h-screen bg-black flex flex-col">
      {currentCamera && isStreaming ? (
        <div className="relative w-full h-full overflow-hidden">
          <WebRTCPlayer key={videoKey} devicePath={currentCamera.path} forwardedRef={videoRef} />

          {/* Camera Info Top Left */}
          <div className="absolute top-4 left-4 text-white text-base bg-black/50 px-4 py-2 rounded-lg">
            <strong>{currentCamera.name}</strong><br />
            <span className={`inline-block w-2.5 h-2.5 rounded-full mr-2 ${isLive ? 'bg-lime-400' : 'bg-red-500'}`}></span>
            Live
          </div>

          {/* Stream Stats Top Right */}
          <div className="absolute top-4 right-4 text-white text-sm bg-black/50 px-4 py-2 rounded-lg text-right">
            FPS: {fps}<br />
            Bitrate: {bitrate_kbps ? `${bitrate_kbps} kbps` : "N/A"}<br />
            Ping: {ping_ms ? `${ping_ms} ms` : "N/A"}
          </div>

          {/* Previous Button */}
          <button onClick={handlePrevious} className="absolute left-4 top-1/2 -translate-y-1/2 w-12 h-12 text-2xl rounded-full bg-black/50 text-white hover:bg-black/70">
            ⟵
          </button>

          {/* Next Button */}
          <button onClick={handleNext} className="absolute right-4 top-1/2 -translate-y-1/2 w-12 h-12 text-2xl rounded-full bg-black/50 text-white hover:bg-black/70">
            ⟶
          </button>

          {/* DPad Overlay */}
          {currentCamera.name.includes("USB 2.0 Camera") && (
            <div className="absolute bottom-8 right-8 w-[100px] h-[100px]">
              <DPad inputStream="up" />
            </div>
          )}
        </div>
      ) : (
        <div className="absolute inset-0 flex flex-col justify-center items-center bg-black/60 text-white text-center z-10">
          <p className="bg-white/10 px-6 py-4 rounded-xl font-medium backdrop-blur-md">
            No stream yet.<br />Click <strong>Start</strong> to begin.
          </p>
        </div>
      )}

      {/* Power Button */}
      <div className="absolute bottom-[-75px] left-1/2 -translate-x-1/2 z-20">
        {/* <PowerButton
          isActive={isStreaming}
          onClick={() => (isStreaming ? stopStream() : startStream())}
        /> */}
      </div>
    </div>
  );
};

export default CameraView;
