"use client";

import React, { useEffect, useState } from "react";
import { useWebRTCStream } from "@/hooks/useWebRTCStreams";
import { useBandwidthStats } from "@/hooks/useBandwidthStats";

import DPad from "./DPadController";
import PowerButton from "@/components/ui/PowerButton";
import ArrowButton from "@/components/ui/ArrowButton";

interface CameraInfo {
  name: string;
  id: string;
}

const CameraView: React.FC = () => {
  const [cameras, setCameras] = useState<CameraInfo[]>([]);
  const [currentIndex, setCurrentIndex] = useState(0);
  const [selectedCamera, setSelectedCamera] = useState<CameraInfo | null>(null);

  const {
    isStreaming,
    startStream,
    stopStream,
    videoKey,
    videoRef,
    fps,
    isLive,
  } = useWebRTCStream({ cameraName: selectedCamera?.name || "" });

  const { bitrateKbps, pingMs } = useBandwidthStats(isStreaming);

  useEffect(() => {
    const fetchCameraList = async () => {
      try {
        const res = await fetch(`http://${location.hostname}:8081/video-devices`);
        const data = await res.json();
        const all: CameraInfo[] = [];

        data.devices.forEach((entry: { name: string; devices: string[] }) => {
          entry.devices.forEach((dev) => {
            all.push({ name: entry.name, id: dev });
          });
        });

        setCameras(all);
        setSelectedCamera(all[0] || null);
      } catch (err) {
        console.error("Failed to fetch camera list:", err);
      }
    };

    fetchCameraList();
  }, []);

  const handleNext = () => {
    if (cameras.length === 0) return;
    const nextIndex = (currentIndex + 1) % cameras.length;
    setCurrentIndex(nextIndex);
    setSelectedCamera(cameras[nextIndex]);
  };

  const handlePrevious = () => {
    if (cameras.length === 0) return;
    const prevIndex = (currentIndex - 1 + cameras.length) % cameras.length;
    setCurrentIndex(prevIndex);
    setSelectedCamera(cameras[prevIndex]);
  };

  return (
    <div className="relative w-full h-screen bg-black flex flex-col">
      {selectedCamera && isStreaming ? (
        <div className="relative w-full h-full overflow-hidden">
          <video
            key={videoKey}
            ref={videoRef}
            autoPlay
            playsInline
            muted
            className="w-full h-full object-cover"
          />

          <div className="absolute top-4 left-4 text-white text-base bg-black/50 px-4 py-2 rounded-lg">
            <strong>{selectedCamera.name}</strong><br />
            <span
              className={`inline-block w-2.5 h-2.5 rounded-full mr-2 ${
                isLive ? "bg-lime-400" : "bg-red-500"
              }`}
            ></span>
            Live
          </div>

          <div className="absolute top-4 right-4 text-white text-sm bg-black/50 px-4 py-2 rounded-lg text-right">
            FPS: {fps}
            <br />
            Bitrate: {bitrateKbps !== "0" ? `${bitrateKbps} kbps` : "N/A"}
            <br />
            Ping: {pingMs !== null ? `${pingMs} ms` : "N/A"}
          </div>

          <ArrowButton direction="left" onClick={handlePrevious} />
          <ArrowButton direction="right" onClick={handleNext} />

          {selectedCamera.name.includes("USB 2.0 Camera") && (
            <div className="absolute bottom-4 right-8 w-[100px] h-[100px]">
              <DPad inputStream="up" />
            </div>
          )}
        </div>
      ) : (
        <div className="absolute inset-0 flex flex-col justify-center items-center bg-black/60 text-white text-center z-10">
          <p className="bg-white/10 px-6 py-4 rounded-xl font-medium backdrop-blur-md">
            No stream yet.
            <br />
            Click <strong>Start</strong> to begin.
          </p>
        </div>
      )}

      <div className="absolute bottom-2 left-1/2 -translate-x-1/2 z-20">
        <PowerButton
          onClick={() => (isStreaming ? stopStream() : startStream())}
          size="small"
          isActive={isStreaming}
          label="Start"
        />
      </div>
    </div>
  );
};

export default CameraView;
