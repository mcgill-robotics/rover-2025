"use client";

import React, { useState } from "react";

import { useWebRTCStream } from "@/hooks/useWebRTCStreams";
import { useCameraList } from "@/hooks/useCameraList";
import { useBandwidthStats } from "@/hooks/useBandwidthStats";

import DPad from "./DPadController";
import PowerButton from "@/components/ui/PowerButton";
import ArrowButton from "@/components/ui/ArrowButton";

const CameraView: React.FC = () => {
  const cameras = useCameraList();
  const [currentIndex, setCurrentIndex] = useState(0);
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

  const { bitrateKbps, pingMs } = useBandwidthStats(isStreaming);

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
          {/* Embedded video element directly */}
          <video
            key={videoKey}
            ref={videoRef}
            autoPlay
            playsInline
            muted
            className="w-full h-full object-cover"
          />

          {/* Camera Info Top Left */}
          <div className="absolute top-4 left-4 text-white text-base bg-black/50 px-4 py-2 rounded-lg">
            <strong>{currentCamera.name}</strong><br />
            <span className={`inline-block w-2.5 h-2.5 rounded-full mr-2 ${isLive ? 'bg-lime-400' : 'bg-red-500'}`}></span>
            Live
          </div>

          {/* Stream Stats Top Right */}
          <div className="absolute top-4 right-4 text-white text-sm bg-black/50 px-4 py-2 rounded-lg text-right">
            FPS: {fps}<br />
            Bitrate: {bitrateKbps !== "0" ? `${bitrateKbps} kbps` : "N/A"}<br />
            Ping: {pingMs !== null ? `${pingMs} ms` : "N/A"}
          </div>

          {/* Navigation Buttons */}
          <ArrowButton direction="left" onClick={handlePrevious} />
          <ArrowButton direction="right" onClick={handleNext} />

          {/* DPad Overlay */}
          {currentCamera.name.includes("USB 2.0 Camera") && (
            <div className="absolute bottom-4 right-8 w-[100px] h-[100px]">
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
