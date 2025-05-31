// hooks/useWebRTCStream.ts
import { useCallback, useEffect, useRef, useState } from "react";

export interface WebRTCStreamOptions {
  devicePath: string;
  stallTimeout?: number; // in ms
  restartDelay?: number; // in ms
}

export function useWebRTCStream({ devicePath, stallTimeout = 3000, restartDelay = 500 }: WebRTCStreamOptions) {
  const [isStreaming, setIsStreaming] = useState(false);
  const [videoKey, setVideoKey] = useState(0);
  const [frameTimestamps, setFrameTimestamps] = useState<number[]>([]);

  const videoRef = useRef<HTMLVideoElement>(null);
  const lastSeenTimeRef = useRef<number>(0);
  const lastFrameTimeRef = useRef<number | null>(null);

  const resetStats = useCallback(() => {
    setFrameTimestamps([]);
    lastSeenTimeRef.current = 0;
    lastFrameTimeRef.current = null;
  }, []);

  const startStream = useCallback(() => {
    if (!devicePath) return;
    setIsStreaming(true);
    resetStats();
  }, [devicePath, resetStats]);

  const stopStream = useCallback(() => {
    setIsStreaming(false);
  }, []);

  useEffect(() => {
    let animationId: number;

    const monitor = () => {
      const video = videoRef.current;
      if (video && video.readyState >= 2) {
        const currentTime = Date.now();
        const videoTime = video.currentTime;

        if (videoTime !== lastSeenTimeRef.current) {
          lastSeenTimeRef.current = videoTime;
          setFrameTimestamps((prev) => {
            const updated = [...prev, currentTime].filter((t) => currentTime - t <= 2000);
            return updated;
          });
          lastFrameTimeRef.current = currentTime;
        }
      }

      if (lastFrameTimeRef.current && Date.now() - lastFrameTimeRef.current > stallTimeout) {
        console.warn("Stream stalled. Reconnecting...");
        setIsStreaming(false);
        setTimeout(() => {
          resetStats();
          setVideoKey((prev) => prev + 1);
          setIsStreaming(true);
        }, restartDelay);
      }

      animationId = requestAnimationFrame(monitor);
    };

    if (isStreaming && devicePath) {
      monitor();
    }

    return () => cancelAnimationFrame(animationId);
  }, [isStreaming, devicePath, stallTimeout, restartDelay, resetStats]);

  const fps = frameTimestamps.length > 1
    ? ((frameTimestamps.length - 1) / ((frameTimestamps.at(-1)! - frameTimestamps[0]) / 1000)).toFixed(1)
    : "0.0";

  const isLive = lastFrameTimeRef.current && Date.now() - lastFrameTimeRef.current < 2000;

  return {
    isStreaming,
    startStream,
    stopStream,
    videoKey,
    videoRef,
    fps,
    isLive,
  };
}
