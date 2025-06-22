import { useCallback, useEffect, useRef, useState } from "react";

export interface WebRTCStreamOptions {
  devicePath: string;
  stallTimeout?: number;
  restartDelay?: number;
}

export function useWebRTCStream({
  devicePath,
  stallTimeout = 3000,
  restartDelay = 500,
}: WebRTCStreamOptions) {
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
    if (isStreaming) {
      resetStats();
      setVideoKey((prev) => prev + 1);
    }
  }, [devicePath, isStreaming, resetStats]);

  useEffect(() => {
    if (!isStreaming || !devicePath) return;

    const pc = new RTCPeerConnection();
    let isCancelled = false;

    const attachStream = (stream: MediaStream) => {
      if (videoRef.current) {
        videoRef.current.srcObject = stream;
      }
    };

    const negotiate = async () => {
      try {
        pc.addTransceiver("video", { direction: "recvonly" });

        pc.ontrack = (event) => {
          if (!isCancelled) attachStream(event.streams[0]);
        };

        pc.oniceconnectionstatechange = () => {
          const state = pc.iceConnectionState;
          console.log("[ICE] State:", state);
          if (["disconnected", "failed"].includes(state)) {
            console.warn("[WebRTC] Connection lost");
          }
        };

        await pc.setLocalDescription(await pc.createOffer());

        const response = await fetch(
          `http://${location.hostname}:8081/offer?id=${encodeURIComponent(devicePath)}`,
          {
            method: "POST",
            body: JSON.stringify(pc.localDescription),
            headers: { "Content-Type": "application/json" },
          }
        );

        const answer = await response.json();
        if (!answer?.sdp || !answer?.type) throw new Error("Invalid SDP");

        await pc.setRemoteDescription(new RTCSessionDescription(answer));
      } catch (err) {
        if (!isCancelled) console.error("[WebRTC] Negotiation failed:", err);
        pc.close();
      }
    };

    negotiate();

    return () => {
      isCancelled = true;
      pc.getSenders().forEach((s) => s.track?.stop());
      pc.getReceivers().forEach((r) => r.track?.stop());
      pc.close();
    };
  }, [devicePath, isStreaming]);

  useEffect(() => {
    if (!isStreaming || !devicePath) return;

    let animationId: number;

    const monitor = () => {
      const video = videoRef.current;
      if (!video || video.readyState < 2) {
        animationId = requestAnimationFrame(monitor);
        return;
      }

      const now = Date.now();
      const currentTime = video.currentTime;

      if (currentTime !== lastSeenTimeRef.current) {
        lastSeenTimeRef.current = currentTime;
        setFrameTimestamps((prev) =>
          [...prev, now].filter((t) => now - t <= 2000)
        );
        lastFrameTimeRef.current = now;
      }

      if (
        lastFrameTimeRef.current &&
        now - lastFrameTimeRef.current > stallTimeout
      ) {
        console.warn("[WebRTC] Stream stalled. Reconnecting...");
        setIsStreaming(false);
        setTimeout(() => {
          resetStats();
          setVideoKey((k) => k + 1);
          setIsStreaming(true);
        }, restartDelay);
      }

      animationId = requestAnimationFrame(monitor);
    };

    monitor();

    return () => cancelAnimationFrame(animationId);
  }, [devicePath, isStreaming, stallTimeout, restartDelay, resetStats]);

  const fps =
    frameTimestamps.length > 1
      ? (
          (frameTimestamps.length - 1) /
          ((frameTimestamps.at(-1)! - frameTimestamps[0]) / 1000)
        ).toFixed(1)
      : "0.0";

  const isLive =
    lastFrameTimeRef.current != null &&
    Date.now() - lastFrameTimeRef.current < 2000;

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
