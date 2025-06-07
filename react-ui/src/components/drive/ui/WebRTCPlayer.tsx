import React, { useEffect, useRef } from "react";
import { CAMERAIP } from "../../../config/config";

interface WebRTCPlayerProps {
  devicePath: string;
  forwardedRef?: React.RefObject<HTMLVideoElement>;
}

const WebRTCPlayer: React.FC<WebRTCPlayerProps> = ({ devicePath, forwardedRef }) => {
  const localRef = useRef<HTMLVideoElement>(null);
  const videoRef = forwardedRef || localRef;

  useEffect(() => {
    const pc = new RTCPeerConnection();
    let isCancelled = false;

    pc.addTransceiver("video", { direction: "recvonly" });

    pc.ontrack = (event) => {
      if (!isCancelled && videoRef.current) {
        videoRef.current.srcObject = event.streams[0];
      }
    };

    pc.oniceconnectionstatechange = () => {
      console.log("[ICE] Connection state:", pc.iceConnectionState);
      if (pc.iceConnectionState === "failed" || pc.iceConnectionState === "disconnected") {
        console.warn("[ICE] Connection lost.");
      }
    };

    const negotiate = async () => {
      try {
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);

        const start = performance.now();
        const response = await fetch(
          `http://${CAMERAIP}:8081/offer?id=${encodeURIComponent(devicePath)}`,
          {
            method: "POST",
            body: JSON.stringify({
              sdp: pc.localDescription?.sdp,
              type: pc.localDescription?.type,
            }),
            headers: { "Content-Type": "application/json" },
          }
        );

        const latency = performance.now() - start;
        const data = await response.json();

        if (!data?.sdp || !data?.type) {
          throw new Error("Invalid response from WebRTC server");
        }

        await pc.setRemoteDescription(new RTCSessionDescription(data));
        console.log(`[PING] Offer/answer exchange latency: ${latency.toFixed(1)} ms`);
      } catch (err) {
        if (!isCancelled) console.error("[WebRTC] Negotiation failed:", err);
        pc.close();
      }
    };

    negotiate();

    return () => {
      isCancelled = true;
      pc.getSenders().forEach((sender) => sender.track?.stop());
      pc.getReceivers().forEach((receiver) => receiver.track?.stop());
      pc.close();
    };
  }, [devicePath]);

  return (
    <video
      ref={videoRef}
      autoPlay
      playsInline
      muted
      style={{ width: "100%", height: "100%", objectFit: "cover" }}
    />
  );
};

export default WebRTCPlayer;
