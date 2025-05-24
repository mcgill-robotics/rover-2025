import React, { useEffect, useRef } from "react";

interface WebRTCPlayerProps {
  devicePath: string;
  forwardedRef?: React.RefObject<HTMLVideoElement>;
}

const WebRTCPlayer: React.FC<WebRTCPlayerProps> = ({ devicePath, forwardedRef }) => {
  const localRef = useRef<HTMLVideoElement>(null);
  const videoRef = forwardedRef || localRef;

  useEffect(() => {
    const pc = new RTCPeerConnection();

    pc.addTransceiver("video", { direction: "recvonly" });

    pc.ontrack = (event) => {
      if (videoRef.current) {
        videoRef.current.srcObject = event.streams[0];
      }
    };

    const negotiate = async () => {
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);

      const response = await fetch(
        `http://localhost:8081/offer?id=${encodeURIComponent(devicePath)}`,
        {
          method: "POST",
          body: JSON.stringify({
            sdp: pc.localDescription?.sdp,
            type: pc.localDescription?.type,
          }),
          headers: { "Content-Type": "application/json" },
        }
      );

      const data = await response.json();
      await pc.setRemoteDescription(new RTCSessionDescription(data));
    };

    negotiate();

    return () => {
      pc.close();
    };
  }, [devicePath]);

  return (
    <video
      ref={videoRef}
      autoPlay
      playsInline
      muted
      style={{ width: "100%", height: "auto", objectFit: "cover" }}
    />
  );
};

export default WebRTCPlayer;
