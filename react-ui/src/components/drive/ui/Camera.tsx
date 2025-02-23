import React, { useEffect, useRef, useState } from 'react';
import './styles/Camera.css'

const waitForIceGatheringComplete = (pc: RTCPeerConnection): Promise<void> => {
  return new Promise((resolve) => {
    const checkState = () => {
      if (pc.iceGatheringState === 'complete') {
        pc.removeEventListener('icegatheringstatechange', checkState);
        resolve();
      }
    };

    if (pc.iceGatheringState === 'complete') {
      resolve();
    } else {
      pc.addEventListener('icegatheringstatechange', checkState);
    }
  });
};

const VideoFeed: React.FC = () => {
  const [pc, setPc] = useState<RTCPeerConnection | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [deviceId, setDeviceId] = useState<number>(0);
  const videoElementRef = useRef<HTMLVideoElement | null>(null);
  const hostIp = "127.0.0.1";

  useEffect(() => {
    return () => {
      stop();
    };
  }, []);

  const negotiate = async (): Promise<void> => {
    try {
      if (!pc) return;

      pc.addTransceiver('video', { direction: 'recvonly' });

      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);

      await waitForIceGatheringComplete(pc);

      const offerData = { sdp: pc.localDescription?.sdp, type: pc.localDescription?.type };
      console.log("Offer data:", offerData);

      const response = await fetch(`http://${hostIp}:8081/offer?id=${deviceId}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(offerData),
      });

      console.log("Fetch response:", response);

      const answer = await response.json();
      console.log("Answer data:", answer);

      await pc.setRemoteDescription(new RTCSessionDescription(answer));
    } catch (err: any) {
      console.error('Error in negotiate:', err);
      setError(`Error: ${err.message || err}`);
    }
  };

  const start = async (): Promise<void> => {
    alert("Hello");
    const config: RTCConfiguration = { iceServers: [] };
    const peerConnection = new RTCPeerConnection(config);

    peerConnection.addEventListener('track', (evt: RTCTrackEvent) => {
      if (evt.track.kind === 'video' && videoElementRef.current) {
        videoElementRef.current.srcObject = evt.streams[0];
      } else {
        console.error("Video element not found");
        setError("Video element not found");
      }
    });

    setPc(peerConnection);
    await negotiate();
  };

  const stop = (): void => {
    if (pc) {
      setTimeout(() => {
        pc.close();
        setPc(null);
      }, 500);
    }
    setError(null);
  };

  return (
    <div>
      <input
        type="number"
        value={deviceId}
        onChange={(e) => setDeviceId(Number(e.target.value))}
        placeholder="Enter Device ID"
        style={{ marginBottom: '10px' }}
      />
      <video
        className="feed"
        ref={videoElementRef}
        autoPlay
        playsInline
      />
      {error && <div style={{ color: 'red' }}>{`Error: ${error}`}</div>}
      <button onClick={start}>Start</button>
      <button onClick={stop}>Stop</button>
    </div>
  );
};

export default VideoFeed;