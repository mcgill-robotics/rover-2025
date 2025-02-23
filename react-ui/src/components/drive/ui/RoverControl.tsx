import React, { useEffect, useRef, useState } from 'react';
import Knob from './Knob';
import Headlight from './Headlight';
import StreamControl from './StreamControl';
import './styles/RoverControl.css';

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

interface RoverControlProps {
  streams: (MediaStream | null)[];
  setStreams: React.Dispatch<React.SetStateAction<(MediaStream | null)[]>>;
}

const RoverControl: React.FC<RoverControlProps> = ({ streams, setStreams }) => {
  const [deviceIds, setDeviceIds] = useState<number[]>([0, 1, 2, 3]);
  const [pc, setPc] = useState<RTCPeerConnection | null>(null);
  const [error, setError] = useState<string | null>(null);
  const videoElementRef = useRef<HTMLVideoElement | null>(null);
  const hostIp = "127.0.0.1";

  useEffect(() => {
    return () => {
      if (pc) {
        stop(pc);
      }
    };
  }, [pc]);

  const startCamera = async (deviceId: number, index: number): Promise<void> => {
    try {
      if (pc?.signalingState === 'closed') {
        console.log("Peer connection is closed. Recreating peer connection...");
        const config: RTCConfiguration = { iceServers: [] };
        const peerConnection = new RTCPeerConnection(config);
        setPc(peerConnection); // Update the state with the new peer connection
      }

      const peerConnection = pc || new RTCPeerConnection({ iceServers: [] });

      peerConnection.addEventListener('track', (evt: RTCTrackEvent) => {
        if (evt.track.kind === 'video' && videoElementRef.current) {
          videoElementRef.current.srcObject = evt.streams[0];
          setStreams((prev) => {
            const newStreams = [...prev];
            newStreams[index] = evt.streams[0];
            return newStreams;
          });
        } else {
          console.error("Video element not found");
          setError("Video element not found");
        }
      });

      setPc(peerConnection);
      await negotiate(deviceId);
    } catch (err: any) {
      console.error('Error in startCamera:', err);
      setError(`Error: ${err.message || err}`);
    }
  };

  const negotiate = async (deviceId: number): Promise<void> => {
    try {
      if (!pc || pc.signalingState === 'closed') return;

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

      // Check if the connection is still open before setting remote description
      if (pc.signalingState === 'closed' as string) {
        console.error("Peer connection has been closed before setting remote description");
        return;
      }

      await pc.setRemoteDescription(new RTCSessionDescription(answer));
    } catch (err: any) {
      console.error('Error in negotiate:', err);
      setError(`Error: ${err.message || err}`);
    }
  };

  const stop = (pc: RTCPeerConnection): void => {
    if (pc) {
      setTimeout(() => {
        pc.close();
        setPc(null);
      }, 500);
    }
    setError(null);
  };

  const handleDeviceIdChange = (index: number, event: React.ChangeEvent<HTMLInputElement>) => {
    const updatedDeviceIds = [...deviceIds];
    updatedDeviceIds[index] = parseInt(event.target.value, 10) || 0;
    setDeviceIds(updatedDeviceIds);
  };

  return (
    <div className="drive-info-container">
      <div className="knob-light-container">
        <div className="knob-container">
          <h2 className="knob-title">Maximum speed</h2>
          <Knob />
        </div>

        <div className="headlight-container">
          <h2 className="headlight-title">Brightness</h2>
          <Headlight />
        </div>
      </div>

      <div className="camera-controls-container">
        {deviceIds.map((deviceId, index) => (
          <StreamControl
            key={index}
            deviceId={deviceId}
            onDeviceIdChange={(event) => handleDeviceIdChange(index, event)}
            onStart={() => startCamera(deviceId, index)}
            onStop={() => stop(pc!)}
          />
        ))}
      </div>

      <video
        className="feed"
        ref={videoElementRef}
        autoPlay
        playsInline
      />
      {error && <div style={{ color: 'red' }}>{`Error: ${error}`}</div>}
    </div>
  );
};

export default RoverControl;