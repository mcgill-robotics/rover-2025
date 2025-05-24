import React, { useEffect, useState } from 'react';
import Knob from './Knob';
import Headlight from './Headlight';
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
  const [pc, setPc] = useState<RTCPeerConnection | null>(null);
  const [error, setError] = useState<string | null>(null);
  const hostIp = "127.0.0.1";

  useEffect(() => {
    return () => {
      if (pc) {
        stop(pc); // Clean up peer connection when component unmounts
      }
    };
  }, [pc]);

  const startCamera = async (deviceId: number, index: number): Promise<void> => {
    try {
      let peerConnection: RTCPeerConnection;

      // If pc exists and is open, reuse it; otherwise, create a new one
      if (pc && pc.signalingState !== 'closed') {
        peerConnection = pc;
      } else {
        console.log("Peer connection is closed. Recreating peer connection...");
        peerConnection = new RTCPeerConnection({ iceServers: [] });
        setPc(peerConnection);  // Set the state with the new peer connection for future reuse
      }

      peerConnection.addEventListener('track', (evt: RTCTrackEvent) => {
        if (evt.track.kind === 'video') {
          setStreams((prev) => {
            const newStreams = [...prev];
            newStreams[index] = evt.streams[0];
            return newStreams;
          });
        } else {
          console.error("Video track not found");
          setError("Video track not found");
        }
      });

      await negotiate(deviceId, peerConnection); // Pass the peerConnection into the negotiate function
    } catch (err: any) {
      console.error('Error in startCamera:', err);
      setError(`Error: ${err.message || err}`);
    }
  };

  const negotiate = async (deviceId: number, peerConnection: RTCPeerConnection): Promise<void> => {
    try {
      if (!peerConnection || peerConnection.signalingState === 'closed') return;

      peerConnection.addTransceiver('video', { direction: 'recvonly' });

      const offer = await peerConnection.createOffer();
      await peerConnection.setLocalDescription(offer);

      await waitForIceGatheringComplete(peerConnection);

      const offerData = { sdp: peerConnection.localDescription?.sdp, type: peerConnection.localDescription?.type };
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

      await peerConnection.setRemoteDescription(new RTCSessionDescription(answer));
    } catch (err: any) {
      console.error('Error in negotiate:', err);
      setError(`Error: ${err.message || err}`);
    }
  };

  const stop = (pc: RTCPeerConnection): void => {
    if (pc) {
      // Stop all tracks of the peer connection before closing it
      const tracks = pc.getReceivers().map(receiver => receiver.track);
      tracks.forEach(track => track.stop());

      // Close the peer connection after stopping tracks
      setTimeout(() => {
        pc.close();
        setPc(null);
      }, 500);
    }

    setError(null);
    // Update the streams state to null for the corresponding camera
    setStreams((prev) => prev.map((stream) => null));
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

      <div className="connection-container">
          <div className="camera-control-container">
            <h2 className="camera-control-title">Available Devices</h2>
            <StreamControl
              onStart={startCamera}
              onStop={() => stop(pc!)}
            />
          </div>
        </div>
      {/* Error handling */}
      {error && <div className="error">{error}</div>}
    </div>
  );
};

export default RoverControl;