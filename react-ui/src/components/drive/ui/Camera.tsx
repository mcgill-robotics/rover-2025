import React, { useEffect, useRef } from 'react';
import Peer from 'peerjs'; // Import Peer from PeerJS
import './styles/Camera.css';

const Camera: React.FC = () => {
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const peerRef = useRef<Peer | null>(null); // Use `Peer` type from PeerJS

  useEffect(() => {
    const connectToWebRTC = async () => {
      try {
        // Initialize PeerJS connection with an explicit peer ID
        const peer = new Peer('peer1', {
          host: 'localhost', // Replace with your signaling server host
          port: 9000, // Replace with your signaling server port
          // path: '/myapp', // Replace with your signaling server path
        });
        peerRef.current = peer;

        // Listen for when the connection is open
        peer.on('open', async (id) => {
          console.log('PeerJS connection established with ID:', id);

          // Get local media stream
          const localStream = await navigator.mediaDevices.getUserMedia({ video: true, audio: true });

          // Call another peer (replace 'peer2' with the ID of the peer you want to connect to)
          const call = peer.call('peer2', localStream);

          // Handle incoming stream
          call.on('stream', (remoteStream) => {
            if (videoRef.current) {
              videoRef.current.srcObject = remoteStream;
            }
          });

          call.on('error', (err) => {
            console.error('Error in call:', err);
          });
        });

        // Handle incoming calls
        peer.on('call', async (call) => {
          // Answer the call with local media stream
          try {
            const localStream = await navigator.mediaDevices.getUserMedia({ video: true, audio: true });
            call.answer(localStream); // Answer the call with the local stream

            // Handle incoming stream
            call.on('stream', (remoteStream) => {
              if (videoRef.current) {
                videoRef.current.srcObject = remoteStream;
              }
            });
          } catch (error) {
            console.error('Error accessing media devices:', error);
          }
        });

        peer.on('error', (err) => {
          console.error('Error in peer connection:', err);
        });

        peer.on('close', () => {
          console.log('Peer connection closed');
        });
      } catch (error) {
        console.error('Error connecting to WebRTC server:', error);
      }
    };

    connectToWebRTC();

    return () => {
      // Cleanup: close peer connection and stop media tracks
      if (peerRef.current) {
        peerRef.current.destroy();
      }
      if (videoRef.current?.srcObject) {
        (videoRef.current.srcObject as MediaStream).getTracks().forEach((track) => track.stop());
      }
    };
  }, []);

  return <video className="video" ref={videoRef} autoPlay playsInline />;
};

export default Camera;