import React, { useEffect, useRef } from 'react';
import './styles/Camera.css';

interface VideoFeedProps {
  stream: MediaStream | null;
}

const VideoFeed: React.FC<VideoFeedProps> = ({ stream }) => {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    if (videoRef.current) {
      videoRef.current.srcObject = stream ?? null;
    }
  }, [stream]);

  return (
    <div className="video-container">
      <video className="video" ref={videoRef} autoPlay playsInline />
    </div>
  );
};

export default VideoFeed;