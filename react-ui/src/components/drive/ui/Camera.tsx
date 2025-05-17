import React from 'react';

interface MJPEGFeedProps {
  url: string; // MJPEG stream URL (e.g. http://localhost:8080)
}

const MJPEGFeed: React.FC<MJPEGFeedProps> = ({ url }) => {
  return (
    <div className="video-container">
      <img
        src={url}
        alt="MJPEG Stream"
        className="video"
        style={{ width: '100%', height: 'auto', objectFit: 'cover' }}
      />
    </div>
  );
};

export default MJPEGFeed;
