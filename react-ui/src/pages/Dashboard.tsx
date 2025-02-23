// Dashboard.tsx
// Pages for drive controls

import { useState } from 'react';
import CameraView from '../components/drive/CameraView';
import RoverHub from '../components/drive/RoverHub';
import './styles/Dashboard.css';

const Dashboard: React.FC = () => {
  // Shared state for camera streams
  const [streams, setStreams] = useState<(MediaStream | null)[]>(Array(4).fill(null));

  return (
    <div className="dashboard">
      <div className="dashboard-content">
        <CameraView streams={streams} />
        <RoverHub streams={streams} setStreams={setStreams} />
      </div>
    </div>
  );
};

export default Dashboard;
