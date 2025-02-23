// Dashboard.tsx
// Pages for drive controls

import CameraView from "../components/drive/CameraView";
import RoverHub from "../components/drive/RoverHub";
import "./styles/Dashboard.css";

const Dashboard: React.FC = () => {
  return (
    <div className="dashboard">
      <div className="dashboard-content">
        <CameraView />
        <RoverHub/>
      </div>
    </div>
  );
};

export default Dashboard;