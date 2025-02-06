// Dashboard.tsx
// Pages for drive controls

import CameraView from "../components/drive/CameraView";
import RoverInfo from "../components/drive/RoverInfo";
import "./styles/Dashboard.css";

const Dashboard: React.FC = () => {
  return (
    <div className="dashboard">
      <div className="dashboard-content">
        <CameraView />
        <RoverInfo />
      </div>
    </div>
  );
};

export default Dashboard;