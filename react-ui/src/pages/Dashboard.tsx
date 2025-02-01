import React from "react";
import CameraView from "../components/CameraView";
import RoverInfo from "../components/RoverInfo";
import "./Dashboard.css";

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