import "./styles/RoverInfo.css";

const RoverInfo: React.FC = () => {
  return (
    <div className="rover-info">
      <h2 className="info-title">Information</h2>
      <div className="info-content">
        <div className="info-item">
          <span>Power Speed</span>
          <span className="info-value">3.0</span>
        </div>
        <div className="info-item">
          <span>Speed</span>
          <span className="info-value">2.5</span>
        </div>
        <div className="info-item">
          <span>Wheel</span>
          <span className="info-value">x 3.0</span>
        </div>
      </div>
    </div>
  );
};

export default RoverInfo;