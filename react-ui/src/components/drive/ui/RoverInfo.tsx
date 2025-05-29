import WheelsContainer from "./WheelsContainer";
import Speedometer from "./Speedometer";
import GPS from './GPS';
import "./styles/RoverInfo.css";

const RoverInfo: React.FC = () => {
  return (
    <div className="drive-info-container"> 
      <div className="speedometer-container">
        <h2 className="speed-title">Rover Speed</h2>
        {/* Main speedometer */}
        <div className="main-speedometer">
          <Speedometer initialSpeed={80} />
        </div>
      
        {/* Four smaller speedometers */}
        <div className="small-speedometers">
          <div className="speedometer-item fl">
            <Speedometer initialSpeed={80} />
            <span className="label">FL</span>
          </div>
          <div className="speedometer-item fr">
            <Speedometer initialSpeed={80} />
            <span className="label">FR</span>
          </div>
          <div className="speedometer-item bl">
            <Speedometer initialSpeed={80} />
            <span className="label">BL</span>
          </div>
          <div className="speedometer-item br">
            <Speedometer initialSpeed={80} />
            <span className="label">BR</span>
          </div>
        </div>
      </div>

      <div className="wheels-minimap-container">
        <WheelsContainer />
      </div>

      <div className="gps-container">
        <GPS />
      </div>
    </div>
  );
};

export default RoverInfo;