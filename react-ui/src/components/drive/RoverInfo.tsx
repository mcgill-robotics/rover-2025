import Speedometer from "./ui/Speedometer";
import "./styles/RoverInfo.css";

const RoverInfo: React.FC = () => {
  return (
    <div className="info-container">
      <h1 className="info-title">Information</h1>
      

      <div className="speedometers">
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
      
    </div>
  );
};

export default RoverInfo;