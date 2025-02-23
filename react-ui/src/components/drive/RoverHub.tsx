import { useState } from 'react';
import RoverInfo from './ui/RoverInfo';
import RoverControl from './ui/RoverControl';
import './styles/RoverHub.css';

const RoverHub: React.FC = () => {
  const [isInfoMode, setIsInfoMode] = useState(true);

  return (
    <div className="hub-view">
        <div className="hub">
            <div className="hub-toggle">
                <div
                className={`hub-option ${isInfoMode ? "active" : ""}`}
                onClick={() => setIsInfoMode(true)}
                >
                Information
                </div>
                <div
                className={`hub-option ${!isInfoMode ? "active" : ""}`}
                onClick={() => setIsInfoMode(false)}
                >
                Controls
                </div>
            </div>
        </div>
      {isInfoMode ? <RoverInfo /> : <RoverControl />}
    </div>
  );
};
  
export default RoverHub;