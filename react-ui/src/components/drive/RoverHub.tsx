import { useState } from 'react';
import RoverInfo from './ui/RoverInfo';
import RoverControl from './ui/RoverControl';
import './styles/RoverHub.css';

interface RoverHubProps {
  streams: (MediaStream | null)[];
  setStreams: React.Dispatch<React.SetStateAction<(MediaStream | null)[]>>;
}

const RoverHub: React.FC<RoverHubProps> = ({ streams, setStreams }) => {
  const [isInfoMode, setIsInfoMode] = useState(true);

  return (
    <div className="hub-view">
      <div className="hub">
        <div className="hub-toggle">
          <div
            className={`hub-option ${isInfoMode ? 'active' : ''}`}
            onClick={() => setIsInfoMode(true)}
          >
            Information
          </div>
          <div
            className={`hub-option ${!isInfoMode ? 'active' : ''}`}
            onClick={() => setIsInfoMode(false)}
          >
            Controls
          </div>
        </div>
      </div>

      {isInfoMode ? (
        <RoverInfo />
      ) : (
        <RoverControl streams={streams} setStreams={setStreams} />
      )}
    </div>
  );
};

export default RoverHub;