import { useState } from 'react';
import RoverInfo from './ui/RoverInfo';
import RoverControl from './ui/RoverControl';
import GPS from './ui/GPS';
import './styles/RoverHub.css';

interface RoverHubProps {
  streams: (MediaStream | null)[];
  setStreams: React.Dispatch<React.SetStateAction<(MediaStream | null)[]>>;
}

const RoverHub: React.FC<RoverHubProps> = ({ streams, setStreams }) => {
  const [mode, setMode] = useState<"info"|"control"|"gps">("info");

  return (
    <div className="hub-view">
      <div className="hub">
        <div
          className={`hub-toggle ${mode == "info" ? 'info-mode' : 'control-mode'}`}
        >
          <div
            className={`hub-option ${mode == "info" ? 'active' : ''}`}
            onClick={() => setMode("info")}
          >
            Information
          </div>
          <div
            className={`hub-option ${mode == "control" ? 'active' : ''}`}
            onClick={() => setMode("control")}
          >
            Controls
          </div>
          <div
            className={`hub-option ${mode == "gps" ? 'active' : ''}`}
            onClick={() => setMode("gps")}
          >
            GPS
          </div>
        </div>
      </div> 

      {mode == "info" && <RoverInfo />}
      {mode == "control" && <RoverControl streams={streams} setStreams={setStreams} />}
      {mode == "gps" && <GPS />}
    </div>
  );
};

export default RoverHub;