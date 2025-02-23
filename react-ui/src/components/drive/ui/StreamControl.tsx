import React from "react";
import "./styles/StreamControl.css";

interface StreamControlProps {
  deviceId: number; // Only one deviceId
  onDeviceIdChange: (event: React.ChangeEvent<HTMLInputElement>) => void;
  onStart: (deviceId: number) => void;
  onStop: () => void;
}

const StreamControl: React.FC<StreamControlProps> = ({
  deviceId,
  onDeviceIdChange,
  onStart,
  onStop,
}) => (
  <div className="stream-controls-container">
    <input
      type="number"
      value={deviceId}
      onChange={onDeviceIdChange}
      placeholder="Enter Device ID"
    />
    <div>
      <button onClick={() => onStart(deviceId)}>Start</button>
      <button onClick={onStop}>Stop</button>
    </div>
  </div>
);

export default StreamControl;