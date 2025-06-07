import { useState, useRef } from 'react';
import PowerButton from './ui/PowerButton';
import CommandLog from './ui/CommandLog';
import PS4Controller from './ui/PS4Controller';
import './styles/ArmControl.css';

const ArmControl: React.FC = () => {
  const [output, setOutput] = useState<string[]>([]);
  const [command, setCommand] = useState<string | undefined>(undefined);
  const [isRunning, setIsRunning] = useState<boolean>(false);
  const eventSourceRef = useRef<EventSource | null>(null); // Ref to store EventSource instance

  const startRosNode = () => {
    if (isRunning) {
      // If already running, stop the EventSource connection
      if (eventSourceRef.current) {
        eventSourceRef.current.close();
        console.log("Connection closed");
      }
      setIsRunning(false);
    } else {
      // If not running, start the EventSource connection
      setIsRunning(true);
      eventSourceRef.current = new EventSource('http://localhost:5000/start-ros');

      clearTerminal();
      setCommand("ros2 run cpp_pubsub talker")

      eventSourceRef.current.onmessage = (event) => {
        setOutput((prev) => [...prev, event.data]);
      };

      eventSourceRef.current.onerror = () => {
        console.error('Error receiving ROS output');
        eventSourceRef.current?.close();
        setIsRunning(false);
      };
    }
  };

  const clearTerminal = () => {
    setOutput([]);
    setCommand("");
  };

  return (
    <div className="arm-control-container">
      <PowerButton onClick={startRosNode} />
      <CommandLog command={command} output={output} id={1} onClear={clearTerminal} />
      <PS4Controller />
    </div>
  );
};

export default ArmControl;