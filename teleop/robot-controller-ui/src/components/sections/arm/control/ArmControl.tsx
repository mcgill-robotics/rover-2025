'use client';

import { useState, useRef } from 'react';
import PowerButton from './PowerButton';
import CommandLog from './CommandLog';
import PS4Controller from './PS4Controller/PS4Controller';

const ArmControl: React.FC = () => {
  const [output, setOutput] = useState<string[]>([]);
  const [command, setCommand] = useState<string | undefined>(undefined);
  const [isRunning, setIsRunning] = useState<boolean>(false);
  const eventSourceRef = useRef<EventSource | null>(null);

  const startRosNode = () => {
    if (isRunning) {
      if (eventSourceRef.current) {
        eventSourceRef.current.close();
        console.log('Connection closed');
      }
      setIsRunning(false);
    } else {
      setIsRunning(true);
      eventSourceRef.current = new EventSource('http://localhost:5000/start-ros');
      clearTerminal();
      setCommand('ros2 run cpp_pubsub talker');

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
    setCommand('');
  };

  return (
    <div className="w-full h-full flex flex-col">
      <PowerButton onClick={startRosNode} />
      <CommandLog command={command} output={output} id={1} onClear={clearTerminal} />
      <PS4Controller />
    </div>
  );
};

export default ArmControl;
