import React, { useEffect, useRef, useState } from 'react';
import { Terminal } from 'lucide-react';
import './styles/CommandLog.css';

interface CommandLogProps {
  command?: string;
  output: string | string[];
  showPrompt?: boolean;
  id: number;  // Unique identifier for each terminal (optional)
  onClear: () => void;
}

const CommandLog: React.FC<CommandLogProps> = ({
  command,
  output,
  showPrompt = true,
  id,
  onClear
}) => {
  const [selectedTerminal, setSelectedTerminal] = useState<number | null>(null);
  const [isMouseInside, setIsMouseInside] = useState(false);
  const outputArray = Array.isArray(output) ? output : output.split('\n');
  const terminalOutputRef = useRef<HTMLDivElement | null>(null);

  const handleSelectTerminal = () => {
    setSelectedTerminal(id);
  };

  useEffect(() => {
    if (terminalOutputRef.current) {
      if (!isMouseInside) {
        terminalOutputRef.current?.lastElementChild?.scrollIntoView({
          behavior: 'smooth', 
          block: 'end'
        });
      }
    }
  }, [output, isMouseInside]);

  const handleMouseEnter = () => {
    setIsMouseInside(true); 
  };

  const handleMouseLeave = () => {
    setIsMouseInside(false); 
  };

  const isSelected = selectedTerminal === id;

  return (
    <div 
      className={`terminal-container ${isSelected ? 'selected' : ''}`}
      onClick={handleSelectTerminal} 
    >
      <div className="terminal-header">
        <Terminal size={18} className="terminal-icon" />
        <span className="terminal-title">Command Line {id}</span>
        <button className="clear-terminal-btn" onClick={onClear}>
          Clear
        </button>
      </div>
      <div 
        className="terminal-body" 
        ref={terminalOutputRef}
        onMouseEnter={handleMouseEnter}
        onMouseLeave={handleMouseLeave}
      >
        {command && showPrompt && (
          <div className="terminal-command-line">
            <span className="terminal-prompt">$</span>
            <span className="terminal-command">{command}</span>
          </div>
        )}
        <div className="terminal-output">
          {outputArray.map((line, index) => (
            <div key={index} className="terminal-output-line">
              {line}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};

export default CommandLog;