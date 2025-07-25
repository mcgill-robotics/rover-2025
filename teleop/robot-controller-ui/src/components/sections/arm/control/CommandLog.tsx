'use client';

import React, { useEffect, useRef, useState } from 'react';
import { Terminal } from 'lucide-react';

interface CommandLogProps {
  command?: string;
  output: string | string[];
  showPrompt?: boolean;
  id: number;
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
    if (terminalOutputRef.current && !isMouseInside) {
      terminalOutputRef.current.lastElementChild?.scrollIntoView({
        behavior: 'smooth',
        block: 'end',
      });
    }
  }, [output, isMouseInside]);

  const handleMouseEnter = () => setIsMouseInside(true);
  const handleMouseLeave = () => setIsMouseInside(false);

  const isSelected = selectedTerminal === id;

  return (
    <div
      className={`w-full flex-[4] rounded-md shadow-md my-4 font-mono bg-[#1e1e1e] text-[#f8f8f8] ${
        isSelected ? 'ring-2 ring-[#d63f3f]' : ''
      }`}
      onClick={handleSelectTerminal}
    >
      {/* Header */}
      <div className="flex items-center h-[10%] px-3 py-2 bg-[#2d2d2d] border-b border-white/10">
        <Terminal size={18} className="text-[#d63f3f] mr-2" />
        <span className="font-semibold text-sm">Command Line {id}</span>
        <button
          onClick={onClear}
          className="ml-auto text-sm text-white hover:text-[#d63f3f] hover:underline"
        >
          Clear
        </button>
      </div>

      {/* Body */}
      <div
        className="w-full h-[80%] px-4 py-3 text-base overflow-y-scroll"
        ref={terminalOutputRef}
        onMouseEnter={handleMouseEnter}
        onMouseLeave={handleMouseLeave}
      >
        {command && showPrompt && (
          <div className="flex text-xs mb-2">
            <span className="text-[#d63f3f] mr-2">$</span>
            <span className="font-medium text-white">{command}</span>
          </div>
        )}
        <div className="text-[0.6rem] text-[#b2b2b2] whitespace-pre-wrap break-words">
          {outputArray.map((line, index) => (
            <div key={index} className="leading-[1.5] mb-[2px]">
              {line}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};

export default CommandLog;
