import React, { useRef, useState } from 'react';
import './styles/Knob.css';

// CodePen Example Credit:
// Original design and implementation by Kevin Lam
// URL: https://codepen.io/kevin-lam-the-scripter/pen/QWNeNvo

const Knob: React.FC = () => {
  const [speedIndex, setSpeedIndex] = useState<number>(0);
  const knobRef = useRef<HTMLDivElement | null>(null);
  const tickContainerRef = useRef<HTMLDivElement | null>(null);
  const minRef = useRef<HTMLSpanElement | null>(null);
  const maxRef = useRef<HTMLSpanElement | null>(null);

  const speedValues = [10, 30, 50, 70, 80]; // Speed levels
  const numTicks = speedValues.length;
  const stepAngle = 270 / (numTicks - 1); // Step angle for ticks

  const handleKnobClick = () => {
    const nextIndex = (speedIndex + 1) % numTicks; // Cycle through speed values
    setSpeedIndex(nextIndex);
    updateVisuals(nextIndex);
  };

  const updateVisuals = (index: number) => {
    const newAngle = index * stepAngle;
    if (knobRef.current) knobRef.current.style.transform = `rotate(${newAngle}deg)`;
    if (tickContainerRef.current) createTicks(tickContainerRef.current, numTicks, index);
    if (minRef.current) minRef.current.classList.toggle('neon', index === 0);
    if (maxRef.current) maxRef.current.classList.toggle('neon', index === numTicks - 1);
  };

  const createTicks = (container: HTMLDivElement, numTicks: number, highlightNumTicks: number) => {
    container.innerHTML = '';
    let startingTickAngle = -135;

    for (let i = 0; i < numTicks; i++) {
      const tick = document.createElement('div');
      tick.className = `tick ${i <= highlightNumTicks ? 'activetick' : ''}`;
      tick.style.transform = `rotate(${startingTickAngle}deg)`;
      container.appendChild(tick);
      startingTickAngle += stepAngle;
    }
  };

  return (
    <div className="knob">
      <div className="wheel-surround">
        <div ref={knobRef} className="wheel" onClick={handleKnobClick}></div>
        <span className="current-speed">{speedValues[speedIndex]} km/h</span>
        <span ref={minRef} className="min">Min</span>
        <span ref={maxRef} className="max">Max</span>
        <div ref={tickContainerRef} className="ticks"></div>
      </div>
    </div>
  );
};

export default Knob;