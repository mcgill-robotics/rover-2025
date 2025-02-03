import React, { useEffect, useRef, useState } from 'react';
import './styles/Knob.css';

// CodePen Example Credit:
// Original design and implementation by Kevin Lam
// URL: https://codepen.io/kevin-lam-the-scripter/pen/QWNeNvo

const Knob: React.FC = () => {
  const [speed, setSpeed] = useState<number>(0); // For tracking speed
  const knobRef = useRef<HTMLDivElement | null>(null);
  const tickContainerRef = useRef<HTMLDivElement | null>(null);
  const minRef = useRef<HTMLSpanElement | null>(null);
  const maxRef = useRef<HTMLSpanElement | null>(null);

  useEffect(() => {
    const knob = knobRef.current;
    const tickContainer = tickContainerRef.current;
    const minLabel = minRef.current;
    const maxLabel = maxRef.current;

    const numTicks = 5; // Number of ticks
    const stepAngle = 270 / (numTicks - 1); // Calculate step angle for resistance
    const speedValues = [10, 30, 50, 70, 80]; // Speed values for each tick

    const handleMouseDown = () => {
      if (knob) {
        document.addEventListener('mousemove', handleMouseMove);
      }
    };

    const handleMouseUp = () => {
      document.removeEventListener('mousemove', handleMouseMove);
    };

    const handleMouseMove = (event: MouseEvent) => {
      if (!knob || !tickContainer) return;

      const boundingRect = knob.getBoundingClientRect();
      const knobCenterX = boundingRect.width / 2 + boundingRect.left;
      const knobCenterY = boundingRect.height / 2 + boundingRect.top;

      const mouseX = event.pageX;
      const mouseY = event.pageY;

      const adjacentSide = knobCenterX - mouseX;
      const oppositeSide = knobCenterY - mouseY;
      const currentRadiansAngle = Math.atan2(adjacentSide, oppositeSide);
      const getRadiansInDegrees = currentRadiansAngle * 180 / Math.PI;
      let finalAngleInDegrees = -(getRadiansInDegrees - 135);

      if (finalAngleInDegrees >= 0 && finalAngleInDegrees <= 270) {
        // Snap the angle to the nearest tick step
        finalAngleInDegrees = Math.round(finalAngleInDegrees / stepAngle) * stepAngle;

        knob.style.transform = `rotate(${finalAngleInDegrees}deg)`;

        // Calculate the current speed based on the tick
        const tickIndex = Math.round(finalAngleInDegrees / stepAngle);
        const newSpeed = speedValues[tickIndex];

        setSpeed(newSpeed);
        createTicks(tickContainer, numTicks, tickIndex);

        if (newSpeed === 10 && minLabel) {
          minLabel.classList.add('neon');
        } else if (minLabel) {
          minLabel.classList.remove('neon');
        }

        if (newSpeed === 80 && maxLabel) {
          maxLabel.classList.add('neon');
        } else if (maxLabel) {
          maxLabel.classList.remove('neon');
        }
      }
    };

    const createTicks = (container: HTMLDivElement, numTicks: number, highlightNumTicks: number) => {
      if (!container) return;

      container.innerHTML = '';
      let startingTickAngle = -135;

      for (let i = 0; i < numTicks; i++) {
        const tick = document.createElement('div');
        tick.className = `tick ${i <= highlightNumTicks ? 'activetick' : ''}`;
        container.appendChild(tick);
        tick.style.transform = `rotate(${startingTickAngle}deg)`;

        startingTickAngle += stepAngle;
      }
    };

    if (knob) {
      knob.addEventListener('mousedown', handleMouseDown);
    }
    document.addEventListener('mouseup', handleMouseUp);

    if (tickContainer) {
      createTicks(tickContainer, numTicks, 0);
    }

    return () => {
      if (knob) {
        knob.removeEventListener('mousedown', handleMouseDown);
      }
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  return (
    <div className="knob">
      <div className="wheel-surround">
        <div ref={knobRef} className="wheel"></div>
        <span className="current-speed">{speed} km/h</span>
        <span ref={minRef} className="min">Min</span>
        <span ref={maxRef} className="max">Max</span>
        <div ref={tickContainerRef} className="ticks"></div>
      </div>
    </div>
  );
};

export default Knob;