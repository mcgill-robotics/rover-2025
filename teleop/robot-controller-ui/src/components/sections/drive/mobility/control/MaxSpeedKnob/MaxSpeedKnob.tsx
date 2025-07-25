'use client';

import React, { useRef, useState, useEffect } from 'react';
import './MaxSpeedKnob.css';

// CodePen Example Credit:
// Original design and implementation by Kevin Lam
// URL: https://codepen.io/kevin-lam-the-scripter/pen/QWNeNvo

const speedValues = [10, 30, 50, 70, 80];
const stepAngle = 270 / (speedValues.length - 1);

export default function Knob() {
  const [speedIndex, setSpeedIndex] = useState(0);
  const knobRef = useRef<HTMLDivElement>(null);
  const tickContainerRef = useRef<HTMLDivElement>(null);
  const minRef = useRef<HTMLSpanElement>(null);
  const maxRef = useRef<HTMLSpanElement>(null);

  useEffect(() => {
    updateVisuals(speedIndex);
  });

  const updateVisuals = (index: number) => {
    const newAngle = index * stepAngle;
    if (knobRef.current) knobRef.current.style.transform = `rotate(${newAngle}deg)`;
    if (tickContainerRef.current) createTicks(tickContainerRef.current, speedValues.length, index);
    if (minRef.current) minRef.current.classList.toggle('neon', index === 0);
    if (maxRef.current) maxRef.current.classList.toggle('neon', index === speedValues.length - 1);
  };

  const handleKnobClick = () => {
    setSpeedIndex((prev) => (prev + 1) % speedValues.length);
  };

  const createTicks = (container: HTMLDivElement, total: number, active: number) => {
    container.innerHTML = '';
    let angle = -135;

    for (let i = 0; i < total; i++) {
      const tick = document.createElement('div');
      tick.className = `absolute w-full h-full top-0 left-0 z-5 tick ${i <= active ? 'activetick' : ''}`;
      tick.style.transform = `rotate(${angle}deg)`;
      container.appendChild(tick);
      angle += stepAngle;
    }
  };

  return (
    <div className="w-full h-full text-center select-none">
      <div className="relative w-[7.5rem] h-[7.5rem] mx-auto rounded-full border-[0.25em] border-[#0e0e0e] bg-[#181818] shadow-[inset_0_0.2em_0.1em_0.05em_rgba(255,255,255,0.1),inset_0_-0.2em_0.1em_0.05em_rgba(0,0,0,0.5),0_0.5em_0.65em_0_rgba(0,0,0,0.3)]">
        <div
          ref={knobRef}
          onClick={handleKnobClick}
          className="absolute w-full h-full rounded-full z-10 cursor-pointer wheel"
        />
        <span className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 text-[#d63f3f] font-bold text-sm">
          {speedValues[speedIndex]} km/h
        </span>
        <span
          ref={minRef}
          className="absolute bottom-[1em] left-[-2.5em] text-[70%] uppercase text-white/40 opacity-50"
        >
          Min
        </span>
        <span
          ref={maxRef}
          className="absolute bottom-[1em] right-[-2.5em] text-[70%] uppercase text-white/40 opacity-50"
        >
          Max
        </span>
        <div ref={tickContainerRef} className="ticks" />
      </div>
    </div>
  );
}
