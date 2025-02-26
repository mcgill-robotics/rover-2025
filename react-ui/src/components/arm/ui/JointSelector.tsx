import React, { useState, useRef, useEffect, CSSProperties } from 'react';
import './styles/JointSelector.css';

type Joint = 'Waist' | 'Shoulder' | 'Elbow' | 'Wrist' | 'Hand' | 'Claw';

interface JointSelectorProps {
  onChange?: (joint: Joint) => void;
}

const JointSelector: React.FC<JointSelectorProps> = ({ onChange }) => {
  const joints: Joint[] = ['Waist', 'Shoulder', 'Elbow', 'Wrist', 'Hand', 'Claw'];
  const [selectedJoint, setSelectedJoint] = useState<Joint>('Elbow');
  const [isDragging, setIsDragging] = useState(false);
  const [startY, setStartY] = useState(0);
  const [scrollOffset, setScrollOffset] = useState(0);
  const [controlMode, setControlMode] = useState<'JOINT' | 'CTRL'>('JOINT');
  
  const scrollerRef = useRef<HTMLDivElement>(null);
  const itemHeight = 30; // Height of each item in pixels
  
  // Initialize scroll position to match the default selected joint
  useEffect(() => {
    const index = joints.indexOf(selectedJoint);
    setScrollOffset(index * itemHeight);
  }, []);
  
  // Calculate which joint should be selected based on scroll position
  useEffect(() => {
    const index = Math.floor(scrollOffset / itemHeight);
    // Ensure index is within bounds
    const boundedIndex = Math.max(0, Math.min(index, joints.length - 1));
    const newSelectedJoint = joints[boundedIndex];
    
    if (newSelectedJoint !== selectedJoint) {
      setSelectedJoint(newSelectedJoint);
      onChange?.(newSelectedJoint);
    }
  }, [scrollOffset, joints, selectedJoint, onChange]);

  // Handle mouse/touch events for scrolling
  const handleDragStart = (e: React.MouseEvent | React.TouchEvent) => {
    setIsDragging(true);
    const clientY = 'touches' in e ? e.touches[0].clientY : e.clientY;
    setStartY(clientY);
  };

  const handleDragMove = (e: React.MouseEvent | React.TouchEvent) => {
    if (!isDragging) return;
    
    const clientY = 'touches' in e ? e.touches[0].clientY : e.clientY;
    const deltaY = startY - clientY;
    setStartY(clientY);
    
    // Calculate new scroll offset with bounds
    const newOffset = scrollOffset + deltaY;
    const maxOffset = (joints.length - 1) * itemHeight;
    setScrollOffset(Math.max(0, Math.min(newOffset, maxOffset)));
    
    e.preventDefault();
  };

  const handleDragEnd = () => {
    setIsDragging(false);
    
    // Snap to nearest item
    const snappedOffset = Math.round(scrollOffset / itemHeight) * itemHeight;
    const maxOffset = (joints.length - 1) * itemHeight;
    setScrollOffset(Math.max(0, Math.min(snappedOffset, maxOffset)));
  };

  // Handle wheel events for scrolling
  const handleWheel = (e: React.WheelEvent) => {
    const newOffset = scrollOffset + e.deltaY;
    const maxOffset = (joints.length - 1) * itemHeight;
    setScrollOffset(Math.max(0, Math.min(newOffset, maxOffset)));
    e.preventDefault();
  };

  // Select a joint directly from dropdown
  const selectJoint = (joint: Joint) => {
    setSelectedJoint(joint);
    onChange?.(joint);
    
    // Update scroll position to match selected joint
    const index = joints.indexOf(joint);
    setScrollOffset(index * itemHeight);
  };

  // Calculate visible position for each item
  const getItemStyle = (index: number): CSSProperties => {
    const position = index * itemHeight;
    const translateY = position - scrollOffset;
    
    return {
      transform: `translateY(${translateY}px)`,
      opacity: 1,
      position: 'absolute',
      width: '100%',
      top: '50%',
      marginTop: -itemHeight / 2
    };
  };

  return (
    <div className="joint-selector-container">
      {/* Scroller */}
      {controlMode === 'JOINT' && (
        <div 
          className="joint-selector-scroller"
          onMouseDown={handleDragStart}
          onMouseMove={handleDragMove}
          onMouseUp={handleDragEnd}
          onMouseLeave={handleDragEnd}
          onWheel={handleWheel}
          ref={scrollerRef}
        >
          {/* Selection indicator */}
          <div className="joint-selector-indicator">
            <span className="joint-selector-indicator-arrow">▶</span>
          </div>
          
          {/* Scrollable items */}
          <div className="joint-selector-items-container">
            {joints.map((joint, index) => (
              <div 
                key={joint}
                className={`joint-selector-item ${
                  joint === selectedJoint ? 'selected' : 'dimmed'
                }`}
                style={getItemStyle(index)}
                onClick={() => {
                  selectJoint(joint)
                }}
              >
                {joint}
              </div>
            ))}
          </div>
        </div>
      )}
      
      {/* Toggle button */}
      <div className="joint-selector-toggle">
        <div
          className={`joint-selector-toggle-container ${controlMode === 'CTRL' ? 'ctrl-mode' : 'joint-mode'}`}
        >
          <div
            className={`joint-selector-toggle-button ${controlMode === 'JOINT' ? 'active' : ''}`}
            onClick={() => setControlMode('JOINT')}
          >
            JOINT
          </div>

          <div
            className={`joint-selector-toggle-button ${controlMode === 'CTRL' ? 'active' : ''}`}
            onClick={() => setControlMode('CTRL')}
          >
            CTRL
          </div>
        </div>
      </div>
    </div>
  );
};

export default JointSelector;