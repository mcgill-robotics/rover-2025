'use client';

// import React, { useState, useRef, useEffect, CSSProperties } from 'react';

type Joint = 'Waist' | 'Shoulder' | 'Elbow' | 'Wrist' | 'Hand' | 'Claw';

interface JointSelectorProps {
  onChange?: (joint: Joint) => void;
}

const JointSelector: React.FC<JointSelectorProps> = ({ onChange }) => {
  // const joints = React.useMemo<Joint[]>(() => ['Waist', 'Shoulder', 'Elbow', 'Wrist', 'Hand', 'Claw'], []);
  // const [selectedJoint, setSelectedJoint] = useState<Joint>('Elbow');
  // const [isDragging, setIsDragging] = useState(false);
  // const [startY, setStartY] = useState(0);
  // const [scrollOffset, setScrollOffset] = useState(0);
  // const [controlMode, setControlMode] = useState<'JOINT' | 'CTRL'>('JOINT');

  // const scrollerRef = useRef<HTMLDivElement>(null);
  // const itemHeight = 30;

  // useEffect(() => {
  //   const index = joints.indexOf(selectedJoint);
  //   setScrollOffset(index * itemHeight);
  // }, [selectedJoint, joints]);

  // useEffect(() => {
  //   const index = Math.floor(scrollOffset / itemHeight);
  //   const boundedIndex = Math.max(0, Math.min(index, joints.length - 1));
  //   const newSelectedJoint = joints[boundedIndex];

  //   if (newSelectedJoint !== selectedJoint) {
  //     setSelectedJoint(newSelectedJoint);
  //     onChange?.(newSelectedJoint);
  //   }
  // }, [scrollOffset, selectedJoint, joints, onChange]);

  // const handleDragStart = (e: React.MouseEvent | React.TouchEvent) => {
  //   setIsDragging(true);
  //   const clientY = 'touches' in e ? e.touches[0].clientY : e.clientY;
  //   setStartY(clientY);
  // };

  // const handleDragMove = (e: React.MouseEvent | React.TouchEvent) => {
  //   if (!isDragging) return;
  //   const clientY = 'touches' in e ? e.touches[0].clientY : e.clientY;
  //   const deltaY = startY - clientY;
  //   setStartY(clientY);
  //   const newOffset = scrollOffset + deltaY;
  //   const maxOffset = (joints.length - 1) * itemHeight;
  //   setScrollOffset(Math.max(0, Math.min(newOffset, maxOffset)));
  //   e.preventDefault();
  // };

  // const handleDragEnd = () => {
  //   setIsDragging(false);
  //   const snappedOffset = Math.round(scrollOffset / itemHeight) * itemHeight;
  //   const maxOffset = (joints.length - 1) * itemHeight;
  //   setScrollOffset(Math.max(0, Math.min(snappedOffset, maxOffset)));
  // };

  // const handleWheel = (e: React.WheelEvent) => {
  //   const newOffset = scrollOffset + e.deltaY;
  //   const maxOffset = (joints.length - 1) * itemHeight;
  //   setScrollOffset(Math.max(0, Math.min(newOffset, maxOffset)));
  //   e.preventDefault();
  // };

  // const selectJoint = (joint: Joint) => {
  //   setSelectedJoint(joint);
  //   onChange?.(joint);
  //   const index = joints.indexOf(joint);
  //   setScrollOffset(index * itemHeight);
  // };

  // const getItemStyle = (index: number): CSSProperties => {
  //   const position = index * itemHeight;
  //   const translateY = position - scrollOffset;

  //   return {
  //     transform: `translateY(${translateY}px)`,
  //     opacity: 1,
  //     position: 'absolute',
  //     width: '100%',
  //     top: '50%',
  //     marginTop: -itemHeight / 2,
  //   };
  // };

  return (
    <div>Place holder</div>
    // <div className="w-full h-full flex flex-col items-center mx-auto text-white overflow-visible">
    //   {controlMode === 'JOINT' && (
    //     <div
    //       className="w-4/5 h-[40%] relative bg-[#24252A] rounded-xl my-4 overflow-hidden"
    //       onMouseDown={handleDragStart}
    //       onMouseMove={handleDragMove}
    //       onMouseUp={handleDragEnd}
    //       onMouseLeave={handleDragEnd}
    //       onWheel={handleWheel}
    //       ref={scrollerRef}
    //     >
    //       <div className="absolute left-0 right-0 top-1/2 -translate-y-1/2 h-[30px] pointer-events-none z-10">
    //         <span className="absolute left-2 top-1/2 -translate-y-1/2">▶</span>
    //       </div>
    //       <div className="relative w-full h-full">
    //         {joints.map((joint, index) => (
    //           <div
    //             key={joint}
    //             className={`h-[30px] text-center text-base flex items-center justify-center cursor-pointer transition-all ${
    //               joint === selectedJoint
    //                 ? 'font-bold text-white'
    //                 : 'text-gray-500'
    //             }`}
    //             style={getItemStyle(index)}
    //             onClick={() => selectJoint(joint)}
    //           >
    //             {joint}
    //           </div>
    //         ))}
    //       </div>
    //     </div>
    //   )}

    //   <div className="w-full flex justify-center items-center mb-4">
    //     <div
    //       className={`flex justify-between items-center bg-[#26272e] rounded-full p-2 relative shadow transition-shadow ${
    //         controlMode === 'CTRL' ? 'ctrl-mode' : 'joint-mode'
    //       }`}
    //     >
    //       <div
    //         className={`absolute h-[80%] bg-gradient-to-t from-[#1d1d1d] to-[#131313] rounded-full transition-all w-[calc(50%-1rem)] top-1/2 -translate-y-1/2 ${
    //           controlMode === 'CTRL'
    //             ? 'left-[calc(50%+0.5rem-5px)]'
    //             : 'left-2'
    //         }`}
    //       />
    //       <div
    //         className={`flex-1 text-center py-2 px-4 rounded-full font-bold cursor-pointer relative z-10 transition-all ${
    //           controlMode === 'JOINT'
    //             ? 'text-[#d63f3f] bg-gradient-to-t from-[#1d1d1d] to-[#131313] shadow-md'
    //             : 'text-gray-500'
    //         }`}
    //         onClick={() => setControlMode('JOINT')}
    //       >
    //         JOINT
    //       </div>
    //       <div
    //         className={`flex-1 text-center py-2 px-4 rounded-full font-bold cursor-pointer relative z-10 transition-all ${
    //           controlMode === 'CTRL'
    //             ? 'text-[#d63f3f] bg-gradient-to-t from-[#1d1d1d] to-[#131313] shadow-md'
    //             : 'text-gray-500'
    //         }`}
    //         onClick={() => setControlMode('CTRL')}
    //       >
    //         CTRL
    //       </div>
    //     </div>
    //   </div>
    // </div>
  );
};

export default JointSelector;
