'use client';

import React, { useEffect, useState } from 'react';
import DPad from '@/components/ui/DPad';

interface DPadControllerProps {
  inputStream: string;
}

const DPadController: React.FC<DPadControllerProps> = ({ inputStream }) => {
  const [activeDirection, setActiveDirection] = useState<
    'up' | 'down' | 'left' | 'right' | null
  >(null);

  useEffect(() => {
    if (['up', 'down', 'left', 'right'].includes(inputStream)) {
      setActiveDirection(inputStream as 'up' | 'down' | 'left' | 'right');
    } else {
      setActiveDirection(null);
    }
  }, [inputStream]);

  return <DPad activeDirection={activeDirection} />;
};

export default DPadController;
