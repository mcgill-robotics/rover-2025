import React, { useState } from 'react';
import { useGamepads } from 'react-gamepads';
import './styles/PS4Controller.css';

const PS4Controller: React.FC = () => {
  const [controllers, setController] = useState<any>({});
  useGamepads((controllers) => setController(controllers));

  const buttonLabels = [
    'A', 'B', 'X', 'Y', 'LB', 'RB', 'LT', 'RT',
    'Back', 'Start', 'LS', 'RS', 'DPad-Up', 'DPad-Down',
    'DPad-Left', 'DPad-Right', 'Home'
  ];

  return (
    <div className="controller-container">
      {Object.keys(controllers).map((index) => {
        const gamepad = controllers[index];
        return (
          <div key={index} className="controller">
            <h2>{gamepad.id}</h2>
            <div className="controller-buttons">
              {gamepad.buttons.map((button: any, i: number) => (
                <div
                  key={i}
                  className={`controller-button ${button.pressed ? 'pressed' : ''}`}
                >
                  {buttonLabels[i]}
                </div>
              ))}
            </div>
          </div>
        );
      })}
    </div>
  );
};

export default PS4Controller;