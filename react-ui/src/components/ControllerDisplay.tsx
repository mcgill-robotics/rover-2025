import { useState } from 'react';
import { useGamepads } from 'react-gamepads';

const buttonLabels = [
  "X",
  "O",
  "[]",
  "/\\",
  "L1",
  "R1",
  "L2",
  "R2",
  "Share",
  "Options",
  "L3",
  "R3",
  "UP",
  "DOWN",
  "LEFT",
  "RIGHT",
  "PS",
  "Touchpad"
]

const axesLabels = [
  "LX",
  "LY",
  "RX",
  "RY",
]

export function ControllerDisplay({controllerConnected} : {controllerConnected : boolean}) {
  const [gamepads, setGamepads] = useState<Gamepad[]>([]);
  useGamepads(_gamepads => {
    setGamepads(Object.values(_gamepads));
    console.log(gamepads);
  });

  if (!controllerConnected || gamepads.length === 0) {
    return (<div>Disconnected</div>);
  }
  return (
    <div
      style={{ background: `rgb(${128 + (gamepads[0].axes[0] * 128)},128,128)` }}>
        <p>Connected</p>
      {gamepads.length && gamepads.map(gp => {
        return (
          <div>
            <div><span>ID:</span>{gp.id}</div>
            {gp.buttons.map((button, index) => {
              return (
                <div><span>{buttonLabels[index]}:</span><span>{button.value}</span></div>
              )
            })}
            {gp.axes.map((stick, index) => {
              return (
                <div><span>{axesLabels[index]}:</span><span>{stick.toFixed(2)}</span></div>
              )
            })}
          </div>
        )
      })}
    </div>
  );
}


