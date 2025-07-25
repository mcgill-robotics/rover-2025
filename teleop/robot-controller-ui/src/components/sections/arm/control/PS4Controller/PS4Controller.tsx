'use client';

import { useEffect, useState } from 'react';
import './PS4Controller.css'; 

interface GamepadState {
  buttons: boolean[];
  axes: number[];
}

const PS4Controller = () => {
  const [gamepadState, setGamepadState] = useState<GamepadState>({
    buttons: Array(17).fill(false),
    axes: [0, 0, 0, 0, 0, 0],
  });

  const [isConnected, setIsConnected] = useState(false);

  const pollGamepad = () => {
    const gamepads = navigator.getGamepads();
    if (gamepads[0]) {
      const gp = gamepads[0];
      setGamepadState({
        buttons: gp.buttons.map((b) => b.pressed),
        axes: gp.axes.map((a) => a),
      });
    }
    requestAnimationFrame(pollGamepad);
  };

  useEffect(() => {
    const handleConnect = () => {
      setIsConnected(true);
      pollGamepad();
    };
    const handleDisconnect = () => {
      setIsConnected(false);
      setGamepadState({ buttons: Array(17).fill(false), axes: [0, 0, 0, 0, 0, 0] });
    };

    window.addEventListener('gamepadconnected', handleConnect);
    window.addEventListener('gamepaddisconnected', handleDisconnect);
    return () => {
      window.removeEventListener('gamepadconnected', handleConnect);
      window.removeEventListener('gamepaddisconnected', handleDisconnect);
    };
  });

  const isActive = (i: number) => (gamepadState.buttons[i] ? 'active' : '');

  return (
    <div className="w-[357px] h-[267px] bg-controller-base bg-cover bg-center bg-no-repeat">
      <div className="relative w-full h-full m-[5px] pt-[5px]">
        <div className={`meta ${isActive(16)} ${isConnected ? 'connected' : 'disconnected'}`} />

        {/* Triggers */}
        <div className="absolute w-[74%] h-[14.65%] top-[-1.83%] left-[11.3%]">
          <div className="absolute w-[38px] h-[35px] top-[2px] left-[5px]">
            <span className={`rtrigger ${isActive(6)}`} />
          </div>
          <div className="absolute w-[38px] h-[35px] top-[2px] right-[3px]">
            <span className={`ltrigger ${isActive(7)}`} />
          </div>
        </div>

        {/* Bumpers */}
        <div className="absolute w-[74%] h-[3.5%] top-[13.8%] left-[11.3%]">
          <div className="absolute w-[45px] h-[10px] left-[2px]">
            <span className={`lbumper ${isActive(4)}`} />
          </div>
          <div className="absolute w-[45px] h-[10px] right-[0px]">
            <span className={`rbumper ${isActive(5)}`} />
          </div>
        </div>

        {/* Share & Options */}
        <div className="absolute w-[44.5%] h-[7.5%] top-[21.8%] left-[26.4%]">
          <div className="absolute w-[13px] h-[20px] left-0">
            <span className={`share ${isActive(8)}`} />
          </div>
          <div className="absolute w-[13px] h-[20px] right-0">
            <span className={`options ${isActive(9)}`} />
          </div>
        </div>

        {/* Touchpad */}
        <div className={`touchpad ${isActive(13)}`} />

        {/* ABXY */}
        <div className="absolute w-[21.5%] h-[27.6%] top-[24.75%] left-[69.25%]">
          <div className="absolute w-[22.5px] h-[22.5px] bottom-0 left-[26.5px]">
            <span className={`a ${isActive(0)}`} />
          </div>
          <div className="absolute w-[22.5px] h-[22.5px] top-[26.5px] right-[2px]">
            <span className={`b ${isActive(1)}`} />
          </div>
          <div className="absolute w-[22.5px] h-[22.5px] top-[26.5px] left-0">
            <span className={`x ${isActive(2)}`} />
          </div>
          <div className="absolute w-[22.5px] h-[22.5px] left-[26.5px] top-0">
            <span className={`y ${isActive(3)}`} />
          </div>
        </div>

        {/* Sticks */}
        <div className="absolute w-[45.5%] h-[17%] top-[48%] left-[26%]">
          <div className="absolute w-[50px] h-[50px] left-[-2px]">
            <span
              className={`lstick ${isActive(10)} ${
                gamepadState.axes[0] !== 0 || gamepadState.axes[1] !== 0 ? 'moving' : ''
              }`}
              style={{
                transform: `translate(${gamepadState.axes[0] * 20}px, ${gamepadState.axes[1] * 20}px)`,
              }}
            />
          </div>
          <div className="absolute w-[50px] h-[50px] right-[-2px]">
            <span
              className={`rstick ${isActive(11)} ${
                gamepadState.axes[2] !== 0 || gamepadState.axes[3] !== 0 ? 'moving' : ''
              }`}
              style={{
                transform: `translate(${gamepadState.axes[2] * 20}px, ${gamepadState.axes[3] * 20}px)`,
              }}
            />
          </div>
        </div>

        {/* Dpad */}
        <div className="absolute w-[16.2%] h-[21.1%] top-[27.75%] left-[9.5%]">
          <div className="absolute w-[45px] h-[45px] top-[-20px] left-[20px]">
            <span className={`fu ${isActive(12)}`} />
          </div>
          <div className="absolute w-[40px] h-[45px] bottom-0 left-[20px]">
            <span className={`fd ${isActive(13)}`} />
          </div>
          <div className="absolute w-[30px] h-[45px] top-[-1px] left-[-10px]">
            <span className={`fl ${isActive(14)}`} />
          </div>
          <div className="absolute w-[30px] h-[45px] top-[-1px] right-0">
            <span className={`fr ${isActive(15)}`} />
          </div>
        </div>
      </div>
    </div>
  );
};

export default PS4Controller;
