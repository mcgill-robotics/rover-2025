import { useEffect, useState } from 'react';
import './styles/PS4Controller.css';

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
      console.log(gp);
      setGamepadState({
        buttons: gp.buttons.map((button) => button.pressed),
        axes: gp.axes.map((axis) => axis),
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
      setGamepadState({ buttons: Array(17).fill(false), axes: [0, 0, 0, 0, 0, 0]});
    };

    window.addEventListener("gamepadconnected", handleConnect);
    window.addEventListener("gamepaddisconnected", handleDisconnect);

    return () => {
      window.removeEventListener("gamepadconnected", handleConnect);
      window.removeEventListener("gamepaddisconnected", handleDisconnect);
    };
  }, []);

  const isActive = (index: number) => (gamepadState.buttons[index] ? "active" : "");

  return (
    <div className="controller-container">
      <div className="controller">
        <div className={`meta ${isActive(16)} ${isConnected ? "connected" : "disconnected"}`} data-name="button-meta"></div>

        <div className="triggers">
          <div className="trigger-left">
            <span 
            className={`rtrigger ${isActive(6)}`} 
            data-name="button-left-shoulder-bottom">
            </span>
          </div>
          <div className="trigger-right">
            <span 
            className={`ltrigger ${isActive(7)}`} 
            data-name="button-right-shoulder-bottom">
            </span>
          </div>
        </div>

        <div className="bumpers">
          <div className="bumper-left">
            <span className={`lbumper ${isActive(4)}`} data-name="button-left-shoulder-top"></span>
          </div>
          <div className="bumper-right">
            <span className={`rbumper ${isActive(5)}`} data-name="button-right-shoulder-top"></span>
          </div>
        </div>

        <div className="buttons">
          <div className="button-share">
            <span className={`share ${isActive(8)}`} data-name="button-select"></span>
          </div>
          <div className="button-options">
            <span className={`options ${isActive(9)}`} data-name="button-start"></span>
          </div>
        </div>

        <div className={`touchpad`}></div>

        <div className="abxy">
          <div className="button-a">
            <span className={`a ${isActive(0)}`} data-name="button-1"></span>
          </div>
          <div className="button-b">
            <span className={`b ${isActive(1)}`} data-name="button-2"></span>
          </div>
          <div className="button-x">
            <span className={`x ${isActive(2)}`} data-name="button-3"></span>
          </div>
          <div className="button-y">
            <span className={`y ${isActive(3)}`} data-name="button-4"></span>
          </div>
        </div>

        <div className="sticks">
          <div className="stick-left">
            <span
              className={`lstick ${isActive(10)} ${gamepadState.axes[0] !== 0 || gamepadState.axes[1] !== 0 ? 'moving' : ''}`}
              data-name="stick-1"
              style={{
                transform: `translate(${gamepadState.axes[0] * 20}px, ${gamepadState.axes[1] * 20}px)`,
              }}>
              </span>
          </div>
          <div className="stick-right">
            <span
              className={`rstick ${isActive(11)} ${gamepadState.axes[2] !== 0 || gamepadState.axes[3] !== 0 ? 'moving' : ''}`}
              data-name="stick-2"
              style={{
                transform: `translate(${gamepadState.axes[2] * 20}px, ${gamepadState.axes[3] * 20}px)`,
              }}>
              </span>
          </div>
        </div>

        <div className="dpads">
          <div className="face-up">
            <span className={`fu ${isActive(12)}`} data-name="button-dpad-top"></span>
          </div>
          <div className="face-down">
            <span className={`fd ${isActive(13)}`} data-name="button-dpad-bottom"></span>
          </div>
          <div className="face-left">
            <span className={`fl ${isActive(14)}`} data-name="button-dpad-left"></span>
          </div>
          <div className="face-right">
            <span className={`fr ${isActive(15)}`} data-name="button-dpad-right"></span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default PS4Controller;