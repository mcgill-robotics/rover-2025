import React, { useState } from "react";
import { useGamepads } from "react-gamepads";
import "./styles/XboxController.css";

const XboxController: React.FC = () => {
  const [gamepads, setGamepads] = useState<any>({});
  useGamepads((gamepads) => setGamepads(gamepads));

  const xboxButtons = [
    "A",
    "B",
    "X",
    "Y",
    "LB",
    "RB",
    "LT",
    "RT",
    "Back",
    "Start",
    "LS",
    "RS",
    "D-Up",
    "D-Down",
    "D-Left",
    "D-Right",
    "Xbox",
  ];

  return (
    <div className="xbox-container">
      {Object.keys(gamepads).map((index) => {
        const gamepad = gamepads[index];
        return (
          <div key={index} className="xbox-controller">
            <h2>{gamepad.id}</h2>

            {/* Face Buttons */}
            <div className="face-buttons">
              {["Y", "X", "B", "A"].map((btn) => (
                <div
                  key={btn}
                  className={`button ${gamepad.buttons[xboxButtons.indexOf(btn)].pressed ? "active" : ""} ${btn}`}
                >
                  {btn}
                </div>
              ))}
            </div>

            {/* Triggers & Bumpers */}
            <div className="bumpers">
              {["LB", "RB"].map((btn) => (
                <div
                  key={btn}
                  className={`button ${gamepad.buttons[xboxButtons.indexOf(btn)].pressed ? "active" : ""}`}
                >
                  {btn}
                </div>
              ))}
            </div>

            {/* Analog Stick */}
            <div className="sticks">
              <div
                className="left-stick"
                style={{
                  transform: `translate(${gamepad.axes[0] * 30}px, ${gamepad.axes[1] * 30}px)`,
                }}
              />
              <div
                className="right-stick"
                style={{
                  transform: `translate(${gamepad.axes[2] * 30}px, ${gamepad.axes[3] * 30}px)`,
                }}
              />
            </div>

            {/* D-Pad */}
            <div className="dpad">
              {["D-Up", "D-Down", "D-Left", "D-Right"].map((btn) => (
                <div
                  key={btn}
                  className={`button ${gamepad.buttons[xboxButtons.indexOf(btn)].pressed ? "active" : ""}`}
                >
                  {btn.replace("D-", "")}
                </div>
              ))}
            </div>
          </div>
        );
      })}
    </div>
  );
};

export default XboxController;