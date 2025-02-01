import { useEffect, useState } from 'react';
import { useGamepads } from 'react-gamepads';
import ros from "../../services/ros";
import * as ROSLIB from 'roslib';

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
  const [isPublishing, setIsPublishing] = useState(false);

  
       

  useGamepads(_gamepads => {
    setGamepads(Object.values(_gamepads));
  });

  

  useEffect(() => {

      //let publisherInterval: number | null = null;
  // create the topic
    var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name :'/cmd_vel',
    messageType: 'geometry_msgs/Twist'
    });

    // start sending messages when the controller is connected every 100ms
    if (controllerConnected && isPublishing) {
      publishMessage(cmdVel);
    }
  }, [isPublishing, controllerConnected, gamepads]);

  // publush the message
    function publishMessage(cmdVel: ROSLIB.Topic<ROSLIB.Message>) {
const gamepadData: Record<string, { buttons: Record<string, number>; axes: Record<string, string> }> = 
  gamepads.reduce((acc, gp) => {
    acc[gp.id] = {
      buttons: gp.buttons.reduce((buttonAcc, button, index) => {
        buttonAcc[buttonLabels[index]] = button.value;
        return buttonAcc;
      }, {} as Record<string, number>),
      axes: gp.axes.reduce((axesAcc, stick, index) => {
        axesAcc[axesLabels[index]] = stick.toFixed(2);
        return axesAcc;
      }, {} as Record<string, string>),
    };
    return acc;
  }, {} as Record<string, { buttons: Record<string, number>; axes: Record<string, string> }>);
  console.log('num of pads ' + gamepads.length);
console.log(gamepadData);

        var twist = new ROSLIB.Message({
        linear : {
            x : 0.1,
            y : 0.2,
            z : 0.3
        },
        angular : {
            x : -0.1,
            y : -0.2,
            z : -0.3
        }
        });
        cmdVel.publish(twist);
        console.log("hi")
    }

    function handleIsPublishingChange() {
      setIsPublishing(!isPublishing); 
    }



  if (!controllerConnected || gamepads.length === 0) {
    return (<div>Disconnected
    </div>);
  }
  return (
    <div
      style={{ background: `rgb(${128 + (gamepads[0].axes[0] * 128)},128,128)` }}>
        <label >
          <input type="checkbox" checked={isPublishing} onChange={handleIsPublishingChange} />
          Publish Messages
        </label>
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


