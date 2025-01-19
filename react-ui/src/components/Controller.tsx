import { useEffect, useState } from "react";
import { ControllerDisplay } from "./ControllerDisplay";
import ros from "../services/ros";
import * as ROSLIB from 'roslib';
import { clearInterval } from "timers";

export function Controller() {
    const [isConnected, setIsConnected] = useState(false);
    const [isEnabled, setIsEnabled] = useState(false);
    // create the topic
    var cmdVel = new ROSLIB.Topic({
        ros : ros,
        name :'/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    // publish the message
    function publishMessage() {
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
    }

    useEffect(() => {



        const handleGamePadConnected = (event: GamepadEvent) => {
            console.log("A gamepad connected:");
            console.log(event.gamepad);
            setIsConnected(true);
        };

        const handleGamePadDisconnected = (event : GamepadEvent) => {
            console.log("A gamepad disconnected:");
            console.log(event.gamepad);
            setIsConnected(false);
        };

        window.addEventListener("gamepadconnected", handleGamePadConnected);
        window.addEventListener("gamepaddisconnected", handleGamePadDisconnected);

        
        return () => {
            window.removeEventListener("gamepadconnected", handleGamePadConnected);
            window.removeEventListener("gamepaddisconnected", handleGamePadDisconnected);
        };

    }, []);

    function handleCheckboxChange(){
        setIsEnabled(!isEnabled);
    }

    return (
        <>
            <label>
            <input
                type="checkbox"
                checked={isEnabled}
                onChange={handleCheckboxChange}
            />
            Enable Controller
        </label>
        <br />
        <button onClick={publishMessage}>Publish a message</button>
        {isEnabled && <ControllerDisplay controllerConnected={isConnected}/>}            
        </>
    )




}