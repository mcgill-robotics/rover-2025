import { useEffect, useState } from "react";
import { ControllerDisplay } from "./ControllerDisplay";

export function Controller() {
    const [isConnected, setIsConnected] = useState(false);
    const [isEnabled, setIsEnabled] = useState(false);


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
        }

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
        {isEnabled && <ControllerDisplay controllerConnected={isConnected}/>}            
        </>
    )




}