import { useState } from "react";
import Wheel from './Wheel.tsx'
import "../styles/WheelsContainer.css"

export default function WheelsContainer() {

    const [wheelAngles, setWheelAngles] = useState([
        { angle: 0 },
        { angle: 0 },
        { angle: 0 },
        { angle: 0 }
    ]);


    function generateNewValues() {
        var newWheels = [];
        
        for (let i=0; i<4; i++) {
            const angle = Math.floor(Math.random() * 91);
            newWheels.push({ angle });
        }

        return newWheels;
    }

    function handleClick() {
        setWheelAngles(generateNewValues)
    }

    return(
        <div className="wheels-container">
            <div className="wheel-grid">
                {wheelAngles.map((wheel, index) => (
                    <div key={index} className="cell">
                        <Wheel angle={wheel.angle} />
                    </div>
                ))}
            </div>
            <button onClick={handleClick}>Generate new Values</button>
        </div>
    )   
}