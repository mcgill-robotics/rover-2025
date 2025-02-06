import { useState } from 'react'
import './styles/SpeedControl.css'

const SpeedControl: React.FC = () => {
    const [speed, setSpeed] = useState(1.5);
    
    return (
        <div className="speed-control">
            <h2>Speed Control</h2>
            <input type="range" min="0.5" max="2.0" step="0.1" value={speed} onChange={(e) => setSpeed(parseFloat(e.target.value))} />
        </div>
    )
};

export default SpeedControl;