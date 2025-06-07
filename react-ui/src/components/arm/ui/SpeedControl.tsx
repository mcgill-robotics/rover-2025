import { useState } from 'react';
import './styles/SpeedControl.css';

const SpeedControl: React.FC = () => {
    const speeds = [0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0];
    const [speed, setSpeed] = useState(1.5);

    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const newValue = parseFloat(e.target.value);
        
        // Snap to nearest speed mark
        const closestSpeed = speeds.reduce((prev, curr) =>
            Math.abs(curr - newValue) < Math.abs(prev - newValue) ? curr : prev
        );
        setSpeed(closestSpeed);
    };

    return (
        <div className="speed-control">
            {/* Vertical Slider */}
            <input
                className="speed-slider"
                type="range"
                min={Math.min(...speeds)}
                max={Math.max(...speeds)}
                step="0.01"
                value={speed}
                onChange={handleChange}
                style={{writingMode: "vertical-lr", direction: "rtl"}}
            />

            {/* Current Speed Display */}
            <div className="speed-display">Speed: {speed}x</div>
        </div>
    );
};

export default SpeedControl;