import "./styles/Wheel.css"

export default function Wheel({ angle = 45 }) {
    return(
        <div className="wheel-container">
            <div className="wheel-size" style={{transform: `rotate(${-45 + angle}deg)`}}>
                <div className="direction-arrow">
                </div>
                <svg>
                    <rect className="wheel-rect" />
                </svg>
            </div>
            <div className="stats">
                <p>Angle: {angle-45}</p>
            </div>
        </div>
    )
}