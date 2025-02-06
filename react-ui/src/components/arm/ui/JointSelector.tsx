import { useState } from 'react'
import './styles/JointSelector.css'

const JointSelector: React.FC = () => {
    const [selectedJoint, setSelectedJoint] = useState("Elbow");

    return (
        <div className="joint">
            <h2>Joint selector</h2>
            <select value={selectedJoint} onChange={(e) => {
                setSelectedJoint(e.target.value);
                }}>
                <option>Waist</option>
                <option>Shoulder</option>
                <option>Elbow</option>
                <option>Wrist</option>
                <option>Hand</option>
                <option>Claw</option>
            </select>
        </div>
    )
};

export default JointSelector;