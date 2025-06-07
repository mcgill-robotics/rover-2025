import CoordinatesDisplay from './ui/CoordinatesDisplay'
import JointSelector from './ui/JointSelector'
import SpeedControl from './ui/SpeedControl'
import GraphPanel from './ui/GraphPanel'
import './styles/ArmInfo.css'

const ArmInfo: React.FC = () => {

    return (
        <div className="arm-info-container">
            <CoordinatesDisplay /> 

            <div className="joint-speed-container">
                <div className="joint-container">
                    <h1>Joint Selector</h1>
                    <JointSelector/>
                </div>

                <div className="speed-container">
                    <h1>Speed Control</h1>
                    <SpeedControl />
                </div>
            </div>

            <GraphPanel />
        </div>
    )
};

export default ArmInfo;