import CoordinatesDisplay from './ui/CoordinatesDisplay'
import JointSelector from './ui/JointSelector'
import SpeedControl from './ui/SpeedControl'
import GraphPanel from './ui/GraphPanel'
import './styles/ArmInfo.css'

const ArmInfo: React.FC = () => {

    return (
        <div className="arm-info-container">
            <h1 className="arm-info-title">Information</h1>

            <div className="coordinate-container">
                <CoordinatesDisplay /> 
            </div>

            <div className="joint-container">
                <JointSelector/>
            </div>

            <div className="speed-container">
                <SpeedControl />
            </div>

            <div className="graph-container">
                <GraphPanel />
            </div>
        </div>
    )
};

export default ArmInfo;