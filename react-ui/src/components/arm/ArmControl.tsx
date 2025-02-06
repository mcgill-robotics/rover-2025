import PowerButton from './ui/PowerButton'
import CommandLog from './ui/CommandLog'
import ControllerDisplay from './ui/ControllerDisplay'
import './styles/ArmControl.css'

const ArmControl: React.FC = () => {

    return (
        <div className="arm-control-container">
            <h1 className="arm-control-title">Control</h1>
            <div className="power-container">
                <PowerButton />
            </div>
            <div className="log-container">
                <CommandLog commands={["hello", "World"]}/>
            </div>
            <div className="controller-container">
               <ControllerDisplay controllerConnected={true}/> 
            </div>
        </div>
    )
};

export default ArmControl;