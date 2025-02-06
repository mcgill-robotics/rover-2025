import ArmVisualizer from './ui/ArmVisualizer'
import './styles/ArmView.css'

const ArmView: React.FC = () => {
    return (
        <div className="arm-view-container">
            <h1 className="arm-view-title">View</h1>
            <div className="visualizer-container">
                <ArmVisualizer />
            </div>
        </div>
    )
}

export default ArmView;