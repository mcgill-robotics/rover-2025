import ArmVisualizer from './ui/ArmVisualizer'
import './styles/ArmView.css'

const ArmView: React.FC = () => {
    return (
        <div className="arm-view-container">
            <ArmVisualizer />
        </div>
    )
}

export default ArmView;