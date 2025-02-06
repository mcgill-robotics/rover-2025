import './styles/CoordinatesDisplay.css'

const CoordinatesDisplay: React.FC = () => {
    return (
        <div className="coordinate-display">
            <h2>Coordinate Display</h2>
            <div>Position: (10, 10, 10) cm</div>
            <div>Orientation: (0, 90, 0)°</div>
        </div>
    )
};

export default CoordinatesDisplay;