import './styles/CoordinatesDisplay.css'

const CoordinatesDisplay: React.FC = () => {
    return (
        <div className="coordinate-display-container">
            <div className="position-display">
                <h2>Position</h2>
                <div className="position-item">
                   <p>Depth (x): 10.00 cm</p>
                    <p>Width (y): 10.00 cm</p>
                    <p>Height (z): 10.00 cm</p> 
                </div>
            </div>
            <div className="orientation-display">
                <h2>Orientation</h2>
                <div className="orientation-item">
                    <p>Row (x): 0.00°</p>
                    <p>Pitch (y): 90.00°</p>
                    <p>Yaw (z): 0.00°</p>  
                </div>
            </div>
        </div>
        

    )
};

export default CoordinatesDisplay;