import Knob from './Knob';
import Headlight from './Headlight';
import './styles/RoverControl.css';

const RoverControl: React.FC = () =>  {
    return (
        <div className="drive-info-container">
            <div className="knob-light-container">
               <div className="knob-container">
                    <h2 className="knob-title">Maximum speed</h2>
                    <Knob />
                </div>

                <div className="headlight-container">
                    <h2 className="headlight-title">Brightness</h2>
                    < Headlight />
                </div>
            </div>
        </div>
    )

};

export default RoverControl;

