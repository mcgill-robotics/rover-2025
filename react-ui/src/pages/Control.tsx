// Control.tsx
// Pages for arm controls 
import ArmInfo from '../components/arm/ArmInfo'
import ArmView from '../components/arm/ArmView'
import ArmControl from '../components/arm/ArmControl'
import './styles/Control.css'


function Control() {
  return (
    <div className="control">
      <div className="control-content">
        <div className="arm-info">
          <ArmInfo />
        </div>
        <div className="arm-view">
          <ArmView />
        </div>
        <div className="arm-control">
          <ArmControl />
        </div>
      </div>
    </div>
  );
};

export default Control;