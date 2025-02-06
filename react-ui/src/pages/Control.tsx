// Control.tsx
// Pages for arm controls 
import ArmInfo from '../components/arm/ArmInfo'
import ArmView from '../components/arm/ArmView'
import ArmControl from '../components/arm/ArmControl'
import './styles/Control.css'


function Control() {
  return (
    <div className="control">
      <h1>TODO: Currently working on the drive control</h1>
      <div className="control-content">
        <ArmInfo />
        <ArmView />
        <ArmControl />
      </div>
    </div>
  );
};

export default Control;