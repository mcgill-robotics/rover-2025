// Pages are the views that can contain components. They are displayed based on the path.
import {ArmVisuals}from '../components/Armvis1'
import ArmVisualizer from '../components/ArmVisualizer2';

function Sandbox() {
  return (
    <>
      <h2>Sandbox Page</h2>
      <ArmVisuals></ArmVisuals>

      <ArmVisualizer />
    </>

);
}

export default Sandbox;