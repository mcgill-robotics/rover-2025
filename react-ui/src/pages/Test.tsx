import React, { useEffect } from 'react';

// Declare ROS3D globally
declare global {
  interface Window {
    ROS3D: any;
  }
}

const Test: React.FC = () => {
  useEffect(() => {
    // Check if ROS3D is loaded
    if (window.ROS3D) {
      console.log('ROS3D loaded successfully!');

      // Create a Three.js viewer
      const viewer = new window.ROS3D.Viewer({
        divID: 'viewer',
        width: window.innerWidth,
        height: window.innerHeight,
        antialias: true,
      });

      // Create a URDF client to load the URDF model
      const urdfClient = new window.ROS3D.UrdfClient({
        ros: {}, // Empty object, not null, to avoid idCounter error
        tfClient: null, // No transformations needed for now
        path: '/arm.urdf', // Path to the URDF in public/ directory
        rootObject: viewer.scene,
        loader: window.ROS3D.COLLADA_LOADER, // Optional: Specify loader if needed
      });

      console.log('URDF model loaded successfully!');
    } else {
      console.error('ROS3D is not loaded.');
    }
  }, []);

  return <div id="viewer" style={{ width: '100%', height: '100vh' }}></div>;
};

export default Test;