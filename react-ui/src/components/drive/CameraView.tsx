import "./styles/CameraView.css";

const CameraView: React.FC = () => {
  const cameras = [
    { id: 1, name: "Camera 1" },
    { id: 2, name: "Camera 2" },
    { id: 3, name: "Camera 3" },
    { id: 4, name: "Pan Tilt Camera" },
  ];

  return (
    <div className="camera-view">
      <div className="camera-grid">
        {cameras.map((camera) => (
          <div key={camera.id} className="camera-card">
            <div className="camera-placeholder">
              <h2 className="camera-title">{camera.name}</h2>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default CameraView;