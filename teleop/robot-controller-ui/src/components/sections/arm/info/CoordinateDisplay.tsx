'use client';

const CoordinateDisplay: React.FC = () => {
  return (
    <div className="flex flex-row flex-1 gap-[7.5%] mr-[2.5%]">
      <div className="flex flex-col">
        <h2 className="text-xl font-bold my-1 ml-0">Position</h2>
        <div className="ml-4">
          <p>Depth (x): 10.00 cm</p>
          <p>Width (y): 10.00 cm</p>
          <p>Height (z): 10.00 cm</p>
        </div>
      </div>

      <div className="flex flex-col">
        <h2 className="text-xl font-bold my-1 ml-0">Orientation</h2>
        <div className="ml-4">
          <p>Row (x): 90.00°</p>
          <p>Pitch (y): 90.00°</p>
          <p>Yaw (z): 90.00°</p>
        </div>
      </div>
    </div>
  );
};

export default CoordinateDisplay;
