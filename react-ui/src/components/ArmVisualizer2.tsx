import { useState, useRef, useEffect } from 'react';

const ArmVisualizer = () => {

    const [jointAngles, setJointAngles] = useState({ waist: Math.PI, shoulder: 0.25*Math.PI, elbow: Math.PI, wrist: Math.PI, hand: Math.PI });
    const lengths = [0.0575, 0.5, 0.4, 0, 0.034]; // define lengths between each joint, in m

    interface ArmSideViewProps {
        jointAngles: object;
        width: number;
        height: number;
    }

    const ArmSideView = ({ jointAngles, width, height }: ArmSideViewProps) => {
        const ref = useRef<HTMLCanvasElement>(null);
        const meter = width*0.75;

        useEffect(() => {
            const canvas = ref.current;
            if (!canvas) return;
            const context = canvas.getContext("2d");
            if (!context) return;
            context.fillStyle = "red";
            context.fillRect(0, 0, width, height);

            // draw one meter scale as reference
            context.moveTo(width*0.125, height*0.9);
            context.lineTo(width*0.125 + meter, height*0.9);
            context.stroke();

            // draw the shoulder joint
            context.beginPath();
            context.arc(0, height/2, 20, 0, 2 * Math.PI);
            context.fillStyle = "blue";
            context.fill();
        }, []);
        

        return (
            <div>
                <canvas ref={ref} width={width} height={height} />
            </div>
        );
    };

    return (
        <div>
            <ArmSideView jointAngles={jointAngles} width={400} height={400}/>
        </div>
    );
}

export default ArmVisualizer;
