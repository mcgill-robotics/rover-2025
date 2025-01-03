import { useState, useRef, useEffect } from 'react';

const ArmVisualizer = () => {

    const [jointAngles, setJointAngles] = useState({ waist: Math.PI, shoulder: 0.5*Math.PI, elbow: 0, wrist: Math.PI, hand: Math.PI });
    const lengths = [0.0575, 0.5, 0.4, 0, 0.034]; // define lengths between each joint, in m

    interface ArmSideViewProps {
        jointAngles: {
            waist: number,
            shoulder: number,
            elbow: number,
            wrist: number,
            hand: number
        };
        width: number;
        height: number;
    }

    const ArmSideView = ({ jointAngles, width, height }: ArmSideViewProps) => {
        const ref = useRef<HTMLCanvasElement>(null);
        const meter = width*0.6;

        function drawJoint(coords: number[], context: CanvasRenderingContext2D) {
            context.beginPath();
            context.arc(coords[0], coords[1], 5, 0, 2 * Math.PI);
            context.fillStyle = "blue";
            context.fill();
        }

        function drawLink(start: number[], end: number[], context: CanvasRenderingContext2D) {
            context.moveTo(start[0], start[1]);
            context.lineTo(end[0], end[1]);
            context.stroke();
        }


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

            // find coordinates of each joint
            const coords = [[meter*0.5, height/2], [meter*0.5, height/2-lengths[0]*meter], [], []]; 
            coords[2][0] = coords[1][0] + Math.sin(jointAngles.shoulder)*lengths[1]*meter;
            coords[2][1] = coords[1][0] - Math.cos(jointAngles.shoulder)*lengths[1]*meter;
            coords[3][0] = coords[2][0] + Math.cos(jointAngles.elbow)*lengths[2]*meter;
            coords[3][1] = coords[2][1] - Math.sin(jointAngles.elbow)*lengths[2]*meter;

            console.log(coords);


            // draw each joint
            for (let i=0; i<coords.length; i++) {
                drawJoint(coords[i], context);
            }

            // draw the links
            for (let i=1; i<coords.length; i++) {
                drawLink(coords[i-1], coords[i], context);
            }

        }, [jointAngles]);


        return (
            <div>
                <canvas ref={ref} width={width} height={height} />
            </div>
        );
    }

    return (
        <div>
            <ul>
                <li>Waist: {jointAngles.waist} rad</li>
                <li>Shoulder: {jointAngles.shoulder}  rad</li>
                <li>Elbow: {jointAngles.elbow} rad</li>
                <li>Wrist: {jointAngles.wrist} rad</li>
                <li>Hand: {jointAngles.hand} rad</li>
            </ul>
            <ArmSideView jointAngles={jointAngles} width={800} height={400}/>
        </div>
    );
}

export default ArmVisualizer;
