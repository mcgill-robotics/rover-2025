import './styles/ArmVisualizer.css';
import React, { useState, useEffect, useRef, useMemo } from 'react';
import ROSLIB from 'roslib';

const ARM_SEGMENT_LENGTHS = {
    L1: 100, // Shoulder to elbow
    L2: 80,  // Elbow to wrist
};

interface Float32MultiArrayMessage {
    data: number[];
}


const ArmVisualizer: React.FC = () => {
    const [jointAngles, setJointAngles] = useState({ shoulder: Math.PI / 2, elbow: Math.PI / 2, waist: 0 });
    const ros = useRef<ROSLIB.Ros | null>(null);
    const jointStateListener = useRef<ROSLIB.Topic | null>(null);

    const calculateJointPositions = useMemo(() => (angles: { shoulder: number; elbow: number; waist: number }) => {
        const { shoulder, elbow, waist } = angles;
        const { L1, L2 } = ARM_SEGMENT_LENGTHS;

        const shoulderX = 150; // Base X position on SVG
        const shoulderY = 250; // Base Y position on SVG (visualizing from top-down or side)

        // Elbow position
        const elbowX = shoulderX + L1 * Math.cos(shoulder);
        const elbowY = shoulderY - L1 * Math.sin(shoulder); // Y decreases upwards in typical SVG coords

        // Wrist position
        const waistX = elbowX + L2 * Math.cos(shoulder + elbow);
        const waistY = elbowY - L2 * Math.sin(shoulder + elbow);

        return {
            shoulder: { x: shoulderX, y: shoulderY },
            elbow: { x: elbowX, y: elbowY },
            waist: { x: waistX, y: waistY },
        };
    }, []);

    const jointPositions = calculateJointPositions(jointAngles);


    useEffect(() => {
        if (!ros.current) {
            ros.current = new ROSLIB.Ros({
                url: 'ws://localhost:9090' // rosbridge_server WebSocket URL
            });

            ros.current.on('connection', () => {
                console.log('Connected to rosbridge_server.');
            });

            ros.current.on('error', (error) => {
                console.error('Error connecting to rosbridge_server:', error);
            });

            ros.current.on('close', () => {
                console.log('Disconnected from rosbridge_server.');
            });
        }

        if (ros.current && !jointStateListener.current) {
            jointStateListener.current = new ROSLIB.Topic({
                ros: ros.current,
                name: '/armBrushlessFb',
                messageType: 'std_msgs/Float32MultiArray'
            });

            jointStateListener.current.subscribe((message: ROSLIB.Message) => {
                const floatArrayMessage = message as Float32MultiArrayMessage;
                // Check if data has 3 values and is an arrary plss
                if (floatArrayMessage.data && floatArrayMessage.data.length === 3) {
                    // const elbowDegrees = floatArrayMessage.data[0];
                    // const shoulderDegrees = floatArrayMessage.data[1];
                    // const waistDegrees = floatArrayMessage.data[2];

                    // const elbowRad = elbowDegrees * Math.PI / 180;
                    // const shoulderRad = shoulderDegrees * Math.PI / 180;
                    // const waistRad = waistDegrees * Math.PI / 180;

                    setJointAngles({
                        elbow: floatArrayMessage.data[0],
                        shoulder: floatArrayMessage.data[1],
                        waist: floatArrayMessage.data[2]
                    });
                } else {
                    console.warn("Received message does not have 'data' property", message);
                }
            });
        }

        // Cleanup
        return () => {
            if (jointStateListener.current) {
                jointStateListener.current.unsubscribe();
                jointStateListener.current = null;
            }
            if (ros.current) {
                ros.current.close();
                ros.current = null;
            }
        };
    }, []);


    return (
        <div className="visualizer-container">
            <h2>2D Arm Visualizer</h2>
            {/* The SVG element will hold our arm drawing */}
            <svg width="100%" height="100%" viewBox="0 0 300 300">
                {/* Base / Shoulder Joint */}
                <circle cx={jointPositions.shoulder.x} cy={jointPositions.shoulder.y} r="5" fill="red" />

                {/* Upper Arm (Shoulder to Elbow Link) */}
                <line
                    x1={jointPositions.shoulder.x} y1={jointPositions.shoulder.y}
                    x2={jointPositions.elbow.x} y2={jointPositions.elbow.y}
                    stroke="blue" strokeWidth="5"
                />
                {/* Elbow Joint */}
                <circle cx={jointPositions.elbow.x} cy={jointPositions.elbow.y} r="5" fill="green" />

                {/* Forearm (Elbow to Wrist Link) */}
                <line
                    x1={jointPositions.elbow.x} y1={jointPositions.elbow.y}
                    x2={jointPositions.waist.x} y2={jointPositions.waist.y}
                    stroke="purple" strokeWidth="5"
                />
                {/* Wrist Joint */}
                <circle cx={jointPositions.waist.x} cy={jointPositions.waist.y} r="5" fill="orange" />


                {/* Display current joint angles as text */}
                <text x="10" y="20" fontSize="11" fill="white">
                    Shoulder: {jointAngles.shoulder.toFixed(2)} rad
                </text>
                <text x="10" y="35" fontSize="11" fill="white">
                    Elbow: {jointAngles.elbow.toFixed(2)} rad
                </text>
                <text x="10" y="50" fontSize="11" fill="white">
                    Waist: {jointAngles.waist.toFixed(2)} rad
                </text>
            </svg>
            <p>Current Angles: Shoulder: {jointAngles.shoulder.toFixed(2)}, Elbow: {jointAngles.elbow.toFixed(2)}, Wrist: {jointAngles.waist.toFixed(2)}</p>
            <p>View (From Unity later) - This text is kept as per your original file structure for now.</p>
        </div>
    );
};

export default ArmVisualizer;