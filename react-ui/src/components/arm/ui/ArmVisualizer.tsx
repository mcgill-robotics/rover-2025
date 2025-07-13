import './styles/ArmVisualizer.css';
import React, { useState, useEffect, useRef, useMemo } from 'react';
import ROSLIB from 'roslib';

const ARM_SEGMENT_LENGTHS = {
    L1: 100, // Shoulder to elbow
    L2: 80,  // Elbow to wrist
    L3: 50   // Wrist to end effector
};

interface Float32MultiArrayMessage {
    data: number[];
}


const ArmVisualizer: React.FC = () => {
    const [jointAngles, setJointAngles] = useState({ shoulder: 0, elbow: 0, wrist: 0 });
    const ros = useRef<ROSLIB.Ros | null>(null);
    const jointStateListener = useRef<ROSLIB.Topic | null>(null);

    // Forward Kinematics function (moved outside render or useMemo to prevent re-creation)
    // Using useMemo to memoize the function, though for a simple calculation it's not strictly necessary,
    // but a good pattern if it were more complex or if it took props that changed frequently.
    const calculateJointPositions = useMemo(() => (angles: { shoulder: number; elbow: number; wrist: number }) => {
        const { shoulder, elbow, wrist } = angles;
        const { L1, L2, L3 } = ARM_SEGMENT_LENGTHS;

        const shoulderX = 150; // Base X position on SVG
        const shoulderY = 250; // Base Y position on SVG (visualizing from top-down or side)

        // Elbow position
        const elbowX = shoulderX + L1 * Math.cos(shoulder);
        const elbowY = shoulderY - L1 * Math.sin(shoulder); // Y decreases upwards in typical SVG coords

        // Wrist position (relative to elbow, adjusted by previous angles)
        const wristX = elbowX + L2 * Math.cos(shoulder + elbow); // Cumulative angle
        const wristY = elbowY - L2 * Math.sin(shoulder + elbow);

        // End-effector position (relative to wrist, adjusted by cumulative angles)
        const endEffectorX = wristX + L3 * Math.cos(shoulder + elbow + wrist);
        const endEffectorY = wristY - L3 * Math.sin(shoulder + elbow + wrist);

        return {
            shoulder: { x: shoulderX, y: shoulderY },
            elbow: { x: elbowX, y: elbowY },
            wrist: { x: wristX, y: wristY },
            endEffector: { x: endEffectorX, y: endEffectorY }
        };
    }, []); // Empty dependency array, as ARM_SEGMENT_LENGTHS is constant

    // Calculate joint positions based on current angles
    // This must be called *before* the return statement where jointPositions is used.
    const jointPositions = calculateJointPositions(jointAngles);


    useEffect(() => {
        if (!ros.current) {
            ros.current = new ROSLIB.Ros({
                url: 'ws://localhost:9090' // Your rosbridge_server WebSocket URL
            });

            // Set up event listeners for the ROS connection
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

        // Subscribe to your joint states topic if ros.current is available and not already subscribed
        if (ros.current && !jointStateListener.current) {
            jointStateListener.current = new ROSLIB.Topic({
                ros: ros.current,
                name: '/arm_joint_states_2d', // Your ROS 2 topic name
                messageType: 'std_msgs/Float32MultiArray' // Correct message type
            });

            // Subscribe to the topic and handle incoming messages
            // Using the custom interface Float32MultiArrayMessage for type safety
            jointStateListener.current.subscribe((message: ROSLIB.Message) => {
                const floatArrayMessage = message as Float32MultiArrayMessage;
                // Ensure message.data exists and has at least 3 elements
                if (floatArrayMessage.data && floatArrayMessage.data.length >= 3) {
                    setJointAngles({
                        shoulder: floatArrayMessage.data[0],
                        elbow: floatArrayMessage.data[1],
                        wrist: floatArrayMessage.data[2]
                    });
                } else {
                    console.warn("Received message does not have 'data' property", message);
                }
            });
        }

        // Cleanup function: unsubscribe and close ROS connection when component unmounts
        return () => {
            if (jointStateListener.current) {
                jointStateListener.current.unsubscribe();
                jointStateListener.current = null; // Clear ref
            }
            if (ros.current) {
                ros.current.close();
                ros.current = null; // Clear ref
            }
        };
    }, []); // Empty dependency array means this effect runs once on mount and cleans up on unmount


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
                    x2={jointPositions.wrist.x} y2={jointPositions.wrist.y}
                    stroke="purple" strokeWidth="5"
                />
                {/* Wrist Joint */}
                <circle cx={jointPositions.wrist.x} cy={jointPositions.wrist.y} r="5" fill="orange" />

                {/* Hand/Tool (Wrist to End-effector Link) */}
                <line
                    x1={jointPositions.wrist.x} y1={jointPositions.wrist.y}
                    x2={jointPositions.endEffector.x} y2={jointPositions.endEffector.y}
                    stroke="black" strokeWidth="5"
                />
                {/* End-effector Point */}
                <circle cx={jointPositions.endEffector.x} cy={jointPositions.endEffector.y} r="5" fill="yellow" />

                {/* Display current joint angles as text */}
                <text x="10" y="20" fontSize="12" fill="black">
                    Shoulder: {jointAngles.shoulder.toFixed(2)} rad
                </text>
                <text x="10" y="35" fontSize="12" fill="black">
                    Elbow: {jointAngles.elbow.toFixed(2)} rad
                </text>
                <text x="10" y="50" fontSize="12" fill="black">
                    Wrist: {jointAngles.wrist.toFixed(2)} rad
                </text>
            </svg>
            <p>Current Angles: Shoulder: {jointAngles.shoulder.toFixed(2)}, Elbow: {jointAngles.elbow.toFixed(2)}, Wrist: {jointAngles.wrist.toFixed(2)}</p>
            <p>View (From Unity later) - This text is kept as per your original file structure for now.</p>
        </div>
    );
};

export default ArmVisualizer;