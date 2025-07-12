import './styles/ArmVisualizer.css'
import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';


const ARM_SEGMENT_LENGTHS = {
    L1: 100, // Shoulder to elbow
    L2: 80,  // Elbow to wrist
    L3: 50   // Wrist to end effector
};

// container name visualizer-container
const ArmVisualizer: React.FC = () => {
    const [jointAngles, setJointAngles] = useState({ shoulder: 0, elbow: 0, wrist: 0 });
    const ros = useRef<ROSLIB.Ros | null>(null);
    const jointStateListener = useRef<ROSLIB.Topic | null>(null);

    useEffect(() => {
        if (ros.current) {
            ros.current.on('connection', () => {
                console.log('Connected to rosbridge_server');
            });
            ros.current.on('error', (error) => {
                console.error('Error connecting to rosbridge_server:', error);
            });
            ros.current.on('close', () => {
                console.log('Connection to rosbridge_server closed');
            });
            // subsrube to join states topic
        }
        if (!jointStateListener.current && ros.current) {
            jointStateListener.current = new ROSLIB.Topic({
                ros: ros.current,
                name: '/arm_joint_states_2d',
                messageType: 'std_msgs/Float32MultiArray'
            });
            jointStateListener.current.subscribe((message: ROSLIB.Message) // message.data is assumed to be an array hopefully
                => {
                if ((message as any).data && (message as any).data.length >= 3) {
                    setJointAngles({
                        shoulder: (message as any).data[0],
                        elbow: (message as any).data[1],
                        wrist: (message as any).data[2]
                    });
                }
            });
        }
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
    const calculateJointPositions = (angles: { shoulder: number; elbow: number; wrist: number }) => {
        const { shoulder, elbow, wrist } = angles;
        const { L1, L2, L3 } = ARM_SEGMENT_LENGTHS;

        const shoulderX = 150;
        const shoulderY = 250;

        const elbowX = shoulderX + L1 * Math.cos(shoulder);
        const elbowY = shoulderY - L1 * Math.sin(shoulder);

        const wristX = elbowX + L2 * Math.cos(shoulder + elbow);
        const wristY = elbowY - L2 * Math.sin(shoulder + elbow);

        const endEffectorX = wristX + L3 * Math.cos(shoulder + elbow + wrist);
        const endEffectorY = wristY - L3 * Math.sin(shoulder + elbow + wrist);

        return {
            shoulder: { x: shoulderX, y: shoulderY },
            elbow: { x: elbowX, y: elbowY },
            wrist: { x: wristX, y: wristY },
            endEffector: { x: endEffectorX, y: endEffectorY }
        };
    }; // to be continued
};

export default ArmVisualizer;