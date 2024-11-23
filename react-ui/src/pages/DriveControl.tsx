// DriveControl.tsx
import React, { useEffect, useState, useRef } from 'react';

import { Container } from "../components/Container";
import './DriveControl.css';

function DriveControl() {
    const [linearVelocity, setLinearVelocity] = useState(0.0);
    const [angularVelocity, setAngularVelocity] = useState(0.0);
    const [activeKeys, setActiveKeys] = useState<{ [key: string]: boolean }>({});

    const ws = useRef<WebSocket | null>(null);

    useEffect(() => {
        let socket: WebSocket;
    
        const connectWebSocket = () => {
          // Update the WebSocket URL to match the backend server
          socket = new WebSocket(`ws://${window.location.hostname}:8080`);
    
          socket.onopen = () => {
            console.log("Connected to WebSocket server");
          };
    
          socket.onmessage = (event) => {
            const msg = JSON.parse(event.data);
            console.log(msg);
            if (msg.type === 'status') {
              setLinearVelocity(msg.data.linearVelocity);
              setAngularVelocity(msg.data.angularVelocity);
            }
          };
    
          socket.onclose = () => {
            console.log("Disconnected from WebSocket server. Reconnecting...");
            setTimeout(connectWebSocket, 3000);
          };
    
          socket.onerror = (err) => {
            console.error("WebSocket encountered error: ", err, "Closing socket");
            socket.close();
          };
    
          ws.current = socket;
        };
    
        connectWebSocket();
    
        // Cleanup websocket on unmount
        return () => {
          if (ws.current) {
            ws.current.close();
          }
        };
      }, []);

    // send key via websocket helper function
    const sendKeyEvent = (type: string, key: string) => {
        if (ws.current && ws.current.readyState == WebSocket.OPEN) {
            ws.current.send(JSON.stringify({ type, key }));
            setActiveKeys((prev) => ({
                ...prev,
                [key]: type === 'keydown',
            }));
        }
    }

    // handle keyboard events
    useEffect(() => {
        const handleKeyDown = (e: KeyboardEvent) => {
            const key = e.key.toLowerCase(); // normalize key
            if (['w', 'a', 's', 'd', ' '].includes(key)) {
                sendKeyEvent('keydown', key === ' ' ? 'space' : key);
            }
        }

        const handleKeyUp = (e: KeyboardEvent) => {
            const key = e.key.toLowerCase();
            if (['w', 'a', 's', 'd', ' '].includes(key)) {
                sendKeyEvent('keyup', key === ' ' ? 'space' : key);
            }
        };

        window.addEventListener('keydown', handleKeyDown);
        window.addEventListener('keyup', handleKeyUp);

        // cleanup event listeners on unmount
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
            window.removeEventListener('keyup', handleKeyUp);
        }
    }, [])

    // mouse click interactions
    const handleMouseDown = (key: string) => {
        sendKeyEvent('keydown', key);
    };

    const handleMouseUp = (key: string) => {
        sendKeyEvent('keyup', key);
    };

    const handleMouseLeave = (key: string) => {
        sendKeyEvent('keyup', key);
    };

    return (
        <Container title="Drive Control" width="100%" height="100%" border='none'>
            <p>Use your keyboard's WASD keys or the buttons below to control the rover.</p>

            <div className="control-buttons">
                <div>
                    <button
                        id="w"
                        className={activeKeys['w'] ? 'active' : ''}
                        onMouseDown={() => handleMouseDown('w')}
                        onMouseUp={() => handleMouseUp('w')}
                        onMouseLeave={() => handleMouseLeave('w')}
                        aria-label="Move Forward"
                    >
                        W
                    </button>
                </div>
                <div>
                    <button
                        id="a"
                        className={activeKeys['a'] ? 'active' : ''}
                        onMouseDown={() => handleMouseDown('a')}
                        onMouseUp={() => handleMouseUp('a')}
                        onMouseLeave={() => handleMouseLeave('a')}
                        aria-label="Turn Left"
                    >
                        A
                    </button>
                    <button
                        id="s"
                        className={activeKeys['s'] ? 'active' : ''}
                        onMouseDown={() => handleMouseDown('s')}
                        onMouseUp={() => handleMouseUp('s')}
                        onMouseLeave={() => handleMouseLeave('s')}
                        aria-label="Move Backward"
                    >
                        S
                    </button>
                    <button
                        id="d"
                        className={activeKeys['d'] ? 'active' : ''}
                        onMouseDown={() => handleMouseDown('d')}
                        onMouseUp={() => handleMouseUp('d')}
                        onMouseLeave={() => handleMouseLeave('d')}
                        aria-label="Turn Right"
                    >
                        D
                    </button>
                </div>
                <div>
                    <button
                        id="space"
                        className={activeKeys['space'] ? 'active' : ''}
                        onMouseDown={() => handleMouseDown('space')}
                        onMouseUp={() => handleMouseUp('space')}
                        onMouseLeave={() => handleMouseLeave('space')}
                        aria-label="Stop Rover"
                    >
                        Space
                    </button>
                </div>
            </div>

            <div id="status">
                <h2>Current Velocities</h2>
                <p>Linear Velocity: <span id="linear">{linearVelocity.toFixed(2)}</span> m/s</p>
                <p>Angular Velocity: <span id="angular">{angularVelocity.toFixed(2)}</span> rad/s</p>
            </div>
        </Container>
    )
}

  export default DriveControl;