// DriveControl.tsx
// Drive control TEST
import './styles/DriveControl.css';

import { useEffect, useState, useRef } from 'react';

// import Controller from "../components/drive/ui/Controller";
import Container from "../Container";

function DriveControl() {
    const [linearVelocity, setLinearVelocity] = useState(0.0);
    const [angularVelocity, setAngularVelocity] = useState(0.0);
    const [activeKeys, setActiveKeys] = useState<{ [key: string]: boolean }>({});
    const [status, setStatus] = useState(false)
    const [connect, setConnect] = useState(false)

    const ws = useRef<WebSocket | null>(null);

    useEffect(() => {
        if (connect) {
            let socket: WebSocket;
    
            const connectWebSocket = () => {
              socket = new WebSocket(`ws://${window.location.hostname}:8080`);
        
              socket.onopen = () => {
                setStatus(true)
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
                setStatus(false)
                console.log("Disconnected from WebSocket server. Reconnecting...");
                //remove autoconnect out(connectWebSocket, 3000);
              };
        
              socket.onerror = (err) => {
                setStatus(false)
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
        }
      }, [connect]);

    // send key via websocket to rclnodejs
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

    // connect and disconnect handlers
    const handleDisconnect = () => {
        if (ws.current) {
            ws.current.close();
            setStatus(false);
        }
    }

    const handleConnect = () => {
        setConnect(prev => !prev);
    }

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
            <div style={{
            }}>
                
                <div id='keyboard-controls'>
                    <p>Run "npm start:all" to start node server for drive control</p>
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

                    <div className='connection-status'>
                        <p>Connection status: {status? "Connected" : "Disconnected" }</p>
                        {status ? (
                            <button onClick={handleDisconnect} onKeyUp={e => e.preventDefault()}>Disconnect</button>
                        ) : (
                            <button onClick={handleConnect} onKeyUp={e => e.preventDefault()}>Connect</button>
                        )}
                    </div>

                    {(status &&
                        <div className='status'>
                            <h2>Current Velocities</h2>
                            <p>Linear Velocity: <span id="linear">{linearVelocity.toFixed(2)}</span> m/s</p>
                            <p>Angular Velocity: <span id="angular">{angularVelocity.toFixed(2)}</span> rad/s</p>
                        </div>
                    )}
                </div>
            </div>
        </Container>
    )
}

  export default DriveControl;