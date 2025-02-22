import express, { json } from 'express';
import Peer from 'peerjs'; 
import { spawn } from 'child_process';
import cors from 'cors';

const app = express();
const pcs = new Set(); // Set to track active peer connections

// Middleware for JSON parsing
app.use(json());
app.use(cors()); // Enable CORS for all origins

// POST /offer endpoint to handle incoming WebRTC offers
app.post('/offer', async (req, res) => {
    const id = req.query.id;  // Get the device ID (e.g., webcam)
    const params = req.body;  // Get the SDP offer from the request body

    // Create a new PeerJS instance for WebRTC
    const pc = new Peer({
        initiator: false, // The server is not the initiator
        trickle: false,   // No trickle ICE
    });

    pcs.add(pc);  // Add to the set of peer connections

    pc.on('signal', (answer) => {
        // Send the SDP answer back to the client
        res.json(answer);
    });

    pc.on('error', (err) => {
        console.error('Error in peer connection:', err);
        res.status(500).send('Internal Server Error');
    });

    pc.on('close', () => {
        console.log('Peer connection closed');
        pcs.delete(pc);
    });

    // MediaPlayer simulation with ffmpeg
    const player = spawn('ffmpeg', [
        '-f', 'v4l2',  // Video4Linux2 format
        '-framerate', '20',
        '-video_size', '640x480',
        '-i', `/dev/video${id}`,  // Use the correct device ID
        '-f', 'mpegts',
        'pipe:1',  // Stream output to the pipe (stdout)
    ]);

    // Attach the video stream to the peer connection
    player.stdout.on('data', (data) => {
        // Normally, here you'd send the video stream to the WebRTC connection
        // This is just a placeholder for now
    });

    try {
        // Set the remote description with the offer from the client
        pc.signal(params); // `params` contains the offer from the client
    } catch (error) {
        console.error('Error during WebRTC negotiation:', error);
        res.status(500).send('Internal Server Error');
    }
});

// OPTIONS handler for preflight CORS requests
app.options('/offer', (req, res) => {
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', req.headers['access-control-request-headers'] || '');
    res.status(200).send();
});

// Shutdown handler
process.on('SIGINT', async () => {
    console.log('Shutting down...');
    const closePromises = Array.from(pcs).map((pc) => pc.destroy());
    await Promise.all(closePromises);
    process.exit(0);
});

// Start the server
const PORT = 8081;
app.listen(PORT, () => {
    console.log(`Server is running on http://localhost:${PORT}`);
});