// drive.js
import express from 'express';
import http from 'http';
import path from 'path';
import cors from 'cors';
import RosService from './rosService.js';
import WebSocketService from '../websocketService.js';
import rclnodejs from 'rclnodejs';
import { fileURLToPath } from 'url';
import { dirname } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

async function initializeBackend() {
  try {
    // initialize ROS2
    await rclnodejs.init();

    // create Express app
    const app = express();

    // middleware
    app.use(cors());
    app.use(express.static(path.join(__dirname, 'public')));

    // create http server
    const server = http.createServer(app);

    // initialize ros service
    const rosService = new RosService();

    // initializat websocket service
    const webSocketService = new WebSocketService(server, rosService);

    // start server
    const PORT = 8080;
    server.listen(PORT, () => {
      console.log(`Backend server is listening on port ${PORT}`);
    });
  } catch (error) {
    console.error('Error initializing the backend:', error);
  }
}

initializeBackend();
