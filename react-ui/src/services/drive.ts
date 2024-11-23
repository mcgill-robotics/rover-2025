import express from 'express';
import http from 'http';
import path from 'path';
import cors from 'cors';
import RosService from './rosService';
import WebSocketService from './webSocketService';
import rclnodejs from 'rclnodejs';

async function initializeBackend() {
  try {
    // Initialize ROS2
    await rclnodejs.init();

    // Create Express app
    const app = express();

    // Middleware
    app.use(cors());
    app.use(express.static(path.join(__dirname, 'public'))); // Serve static files if needed

    // Create HTTP server
    const server = http.createServer(app);

    // Initialize ROS Service
    const rosService = new RosService();

    // Initialize WebSocket Service
    const webSocketService = new WebSocketService(server, rosService);

    // Start HTTP server
    const PORT = 8080;
    server.listen(PORT, () => {
      console.log(`Backend server is listening on port ${PORT}`);
    });
  } catch (error) {
    console.error('Error initializing the backend:', error);
  }
}

initializeBackend();
