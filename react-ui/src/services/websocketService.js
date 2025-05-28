// webSocketService.js

// use websocket to communicate between client and server (rclnodejs)
import { WebSocketServer } from 'ws';
import WebSocket from 'ws';

class WebSocketService {
  constructor(server, rosService) {
    this.wss = new WebSocketServer({ server });
    this.rosService = rosService;
    this.clients = new Set();

    this.wss.on('connection', (ws) => {
      console.log('WebSocket client connected');
      this.clients.add(ws);

      // initial status
      ws.send(JSON.stringify({ type: 'status', data: this.rosService.getStatus() }));

      ws.on('message', (message) => {
        try {
          const data = JSON.parse(message);
          if (data.type === 'keydown') {
            this.rosService.keyDown(data.key || '');
          } else if (data.type === 'keyup') {
            this.rosService.keyUp(data.key || '');
          }
        } catch (err) {
          console.error('Error parsing message:', err);
        }
      });

      ws.on('close', () => {
        console.log('WebSocket client disconnected');
        this.clients.delete(ws);
      });
    });

    // listen to status updates from RosService and broadcast
    this.rosService.on('statusUpdate', (status) => {
      this.broadcastStatus(status);
    });
  }

  broadcastStatus(status) {
    const statusMessage = {
      type: 'status',
      data: status,
    };

    const messageString = JSON.stringify(statusMessage);

    this.wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(messageString);
      }
    });
  }
}

export default WebSocketService;
