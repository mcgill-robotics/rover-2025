import WebSocket, { Server as WebSocketServer } from 'ws';
import RosService from './rosService';

interface Message {
  type: string;
  key?: string;
  data?: any;
}

class WebSocketService {
  private wss: WebSocketServer;
  private rosService: RosService;
  private clients: Set<WebSocket>;

  constructor(server: http.Server, rosService: RosService) {
    this.wss = new WebSocketServer({ server });
    this.rosService = rosService;
    this.clients = new Set();

    this.wss.on('connection', (ws: WebSocket) => {
      console.log('WebSocket client connected');
      this.clients.add(ws);

      // Send initial status
      ws.send(JSON.stringify({ type: 'status', data: this.rosService.getStatus() }));

      ws.on('message', (message: string) => {
        try {
          const data: Message = JSON.parse(message);
          if (data.type === 'keydown') {
            this.rosService.keyDown(data.key || '');
            console.log(`Key down: ${data.key}`);
          } else if (data.type === 'keyup') {
            this.rosService.keyUp(data.key || '');
            console.log(`Key up: ${data.key}`);
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

    // Listen to status updates from RosService and broadcast
    this.rosService.on('statusUpdate', (status: { linearVelocity: number; angularVelocity: number }) => {
      this.broadcastStatus(status);
    });
  }

  private broadcastStatus(status: { linearVelocity: number; angularVelocity: number }) {
    const statusMessage: Message = {
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
