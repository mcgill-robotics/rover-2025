// index.js

const rclnodejs = require('rclnodejs');
const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const path = require('path');

async function main() {
  try {
    // Initialize rclnodejs
    await rclnodejs.init();

    // Create a ROS2 node named 'wasd_control_node'
    const node = rclnodejs.createNode('wasd_control_node');

    // Create a publisher for Twist messages on the '/rover_velocity_controller/cmd_vel' topic
    const publisher = node.createPublisher('geometry_msgs/msg/Twist', '/rover_velocity_controller/cmd_vel');

    // Initialize velocities
    let linearVelocity = 0.0;
    let angularVelocity = 0.0;

    // Set maximum velocities
    const maxLinearVelocity = 3.0;
    const maxAngularVelocity = 20.0; // Adjusted for faster angular changes

    // Set acceleration and deceleration rates
    const acceleration = 0.5; // Increased acceleration for faster changes
    const deceleration = 2.0; // Increased deceleration for faster stopping

    // Object to keep track of key states
    let keyStates = {
      w: false,
      s: false,
      a: false,
      d: false,
      space: false,
    };

    // Set up Express app
    const app = express();
    const server = http.createServer(app);

    // Serve static files from the 'public' directory
    app.use(express.static(path.join(__dirname, 'public')));

    // Create a WebSocket server
    const wss = new WebSocket.Server({ server });

    wss.on('connection', (ws) => {
      console.log('WebSocket client connected');

      // Send initial status
      ws.send(JSON.stringify({ type: 'status', data: { linearVelocity, angularVelocity } }));

      ws.on('message', (message) => {
        try {
          const data = JSON.parse(message);
          if (data.type === 'keydown') {
            if (data.key in keyStates) {
              keyStates[data.key] = true;
              console.log(`Key down: ${data.key}`);
            }
          } else if (data.type === 'keyup') {
            if (data.key in keyStates) {
              keyStates[data.key] = false;
              console.log(`Key up: ${data.key}`);
            }
          }
        } catch (err) {
          console.error('Error parsing message:', err);
        }
      });

      ws.on('close', () => {
        console.log('WebSocket client disconnected');
      });
    });

    function updateVelocity() {
      // Update linear velocity
      if (keyStates.w) {
        linearVelocity += acceleration;
        if (linearVelocity > maxLinearVelocity) {
          linearVelocity = maxLinearVelocity;
        }
      } else if (keyStates.s) {
        linearVelocity -= acceleration;
        if (linearVelocity < -maxLinearVelocity) {
          linearVelocity = -maxLinearVelocity;
        }
      } else {
        // Decelerate when no key is pressed
        if (linearVelocity > 0) {
          linearVelocity -= deceleration;
          if (linearVelocity < 0) linearVelocity = 0;
        } else if (linearVelocity < 0) {
          linearVelocity += deceleration;
          if (linearVelocity > 0) linearVelocity = 0;
        }
      }

      // Update angular velocity
      if (keyStates.a) {
        angularVelocity += acceleration;
        if (angularVelocity > maxAngularVelocity) {
          angularVelocity = maxAngularVelocity;
        }
      } else if (keyStates.d) {
        angularVelocity -= acceleration;
        if (angularVelocity < -maxAngularVelocity) {
          angularVelocity = -maxAngularVelocity;
        }
      } else {
        // Decelerate angular velocity
        if (angularVelocity > 0) {
          angularVelocity -= deceleration;
          if (angularVelocity < 0) angularVelocity = 0;
        } else if (angularVelocity < 0) {
          angularVelocity += deceleration;
          if (angularVelocity > 0) angularVelocity = 0;
        }
      }

      // Handle stop command
      if (keyStates.space) {
        linearVelocity = 0;
        angularVelocity = 0;
        keyStates.space = false; // Reset stop state
        console.log('Space pressed: Stopping rover');
      }

      // Create and publish the Twist message
      const Twist = rclnodejs.require('geometry_msgs/msg/Twist');
      const twist = new Twist();
      twist.linear.x = linearVelocity;
      twist.angular.z = angularVelocity;

      publisher.publish(twist);
      console.log(`Published Twist - Linear: ${linearVelocity.toFixed(2)}, Angular: ${angularVelocity.toFixed(2)}`);

      // Broadcast the updated status to all connected WebSocket clients
      const statusMessage = JSON.stringify({
        type: 'status',
        data: {
          linearVelocity: linearVelocity,
          angularVelocity: angularVelocity,
        },
      });

      wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(statusMessage);
        }
      });
    }

    // Update velocity at a fixed interval
    setInterval(updateVelocity, 100); // Update every 100ms

    // Start spinning the ROS2 node
    rclnodejs.spin(node);

    // Start the HTTP server
    const PORT = 8000;
    server.listen(PORT, () => {
      console.log(`HTTP server is listening on port ${PORT}`);
      console.log(`Open http://localhost:${PORT} in your browser to control the rover.`);
    });
  } catch (error) {
    console.error('Error initializing the application:', error);
  }
}

main();
