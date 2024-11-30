// rosService.js
import rclnodejs from 'rclnodejs';
import EventEmitter from 'events';

class RosService extends EventEmitter {
  constructor() {
    super();
    // Initialize velocities
    this.linearVelocity = 0.0;
    this.angularVelocity = 0.0;

    // Set maximum velocities
    this.maxLinearVelocity = 3.0;
    this.maxAngularVelocity = 20.0; // Adjusted for faster angular changes

    // Set acceleration and deceleration rates
    this.acceleration = 0.5;
    this.deceleration = 2.0; 

    // Key states
    this.keyStates = {
      w: false,
      s: false,
      a: false,
      d: false,
      space: false,
    };
  
    // interval of update in ms
    this.updateInterval = 200; 

    this.initializeROS2();
    this.setupVelocityUpdate();
  }

  initializeROS2() {
    // initialize a ROS2 node
    this.node = rclnodejs.createNode('wasd_control_node');

    // publisher for Twist messages that publish to cmd_vel
    this.publisher = this.node.createPublisher('geometry_msgs/msg/Twist', '/rover_velocity_controller/cmd_vel');

    // spin this node with the publisher that publishes Twist messages
    rclnodejs.spin(this.node);
  }
  setupVelocityUpdate() {
    setInterval(() => this.updateVelocity(), this.updateInterval);
  }

  keyDown(key) {
    if (key in this.keyStates) {
      this.keyStates[key] = true;
    }
  }

  keyUp(key) {
    if (key in this.keyStates) {
      this.keyStates[key] = false;
    }
  }

  updateVelocity() {
    // Update linear velocity (forward backward)
    if (this.keyStates.w) {
      this.linearVelocity += this.acceleration;
      if (this.linearVelocity > this.maxLinearVelocity) {
        this.linearVelocity = this.maxLinearVelocity;
      }
    } else if (this.keyStates.s) {
      this.linearVelocity -= this.acceleration;
      if (this.linearVelocity < -this.maxLinearVelocity) {
        this.linearVelocity = -this.maxLinearVelocity;
      }
    } else {
      // Decelerate when no key is pressed
      if (this.linearVelocity > 0) {
        this.linearVelocity -= this.deceleration;
        if (this.linearVelocity < 0) this.linearVelocity = 0;
      } else if (this.linearVelocity < 0) {
        this.linearVelocity += this.deceleration;
        if (this.linearVelocity > 0) this.linearVelocity = 0;
      }
    }

    // Update angular velocity (turning)
    if (this.keyStates.a) {
      this.angularVelocity += this.acceleration;
      if (this.angularVelocity > this.maxAngularVelocity) {
        this.angularVelocity = this.maxAngularVelocity;
      }
    } else if (this.keyStates.d) {
      this.angularVelocity -= this.acceleration;
      if (this.angularVelocity < -this.maxAngularVelocity) {
        this.angularVelocity = -this.maxAngularVelocity;
      }
    } else {
      // Decelerate angular velocity
      if (this.angularVelocity > 0) {
        this.angularVelocity -= this.deceleration;
        if (this.angularVelocity < 0) this.angularVelocity = 0;
      } else if (this.angularVelocity < 0) {
        this.angularVelocity += this.deceleration;
        if (this.angularVelocity > 0) this.angularVelocity = 0;
      }
    }

    // Handle stop command
    if (this.keyStates.space) {
      this.linearVelocity = 0;
      this.angularVelocity = 0;
      this.keyStates.space = false; // Reset stop state
      console.log('stopping rover');
    }

    // create and publish the Twist message
    const Twist = rclnodejs.require('geometry_msgs/msg/Twist');
    const twist = new Twist();
    twist.linear.x = this.linearVelocity; // add linear velocity to Twist msg
    twist.angular.z = this.angularVelocity; // add angular velocity to Twist msg

    this.publisher.publish(twist); // publish the Twist msg
    console.log(`Published Twist - Linear: ${this.linearVelocity.toFixed(2)}, Angular: ${this.angularVelocity.toFixed(2)}`);

    // Emit status update
    const status = {
      linearVelocity: this.linearVelocity,
      angularVelocity: this.angularVelocity,
    };

    this.emit('statusUpdate', status);
  }

  getStatus() {
    const status = {
      linearVelocity: this.linearVelocity,
      angularVelocity: this.angularVelocity,
    };
    return status;
  }
}

export default RosService;
