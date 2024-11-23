import rclnodejs from 'rclnodejs';
import { EventEmitter } from 'events';

interface Status {
  linearVelocity: number;
  angularVelocity: number;
}

class RosService extends EventEmitter {
  private node: any;
  private publisher: any;

  // Initialize velocities
  private linearVelocity: number = 0.0;
  private angularVelocity: number = 0.0;

  // Set maximum velocities
  private maxLinearVelocity: number = 3.0;
  private maxAngularVelocity: number = 20.0; // Adjusted for faster angular changes

  // Set acceleration and deceleration rates
  private acceleration: number = 0.5; // Increased acceleration for faster changes
  private deceleration: number = 2.0; // Increased deceleration for faster stopping

  // Key states
  private keyStates: { [key: string]: boolean } = {
    w: false,
    s: false,
    a: false,
    d: false,
    space: false,
  };

  constructor() {
    super();
    this.initializeROS2();
    this.setupVelocityUpdate();
  }

  private initializeROS2() {
    // Initialize ROS2 node
    this.node = rclnodejs.createNode('wasd_control_node');

    // Create a publisher for Twist messages
    this.publisher = this.node.createPublisher('geometry_msgs/msg/Twist', '/rover_velocity_controller/cmd_vel');

    // Start spinning the ROS2 node
    rclnodejs.spin(this.node);
  }

  private setupVelocityUpdate() {
    setInterval(() => this.updateVelocity(), 100); // Update every 100ms
  }

  public keyDown(key: string) {
    if (key in this.keyStates) {
      this.keyStates[key] = true;
    }
  }

  public keyUp(key: string) {
    if (key in this.keyStates) {
      this.keyStates[key] = false;
    }
  }

  private updateVelocity() {
    // Update linear velocity
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

    // Update angular velocity
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
      console.log('Space pressed: Stopping rover');
    }

    // Create and publish the Twist message
    const Twist = rclnodejs.require('geometry_msgs/msg/Twist');
    const twist = new Twist();
    twist.linear.x = this.linearVelocity;
    twist.angular.z = this.angularVelocity;

    this.publisher.publish(twist);
    console.log(`Published Twist - Linear: ${this.linearVelocity.toFixed(2)}, Angular: ${this.angularVelocity.toFixed(2)}`);

    // Emit status update
    const status: Status = {
      linearVelocity: this.linearVelocity,
      angularVelocity: this.angularVelocity,
    };

    this.emit('statusUpdate', status);
  }

  public getStatus(): Status {
    return {
      linearVelocity: this.linearVelocity,
      angularVelocity: this.angularVelocity,
    };
  }
}

export default RosService;
