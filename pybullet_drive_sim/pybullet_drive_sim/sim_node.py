import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pybullet as p
import pybullet_data
import time, math

class drive_simulation_node(Node):
    def __init__(self):
        super().__init__('drive_simulation_node')

        # Start PyBullet GUI
        p.connect(p.GUI)

        # Add default PyBullet search path
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load world
        p.loadURDF("plane.urdf", [0, 0, -0.1])
        p.setGravity(0, 0, -9.81)

        # Load your actual rover URDF
        urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../urdf/ROVER_DRIVE.urdf"
        self.roverID = p.loadURDF(urdf_path, [0, 0, 1], useFixedBase=False)
        self.get_logger().info(f"Loaded URDF: {urdf_path}")

        self.t = 0.0
        p.setRealTimeSimulation(0)

        info = [p.getJointInfo(self.roverID, i) for i in range(p.getNumJoints(self.roverID))]
        name_to_idx = {bytes(name).decode(): idx for idx, (_, name, *_ ) in enumerate(info)}
        self.drive_joints = {
            'FL': name_to_idx['FL drive'],
            'FR': name_to_idx['FR drive'],
            'BL': name_to_idx['BL drive'],
            'BR': name_to_idx['BR drive'],
        }

        self.steering_joints = {
            'FL': name_to_idx['FL Steering'],
            'FR': name_to_idx['FR Steering'],
            'BL': name_to_idx['BL Steering'],
            'BR': name_to_idx['BR Steering'],
        }

        self.drive_speed_subscriber = self.create_subscription(Float32MultiArray, "drive_speed_input",   self.speed_cmd_callback, 10)
        self.drive_cmd = {'FL':0.0, 'FR':0.0, 'BL':0.0, 'BR':0.0}

        self.drive_steering_subscriber = self.create_subscription(Float32MultiArray, "drive_steering_input",   self.steering_cmd_callback, 10)
        self.steering_cmd = {'FL': math.pi/2, 'FR': math.pi/2, 'BL': math.pi/2, 'BR': math.pi/2}

        self.pub_speed_fb  = self.create_publisher(Float32MultiArray, '/sim_drive_speeds', 10)
        self.pub_steering_fb  = self.create_publisher(Float32MultiArray, '/sim_drive_steering_angles', 10)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.run)

    def speed_cmd_callback(self, msg: Float32MultiArray):
        keys = ['FL','FR','BL','BR']
        for k, rpm in zip(keys, msg.data):
            self.drive_cmd[k] = rpm

    def steering_cmd_callback(self, msg: Float32MultiArray):
        keys = ['FL','FR','BL','BR']
        for k, rpm in zip(keys, msg.data):
            self.steering_cmd[k] = rpm


    def run(self):
        self.t = self.t + 0.01
        p.stepSimulation()

        # Drive Motors
        for k, rpm in self.drive_cmd.items():
            rad_per_sec = rpm * (2 * math.pi / 60.0)
            joint_index = self.drive_joints[k]
            p.setJointMotorControl2(
                bodyIndex=self.roverID,
                jointIndex=joint_index,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=rad_per_sec,
                force=50  # adjust max torque
            )

        fb = Float32MultiArray()
        fb.data = []
        for k in ['FL','FR','BL','BR']:
            joint_index = self.drive_joints[k]
            joint_state = p.getJointState(self.roverID, joint_index)
            actual_vel = joint_state[1] * (60 / (2 * math.pi)) # in rpm
            fb.data.append(actual_vel)
        self.pub_speed_fb.publish(fb)

        # Steering Motors
        for k, angle in self.steering_cmd.items():
            angle -= math.pi/2
            joint_index = self.steering_joints[k]
            p.setJointMotorControl2(
                bodyIndex=self.roverID,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle,
                force=50,  # adjust max torque
                positionGain=0.03,
                velocityGain=1,
            )

        fb = Float32MultiArray()
        fb.data = []
        for k in ['FL','FR','BL','BR']:
            joint_index = self.steering_joints[k]
            joint_state = p.getJointState(self.roverID, joint_index)
            actual_pos = joint_state[0]
            fb.data.append(actual_pos)
        self.pub_steering_fb.publish(fb)

def main(args=None):
    rclpy.init(args=args)
    drive_sim_node = drive_simulation_node()
    try:
        rclpy.spin(drive_sim_node)
    except KeyboardInterrupt:
        pass
    finally:
        drive_sim_node.destroy_node()
        rclpy.shutdown()
        p.disconnect()

if __name__ == "__main__":
    main()