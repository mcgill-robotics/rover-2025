from ..src import human_arm_control
import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Float32MultiArray


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.brushed_angles = [0.0, 0.0, 0.0]
        self.brushless_angles = [0.0, 0.0, 0.0]
        self.brushless_publisher_ = self.create_publisher(Float32MultiArray, 'armBrushlessCmd', 10)
        self.brushed_publisher_ = self.create_publisher(Float32MultiArray, 'armBrushedCmd', 10)
        self.armBrushedSubscriber = self.create_subscription(
            Float32MultiArray,
            "armBrushedFb",
            self.updateArmBrushedSim,
            10
        )

        self.armBrushlessSubscriber = self.create_subscription(
            Float32MultiArray,
            "armBrushlessFb",
            self.updateArmBrushlessSim,
            10
        )
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.motion_callback)
        self.i = 0

    def updateArmBrushedSim(self, cmds):
        print("brushed_update")
        self.brushed_angles = cmds.data
        #print("Brushed:", self.brushed_angles)
    
    def updateArmBrushlessSim(self, cmds):
        print("brushless_update")
        self.brushless_angles = cmds.data
        #print("Brushless:", self.brushless_angles)
    
    def motion_callback(self):
        brushless_msg = Float32MultiArray()
        brushed_msg = Float32MultiArray()
        motion = input("Motion type: (d, h, v)")
        joystick = float(input("Joystick: (-1.0 - 1.0)"))
        cur_angles = list(self.brushless_angles)
        cur_angles.reverse()
        cur_angles.append(self.brushed_angles[2])
        cur_angles.append(self.brushed_angles[1])
        cur_angles_rad = [x*math.pi/180 for x in cur_angles]
        #print(cur_angles)
        if motion == "d":
            #print(human_arm_control.depth_motion(joystick, cur_angles))
            new_angles = human_arm_control.depth_motion(joystick, cur_angles)
            brushless_msg.data = (
                new_angles[2] * 180 / math.pi,
                new_angles[1] * 180 / math.pi,
                new_angles[0] * 180 / math.pi
            )
            print(brushless_msg.data)
            brushed_msg.data = (
                0.0,
                new_angles[4] * 180 / math.pi,
                new_angles[3] * 180 / math.pi
            )
            print(brushed_msg.data)
            self.brushless_publisher_.publish(brushless_msg)
            self.brushed_publisher_.publish(brushed_msg)
        elif motion == "h":
            new_angles = human_arm_control.horizontal_motion(joystick, cur_angles)
            brushless_msg.data = (
                new_angles[2] * 180 / math.pi,
                new_angles[1] * 180 / math.pi,
                new_angles[0] * 180 / math.pi
            )
            print(brushless_msg.data)
            brushed_msg.data = (
                0.0,
                new_angles[4] * 180 / math.pi,
                new_angles[3] * 180 / math.pi
            )
            print(brushed_msg.data)
            self.brushless_publisher_.publish(brushless_msg)
            self.brushed_publisher_.publish(brushed_msg)
        else:
            print(cur_angles)
            new_angles = human_arm_control.vertical_motion(joystick, cur_angles)
            print(new_angles)
            brushless_msg.data = (
                new_angles[2] * 180 / math.pi,
                new_angles[1] * 180 / math.pi,
                new_angles[0] * 180 / math.pi
            )
            #print(brushless_msg.data)
            brushed_msg.data = (
                0.0,
                new_angles[4] * 180 / math.pi,
                new_angles[3] * 180 / math.pi
            )
            #print(brushed_msg.data)
            self.brushless_publisher_.publish(brushless_msg)
            print("hi")
            self.brushed_publisher_.publish(brushed_msg)
            print("hi2")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

