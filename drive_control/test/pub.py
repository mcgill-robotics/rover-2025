import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from msg_srv_interface.msg import WheelSpeed
    
class wheelspeed_publisher(Node):
    def __init__(self):
        super().__init__('test_sub')
        self.twist_pub    = self.create_publisher(Twist,      "rover_velocity_controller/cmd_vel", 10)
        self.feedback_pub = self.create_publisher(WheelSpeed, "/feedback_velocity",                10)
        self.position_pub = self.create_publisher(Pose,       "/position_pose",                    10)
        self.time_period  = 0.35
        self.timer        = self.create_timer(self.time_period, self.timer_callback)
        self.init_params()
        self.iter = 0
    
    def timer_callback(self):
        self.robot_twist.linear.x = float(30 + self.iter)
        self.twist_pub.publish(self.robot_twist)
        self.position_pub.publish(self.rover_position)
        self.feedback_pub.publish(self.feedback)
        self.iter += 10
        self.get_logger().info('Publishing: {}'.format(self.rover_position)) 

    def init_params(self):
        robot_twist = Twist()
        robot_twist.angular.z = 1.0
        robot_twist.linear.x = 1.0
        self.robot_twist = robot_twist
        
        feedback = WheelSpeed()
        feedback.left[0] = 10
        feedback.left[1] = 12
        feedback.right[0] = 20
        feedback.right[1] = 22
        self.feedback = feedback

        rover_position = Pose()
        rover_position.position.x = 2.0
        rover_position.position.y = 3.0
        rover_position.position.z = 1.0
        rover_position.orientation.x = 0.778
        rover_position.orientation.y = 0.34
        rover_position.orientation.z = 0.35
        rover_position.orientation.w = 0.395
        self.rover_position = rover_position 
        
def main():
    rclpy.init()
    test_pub = wheelspeed_publisher()
    rclpy.spin(test_pub)
    test_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    