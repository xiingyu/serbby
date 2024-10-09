import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import math

class CmdVelToWheelNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wheel_node')
        self.ppr = 1000
        self.wheel_radius = 0.075
        self.wheel_separation = 0.35
        self.linear_x = 0
        self.angular_z = 0
        self.left_wheel_rps = 0  
        self.right_wheel_rps = 0
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, QoSProfile(depth=10))
        self.wheel_command_pub = self.create_publisher(Float32MultiArray, 'wheel_command', QoSProfile(depth=10))
        self.timer = self.create_timer(1/10, self.update_rps)

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x / 2
        self.angular_z = msg.angular.z / 2

        self.v_left = (self.linear_x - self.angular_z * self.wheel_separation / 2)
        self.v_right = (self.linear_x + self.angular_z * self.wheel_separation / 2)
        self.left_wheel_rps = (self.v_left / (2 * math.pi * self.wheel_radius)) * self.ppr
        self.right_wheel_rps = (self.v_right / (2 * math.pi * self.wheel_radius)) * self.ppr
    
    def update_rps(self):
        ###모터 회전값 수정 가능
        self.left_wheel_rps = int(self.left_wheel_rps)
        self.right_wheel_rps = int(self.right_wheel_rps)

        # self.left_wheel_rps = min(self.left_wheel_rps, 50)
        # self.right_wheel_rps = min(self.right_wheel_rps, 50)

        # if self.angular_z != 0:
        #     if self.angular_z < -0.01:
        #         self.right_wheel_rps = 0
        #     elif self.angular_z > 0.01:
        #         self.left_wheel_rps = 0

        wheel = Float32MultiArray()
        wheel.data = [float(self.left_wheel_rps), float(self.right_wheel_rps)]

        self.wheel_command_pub.publish(wheel)
        self.get_logger().info(f"left_wheel_rps : {self.left_wheel_rps}, right_wheel_rps : {self.right_wheel_rps}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
