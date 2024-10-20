import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import math

class EncoderMotorControlNode(Node):
    def __init__(self):
        super().__init__('wheel_to_odometry_node')
        self.wheel_radius = 0.075  
        self.wheel_separation = 0.594  
        self.ppr = 1000

        self.create_subscription(Float32MultiArray, 'Odrive_encoder', self.wheel_command_callback, QoSProfile(depth=10))
        self.cmd_vel_pub = self.create_publisher(Twist, 'wheel_cmd_vel', QoSProfile(depth=10))

    def wheel_command_callback(self, msg):
        try:
            ###수동 입력이 필요시 
            # left_wheel_rps, right_wheel_rps = map(float, msg.data.split())

            ###자율 시스템
            left_wheel_rps = msg.data[0]
            right_wheel_rps = msg.data[1]

            v_left = (2 * math.pi * self.wheel_radius * left_wheel_rps) / self.ppr
            v_right = (2 * math.pi * self.wheel_radius * right_wheel_rps) / self.ppr

            linear_speed = (v_left + v_right) / 2 
            angular_speed = (v_right - v_left) / self.wheel_separation

            twist = Twist()
            twist.linear.x = -linear_speed
            twist.angular.z = angular_speed
            self.cmd_vel_pub.publish(twist)

            self.get_logger().info(f"linear.x = {linear_speed}, angular.z = {angular_speed}")
        except ValueError:
            self.get_logger().info("error")


def main(args=None):
    rclpy.init(args=args)
    node = EncoderMotorControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
