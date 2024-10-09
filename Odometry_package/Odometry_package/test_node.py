import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile

class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.keyboard_pub = self.create_publisher(PoseStamped, 'goal_pose', QoSProfile(depth=10))

    def get_user_input(self):
        while True:
            try:
                user_input = input("(left_wheel right_wheel)")
                self.publish_wheel_command(user_input)
            except EOFError:
                break

    def publish_wheel_command(self, command):
        self.keyboard_pub.publish(String(data=command))

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    try:
        node.get_user_input() 
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
