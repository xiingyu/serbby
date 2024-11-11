import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import String

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('current_coordinate')
        self.pose_pub = self.create_publisher(PoseStamped, 'current_location', 10)
        self.request_sub = self.create_subscription(String, 'request_coordinate', self.publish_current_position, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def publish_current_position(self, msg):
        self.get_logger().info('좌표 생성중...')
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            current_pose = PoseStamped()
            current_pose.header.stamp = self.get_clock().now().to_msg()
            current_pose.header.frame_id = 'map'
            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.orientation.z = trans.transform.rotation.z
            current_pose.pose.orientation.w = trans.transform.rotation.w

            self.pose_pub.publish(current_pose)
            self.get_logger().info('좌표 전송')
        except Exception as e:
            self.get_logger().warn(f'실패 : {e}')

    # def request_callback(self, msg):
    #     
    #     self.publish_current_position()

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
