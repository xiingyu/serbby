import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile
import math

class PlanFollowerNode(Node):
    def __init__(self):
        super().__init__('plan_follower_node')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        self.plan_sub = self.create_subscription(
            PoseStamped,
            '/plan',
            self.plan_callback,
            QoSProfile(depth=10)
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.goal_position = None
        self.goal_orientation = None

        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.5  # rad/s

    def plan_callback(self, msg):
        self.goal_position = msg.pose.position
        self.goal_orientation = msg.pose.orientation

    def control_loop(self):
        if self.goal_position is None:
            return
        
        # 현재 위치 및 목표 위치 계산 (base_link 기준으로 현재 위치가 (0, 0, 0)이라 가정)
        current_x = 0.0
        current_y = 0.0
        goal_x = self.goal_position.x
        goal_y = self.goal_position.y

        # 목표 위치까지의 거리와 각도 계산
        distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)

        if distance < 0.1: 
            self.stop_robot()
            self.get_logger().info("Reached goal position!")
            return
        
        twist = Twist()
        twist.linear.x = min(self.linear_speed, distance)
        twist.angular.z = self.angular_speed * angle_to_goal
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PlanFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
