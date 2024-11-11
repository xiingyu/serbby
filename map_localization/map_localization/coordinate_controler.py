import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile
from rclpy.clock import Clock

class LocationSaver(Node):
    def __init__(self):
        super().__init__('coordinate_controler')
        self.location_sub = self.create_subscription(PoseStamped, 'drive', self.go_goal, 10)
        self.state = self.create_publisher(String, 'state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', QoSProfile(depth=10))
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def go_goal(self, msg):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = Clock().now().to_msg()  
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = msg.pose.position.x
        goal_pose.pose.position.y = msg.pose.position.y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = msg.pose.orientation.x
        goal_pose.pose.orientation.y = msg.pose.orientation.y
        goal_pose.pose.orientation.z = msg.pose.orientation.z
        goal_pose.pose.orientation.w = msg.pose.orientation.w

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        state_msg = String()

        if not goal_handle.accepted:
            state_msg.data = '목표 전송 실패'
            self.state.publish(state_msg)
            return

        state_msg.data = '목표 전송 성공'
        self.state.publish(state_msg)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            status = future.result().status
            state_msg = String() 

            if status == GoalStatus.STATUS_SUCCEEDED:
                state_msg.data = '도착'
            else:
                state_msg.data = f'실패 : {status}'
                
            self.state.publish(state_msg)
        except Exception as e:
            state_msg.data = f'에러 : {str(e)}'
            self.state.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocationSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
