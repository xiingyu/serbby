import rclpy  
import yaml
import os

from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
params_dir = '/home/skh/semi_projects/jongsul/src/map_localization/params/coordinate.yaml'
coord_lists = {}

class serbby_order(Node):
    def __init__(self):
        global coord_lists
        super().__init__('order_keyboard')
        self.location_sub = self.create_subscription(PoseStamped, 'current_location', self.coordinate_callback, 10)
        self.request_pub = self.create_publisher(String, 'request_coordinate', 10)
        self.drive_coord = self.create_publisher(PoseStamped, 'drive', 10)
        self.state_sub = self.create_subscription(String, 'state', self.state_callback, 10)
        
        #추가
        self.order_sub = self.create_subscription(String, '/order', self.sent_message, 10)
        self.move_num_pub = self.create_publisher(String, '/num', 10)

        self.coordinate_list = coord_lists

        #### addition ####
        with open(params_dir, 'r') as f:
            file = yaml.full_load(f)
            self.coordinate_list = file
        coord_lists = self.coordinate_list
        #####################
        
        self.num_store = None

    def state_callback(self, msg):
        self.get_logger().info(f"이동 상태: {msg.data}")

    def coordinate_callback(self, msg):
        global coord_lists
        if self.num_store is not None:
            self.coordinate_list[self.num_store] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
            self.get_logger().info(f"좌표 저장 완료 : 번호 {self.num_store}")
            coord_lists = self.coordinate_list
            self.num_store = None

    def request_current_location(self):
        self.get_logger().info("좌표 요청중")
        self.request_pub.publish(String(data="save"))

    def send_num_msg(self, num):
        self.move_num_pub.publish(String(data = num))

    def send_drive_command(self, num):
        if num in self.coordinate_list:
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = self.coordinate_list[num]['x']
            goal_msg.pose.position.y = self.coordinate_list[num]['y']
            goal_msg.pose.orientation.z = self.coordinate_list[num]['z']
            goal_msg.pose.orientation.w = self.coordinate_list[num]['w']
            
            self.drive_coord.publish(goal_msg)
            self.get_logger().info(f"이동 명령 전송: 번호 {num}")
        else:
            self.get_logger().warn("유효하지 않은 번호입니다.")

    def print_coordinates(self):
        self.get_logger().info("저장된 좌표 목록:")
        for num, coords in self.coordinate_list.items():
            self.get_logger().info(f"번호 {num}: \nx = {coords['x']} \ny = {coords['y']} \nz = {coords['z']} \nw = {coords['w']}")

    def sent_message(self, msg):
        order_data = msg.data.split(' ')
        order_word = order_data[0]
        order_num = order_data[1]
        
        if order_word == 's':
            self.num_store = order_num
            self.request_current_location()
        elif order_word == 'g':
            self.send_drive_command(order_num)
            self.send_num_msg(order_num)
        elif order_word == 'd':
            self.print_coordinates()
        else:
            self.get_logger().info("잘못된 입력입니다. 다시 입력해주세요.")

def main(args=None):
    rclpy.init(args=args)
    node = serbby_order()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        with open(params_dir, 'w') as f:
            yaml.dump(coord_lists, f)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()