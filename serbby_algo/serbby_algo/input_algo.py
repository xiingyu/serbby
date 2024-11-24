import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, String, Bool, Int32
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge
import time

# import serial
import sys
params_dir = '/home/skh/semi_projects/jongsul/src/map_localization/params/coordinate.yaml'
coord_lists = {}


## robot state 
# got_goal
# arrive
# got_goal2




## main state
# idle
# setting
# decay


class SpringColorChecker(Node):
    def __init__(self):
        global coord_lists
        super().__init__('spring_color_checker')
        
        qos_profile = QoSProfile(depth=10)
        
        self.control_publisher = self.create_publisher(
            Float32MultiArray, 
            'Odrive_control', 
            qos_profile)
        
        self.request_pub = self.create_publisher(String, 'request_coordinate', 10)
        self.drive_coord = self.create_publisher(PoseStamped, 'drive', 10)
        ##
        self.robot_state_pub= self.create_publisher(
            String,
            'robot_state',
            qos_profile)
        self.main_state_pub = self.create_publisher(
            String,
            'main_state',
            qos_profile)

        #1
        self.order_pub = self.create_publisher(
            String, 
            '/order', 
            qos_profile)
        
        self.state_sub = self.create_subscription(
            String, 
            'state', 
            self.state_callback, 
            10)
        
        
        self.cam_flagger = self.create_subscription(
            Bool, 
            'cam_flag_topic', 
            self.cam_flag_callback, 
            10)
        

        self.location_sub = self.create_subscription(PoseStamped, 'current_location', self.coordinate_callback, 10)
        
        self.move_num_pub = self.create_publisher(String, '/num',10)

        ##################################
        self.coordinate_list = coord_lists

        #### addition ####
        with open(params_dir, 'r') as f:
            file = yaml.full_load(f)
            self.coordinate_list = file
        coord_lists = self.coordinate_list
        #####################
        
        self.num_store = None
        
        self.max_speed = 5
        self.odrive_mode = 1.
        self.cvbrid = CvBridge()
        
        ### paramas declare ###
        
        self.robot_state = "idle"
        self.main_state = "idle"
        
        
        self.arm_prev_state = "nothing"
        self.arrival_flag = "안도착"
        
        self.cam_flag = False
        #######################
        self.timer = self.create_timer(1/30, self.input_command) 

        self.state_nav = None
    
    def cam_flag_callback(self, msg) :
        in_data = msg.data
        self.cam_flag = in_data
        

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






    def send_table_num(self, num):
        number = String()
        self.get_logger().info(f"{num} 번 테이블로 이동합니다.")
        number.data = num
        self.order_pub.publish(number)







    def state_callback(self, msg):
        robot_state_msg = String()
        main_state_msg = String()
        self.arrival_flag = msg.data
        print(f'arrival flag : {self.arrival_flag}, {self.robot_state}')
        if (self.arrival_flag == "도착") & (self.robot_state == "got_goal") :
            self.robot_state = 'arrive'
            robot_state_msg.data = self.robot_state
            self.robot_state_pub.publish(robot_state_msg)
            print(f'robot state changed')
            self.get_logger().info(f"\033[1;32m 이동 상태: {self.arrival_flag} \033[0m")
        elif (self.arrival_flag == "도착") & (self.robot_state == "go_home") :
            self.robot_state = 'idle'
            robot_state_msg.data = self.robot_state
            self.robot_state_pub.publish(robot_state_msg)
            self.get_logger().info(f"\033[1;32m 이동 상태: {self.arrival_flag} \033[0m")
        elif (self.arrival_flag == "도착") & (self.robot_state == "got_goal2") :
            self.robot_state = 'idle'
            self.main_state = 'idle'
            
            robot_state_msg.data = self.robot_state
            self.robot_state_pub.publish(robot_state_msg)
            main_state_msg.data = self.main_state
            self.main_state_pub.publish(main_state_msg)
            
            self.get_logger().info(f"\033[1;32m 이동 상태: {self.arrival_flag} \033[0m")
        self.get_logger().info(f"이동 상태: {self.arrival_flag}")

    def input_command(self) :
        main_state_msg = String()
        robot_state_msg = String()
        
        if (self.robot_state == "idle") :
        
            self.get_logger().info(f'setting or decay')
            self.main_state = sys.stdin.readline().strip()
            if (self.main_state == "s") or (self.main_state == "d") :
                self.robot_state = "set"
            
            if self.main_state == "setting" :
                main_state_msg.data = self.main_state
                self.main_state_pub.publish(main_state_msg)
                self.get_logger().info(f'set main state setting')

                
                self.get_logger().info(f'inout goal (ex, g 1)')
                mod, num_table = sys.stdin.readline().split()
                self.robot_state = "got_goal"
                robot_state_msg.data = "got_goal"
                self.robot_state_pub.publish(robot_state_msg)
                
                if mod == "g" :
                    self.send_table_num(num_table)
                    self.send_drive_command(num_table)
                    self.send_num_msg(num_table)
                else :
                    self.robot_state = "idle"
                    robot_state_msg.data = "idle"
                    self.robot_state_pub.publish(robot_state_msg)
                    self.get_logger().info("Invalid input data")
                
                
                # self.main_state = sys.stdin.readline().strip()
                # while success == False:
                #     if self.state_nav == '도착' :
                #         self.endgoal_pub.publish(String(data = 'setting'))
                #         robot_state_msg.data = "arrive"
                #         self.robot_state_pub.publish(robot_state_msg)
                #         success = True
                #     elif self.state_nav == '실패' :
                #         num_table = 'g 1'
                #         self.send_table_num(num_table)
                #         success = True
                #     else :
                #         success = False


            elif self.main_state == "decay" :
                    main_state_msg.data = self.main_state
                    self.main_state_pub.publish(main_state_msg)
                    self.get_logger().info(f'set main state setting')

                    
                    self.get_logger().info(f'inout goal (ex, g 1)')
                    mod, num_table = sys.stdin.readline().split()
                    self.robot_state = "got_goal"
                    robot_state_msg.data = "got_goal"
                    self.robot_state_pub.publish(robot_state_msg)
                    
                    if mod == "g" :
                        self.send_table_num(num_table)
                        self.send_drive_command(num_table)
                        self.send_num_msg(num_table)
                    else :
                        self.robot_state = "idle"
                        robot_state_msg.data = "idle"
                        self.robot_state_pub.publish(robot_state_msg)
                        self.get_logger().info("Invalid input data")
                    
                    
            # main_state_msg.data = self.main_state
            # self.main_state_pub.publish(main_state_msg)
            # self.get_logger().info(f'set main state decay')

            # self.get_logger().info(f'inout goal (ex, g 1)')
            # num_table = sys.stdin.readline().strip()
            # self.send_table_num(num_table)
            
            # while success == False:
            #     if self.state_nav == '도착' :
            #         success = True
            #     elif self.state_nav == '실패' :
            #         num_table = 'g 1'
            #         self.send_table_num(num_table)
            #         success = True
            #     else :
            #         success = False
                    
        # elif self.robot_state == "got_goal" :
        #     self.get_logger().info(f'inout goal (ex, g 1)')
        #     self.robot_state = "got_goal"
        #     robot_state_msg.data = "got_goal"
        #     self.robot_state_pub.publish(robot_state_msg)
            
        #     mod, num_table = sys.stdin.readline().split(" ")
        #     if mod == "g" :
        #         self.send_table_num(num_table)
        #         self.send_drive_command(num_table)
        #         self.send_num_msg(num_table)
        #     else :
        #         self.get_logger().info("Invalid input data")
                
                
        elif self.main_state == "s" :
            self.get_logger().info(f'save place number')
            self.num_store = sys.stdin.readline().strip()
            self.request_current_location()
            self.main_state = "idle"
            self.robot_state = "idle"
            return
        elif self.main_state == "d" :
            self.print_coordinates()
            self.main_state = "idle"
            self.robot_state = "idle"
            
            return 
        elif self.robot_state == "arrive" :
            if self.cam_flag == True :
                self.send_table_num("1")
                self.send_drive_command("1")
                self.send_num_msg("1")
                self.robot_state = "got_goal2"
                self.get_logger().info(f'go state got_goal2')
                
        elif self.robot_state == "got_goal2" :
            # self.get_logger().info(f'go to home in 3 seconds...')
            # time.sleep(1)
            # self.get_logger().info(f'go to home in 2 seconds...')
            # time.sleep(1)
            # self.get_logger().info(f'go to home in 1 seconds...')
            # time.sleep(1)
            
            
            # self.send_drive_command(1)
            pass
                
        else :
            # self.get_logger().info(f'Invalid command : {self.main_state}')
            pass
            
        return
        

def main(args=None):
    rclpy.init(args=args)
    node = SpringColorChecker()
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
