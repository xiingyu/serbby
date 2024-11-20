import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, String


from cv_bridge import CvBridge

import serial
import sys


class SpringColorChecker(Node):
    def __init__(self):
        super().__init__('spring_color_checker')
        
        qos_profile = QoSProfile(depth=10)
        img_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST,
                                    depth=1)
        
        self.control_publisher = self.create_publisher(
            Float32MultiArray, 
            'Odrive_control', 
            qos_profile)
        
        ##
        self.robot_state = self.create_publisher(
            String,
            'robot_state',
            qos_profile)
        self.main_state = self.create_publisher(
            String,
            'main_state',
            qos_profile)
        
        
        ##################################
        self.ser = serial.Serial('/dev/ttyRS485', 9600, timeout=5)
        
        self.max_speed = 5
        self.odrive_mode = 1.
        self.cvbrid = CvBridge()
        
        ### paramas declare ###
        
        
        self.arm_prev_state = "nothing"
        
        
        #######################
        self.timer = self.create_timer(1/30, self.input_command) 
    
    
    def input_command(self) :
        main_state_msg = String()
        robot_state_msg = String()
        
        self.get_logger().info(f'setting or decay')
        in_data = sys.stdin.readline().strip()
        
        if in_data == "setting" :
            main_state_msg.data = in_data
            self.main_state.publish(main_state_msg)
            self.get_logger().info(f'set main state setting')
            in_data = sys.stdin.readline().strip()
            self.get_logger().info(f'inout goal (ex, s 1)')
        elif in_data == "decay" :
            main_state_msg.data = in_data
            self.main_state.publish(main_state_msg)
            self.get_logger().info(f'set main state decay')
        else :
            self.get_logger().info(f'Invalid command : {in_data}')
            
            
            
        
        return
        

def main(args=None):
    rclpy.init(args=args)
    node = SpringColorChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()