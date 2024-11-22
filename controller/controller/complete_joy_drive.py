import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Joy, Image


import cv2
from cv_bridge import CvBridge

import serial
import time



class CompleteJoyDrive(Node):
    def __init__(self):
        super().__init__('complete_joy_drive')
        
        qos_profile = QoSProfile(depth=10)
        img_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST,
                                    depth=1)
        
        self.control_publisher = self.create_publisher(
            Float32MultiArray, 
            'Odrive_control', 
            qos_profile)
        self.serial_read_data = self.create_publisher(
            String, 
            'read_rs485', 
            qos_profile)
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_msg_sampling,
            qos_profile)
        self.slam_subscriber = self.create_subscription(
            Float32MultiArray,
            'wheel_command',
            self.slam_cmd_sampling,
            qos_profile)
        self.img_subscriber = self.create_subscription(
            Image,
            'col_img',
            self.img_indicater,
            img_qos_profile)
        
        
        self.arm_control = self.create_subscription(
            String,
            'arm_control',
            self.arm_control_sub,
            qos_profile)
        
        self.control_subscriber = self.create_subscription(
            Float32MultiArray, 
            'auto_control',
            self.auto_control, 
            qos_profile)
        
        self.control_subscriber = self.create_subscription(
            Float32MultiArray, 
            'abs_control',
            self.abs_control, 
            qos_profile)
        self.ser = serial.Serial('/dev/ttyRS485', 9600, timeout=5)
        
        self.max_speed = 5
        self.odrive_mode = 1.
        self.cvbrid = CvBridge()
        
        ### paramas declare ###
        
        self.L_cmd_vel = 0.
        self.R_cmd_vel = 0.
        
        self.arm_prev_state = "nothing"
        
        
        #######################
        
        self.timer_serial = self.create_timer(1/10, self.serial_read)
        
        self.ser.write('c'.encode()) 
        time.sleep(1)
        
        self.ser.write('u'.encode()) 
        time.sleep(1)
        # self.get_logger().info(f'serial send "d"')
        self.cur_time = time.time()
        
    def abs_control(self, msg) :
        ctl_data = msg.data
        msg = Float32MultiArray()
        # self.odrive_mode = 2. 
        msg.data = [ctl_data[1],ctl_data[1], ctl_data[2] ]
        self.control_publisher.publish(msg)
        self.cur_time = time.time()
        
        
                
        
    def serial_read(self) :
        if self.ser.in_waiting > 0:  # 수신 데이터가 있을 경우
            read_data = String()
            raw_data = self.ser.read(self.ser.in_waiting)  # 받은 데이터 읽기
            
            try:
                read_data.data = raw_data.decode('utf-8')
            except UnicodeDecodeError:
                self.get_logger().error("\033[1;31m Failed to decode serial data. Received raw data: {}\033[0m".format(raw_data))
                return
            
            self.get_logger().info(f"Received: {read_data.data}")
            
            self.serial_read_data.publish(read_data)
            
            
            self.get_logger().info(f"\033[1;36m {read_data.data} \033[0m")
    
    def img_indicater(self, msg) :
        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        y,x,c = current_img.shape
        resized = cv2.resize(current_img, (int(x*1.5),int(y*1.5)), interpolation=cv2.INTER_CUBIC)

        cv2.imshow("col_img", resized)
        cv2.waitKey(1)
        
    def slam_cmd_sampling(self, msg) :
        data = msg.data
        
        self.L_cmd_vel = data[0]
        self.R_cmd_vel = data[1]
        
        
        return
    def auto_control(self, msg) :
        data = msg.data
        self.L_cmd_vel = data[1]
        self.R_cmd_vel = data[2]
        
        
    
    def arm_control_sub(self, msg) :
        data = msg.data
        # print(f'data : {data}')
        
        if self.arm_prev_state == data :
            pass
        
        elif data == "u" :
            self.ser.write(b'u')        
            self.ser.write('u'.encode())
            self.get_logger().info(f'serial send "u"')
        elif data == "d" :
            self.ser.write('d'.encode()) 
            self.get_logger().info(f'serial send "d"')
        elif data == "s" :
            self.ser.write('s'.encode())    
            self.get_logger().info(f'serial send "s"')
        else :
            self.get_logger().info(f'i got data : {data}, and type is {type(data)}')
            pass
        
        
        
        
        self.arm_prev_state = data
        
        
        return


    def joy_msg_sampling(self, msg):
        axes = msg.axes
        btn = msg.buttons

        if not (axes[2] == 1) :
            self.max_speed = 5
            if btn[2] == 1 :
                self.go()
            elif btn[1] == 1 :
                self.turn_right()
            elif btn[3] == 1:
                self.turn_left()
            elif btn[0] == 1 :
                self.back()
            else : 
                self.joy_stick_data = [axes[1], axes[4]]
                self.joy_pub()
        elif not (axes[5] == 1) :
            self.max_speed = 10
            if btn[2] == 1 :
                self.go()
            elif btn[1] == 1 :
                self.turn_right()
            elif btn[3] == 1:
                self.turn_left()
            elif btn[0] == 1 :
                self.back()
            else : 
                self.joy_stick_data = [axes[1], axes[4]]
                self.joy_pub()
                
        elif axes[6] != 0 :
            self.ser.write('s'.encode())
            self.ser.write(b's')    
            self.get_logger().info(f'serial send "s"')
                
        elif axes[7] != 0 :
            if axes[7] >0 :
                self.ser.write(b'u')        
                self.ser.write('u'.encode())
                self.get_logger().info(f'serial send "u"')
            elif axes[7] <0 :
                self.ser.write('d'.encode()) 
                self.get_logger().info(f'serial send "d"')
            else :
                self.ser.write('s'.encode())    
                self.get_logger().info(f'serial send "s"')
        elif btn[9] == 1 :
            self.ser.write('r'.encode())    
            self.get_logger().info(f'serial send "r"')
            
        else :
            if time.time()- self.cur_time > 4 :
                msg = Float32MultiArray()
                self.joy_stick_data = [self.L_cmd_vel, self.R_cmd_vel]
                msg.data = [self.odrive_mode,self.L_cmd_vel/1000*4.8, self.R_cmd_vel/1000*4.8]
                self.control_publisher.publish(msg)
                self.get_logger().info(f"\033[1;32m {msg.data} \033[0m")
            else :
                pass
            # self.L_cmd_vel = 0.
            # self.R_cmd_vel = 0.
            # self.odrive_mode = 1. 
            
    def joy_pub(self) :
        msg = Float32MultiArray()
        self.odrive_mode = 1. 
        msg.data = [self.odrive_mode,self.joy_stick_data[0] * self.max_speed ,self.joy_stick_data[1] * self.max_speed ]
        self.control_publisher.publish(msg)
        
        
        
    ########################################
    ############ control preset ############
    ########################################
    def turn_left(self) :
        msg = Float32MultiArray()
        self.odrive_mode = 1. 
        self.R_joy = self.max_speed * 0.5
        self.L_joy = - self.max_speed * 0.5
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def turn_right(self) :
        msg = Float32MultiArray()
        self.odrive_mode = 1. 
        self.R_joy = - self.max_speed * 0.5
        self.L_joy = self.max_speed * 0.5
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def go(self) :
        msg = Float32MultiArray()
        self.odrive_mode = 1. 
        self.R_joy = self.max_speed * 0.5
        self.L_joy = self.max_speed * 0.5
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
    
    def back(self) :
        msg = Float32MultiArray()
        self.odrive_mode = 1. 
        self.R_joy = - self.max_speed * 0.5
        self.L_joy = - self.max_speed * 0.5
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
        
    def stop(self) :
        msg = Float32MultiArray()
        self.odrive_mode = 1. 
        self.R_joy = 0.
        self.L_joy = 0.
        msg.data = [self.odrive_mode,self.L_joy ,self.R_joy ]
        self.control_publisher.publish(msg)
        
            
            
        

def main(args=None):
    rclpy.init(args=args)
    node = CompleteJoyDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()