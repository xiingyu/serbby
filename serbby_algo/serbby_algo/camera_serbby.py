import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float32MultiArray, String, Bool

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_srvs.srv import SetBool

import time

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

class PersonDistancePub(Node):
    def __init__(self):
        super().__init__('Image_catcher')
        qos_profile = QoSProfile(depth=10)
        
        
        #### params ####
        self.img_size_x = 640
        self.img_size_y = 480
        self.frame_rate = 15
        
        
        

        ##depth setting
        self.depth_frame_pub = self.create_publisher(Image, 'depth_data', qos_profile)
        self.color_frame_pub = self.create_publisher(Image, 'color_data', qos_profile)
        self.distance_data_pub = self.create_publisher(Float64, 'distance_data', qos_profile)
        self.cam_flagger = self.create_publisher(
            Bool, 
            'cam_flag_topic', 
            10)
        self.abs_control_pub = self.create_publisher(
            Float32MultiArray, 
            'abs_control',
            qos_profile)
        
        self.robot_state_pub = self.create_publisher(String, 'robot_state', qos_profile)
        self.goal_sub = self.create_subscription(Float32MultiArray,'goal_serbby',self.goal_sub_callback, qos_profile)
        self.robot_state_sub = self.create_subscription(String,'robot_state',self.robot_state_sub_callback,qos_profile)
        self.main_state_sub = self.create_subscription(String, 'main_state',self.main_state_sub_callback,qos_profile)
        self.move_num_pub = self.create_publisher(String, '/num',10)
        
        self.state_sub = self.create_subscription(String, 'state', self.state_callback, qos_profile)
        
        self.arm_control = self.create_publisher(
            String,
            'arm_control',
            qos_profile)
        self.control_publisher = self.create_publisher(
            Float32MultiArray, 
            'auto_control', 
            qos_profile)
        
        self.auto_controller = self.create_publisher(
            Float32MultiArray,
            'wheel_command',
            qos_profile)
        
        
        self.serial_read_data = self.create_subscription(
            String, 
            'read_rs485', 
            self.serial_read,
            qos_profile)

        self.odrive_direct_control = self.create_publisher(
            Float32MultiArray,
            'Odrive_control',
            qos_profile)
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, self.img_size_x, self.img_size_y, rs.format.z16, self.frame_rate)
        self.config.enable_stream(rs.stream.color, self.img_size_x, self.img_size_y, rs.format.bgr8, self.frame_rate)

        depth_profile = self.pipeline.start(self.config)
        
        ###align setting
        depth_sensor = depth_profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        
        #
        
        
        frames = self.pipeline.wait_for_frames()
        aligned_depth_frame = frames.get_depth_frame()
        depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
        ###align setting end
        
        
        ###end setting###
        
        
        
        #### aruco params ####
        
        self.declare_parameter('type', 'DICT_5X5_100')
        aruco_type = self.get_parameter('type').get_parameter_value().string_value

        if ARUCO_DICT.get(aruco_type, None) is None:
            self.get_logger().error(f"ArUCo tag type '{aruco_type}' is not supported")
            return

        self.aruco_dict_type = ARUCO_DICT[aruco_type]
        fx = depth_intrinsics.fx
        fy = depth_intrinsics.fy
        cx = depth_intrinsics.ppx
        cy = depth_intrinsics.ppy
        print(fx, fy, cx, cy)

        self.k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        self.d = np.zeros((5, 1), dtype=np.float32)
        
        
        
        
            
        ###yolo setting###
        self.model = YOLO('yolov8n.yaml')
        self.model = YOLO('yolov8n.pt')
        
        self.cap = cv2.VideoCapture("/dev/c920")

        # self.timer1 = self.create_timer(1/self.frame_rate, self.cam_cap)
        self.timer2 = self.create_timer(1/self.frame_rate, self.depth_cap)
        self.timer3 = self.create_timer(1/self.frame_rate, self.img_show)
        self.timer4 = self.create_timer(1/self.frame_rate, self.camera_positioning)
        self.cvbrid = CvBridge()
        
        ### image init ###
        
        self.color_image = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.depth_image = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.c920 = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.annotated_img = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.drawed_frame = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        ##################
        
        
        ### param init ###
        self.goal_num = 0
        self.robot_state = "idle"
        self.main_state = "idle"
        self.substate = "aruco"
        # aruco     # aruco
        # lift up 
        # gogo      # liftdown
        # back      # back
        
        self.read_data = "nothing"
        self.center_x = 0
        self.center_y = 0
        ###
        ### flag init ###
        
        self.update_frame_flag = True
        self.bottom_aruco = False
        self.yawing_degree = 0.
        
        ###
        self.robot_width = 560 #mm
        self.wheel_size = 175 #mm
        
        
        self.get_logger().info(f'init clear')
    
    def serial_read(self,msg) :
        self.read_data = msg.data
        self.get_logger().info(f"\033[1;32m 이동 상태: {self.read_data} \033[0m")
        
            
        
    ##aruco marker는,, home이 1
    # 이제 1번테이블은 상판이 10, 바닥이 11  depth가 +1인거
    #     2번테이블은 상판이 20, 바닥이 21    
    def move_num_clear(self, msg) :
        self.goal_num = int(msg.data)
        
    def state_callback(self, msg) :
        self.get_logger().info(f"\033[1;32m 이동 상태: {msg.data} \033[0m")
        #이걸 input하는데에서 받아야됨. 이거 삭제 예정
        # data = msg.data
        # if (data == "도착") & (self.robot_state == "got_goal") :
        #     self.robot_state = 'arrive'
        #     self.get_logger().info(f"\033[1;32m 이동 상태: {msg.data} \033[0m")
        # elif (data == "도착") & (self.robot_state == "go_home") :
        #     self.robot_state = 'idle'
        #     self.get_logger().info(f"\033[1;32m 이동 상태: {msg.data} \033[0m")
        # self.get_logger().info(f"이동 상태: {msg.data}")
        # ## data lists
        # 목표 전송 실패
        # 목표 전송 성공
        # 도착
        # 실패
        # 에러
        return
        
        
    def camera_positioning(self) :
        arm_msg = String()
        arm_cmd = String()
        cnt_data = Float32MultiArray()
        cam_flagger = Bool()
        
        ## if robot state ~~이런거 해야됨
        if self.robot_state == "arrive" or  self.robot_state=="putting" or self.robot_state == "taking":
            ## setting
            #책상의 aruco 찾고 뒤로 빼고 자리 찾고 넣어야함.
            # print("first if passed")
            if self.main_state == "setting" :
                # print("main state is setting")
                # self.update_frame_flag = False
                gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
                aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)
                parameters = cv2.aruco.DetectorParameters_create()
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                print(ids)
                if ids is not None and len(ids) > 0:
                    # print("id is exist")
                    # msg.data = "s"
                    # self.arm_control.publish(msg)
                    for i in range(len(ids)):
                        # print("ive get in for")
                        # if ids[i][0] == (self.goal_num *10 + 1):  # ID가 11인 경우에만 처리
                        if True :
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.k, self.d)
                            cv2.aruco.drawDetectedMarkers(self.color_image, corners)
                            self.color_image = self.draw_axis(self.color_image, self.k, self.d, rvec, tvec, 0.01)
                            # print("pass aruco draw")
                            
                            
                            corner_points = corners[i][0]
                            self.center_x = int(np.mean(corner_points[:, 0]))  # x 좌표의 평균
                            self.center_y = int(np.mean(corner_points[:, 1]))  # y 좌표의 평균
                else :      
                    self.center_x = 0
                    self.center_y = 0
                            # print(self.center_x)
                # self.update_frame_flag = True
                if self.substate == "aruco" :
                    
                    if (self.center_x <= (self.img_size_x * 0.9)) & (self.center_x >= (self.img_size_x * 0.1)) :
                        if (self.center_y <= (self.img_size_y * 0.7)) & (self.center_y >= (self.img_size_y * 0.5)) :
                            if abs(self.yawing_degree) > 1.8 :
                                drive_msg = Float32MultiArray()
                                movement_distance = self.robot_width * np.pi / 180 * self.yawing_degree
                                L_rotate = - (movement_distance /self.wheel_size / np.pi)
                                R_rotate = (movement_distance /self.wheel_size / np.pi)
                                drive_msg.data = [2., L_rotate, R_rotate]
                                self.abs_control_pub.publish(drive_msg)
                                self.get_logger().info(f"\033[1;38m {drive_msg.data[1]}, {drive_msg.data[2]} \033[0m")
                        
                                cur_time = time.time()
                                self.delay(cur_time, 5)
                            else :
                                drive_msg = Float32MultiArray()
                                # self.stop()
                                # self.get_logger().info("\033[1;32m center_x is in range \033[0m")
                        
                                # cnt_data.data = [1.,1000/4.8 *2  ,1000/4.8 *2]
                                # self.control_publisher.publish(cnt_data)
                                cur_time = time.time()
                                # # while (time.time()- cur_time) < 3 :
                                # #     cnt_data.data = [1.,1000/4.8 *2  ,1000/4.8 *2]
                                # #     self.control_publisher.publish(cnt_data)
                                # #     time.sleep(0.1)
                                
                                # cnt_data.data = [1.,1000/4.8 *2  ,1000/4.8 *2]
                                # self.control_publisher.publish(cnt_data)
                                drive_msg.data = [2., 4.,4.]
                                self.abs_control_pub.publish(drive_msg)
                                drive_msg.data = [2., 4.,4.]
                                self.abs_control_pub.publish(drive_msg)
                                drive_msg.data = [2., 4.,4.]
                                self.abs_control_pub.publish(drive_msg)
                                    
                                self.delay(cur_time, 3)
                                # print("before sleep")
                                # time.sleep(8)
                                # print("after sleep")
                                arm_cmd.data = "c"
                                self.arm_control.publish(arm_cmd)
                                self.substate = "liftdown"
                                self.get_logger().info("change substate to liftdown")
                                
                                cnt_data.data = [1., 0. ,0.]
                                self.control_publisher.publish(cnt_data)
                            
                        elif self.center_y >= (self.img_size_y * 0.6) : 
                            cnt_data = Float32MultiArray()
                            cnt_data.data = [1.,1000/4.8 *2  ,1000/4.8 *2]
                            self.control_publisher.publish(cnt_data)
                            
                        elif self.center_y <= (self.img_size_y * 0.4) : 
                            cnt_data = Float32MultiArray()
                            cnt_data.data = [1.,-1000/4.8 *2  ,-1000/4.8 *2]
                            self.control_publisher.publish(cnt_data)
                        else :
                            pass
                    else :
                        self.get_logger().info(f"\033[1;31m center_x is out of range : {self.center_x} \033[0m")
                elif self.substate == "liftdown" :
                    self.get_logger().info(f"\033[1;36m substate : {self.substate}  read_date : {self.read_data} \033[0m")
                    if self.read_data and self.read_data[0] != "o" :
                        arm_cmd.data = "d"
                        self.arm_control.publish(arm_cmd)
                        
                    elif self.read_data and self.read_data[0] == "o" :
                        arm_cmd.data = "s"
                        self.arm_control.publish(arm_cmd)
                        cur_time = time.time()
                        self.delay(cur_time, 3)
                        
                        arm_cmd.data = "d"
                        self.arm_control.publish(arm_cmd)
                        cur_time = time.time()
                        self.delay(cur_time, 3)
                        
                        
                        arm_cmd.data = "s"
                        self.arm_control.publish(arm_cmd)
                        self.read_data = "nothing"
                        
                        self.substate = "back"
                elif self.substate == "back" :
                    
                    cnt_data = Float32MultiArray()
                    cnt_data.data = [1.,-1000/4.8 *2  ,-1000/4.8 *2]
                    self.control_publisher.publish(cnt_data)
                    cur_time = time.time()
                    
                    while (time.time()- cur_time) < 5 :
                        cnt_data.data = [1.,-1000/4.8 *2  ,-1000/4.8 *2]
                        self.control_publisher.publish(cnt_data)
                        time.sleep(0.1)
                                
                    
                    
                    self.delay(cur_time, 5)
                    
                    cam_flagger.data = True
                    self.cam_flagger.publish(cam_flagger)
                    self.get_logger().info("\033[1;36m cam_flagger True is sent \033[0m")
                
                    
            ## decay
            #상판의 aruco 찾고 자리 찾고 넣어야함.
            elif self.main_state == "decay" :
                self.update_frame_flag = False
                gray = cv2.cvtColor(self.c920, cv2.COLOR_BGR2GRAY)
                aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)
                parameters = cv2.aruco.DetectorParameters_create()
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                if ids is not None and len(ids) > 0:
                    # msg.data = "s"
                    # self.arm_control.publish(msg)
                    for i in range(len(ids)):
                        if ids[i][0] == (self.goal_num *10 + 0):  # ID가 11인 경우에만 처리
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.k, self.d)
                            self.c920 = self.draw_axis(self.c920, self.k, self.d, rvec, tvec, 0.01)
                            cv2.aruco.drawDetectedMarkers(self.c920, corners)
                            
                            
                            corner_points = corners[i][0]
                            self.center_x = int(np.mean(corner_points[:, 0]))  # x 좌표의 평균
                            self.center_y = int(np.mean(corner_points[:, 1]))  # y 좌표의 평균
                self.update_frame_flag = True
                
            
            
            else :
                self.update_frame_flag = True
                self.get_logger().info( "\033[1;31m Invalid State \033[0m")
        else :
            self.update_frame_flag = True
        
        
    
    def draw_axis(self, frame, matrix_coefficients, distortion_coefficients, rvec, tvec, axis_length=0.05):
        axis_points_3d = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]])
        axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, matrix_coefficients, distortion_coefficients)
        axis_points_2d = np.array(axis_points_2d, dtype=np.int32)

        # frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[1].ravel()), (0, 0, 255), 2)
        # frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[2].ravel()), (0, 255, 0), 2)
        # frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[3].ravel()), (255, 0, 0), 2)
        
        rot_mat, _ = cv2.Rodrigues(rvec) # 회전 백터 rvec를 회전 행렬 rot_mat로 변환
        theta_x = np.arctan2(rot_mat[2, 1], rot_mat[2, 2]) # x축 회전 각도
        theta_y = np.arctan2(-rot_mat[2, 0], np.sqrt(rot_mat[2, 1] ** 2 + rot_mat[2, 2] ** 2)) # y축 회전 각도
        theta_z = np.arctan2(rot_mat[1, 0], rot_mat[0, 0]) # z축 회전 각도
        # print(axis_points_2d[0][0])
        # for i in range(len(axis_points_2d)) :
            
        #     print(f'axis_points_2d [{i}] : {axis_points_2d[i]}')
            
        # cv2.circle(frame, (self.center_x, self.center_y), 10, (0,0,255), -1 , cv2.LINE_AA)
        #x pitch
        #z roll
        #y yaw
        
        
        theta_x_deg = np.degrees(theta_x) # degree로 변환
        theta_y_deg = np.degrees(theta_y) # degree로 변환
        theta_z_deg = np.degrees(theta_z) # degree로 변환
        self.yawing_degree = theta_z_deg
        print(f'yawing degree is {self.yawing_degree}')

        cv2.putText(frame, f'X: {theta_x_deg:.2f}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) # x축 회전 각도 표시
        cv2.putText(frame, f'Y: {theta_y_deg:.2f}', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) # y축 회전 각도 표시
        cv2.putText(frame, f'Z: {theta_z_deg:.2f}', (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) # z축 회전 각도 표시
        return frame
    
    def delay(self, prev_times, seconds) : 
        while True :
            current_time = time.time()
            if ((current_time - prev_times) <= seconds) :
                cv2.imshow('color', self.color_image)
                # cv2.imshow('depth', self.depth_image)
                cv2.imshow('color', self.annotated_img)
                # cv2.imshow('c920', self.c920)
                cv2.imshow('drawn', self.drawed_frame)
                cv2.waitKey(1)
            else :
                return

    def img_show(self):
        
        cv2.imshow('color', self.color_image)
        # cv2.imshow('depth', self.depth_image)
        cv2.imshow('color', self.annotated_img)
        # cv2.imshow('c920', self.c920)
        cv2.imshow('drawn', self.drawed_frame)
        cv2.waitKey(1)

        
        
    def cam_cap(self) :
        if self.update_frame_flag :
            ret, self.c920 = self.cap.read()
            
            if not ret :
                self.get_logger().info(f'c920 capture fail')
        
            
        
        

    def depth_cap(self):
        if self.update_frame_flag :
            distance_msg = Float64()
            
            self.frames = self.pipeline.wait_for_frames()
            
            self.aligned_frames = self.align.process(self.frames)
            
            self.aligned_depth_frame = self.aligned_frames.get_depth_frame()
            self.color_frame = self.aligned_frames.get_color_frame()
            
            self.depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
            self.color_image = np.asanyarray(self.color_frame.get_data())
            
            
            gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None and len(ids) > 0:
                for i in range(len(ids)):
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.k, self.d)
                    self.color_image = self.draw_axis(self.color_image, self.k, self.d, rvec, tvec, 0.01)
                    cv2.aruco.drawDetectedMarkers(self.color_image, corners)
            
            
            ###yolo's turn
            
            result = self.model.predict(self.color_image, classes=[0., 67.], conf= 0.6, max_det = 1, verbose=False)
            self.annotated_img = result[0].plot()
            if len(result[0].boxes.cls) :
                # print(result[0].boxes.cls)
                object_xy = np.array(result[0].boxes.xywh.detach().numpy().tolist()[0], dtype='int')
                
                # print(object_xy[0], object_xy[1]) ### 640 by 480 
                
                distance = self.depth_image[object_xy[1]][object_xy[0]] * self.depth_scale    
                # print(f'distance between cam and object is {distance:.4f} meters')
                self.get_logger().info(f'distance between cam and object is {distance:.4f} meters')
                
                self.annotated_img = cv2.circle(self.annotated_img,((object_xy[0]),(object_xy[1])),10,(0,0,255), -1, cv2.LINE_AA)
                
                distance_msg.data = float(distance)
                self.distance_data_pub.publish(distance_msg)
                
            else :
                # self.get_logger().info(f'any object detected')
                pass
                
            
            
            
            
            ###end of yolo
            
            
            # grey_color = 0
            # depth_image_3d = np.dstack((depth_image, depth_image, depth_image)) #depth image is 1 channel, color is 3 channels
            # bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image) #need to search what is np.where.
            
            # self.depth_frame_pub.publish(self.cvbrid.cv2_to_imgmsg(bg_removed))
            # self.color_frame_pub.publish(self.cvbrid.cv2_to_imgmsg(self.annotated_img))
        
        
    
        
    def robot_state_sub_callback(self,msg) :
        self.robot_state = msg.data
        self.get_logger().info(f"\033[1;35m robot_state : {self.robot_state} \033[0m")
        
    def main_state_sub_callback(self,msg) :
        self.main_state = msg.data
        self.get_logger().info(f"\033[1;35m main_state : {self.main_state} \033[0m")
        
        
    def goal_sub_callback(self,msg) :
        return
        

def main(args=None):
    rclpy.init(args=args)
    node = PersonDistancePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()