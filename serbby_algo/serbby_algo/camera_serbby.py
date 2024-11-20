import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float32MultiArray, String

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

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
        self.goal_sub = self.create_subscription(
            Float32MultiArray,
            'goal_serbby',
            self.goal_sub_callback,
            qos_profile)
        self.robot_state = self.create_subscription(
            String,
            'robot_state',
            self.goal_sub_callback,
            qos_profile)
        self.robot_state = self.create_subscription(
            String,
            'main_state',
            self.goal_sub_callback,
            qos_profile)
        
        self.state_sub = self.create_subscription(String, 'state', self.state_callback, qos_profile)
        

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

        self.timer1 = self.create_timer(1/self.frame_rate, self.cam_cap)
        self.timer2 = self.create_timer(1/self.frame_rate, self.depth_cap)
        self.timer3 = self.create_timer(1/self.frame_rate, self.img_show)
        self.cvbrid = CvBridge()
        
        ### image init ###
        
        self.color_image = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.depth_image = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.c920 = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.annotated_img = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.drawed_frame = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        ##################
        
        
        ### param init ###
        
        self.robot_state = "idle"
        self.main_state = "idle"
        self.main_algo = "idle"
        
        ###
        ### flag init ###
        
        self.bottom_aruco = False
        
        ###
        
        
        
        self.get_logger().info(f'init clear')
        
        
        
        
    def state_callback(self, msg) :
        self.get_logger().info(f"이동 상태: {msg.data}")
        ## data lists
        # 목표 전송 실패
        # 목표 전송 성공
        # 도착
        # 실패
        # 에러
        
        ## if robot state ~~이런거 해야됨
        
        
        
        
        
        
    
    def draw_axis(self, frame, matrix_coefficients, distortion_coefficients, rvec, tvec, axis_length=0.05):
        axis_points_3d = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]])
        axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, matrix_coefficients, distortion_coefficients)
        axis_points_2d = np.array(axis_points_2d, dtype=np.int32)

        # frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[1].ravel()), (0, 0, 255), 2)
        # frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[2].ravel()), (0, 255, 0), 2)
        # frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[3].ravel()), (255, 0, 0), 2)

        return frame

    def img_show(self):
        
        cv2.imshow('color', self.color_image)
        # cv2.imshow('depth', self.depth_image)
        cv2.imshow('color', self.annotated_img)
        cv2.imshow('c920', self.c920)
        cv2.imshow('drawn', self.drawed_frame)
        cv2.waitKey(1)

        
        
    def cam_cap(self) :
        ret, self.c920 = self.cap.read()
        
        if not ret :
            self.get_logger().info(f'c920 capture fail')
        
            
        
        

    def depth_cap(self):
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
        
        result = self.model.predict(self.color_image, classes=[0., 67.], conf= 0.6, max_det = 1)
        self.annotated_img = result[0].plot()
        if len(result[0].boxes.cls) :
            print(result[0].boxes.cls)
            object_xy = np.array(result[0].boxes.xywh.detach().numpy().tolist()[0], dtype='int')
            
            # print(object_xy[0], object_xy[1]) ### 640 by 480 
            
            distance = self.depth_image[object_xy[1]][object_xy[0]] * self.depth_scale    
            # print(f'distance between cam and object is {distance:.4f} meters')
            self.get_logger().info(f'distance between cam and object is {distance:.4f} meters')
            
            self.annotated_img = cv2.circle(self.annotated_img,((object_xy[0]),(object_xy[1])),10,(0,0,255), -1, cv2.LINE_AA)
            
            distance_msg.data = float(distance)
            self.distance_data_pub.publish(distance_msg)
            
        else :
            self.get_logger().info(f'any object detected')
            
        
        
        
        
        ###end of yolo
        
        
        # grey_color = 0
        # depth_image_3d = np.dstack((depth_image, depth_image, depth_image)) #depth image is 1 channel, color is 3 channels
        # bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image) #need to search what is np.where.
        
        # self.depth_frame_pub.publish(self.cvbrid.cv2_to_imgmsg(bg_removed))
        # self.color_frame_pub.publish(self.cvbrid.cv2_to_imgmsg(self.annotated_img))
        
        
        
        
    def goal_sub_callback(self) :
        
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