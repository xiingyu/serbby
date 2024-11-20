import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

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
        ###align setting end
        
        
        ###end setting###
        
        
            
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
        ##################
        self.get_logger().info(f'init clear')
        
        
    def cam_cap(self) :
        ret, self.c920 = self.cap.read()
        
        if not ret :
            self.get_logger().info(f'c920 capture fail')
        
            
        
        

    def depth_cap(self):
        distance_msg = Float64()
        
        frames = self.pipeline.wait_for_frames()
        
        aligned_frames = self.align.process(frames)
        
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())
        
        
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
        
        
    def img_show(self) :
        cv2.imshow('color', self.color_image)
        # cv2.imshow('depth', self.depth_image)
        cv2.imshow('color', self.annotated_img)
        cv2.imshow('c920', self.c920)
        cv2.waitKey(1)

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