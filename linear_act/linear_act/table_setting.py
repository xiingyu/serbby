import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray, String
import numpy as np

import pyrealsense2 as rs
import cv2
import time

# img_size_x = 1920
# img_size_y = 1080
# dimg_size_x = 1280
# dimg_size_y = 720
# HFOV = 69   # degree
# VFOV = 42
# DFOV = 77
# focal_length = 0.00193  # m
# pixel_size_color = 0.0000014  # m
# pixel_size_depth = 0.000003  # m

frame_rate = 30
img_size_x = 640
img_size_y = 480

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
class TableSetting(Node):
    def __init__(self):
        super().__init__('table_setting')
        qos_profile = QoSProfile(depth=10)
        
        
        self.arm_control = self.create_publisher(
            String, 
            'arm_control', 
            qos_profile)
        
        
        #Encoder Publisher
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'Odrive_encoder', 
            qos_profile
        )

        self.declare_parameter('type', 'DICT_5X5_100')
        aruco_type = self.get_parameter('type').get_parameter_value().string_value

        if ARUCO_DICT.get(aruco_type, None) is None:
            self.get_logger().error(f"ArUCo tag type '{aruco_type}' is not supported")
            return

        self.aruco_dict_type = ARUCO_DICT[aruco_type]

        
        
        # self.pipeline = rs.pipeline()
        # config = rs.config()

        # config.enable_stream(rs.stream.depth, dimg_size_x, dimg_size_y, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, img_size_x, img_size_y, rs.format.bgr8, 30)

        # profile = self.pipeline.start(config)

        # RealSense pipeline 시작
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, img_size_x, img_size_y, rs.format.z16, frame_rate)
        config.enable_stream(rs.stream.color, img_size_x, img_size_y, rs.format.bgr8, frame_rate)
        profile=self.pipeline.start(config)
        
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        
        clipping_distance_in_meters = 1  # 1 meter
        clipping_distance = clipping_distance_in_meters / self.depth_scale
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        # depth_sensor = self.pipeline_profile.get_device().first_depth_sensor()
        # depth_stream = self.pipeline_profile.get_stream(rs.stream.depth) # Depth stream profile
        # intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
                
        # # 카메라 내부 파라미터 출력
        # print(f"fx: {intrinsics.fx}")
        # print(f"fy: {intrinsics.fy}")
        # print(f"cx: {intrinsics.ppx}")  # cx는 ppx로 나타남
        # print(f"cy: {intrinsics.ppy}")  # cy는 ppy로 나타남
        

        
        self.pose_estimate_timer = self.create_timer(1/frame_rate, self.pose_estimation_callback)

        # Depth camera의 intrinsic parameters를 가져옴
        frames = self.pipeline.wait_for_frames()
        aligned_depth_frame = frames.get_depth_frame()
        depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().get_intrinsics()
        
        fx = depth_intrinsics.fx
        fy = depth_intrinsics.fy
        cx = depth_intrinsics.ppx
        cy = depth_intrinsics.ppy
        print(fx, fy, cx, cy)

        # 카메라 매트릭스 정의
        self.k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

        # 왜곡 계수 설정 (실제 카메라에 맞게 수정 가능)
        self.d = np.zeros((5, 1), dtype=np.float32)
        
        self.center_x = 0
        self.center_y = 0
        
        
        msg = String()
        msg.data = "d"
        self.arm_control.publish(msg)
        self.get_logger().info(f'send message {msg.data}')
        time.sleep(4.0)
        
        
        
    def draw_axis(self, frame, matrix_coefficients, distortion_coefficients, rvec, tvec, axis_length=0.05):
        axis_points_3d = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]])
        axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, matrix_coefficients, distortion_coefficients)
        axis_points_2d = np.array(axis_points_2d, dtype=np.int32)

        frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[1].ravel()), (0, 0, 255), 2)
        frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[2].ravel()), (0, 255, 0), 2)
        frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[3].ravel()), (255, 0, 0), 2)
        
        rot_mat, _ = cv2.Rodrigues(rvec) # 회전 백터 rvec를 회전 행렬 rot_mat로 변환
        theta_x = np.arctan2(rot_mat[2, 1], rot_mat[2, 2]) # x축 회전 각도
        theta_y = np.arctan2(-rot_mat[2, 0], np.sqrt(rot_mat[2, 1] ** 2 + rot_mat[2, 2] ** 2)) # y축 회전 각도
        theta_z = np.arctan2(rot_mat[1, 0], rot_mat[0, 0]) # z축 회전 각도
        # print(axis_points_2d[0][0])
        # for i in range(len(axis_points_2d)) :
            
        #     print(f'axis_points_2d [{i}] : {axis_points_2d[i]}')
            
        cv2.circle(frame, (self.center_x, self.center_y), 10, (0,0,255), -1 , cv2.LINE_AA)
        #x pitch
        #z roll
        #y yaw
        
        
        theta_x_deg = np.degrees(theta_x) # degree로 변환
        theta_y_deg = np.degrees(theta_y) # degree로 변환
        theta_z_deg = np.degrees(theta_z) # degree로 변환

        cv2.putText(frame, f'X: {theta_x_deg:.2f}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) # x축 회전 각도 표시
        cv2.putText(frame, f'Y: {theta_y_deg:.2f}', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) # y축 회전 각도 표시
        cv2.putText(frame, f'Z: {theta_z_deg:.2f}', (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) # z축 회전 각도 표시

        return frame

    def pose_estimation_callback(self):
        msg = String()
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) > 0:
            # msg.data = "s"
            # self.arm_control.publish(msg)
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.k, self.d)
                drawed_frame = self.draw_axis(color_image, self.k, self.d, rvec, tvec, 0.01)
                cv2.aruco.drawDetectedMarkers(drawed_frame, corners)
                
                
                corner_points = corners[i][0]
                self.center_x = int(np.mean(corner_points[:, 0]))  # x 좌표의 평균
                self.center_y = int(np.mean(corner_points[:, 1]))  # y 좌표의 평균
                
            if self.center_y <= int(img_size_y * 0.45):
                msg.data = "u"
                self.arm_control.publish(msg)
                self.get_logger().info(f'send message {msg.data}')
            elif self.center_y >= int(img_size_y * 0.55):
                msg.data = "d"
                self.arm_control.publish(msg)
                self.get_logger().info(f'send message {msg.data}')
            else :
                msg.data = "s"
                self.arm_control.publish(msg)
                self.get_logger().info(f'send message {msg.data}')
                
                
                
            
            
                        
                
        else :
            msg.data = "u"
            # self.get_logger().info(f'type is {type(msg.data)}')
            self.arm_control.publish(msg)
            self.get_logger().info(f'send message {msg.data}')
            drawed_frame = color_image
            self.center_x = 0
            self.center_y = 0
            
        
            

        cv2.imshow('Pose Estimation', drawed_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = TableSetting()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
