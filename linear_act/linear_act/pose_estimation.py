import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import time
import pyrealsense2 as rs  # RealSense 라이브러리

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

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')

        self.declare_parameter('type', 'DICT_5X5_100')
        aruco_type = self.get_parameter('type').get_parameter_value().string_value

        if ARUCO_DICT.get(aruco_type, None) is None:
            self.get_logger().error(f"ArUCo tag type '{aruco_type}' is not supported")
            return

        self.aruco_dict_type = ARUCO_DICT[aruco_type]

        # RealSense pipeline 시작
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 24)
        self.pipeline.start(config)

        time.sleep(2.0)
        self.timer = self.create_timer(0.1, self.pose_estimation_callback)

        # Depth camera의 intrinsic parameters를 가져옴
        frames = self.pipeline.wait_for_frames()
        aligned_depth_frame = frames.get_depth_frame()
        depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
        fx = depth_intrinsics.fx
        fy = depth_intrinsics.fy
        cx = depth_intrinsics.ppx
        cy = depth_intrinsics.ppy
        print(fx, fy, cx, cy)

        # 카메라 매트릭스 정의
        self.k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

        # 왜곡 계수 설정 (실제 카메라에 맞게 수정 가능)
        self.d = np.zeros((5, 1), dtype=np.float32)

    def draw_axis(self, frame, matrix_coefficients, distortion_coefficients, rvec, tvec, axis_length=0.05):
        axis_points_3d = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]])
        axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, matrix_coefficients, distortion_coefficients)
        axis_points_2d = np.array(axis_points_2d, dtype=np.int32)

        frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[1].ravel()), (0, 0, 255), 2)
        frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[2].ravel()), (0, 255, 0), 2)
        frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[3].ravel()), (255, 0, 0), 2)

        return frame

    def pose_estimation_callback(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        ret, frame = self.video.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.k, self.d)
                frame = self.draw_axis(frame, self.k, self.d, rvec, tvec, 0.01)
                cv2.aruco.drawDetectedMarkers(frame, corners)

        cv2.imshow('Pose Estimation', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
