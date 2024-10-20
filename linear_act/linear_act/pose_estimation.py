import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import time
import sys

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
        
        # self.declare_parameter('K_Matrix', '/home/dsj/ros2_ws/calibration_matrix.npy')
        # self.declare_parameter('D_Coeff', '/home/dsj/ros2_ws/distortion_coefficients.npy')
        self.declare_parameter('type', 'DICT_5X5_100')

        # calibration_matrix_path = self.get_parameter('K_Matrix').get_parameter_value().string_value
        # distortion_coefficients_path = self.get_parameter('D_Coeff').get_parameter_value().string_value
        aruco_type = self.get_parameter('type').get_parameter_value().string_value

        if ARUCO_DICT.get(aruco_type, None) is None:
            self.get_logger().error(f"ArUCo tag type '{aruco_type}' is not supported")
            return

        self.aruco_dict_type = ARUCO_DICT[aruco_type]
        # self.k = np.load(calibration_matrix_path)
        # self.d = np.load(distortion_coefficients_path)

        self.video = cv2.VideoCapture(8)
        time.sleep(2.0)
        self.timer = self.create_timer(0.1, self.pose_estimation_callback)
        
        
        # Example intrinsic parameters (replace with your actual camera parameters)
        fx, fy = 800, 800  # Focal lengths in pixels
        cx, cy = 320, 240  # Principal point (usually the image center)
# Focal Lengths:
# fx = 1031.16
# fy = 1030.69
# Principal Point:
# cx = 638.145
# cy = 360.15

        # Define the camera matrix
        self.k = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ], dtype=np.float32)

        # Define zero distortion coefficients
        self.d = np.zeros((5, 1), dtype=np.float32)

    def draw_axis(self, frame, matrix_coefficients, distortion_coefficients, rvec, tvec, axis_length=0.05):
        axis_points_3d = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]])
        axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, matrix_coefficients, distortion_coefficients)
        axis_points_2d = np.array(axis_points_2d, dtype=np.int32)
        
        frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[1].ravel()), (0, 0, 255), 2)
        frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[2].ravel()), (0, 255, 0), 2)
        frame = cv2.line(frame, tuple(axis_points_2d[0].ravel()), tuple(axis_points_2d[3].ravel()), (255, 0, 0), 2)

        rot_mat, _ = cv2.Rodrigues(rvec)
        theta_x = np.arctan2(rot_mat[2, 1], rot_mat[2, 2])
        theta_y = np.arctan2(-rot_mat[2, 0], np.sqrt(rot_mat[2, 1] ** 2 + rot_mat[2, 2] ** 2))
        theta_z = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])

        theta_x_deg = np.degrees(theta_x)
        theta_y_deg = np.degrees(theta_y)
        theta_z_deg = np.degrees(theta_z)

        cv2.putText(frame, f'X: {theta_x_deg:.2f}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(frame, f'Y: {theta_y_deg:.2f}', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(frame, f'Z: {theta_z_deg:.2f}', (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
        return frame

    def pose_estimation_callback(self):
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
                for corner in corners[i][0]:
                    x, y = corner.ravel()
                    cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
                    cv2.putText(frame, f'({x:.2f}, {y:.2f})', (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
        
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

