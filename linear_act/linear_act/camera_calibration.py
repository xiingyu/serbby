import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import os

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        self.declare_parameter('dir', '/home/dsj/ros2_ws/src/marker/marker/camera1/')
        self.declare_parameter('width', 7)
        self.declare_parameter('height', 10)
        self.declare_parameter('square_size', 0.015)
        self.declare_parameter('visualize', False)

        dirpath = self.get_parameter('dir').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        square_size = self.get_parameter('square_size').get_parameter_value().double_value
        visualize = self.get_parameter('visualize').get_parameter_value().bool_value

        ret, mtx, dist, rvecs, tvecs = self.calibrate(dirpath, square_size, visualize, width, height)

        self.get_logger().info(f'Calibration matrix:\n{mtx}')
        self.get_logger().info(f'Distortion coefficients:\n{dist}')
        np.save("calibration_matrix.npy", mtx)
        np.save("distortion_coefficients.npy", dist)

        if visualize:
            self.visualize_undistortion(dirpath, mtx, dist)

    def calibrate(self, dirpath, square_size, visualize, width, height):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((height*width, 3), np.float32)
        objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
        objp = objp * square_size
        objpoints = []
        imgpoints = []

        images = os.listdir(dirpath)

        for fname in images:
            img = cv2.imread(os.path.join(dirpath, fname))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)

            if visualize:
                cv2.imshow('img', img)
                cv2.waitKey(0)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        return [ret, mtx, dist, rvecs, tvecs]

    def visualize_undistortion(self, dirpath, mtx, dist):
        images = os.listdir(dirpath)

        for fname in images:
            img = cv2.imread(os.path.join(dirpath, fname))
            h, w = img.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

            # Undistort
            dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

            # Crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]

            cv2.imshow('undistorted_img', dst)
            cv2.waitKey(0)

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

