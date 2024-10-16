import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Camera calibration parameters
        fx = 600
        fy = 600
        cx = 320
        cy = 240

        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        k1 = 0 # no distortion 
        k2 = 0
        k3 = 0
        p1 = 0
        p2 = 0
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

        # Define the marker size in meters 
        self.marker_length = 0.1

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            # pose estimation:
            for i, corner in enumerate(corners):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corner, self.marker_length, self.camera_matrix, self.dist_coeffs)

                print(f"Marker ID: {ids[i][0]}, Position: x={tvec[0][0][0]:.2f}, y={tvec[0][0][1]:.2f}, z={tvec[0][0][2]:.2f}")
                # note: z = depth from camera to marker, x = horiz dist from camera to marker, y = vertical dist 
                # orientation: front facing to marker 
                # draw axis: 
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

        cv2.imshow("Aruco Detector", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
