#!/usr/bin/env python3
"""
ArUco Marker Detection Node for Hand-Eye Calibration.

Subscribes to the camera image topic, detects ArUco markers using OpenCV,
publishes the annotated image AND the detected marker pose in the camera frame.

Outputs:
  /aruco_detected       - annotated image with markers drawn
  /aruco_marker_pose    - PoseStamped: marker pose in camera_optical frame
  /aruco_marker_tf      - TF: marker frame relative to camera_optical

For hand-eye calibration:
  - Record /joint_states for the hand (end-effector) pose
  - Record /aruco_marker_pose for the eye (camera) measurement
  - Solve AX = XB with >10 pose pairs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np


def rvec_tvec_to_pose(rvec: np.ndarray, tvec: np.ndarray):
    """Convert OpenCV rvec/tvec to position (xyz) + quaternion (xyzw)."""
    rot_mat, _ = cv2.Rodrigues(rvec)
    # Convert rotation matrix to quaternion
    # qx, qy, qz, qw
    m = rot_mat
    qw = np.sqrt(1.0 + m[0, 0] + m[1, 1] + m[2, 2]) / 2.0
    qx = (m[2, 1] - m[1, 2]) / (4.0 * qw)
    qy = (m[0, 2] - m[2, 0]) / (4.0 * qw)
    qz = (m[1, 0] - m[0, 1]) / (4.0 * qw)
    return tvec.flatten(), np.array([qx, qy, qz, qw])


class ArUcoDetector(Node):
    """ArUco detection with pose publishing for hand-eye calibration."""

    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('dictionary', 'DICT_ARUCO_ORIGINAL')
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('camera_topic', '/camera_sensor')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_frame', 'camera_link_optical')

        aruco_dict_name = self._get_str_param('dictionary', 'DICT_ARUCO_ORIGINAL')
        self.marker_size = self._get_float_param('marker_size', 0.15)
        camera_topic = self._get_str_param('camera_topic', '/camera_sensor')
        camera_info_topic = self._get_str_param(
            'camera_info_topic', '/camera_info'
        )
        self.camera_frame = self._get_str_param(
            'camera_frame', 'camera_link_optical'
        )

        # ArUco
        dict_id = getattr(cv2.aruco, aruco_dict_name, cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        aruco_params = cv2.aruco.DetectorParameters()
        # Use contour-based corner refinement — more accurate on anti-aliased
        # edges than the default CORNER_REFINE_SUBPIX, which can push corners
        # outward into the white margin of the marker texture.
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        aruco_params.cornerRefinementWinSize = 3
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10
        )

        # Publishers
        self.result_pub = self.create_publisher(Image, '/aruco_detected', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/aruco_marker_pose', 10
        )

        # TF broadcaster (publishes marker frame in camera_optical)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f'ArUco Detector ready: {aruco_dict_name}')
        self.get_logger().info(f'Camera frame: {self.camera_frame}')

    def _get_str_param(self, name: str, default: str) -> str:
        value = self.get_parameter(name).value
        return str(value) if value is not None else default

    def _get_float_param(self, name: str, default: float) -> float:
        value = self.get_parameter(name).value
        return float(value) if value is not None else default

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion: {e}')
            return

        corners, ids, _rejected = self.detector.detectMarkers(cv_image)
        annotated = cv_image.copy()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            if self.camera_matrix is not None and self.dist_coeffs is not None:
                for i, corner in enumerate(corners):
                    try:
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                            corner, self.marker_size,
                            self.camera_matrix, self.dist_coeffs,
                        )
                        cv2.drawFrameAxes(
                            annotated, self.camera_matrix, self.dist_coeffs,
                            rvec[0], tvec[0], self.marker_size * 0.5,
                        )
                        pos, quat = rvec_tvec_to_pose(
                            rvec[0].reshape(3, 1), tvec[0].reshape(3, 1)
                        )

                        # --- Publish PoseStamped ---
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = msg.header.stamp
                        pose_msg.header.frame_id = self.camera_frame
                        pose_msg.pose.position.x = float(pos[0])
                        pose_msg.pose.position.y = float(pos[1])
                        pose_msg.pose.position.z = float(pos[2])
                        pose_msg.pose.orientation.x = float(quat[0])
                        pose_msg.pose.orientation.y = float(quat[1])
                        pose_msg.pose.orientation.z = float(quat[2])
                        pose_msg.pose.orientation.w = float(quat[3])
                        self.pose_pub.publish(pose_msg)

                        # --- Publish TF ---
                        tf_msg = TransformStamped()
                        tf_msg.header.stamp = msg.header.stamp
                        tf_msg.header.frame_id = self.camera_frame
                        tf_msg.child_frame_id = f'aruco_marker_{ids[i][0]}'
                        tf_msg.transform.translation.x = float(pos[0])
                        tf_msg.transform.translation.y = float(pos[1])
                        tf_msg.transform.translation.z = float(pos[2])
                        tf_msg.transform.rotation.x = float(quat[0])
                        tf_msg.transform.rotation.y = float(quat[1])
                        tf_msg.transform.rotation.z = float(quat[2])
                        tf_msg.transform.rotation.w = float(quat[3])
                        self.tf_broadcaster.sendTransform(tf_msg)

                        self.get_logger().info(
                            f'Marker {ids[i][0]}: pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) in {self.camera_frame}',
                            throttle_duration_sec=1.0,
                        )
                    except Exception as e:
                        self.get_logger().debug(f'Pose error: {e}')

        # Publish annotated image
        try:
            result_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            result_msg.header = msg.header
            self.result_pub.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
