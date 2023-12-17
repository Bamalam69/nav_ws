import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point
from sensor_msgs.msg import Image

from typing_extensions import Self

from rclpy.qos import QoSProfile, QoSHistoryPolicy, ReliabilityPolicy, DurabilityPolicy, QoSPolicyEnum

from cv_bridge import CvBridge
import cv2@id:ms-python.python
import cv2.typing as cvt

import numpy as np

class AprilTagVisualizer(Node):
    def __init__(self: Self):
        super().__init__('apriltag_visualizer')

        # --- Initialize private members: --- #
        self.last_image_frame: cvt.MatLike | None = None
        self.cv_bridge = CvBridge()

        # --- Initialize subscriptions: --- #
        # Default profile...
        default_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.window = cv2.namedWindow("Visuals", cv2.WINDOW_NORMAL)

        # TODO: Get camera intrinsics from camera_info topic:
        # TODO: Link all subcriptions into one callback... (Perhaps not the camera info for intrinsics though??)

        # Need to visualize the image, and the detections on the image:
        # Subscribe to camera feed:
        self.camera_feed = self.create_subscription(
            Image,
            '/my_camera/pylon_ros2_camera_node/image_raw',
            self.on_image,
            qos_profile=default_qos
        )

        # Subscribe to detections:
        self.detection_lister = self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self.on_detection,
            qos_profile=default_qos
        )

        self.visual = self.create_publisher(
            Image,
            "/visuals",
            qos_profile=default_qos
        )

        self.get_logger().info("Initialized and subscribed. Node Ready.")
    
    def decompose_homography(self: Self, homography: np.ndarray, camera_intrinsics: TYPE) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        homography = np.array(homography).reshape((3, 3))# TODO: Check if necessary...
        homography /= homography[2, 2]  # normalize homography

        # camera intrinsic parameters
        # focal_length = 700  # replace with your camera's focal length in pixels
        # principal_point = (320, 240)  # replace with your camera's principal point in pixels

        camera_matrix = np.array([
            [camera_intrinsics.focal_length, 0, camera_intrinsics.principal_point[0]],
            [0, camera_intrinsics.focal_length, camera_intrinsics.principal_point[1]],
            [0, 0, 1]
        ])

        # decompose homography
        _, rotation_matrix, translation_vector, _ = cv2.decomposeHomographyMat(homography, camera_matrix)

        # calculate distance
        distance = np.linalg.norm(translation_vector[2])
        self.get_logger().info(f"Distance to Apriltag: {distance}")

    def on_image(self: Self, data: Image):
        # self.get_logger().info("Recieved image!")
        self.last_image_frame = self.cv_bridge.imgmsg_to_cv2(data)
        # self.get_logger().info(f"Image size: {self.last_image_frame.shape}")

    def on_detection(self: Self, data: AprilTagDetectionArray):
        # self.get_logger().info("Recieved detection!")
        if self.last_image_frame is None:
            return
        
        # Display the corner on the image:
        if len(data.detections) == 0:
            # self.get_logger().info("No detections!")
            return
        
        display_img = self.last_image_frame.copy()
        display_img = cv2.cvtColor(display_img, cv2.COLOR_GRAY2BGR)

        detection: AprilTagDetection
        for detection in data.detections:
            corner: Point
            detection.homography
            for corner in detection.corners:
                self.get_logger().info(f"Corner: {corner}")
                cv2.drawMarker(display_img,
                               (int(corner.x), int(corner.y)),
                                 (0, 0, 255),
                                   markerSize=200,
                                     markerType=cv2.MARKER_DIAMOND
                                     )
        cv2.imshow("Visuals", display_img)
        cv2.waitKey(1)
        self.get_logger().info("Publishing image!")
        # self.visual.publish(self.cv_bridge.cv2_to_imgmsg(display_img))

def main(args=None):
    rclpy.init(args=args)

    apriltag_vis = AprilTagVisualizer()
    rclpy.spin(apriltag_vis)

    apriltag_vis.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()