import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
import cv_bridge
from geometry_msgs.msg import PoseStamped
from typing import List

from basler_lib.detector import Detector, Detection, Calibrator

class ApriltagDetector(Node):
    def __init__(self):
        super().__init__('ApriltagDetector')
        
        self.get_logger().info("Initializing detector")
        # TODO: Need to follow ROS2 standard way of providing camera info...
        # But for now, we'll just load the calibration file:
        self.get_logger().info("Loading calibration")
        self.calibrator = Calibrator.load_calibration('calibration_20.pickle')
        self.detector = Detector(tag_size_mm=0.15, calibrator=self.calibrator)
        
        # Subscribe to camera topic
        self.get_logger().info("Subscribing to camera topic")
        self.subscription = self.create_subscription(Image,
            '/camera/image_rect',
            self.listener_callback,
            10)

        ## Publishers:
        # A publisher for the calculated pose:
        self.pose_publisher = self.create_publisher(PoseStamped, '/apriltag/pose', 10)

        # A publisher for the image with the detected tags:
        self.image_publisher = self.create_publisher(Image, '/apriltag/image', 10)
        
    def listener_callback(self, msg):
        self.get_logger().info("Received image")

        # Convert from ros message to cv2 so we can work with it:
        img = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='mono8')

        # Detect the tag:
        tags: List[Detection] = self.detector.detect(img, self.calibrator)
        if len(tags) == 0:
            self.get_logger().info("No tags detected")
            return

        ### Publish the image with the overlay: TODO: All this should be under an
        ###     'if' statement to check if user wants debugging...
        # Firstly, construct the overlay image..
        overlay_img = self.detector.overlay_tags(img, tags, show=False)

        # Image is now in color, so let's display color:
        detected_msg = cv_bridge.CvBridge().cv2_to_imgmsg(overlay_img, encoding="rgb8")
        self.image_publisher.publish(detected_msg)

        # TODO: Now publish the pose information:
        self.get_logger().info("Publishing pose information")
        
        # Let's do this for one tag for now:
        tag = tags[0]
        print(f"Pose_T: {tag.pose_t}, \nPose_R: {tag.pose_R}")
        
        
def main(args=None):
    rclpy.init(args=args)

    apriltagDetector = ApriltagDetector()
    rclpy.spin(apriltagDetector)

    apriltagDetector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# print("RUNNING CALIBRATION")
# camera_matrix = camera.calibrate_camera(cam, 'images/calibrate*.jpg')
# print("COMPLETED CALIBRATION")

# tag_size = 0.04     # 4 cm
# # calibrator = Calibrator()
# print("Loading calibration")
# calibrator = Calibrator.load_calibration('calibration_20.pickle')
# print(str(calibrator))
# # calibrator.calibrate(filename_format='images/calibrate*.jpg')
# detector = Detector(tag_size)
# print("Detecting tags")
# tags = detector.detect(img, calibrator)

# if len(tags) > 0:
#     print(tags[0])
#     distance = max(max(tags[0].pose_t[0][-1], tags[0].pose_t[1][-1]), tags[0].pose_t[2][-1])
#     print(f"Distance is {distance:.2f}")
#     # calculate_distance(tags[-1], 11, 100)
# else:
#     print("No tags found!")