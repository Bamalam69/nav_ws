import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from sensor_msgs.msg import Image
from message_filters import TimeSynchronizer, Subscriber
from typing_extensions import Self
import cv2 as cv
from cv_bridge import CvBridge

class AprilTagVisualizer(Node):
    def __init__(self):
        super().__init__('apriltag_visualizer')
        self.get_logger().info('Hi from AprilTagVisualizer.')

        # Create a subscriber to the apriltag detections
        self.approximateTimeSynchronizer = TimeSynchronizer(
            [
                Subscriber(self, AprilTagDetectionArray, '/detections'),
                Subscriber(self, Image, '/camera/image_raw'),
            ],
            10,
            # 0.1
        )
        self.approximateTimeSynchronizer.registerCallback(self.listener_callback)

        self.apriltag_image = self.create_publisher(Image, '/apriltag_image', 10)

    def listener_callback(self: Self, apriltag_detection_array: AprilTagDetectionArray, image: Image):
        # Need to draw all tag corners on the image and republish the image:

        # Convert the image to a cv2 image
        cv_bridge = CvBridge()
        cv_image = cv_bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        # Need image to be in color
        cv_image = cv.cvtColor(cv_image, cv.COLOR_GRAY2BGR)

        # Draw the tag corners on the image
        detection: AprilTagDetection
        for detection in apriltag_detection_array.detections:
            # Get the corners of the tag
            corners = detection.corners

            # Draw the corners on the image
            for i in range(4):
                cv.line(cv_image, (int(corners[i].x), int(corners[i].y)), (int(corners[(i+1)%4].x), int(corners[(i+1)%4].y)), (0, 255, 0), 8)
        
        # Convert the cv2 image back to a ROS image (color)
        image = cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.apriltag_image.publish(image)

def main():
    rclpy.init()
    apriltag_visualizer = AprilTagVisualizer()
    
    try:
        rclpy.spin(apriltag_visualizer)
    except KeyboardInterrupt:
        pass

    apriltag_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
