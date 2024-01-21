import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge
from rclpy.qos import qos_profile_sensor_data

from basler_lib import Camera, Calibrator
# from camera.detector import Detector

class BaslerCamera(Node):
    def __init__(self):
        super().__init__('BaslerCamera')

        self.calibrator = Calibrator.load_calibration('calibration_20.pickle')
        self.cam = Camera(self.calibrator, color=False, auto=False)
        self.bridge = cv_bridge.CvBridge()

        self.raw_image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.rectified_image_publisher = self.create_publisher(Image, '/camera/image_rect', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        timer_period = 0.01
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Publish raw image:
        img = self.cam.get_image()
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
        image_msg.header.frame_id = "camera"
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.raw_image_publisher.publish(image_msg)

        # Publish rectified image:
        # TODO: Currently not actually rectifying the image... :(
        # self.calibrator.undistort_image(img)
        msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
        self.rectified_image_publisher.publish(msg)

        # Also publish camera info:
        msg = CameraInfo()
        msg.header.frame_id = "camera"
        msg.header.stamp = image_msg.header.stamp
        msg.height = self.cam.height
        msg.width = self.cam.width
        msg.distortion_model = "plumb_bob"
        msg.d = self.cam.distortion_parameters
        msg.k = self.cam.intrinsics
        msg.r = self.cam.rectification
        msg.p = self.cam.projection_matrix
        self.camera_info_publisher.publish(msg)
        
        self.get_logger().info("Published images")

    def destroy_node(self):
        self.cam.close_camera()
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)

    baslerCamera = BaslerCamera()
    rclpy.spin(baslerCamera)

    baslerCamera.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# print(img.shape)
# cv2.namedWindow("Camera Test", cv2.WINDOW_NORMAL)
# cv2.imshow('Camera Test', img)

# cv2.waitKey(5000)

# cv2.destroyAllWindows()


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