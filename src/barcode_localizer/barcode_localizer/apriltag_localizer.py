from threading import Thread

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from typing_extensions import Self

from sensor_msgs.msg import NavSatFix

import yaml

class ApriltagLocalizer(Node):
    def __init__(self):
        super().__init__('apriltag_localizer')

        self.get_logger().info("Initializing")

        self.tag_cache = yaml.load(open('~/tag_gps/test.yaml', 'r'))

        self.create_subscription(TFMessage, '/tf', self.on_tf_message, 10)
        self.approximated_gps = self.create_publisher(NavSatFix, '/approximated_gps', 10)

        self.get_logger().info("Finished initializing")

    def on_tf_message(self: Self, tf_message: TFMessage):
        self.get_logger().info(f"Got a tf message. Processing {len(tf_message.transforms)} transforms")
        
        tf: TransformStamped
        for tf in tf_message.transforms:
            self.get_logger().info(f'Got a tf from {tf.header.frame_id} to {tf.child_frame_id}')

            # With this tag, look up its GPS coordinates and publish a transform from the camera to the tag:
            ## Firstly, get the tag ID:
            tag_id: int = int(tf.child_frame_id.split(':')[1])

            ## Secondly, look up the tag's GPS coordinates:
            tag_gps = self.tag_cache[tag_id]

            ## Thirdly, publish a transform from the camera to the tag:
            nav_sat_fix_msg = NavSatFix()
            nav_sat_fix_msg.header.frame_id = 'camera'
            nav_sat_fix_msg.header.stamp = self.get_clock().now().to_msg()
            nav_sat_fix_msg.latitude = tag_gps['latitude']
            nav_sat_fix_msg.longitude = tag_gps['longitude']
            nav_sat_fix_msg.altitude = tag_gps['altitude']

            self.approximated_gps.publish(nav_sat_fix_msg)

        self.get_logger().info("Here2")

def main(args=None):
    rclpy.init(args=args)

    node = ApriltagLocalizer()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()