from tag_interfaces.srv import QueryTagPose

import rclpy
from rclpy.node import Node
import yaml

from numpy import random

from geometry_msgs.msg import PoseStamped

class TagService(Node):
    def __init__(self):
        super().__init__('tag_service')

        # Open local yaml for reading saved tage info:
        # self.tag_file = yaml.load()

        # Setup service:
        self.srv = self.create_service(QueryTagPose, 'query_tag_pose', self.query_tag_pose_callback)

        self.get_logger().info("Tag service initialized.")

    def query_tag_pose_callback(self, request: int, response: QueryTagPose.Response):
        # Get the tag pose from the yaml file:
        # tag_pose = self.tag_file[request]

        # Fill out the response:
        # response.header.frame_id = "camera"
        # response.header.stamp = self.get_clock().now().to_msg()
        # response.pose.position.x = tag_pose['x']
        # response.pose.position.y = tag_pose['y']
        # response.pose.position.z = tag_pose['z']
        # response.pose.orientation.x = tag_pose['qx']
        # response.pose.orientation.y = tag_pose['qy']
        # response.pose.orientation.z = tag_pose['qz']
        # response.pose.orientation.w = tag_pose['qw']

        response.pose.position.x = random.uniform(-1, 1)
        response.pose.position.y = random.uniform(-1, 1)
        response.pose.position.z = random.uniform(-1, 1)
        response.pose.orientation.x = random.uniform(-1, 1)
        response.pose.orientation.y = random.uniform(-1, 1)
        response.pose.orientation.z = random.uniform(-1, 1)
        response.pose.orientation.w = random.uniform(-1, 1)

        # self.get_logger().info("Processed a query.")

        return response

def main(args=None):
    rclpy.init(args=args)

    node = TagService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()