from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import launch_ros
import os

def generate_launch_description():
    master_pkg_share = launch_ros.substitutions.FindPackageShare(package='master_pkg').find('master_pkg')
    default_model_path = os.path.join(master_pkg_share, 'master.urdf')
    
    return LaunchDescription([
        # Start the robot model:
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[default_model_path]
        ),
        Node(
            package='basler_camera',
            executable='basler_camera',
            name='camera_provider',
            output='screen',
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            output='screen',
            remappings=[
                ('/image_rect', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
            ],
        ),
        Node(
            package='debugging_pkg',
            executable='apriltag_visualizer',
            name='apriltag_visualizer',
            output='screen',
        ),
    ])