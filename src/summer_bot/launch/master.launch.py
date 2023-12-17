from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["ros2", "launch", "pylon_ros2_camera_wrapper", "pylon_ros2_camera.launch.py",
                 "--ros-args", "-p", "frame_rate:=30"],
            output="screen"
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "apriltag_ros", "apriltag_node",
                  "--ros-args",
                    "-r", "image_rect:=/my_camera/pylon_ros2_camera_node/image_raw",
                    "-r", "camera_info:=/my_camera/pylon_ros2_camera_node/camera_info",
                    "--params-file", f"{get_package_share_directory('apriltag_ros')}/cfg/tags_Standard52h13.yaml"],
                      # "--log-level", "DEBUG"],
            output="screen"
        ),
        Node(
            package="summer_bot",
            executable="apriltag_visuals",
            name="apriltag_visuals",
            output="screen"
        )
    ])