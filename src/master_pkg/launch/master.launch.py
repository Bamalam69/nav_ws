from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import launch_ros
import os

def generate_launch_description():
    master_pkg_share = launch_ros.substitutions.FindPackageShare(package='master_pkg').find('master_pkg')
    default_model_path = os.path.join(master_pkg_share, 'master.urdf')

    robot_localization_share = launch_ros.substitutions.FindPackageShare(package='robot_localization').find('robot_localization')

    return LaunchDescription([
        # Start the robot model:
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[default_model_path]
        ),
        # Start the INS:
        # Node(
        #     package='inertiallabs_ins',
        #     executable='inertiallabs_ins',
        #     name='inertiallabs_ins',
        #     output='screen'
        # ),
        # TimerAction(
        #     period=7.5,
        #     actions=[
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_odom',
            output='screen',
            remappings=[
                ('/odometry/filtered', '/odometry/filtered_odom'),
            ],
            # arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[{
                'frequency': 60.0,

                'print_diagnostics': True,

                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',

                'imu0': '/imu/data',
                'imu0_remove_gravitational_acceleration': True,
                'imu0_config': [
                    False, False, False,
                    True,  True,  True,
                    False, False, False,
                    False, False, False,
                    False,  False,  False,
                ],
                'imu0_queue_size': 30,
                'imu0_relative': False,
                'imu0_differential': False,

                # 'odom0': '/odemetry/data',
                # 'odom0_config': [False,  False,  False,
                #                 False, False, False,
                #                 True, True, True,
                #                 False, False, False,
                #                 False, False, False,
                #     ],
                # 'odom0_differential': True,
                # 'odom0_queue_size': 10,
                # 'odom0_relative': True,

                # 'dynamic_process_noise_covariance' : True,
                # 'process_noise_covariance': [ 
                #     #x,     y,      z,      roll,   pitch,  yaw,    vx,     vy,     vz,     vroll,  vpitch, vyaw,   ax,     ay,     az
                #     0.05,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.05,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.06,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.1,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.1,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.1,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.04,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.02,   0.0,    0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,
                #     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.015
                # ],

            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_odom',
            output='screen',
            remappings=[
                ('/odometry/filtered', '/odometry/filtered_map'),
            ],
            parameters=[{
                'frequency': 60.0,

                'debug': True,
                'debug_out_file': '/home/nvidia/debug_ekf.txt',

                'print_diagnostics': True,

                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'map',

                'imu0': '/imu/data',
                'imu0_remove_gravitational_acceleration': True,
                'imu0_config': [
                    False, False, False,
                    True,  True,  True,
                    False, False, False,
                    False, False, False,
                    False,  False,  False,
                ],
                'imu0_queue_size': 30,
                'imu0_relative': False,
                'imu0_differential': False,

                # 'odom0': '/odemetry/data',
                # 'odom0_config': [False,  False,  False,
                #                 False, False, False,
                #                 True, True, True,
                #                 False, False, False,
                #                 False, False, False,
                #     ],
                # 'odom0_differential': True,
                # 'odom0_queue_size': 10,
                # 'odom0_relative': True,

                'dynamic_process_noise_covariance' : True,
                'process_noise_covariance': [ 
                    #x,     y,      z,      roll,   pitch,  yaw,    vx,     vy,     vz,     vroll,  vpitch, vyaw,   ax,     ay,     az
                    0.05,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.05,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.06,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.1,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.1,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.1,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.04,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.02,   0.0,    0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,
                    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.015
                ],

            }],
        ),
            # ]
        # ),
    ])