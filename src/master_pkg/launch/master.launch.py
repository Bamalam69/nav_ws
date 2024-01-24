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
        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform_node',
        #     output='screen',
        #     parameters=[{
        #         'magnetic_declination_radians': 0.3652888603,
        #         'yaw_offset': 0.0,
        #         'zero_altitude': False,
        #         # 'broadcast_utm_transform': True,
        #         # 'broadcast_utm_transform_as_parent_frame': False,
        #         # 'publish_filtered_gps': False,
        #         'use_odometry_yaw': False,
        #         'wait_for_datum': False,
        #         'publish_enu_pose': False,
        #     }],
        # ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_odom',
            output='screen',
            remappings=[
                ('/odometry/filtered', '/odometry/filtered_odom'),
            ],
            parameters=[{
                'frequency': 30.0,

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
                    True, True, True,
                    True,  True,  True,
                ],
                'imu0_queue_size': 30,
                'imu0_relative': False,
                'imu0_differential': False,

                'odom0': '/odemetry/data',
                'odom0_config': [False,  False,  False,
                                True, True, True,
                                True, True, True,
                                False, False, False,
                                False, False, False,
                    ],
                'odom0_differential': False,
                'odom0_queue_size': 10,
                'odom0_relative': True,

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
                'frequency': 30.0,

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
                'imu0_relative': True,
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
        # Start RViz2:
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', os.path.join(master_pkg_share, 'rviz', 'rviz_config.rviz')]
        # ),
    ])