import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='mst110cr_2')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg,

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name
            ),

            Node(
                package='mst110cr_navigation',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
                parameters=[
                    {'odom_topic': 'odom_pose', 
                     'odom_frame': [robot_name, '/odom'],
                     'base_link_frame': [robot_name, '/base_link'],
                     'use_sim_time': use_sim_time,
                    },
                ]
            ),

            Node(
                package = 'd37pxi_navigation',
                executable = 'message_converter_odom',
                name = "message_converter_odom",
                output = "screen",
                parameters=[{'input_topic': "odom_pose",
                            'output_topic': 'fixed_odom_pose',
                            'use_sim_time': use_sim_time}],
            ),

            Node(
                package = 'mst110cr_navigation',
                executable = 'message_converter_gnss',
                name = "message_converter_gnss",
                output = "screen",
                parameters=[{'input_topic': 'global_pose',
                            'output_topic': 'fixed_global_pose',
                            'use_sim_time': use_sim_time}],
            ),

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                output="screen",
                remappings=[
                    ('odometry/filtered', 'odometry/global'),
                    ('odom0', 'odom_pose'),
                    ('odom1', 'gnss_odom'),
                ],
                parameters=[{
                    'debug': False,
                    'frequency': 10.0,
                    'transform_time_offset': 0.0,
                    'transform_timeout': 0.0,
                    'print_diagnostics': True,
                    'publish_acceleration': True,
                    'publish_tf': True,
                    'two_d_mode': True,
                    'map_frame': 'map',
                    'odom_frame': [robot_name, '/odom'],
                    'base_link_frame': [robot_name, '/base_link'],
                    'world_frame': 'map',
                    'use_sim_time': use_sim_time,

                    'odom0': 'fixed_odom_pose',
                    'odom0_config': [
                        True,  True,  False,
                        False, False, True,
                        False, False, False,
                        False, False, False,
                        False, False, False],
                    'odom0_differential': True,

                    'odom1': 'fixed_global_pose',
                    'odom1_config': [
                        True,  True,  True,
                        False, False, True,
                        False, False, False,
                        False, False, False,
                        False, False, False],
                    'odom1_differential': False,
                }]
            ),
        ])
    ])