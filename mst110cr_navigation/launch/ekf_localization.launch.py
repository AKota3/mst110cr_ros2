import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

robot_name="mst110cr_2"
use_namespace=True

def generate_launch_description():

    mst110cr_description_dir=get_package_share_directory("mst110cr_description")
    mst110cr_navigation_dir=get_package_share_directory("mst110cr_navigation")

    mst110cr_ekf_yaml_file = LaunchConfiguration('ekf_yaml_file', default=os.path.join(mst110cr_navigation_dir, 'config', 'mst110cr_ekf.yaml'))
    xacro_model = os.path.join(mst110cr_description_dir, "urdf", "mst110cr.xacro")
    
    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}



    return LaunchDescription([

        DeclareLaunchArgument('robot_name', default_value='mst110cr'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name),
            Node(
                package='mst110cr_navigation',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
                parameters=[
                    {
                        'odom_topic': 'odom_pose',
                        'odom_frame': robot_name + '/odom',
                        'base_link_frame': robot_name + '/base_link',
                        'use_sim_time': bool(LaunchConfiguration('use_sim_time'))
                    }
                ]
            ), 

            Node(
                package='mst110cr_navigation',
                executable='poseStamped2Odometry',
                name='poseStamped2ground_truth_odom',
                output="screen",
                parameters=[
                    {
                        'odom_header_frame': "world",
                        'odom_child_frame': robot_name + "/base_link",
                        'poseStamped_topic_name': "base_link_pose_from_gnss",
                        # 'poseStamped_topic_name': "global_pose",
                        'odom_topic_name': "gnss_odom",
                        'use_sim_time': bool(LaunchConfiguration('use_sim_time'))
                    }
                ]
            ),            

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                output="screen",
                remappings=[('odometry/filtered','odometry/global'),
                            ('odom0','/mst110cr_2/odom_pose'),
                            ('odom1','/mst110cr_2/gnss_odom')],
                parameters=[
                    mst110cr_ekf_yaml_file,
                            {
                                'debug': False,
                                'frequency': 10.0,
                                'transform_time_offset': 0.0,
                                'transform_timeout': 0.0,
                                'print_diagnostics': True,
                                'publish_acceleration': True,
                                'print_diagnostics': True,
                                'publish_tf': True,
                                'two_d_mode': True ,                                               
                                    'map_frame' : 'map',
                                'odom_frame' : robot_name + '/odom',
                                'base_link_frame' : robot_name + '/base_link',
                                'world_frame' : 'map',
                                'odom0' : '/mst110cr_2/odom_pose',
                                'odom0_config': [
                                    True,  True,  False,
                                    False, False, True,
                                    False, False, False,
                                    False, False, False,
                                    False, False, False],
                                'odom0_differential': True,
                                'odom1' : '/mst110cr_2/gnss_odom',
                                'odom1_config': [
                                    True,  True,  True,
                                    False, False, True,
                                    False, False, False,
                                    False, False, False,
                                    False, False, False],
                                'odom1_differential': False,
                                'use_sim_time': bool(LaunchConfiguration('use_sim_time'))
                            }
                ]
),
        ])
    ])