import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

#plane=9
robot_name="mst110cr_2"
use_namespace=True

def generate_launch_description():
    
    mst110cr_description_dir=get_package_share_directory("mst110cr_description")
    mst110cr_navigation_dir=get_package_share_directory("mst110cr_navigation")
    gnss_localizer_ros2 = get_package_share_directory("gnss_localizer_ros2")
    # mst110cr_unity_dir=get_package_share_directory("mst110cr_unity")
    xacro_model = os.path.join(mst110cr_description_dir, "urdf", "mst110cr.xacro")

    gnss_localizer_ros2_launch_file_path=os.path.join(gnss_localizer_ros2, "launch","gnss_localizer_ros2.py")
    ekf_localization_launch_file_path=os.path.join(mst110cr_navigation_dir,"launch","ekf_localization.launch.py")
    mst110cr_navigation_launch_file_path=os.path.join(mst110cr_navigation_dir,"launch","mst110cr_navigation.launch.py")
    mst110cr_standby_rviz_file = os.path.join(mst110cr_navigation_dir, "rviz2", "mst110cr_standby.rviz")

    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mst110cr_navigation_launch_file_path),
        ),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name),

            DeclareLaunchArgument('robot_name', default_value=robot_name),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=['--x','21395.178', 
                           '--y','14034.450', 
                           '--z','28.552', 
                           '--roll','0', 
                           '--pitch','0', 
                           '--yaw','0', 
                           '--frame-id', 'world',
                           '--child-frame-id', 'map']),
            
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[params]
            ),
            
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["--display-config", mst110cr_standby_rviz_file]),
        ]),
    ])