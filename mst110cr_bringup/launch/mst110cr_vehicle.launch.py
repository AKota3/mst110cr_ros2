import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
import xacro

robot_name="mst110cr"

def generate_launch_description():

    mst110cr_com3_ros_dir = get_package_share_directory("mst110cr_com3_ros")
    
    mst110cr_com3_ros_launch_file = os.path.join(mst110cr_com3_ros_dir, 'launch', 'mst110cr_com3_ros.launch.py')
    return LaunchDescription([

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mst110cr_com3_ros_launch_file),
        ),
    ])