import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    mst110cr_navigation_dir = get_package_share_directory("mst110cr_navigation")
    mst110cr_unity_dir = get_package_share_directory("mst110cr_unity")

    ekf_localization_launch_file_path = os.path.join(
        mst110cr_navigation_dir, "launch", "ekf_localization.launch.py"
    )
    mst110cr_navigation_launch_file_path = os.path.join(
        mst110cr_navigation_dir, "launch", "mst110cr_navigation.launch.py"
    )

    mst110cr_standby_rviz_file_path = os.path.join(
        mst110cr_unity_dir, "rviz2", "mst110cr_standby.rviz"
    )

    # ---- Launch Arguments ----
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='mst110cr')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_navigation_xy_goal_tolerance_arg = DeclareLaunchArgument('navigation_xy_goal_tolerance', default_value='1.0')
    use_navigation_yaw_goal_tolerance_arg = DeclareLaunchArgument('navigation_yaw_goal_tolerance', default_value='0.15')


    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_navigation_xy_goal_tolerance = LaunchConfiguration('navigation_xy_goal_tolerance')
    use_navigation_yaw_goal_tolerance = LaunchConfiguration('navigation_yaw_goal_tolerance')

    # ---- robot_description (xacro) ----
    xacro_file = PathJoinSubstitution([
        FindPackageShare('mst110cr_description'),
        'urdf',
        'mst110cr_for_sim.xacro'
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                xacro_file, ' ',
                'prefix:=', robot_name
            ]),
            value_type=str
        )
    }
    

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg,
        use_navigation_xy_goal_tolerance_arg,
        use_navigation_yaw_goal_tolerance_arg,

        GroupAction([

            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
                launch_arguments={
                    'robot_name': robot_name,
                    'use_namespace': 'false',
                    'use_sim_time': use_sim_time,
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mst110cr_navigation_launch_file_path),
                launch_arguments={
                    'robot_name': robot_name,
                    'use_namespace': 'false',
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': "/params/mst110cr_navigate_to_pose_w_replanning_and_recovery_for_sim.xml",
                    'navigation_xy_goal_tolerance': use_navigation_xy_goal_tolerance,
                    'navigation_yaw_goal_tolerance': use_navigation_yaw_goal_tolerance
                }.items(),
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=[
                    '--x','0','--y','0','--z','0',
                    '--roll','0','--pitch','0','--yaw','0',
                    '--frame-id','world',
                    '--child-frame-id','map'
                ]
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[robot_description]
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["--display-config", mst110cr_standby_rviz_file_path]),
        ]),
    ])