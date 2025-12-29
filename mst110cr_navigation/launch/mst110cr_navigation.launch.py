import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


use_autostart=True
use_respawn=True

def generate_launch_description():

    mst110cr_navigation_dir=get_package_share_directory('mst110cr_navigation')

    navigation_parameters_yaml_file = os.path.join(mst110cr_navigation_dir, 'params', 'navigation_parameters.yaml')

    map_yaml_file=LaunchConfiguration('map', default=os.path.join(mst110cr_navigation_dir, 'map', 'map.yaml'))

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='mst110cr_2')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    default_nav_to_pose_bt_xml_arg = DeclareLaunchArgument('default_nav_to_pose_bt_xml', default_value="/params/mst110cr_navigate_to_pose_w_replanning_and_recovery.xml")
    use_navigation_xy_goal_tolerance_arg = DeclareLaunchArgument('navigation_xy_goal_tolerance', default_value='1.0')
    use_navigation_yaw_goal_tolerance_arg = DeclareLaunchArgument('navigation_yaw_goal_tolerance', default_value='0.15')
        
    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    default_nav_to_pose_bt_xml =  LaunchConfiguration('default_nav_to_pose_bt_xml')
    use_navigation_xy_goal_tolerance = LaunchConfiguration('navigation_xy_goal_tolerance')
    use_navigation_yaw_goal_tolerance = LaunchConfiguration('navigation_yaw_goal_tolerance')
    
    lifecycle_nodes_localization = [
                   'map_server']
    
    lifecycle_nodes_navigation = [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother']

    param_substitutions = {
    # map yaml
    'yaml_filename': map_yaml_file,

    # bt_navigator
    'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': [mst110cr_navigation_dir, default_nav_to_pose_bt_xml],
    'bt_navigator.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
    'bt_navigator.ros__parameters.odom_topic': [robot_name, '/odom_pose'],
    'bt_navigator.ros__parameters.use_sim_time': use_sim_time,

    # smoother server
    'smoother_server.ros__parameters.use_sim_time': use_sim_time, 

    # behavior_server
    'behavior_server.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
    'behavior_server.ros__parameters.local_frame': [robot_name, '/odom'],
    'behavior_server.ros__parameters.use_sim_time': use_sim_time,    

    # controller_server
    'controller_server.ros__parameters.odom_topic': [robot_name, '/odom_pose'],
    'controller_server.ros__parameters.use_sim_time': use_sim_time,
    'controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance': use_navigation_xy_goal_tolerance,
    'controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance': use_navigation_yaw_goal_tolerance,

    # behavior_server
    'behavior_server.ros__parameters.local_frame': [robot_name, '/odom'],
    'behavior_server.ros__parameters.robot_base_frame': [robot_name, '/base_link'],

    # waypoint follower
    'waypoint_follower.ros__parameters.use_sim_time': use_sim_time,

    # velocity_smoother
    'velocity_smoother.ros__parameters.odom_topic': [robot_name, '/odom_pose'],
    'velocity_smoother.ros__parameters.use_sim_time': use_sim_time,

    # map server
    'map_server.ros__parameters.use_sim_time': use_sim_time,
    
    # costmaps
    'local_costmap.local_costmap.ros__parameters.use_sim_time': use_sim_time,
    'local_costmap.local_costmap.ros__parameters.global_frame': [robot_name, '/odom'],
    'local_costmap.local_costmap.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
    'global_costmap.global_costmap.ros__parameters.use_sim_time': use_sim_time,
    'global_costmap.global_costmap.ros__parameters.robot_base_frame': [robot_name, '/base_link'],

    # ここにtoleranceの更新プログラムを追記
    }
    
    configured_params = RewrittenYaml(
            source_file=navigation_parameters_yaml_file,
            root_key= robot_name,
            param_rewrites=param_substitutions,
            convert_types=True)
    
    remappings_mst110cr_tf=[('/mst110cr/tf','tf'),
                         ('/mst110cr/tf_static', 'tf_static')]
    
    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg,
        default_nav_to_pose_bt_xml_arg,
        use_navigation_xy_goal_tolerance_arg,
        use_navigation_yaw_goal_tolerance_arg,

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name),

            Node(
                condition=IfCondition('true'),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, 
                            {
                                'autostart': use_autostart,
                                'use_sim_time':use_sim_time,
                            }
                            ],
                remappings=remappings_mst110cr_tf,
                output='screen'),
            
            #########################
            # Localization packages #
            #########################

            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_mst110cr_tf),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': use_autostart,
                             'node_names': lifecycle_nodes_localization}]),

            #######################
            # Navigation packages #
            #######################

            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_mst110cr_tf + [('cmd_vel', 'cmd_vel_nav')]
                ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_mst110cr_tf),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                remappings=remappings_mst110cr_tf),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_mst110cr_tf+[('mst110cr/goal_pose','goal_pose')]),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_mst110cr_tf),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings_mst110cr_tf +
                        [('cmd_vel', 'cmd_vel_nav'), 
                         ('cmd_vel_smoothed', 'cmd_vel')]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': use_autostart},
                            {'node_names': lifecycle_nodes_navigation}]),
            
        ])
    ])