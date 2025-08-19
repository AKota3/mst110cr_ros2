cmake_minimum_required(VERSION 3.8)
project(bt_control_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(com3_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  sensor_msgs
  com3_msgs
  behaviortree_cpp_v3
  nav2_msgs
  tf2_geometry_msgs
  tf2
)

add_library(logger_node SHARED src/logger_node.cpp)
list(APPEND plugin_libs logger_node)

add_library(within_goal_radius_node SHARED src/within_goal_radius.cpp)
list(APPEND plugin_libs within_goal_radius_node)


add_library(rotate_fwdbwd_along_track_node SHARED src/rotate_fwdbwd_along_track_node.cpp)
list(APPEND plugin_libs logger_node rotate_fwdbwd_along_track_node within_goal_radius_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${plugin_libs})

ament_export_dependencies(${dependencies})

ament_package()
