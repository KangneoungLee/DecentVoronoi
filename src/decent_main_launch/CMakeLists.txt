cmake_minimum_required(VERSION 3.5)
project(decent_main_launch)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)

#include_directories(
#  include
#)

#set(executable_name nri_mission_runner_node)
#set(library_name ${executable_name}_core)

#set(dependencies
#  rclcpp
#  rclcpp_action
#  rclcpp_components
#  std_msgs
#  visualization_msgs
#  nav2_msgs
#  nav_msgs
#  nri_msgs
#  geometry_msgs
#  builtin_interfaces
#  tf2_ros
#  nav2_costmap_2d
#)


#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/NriWaypointGps.msg"
#  "msg/NriWaypointlistGps.msg"
#  "msg/NriWaypointlistXyz.msg"
#  "msg/NriWaypointXyz.msg"
# DEPENDENCIES std_msgs
#)

#add_library(${library_name} SHARED
#  src/nri_mission_runner.cpp
#)

#ament_target_dependencies(${library_name}
#  ${dependencies}
#)

#add_executable(${executable_name}
#  src/main.cpp
#)

#target_link_libraries(${executable_name} ${library_name})

#ament_target_dependencies(${executable_name}
#  ${dependencies}
#)

#rclcpp_components_register_nodes(${library_name} "nav2_planner::PlannerServer")

#install(TARGETS ${library_name}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

#install(TARGETS ${executable_name}
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#)

#install(DIRECTORY include/
#  DESTINATION include/
#)

install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#ament_export_include_directories(include)
#ament_export_libraries(${library_name})
#ament_export_dependencies(${dependencies})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
