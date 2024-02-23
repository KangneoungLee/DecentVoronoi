cmake_minimum_required(VERSION 3.5)
project(decent_voronoi)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  {Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(executable_name_decent decent_voronoi_ros_agent_node)
set(executable_name_robot_dyn decent_voronoi_virtual_agent_ros_node)
set(executable_name_offline_metric_cal offline_metric_cal_ros_node)


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

add_library(voro_cell src/voro_cell.cpp)
target_link_libraries(voro_cell ${Boost_LIBRARIES})

add_library(decent_voronoi_core SHARED
  src/decent_voronoi.cpp
)
target_link_libraries(decent_voronoi_core voro_cell ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(${executable_name_decent}
  src/decent_voronoi_ros_agent.cpp
)

add_executable(${executable_name_robot_dyn}
  src/decent_voronoi_virtual_agent_ros.cpp
)

add_executable(${executable_name_offline_metric_cal}
  src/offline_metric_cal.cpp
)

target_link_libraries(${executable_name_decent} decent_voronoi_core ${OpenCV_LIBS})
target_link_libraries(${executable_name_robot_dyn} ${OpenCV_LIBS})
target_link_libraries(${executable_name_offline_metric_cal} decent_voronoi_core ${OpenCV_LIBS})

ament_target_dependencies(${executable_name_decent}
  ${dependencies}
)

ament_target_dependencies(${executable_name_robot_dyn}
  ${dependencies}
)

ament_target_dependencies(${executable_name_offline_metric_cal}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name_decent}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${executable_name_robot_dyn}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${executable_name_offline_metric_cal}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)


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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
#ament_export_dependencies(rosidl_default_runtime)
ament_package()
