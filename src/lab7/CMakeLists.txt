cmake_minimum_required(VERSION 3.8)
project(lab7)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set (UR3E_MOVE_INTERFACE_DEPS
  control_msgs
  Eigen3
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_ros
)

set (EEF_POSE_TRACKER_DEPS
  tf2_geometry_msgs
  tf2_ros
  Matplot++
)

# package dependencies
set(PACKAGE_DEPENDENCIES
  ament_cmake
  rclcpp
  Boost
  rosidl_default_generators
  "${UR3E_MOVE_INTERFACE_DEPS}"
  "${EEF_POSE_TRACKER_DEPS}"
)

# find dependencies
foreach(dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TrackRequest.srv"
)
ament_target_dependencies(draw_shape ${PACKAGE_DEPENDENCIES})
target_include_directories(draw_shape
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

install(TARGETS
  draw_shape
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
