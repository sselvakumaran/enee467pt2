cmake_minimum_required(VERSION 3.8)
project(lab7)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HAND_EYE_CALIB_DEPS
  aruco_opencv
  aruco_opencv_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  OpenCV
)

set(CORE_DEPENDENCIES
  ament_cmake
  rclcpp
  rosidl_default_generators
)

# find dependencies
foreach(dependency IN ITEMS ${CORE_DEPENDENCIES} ${HAND_EYE_CALIB_DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/HandEyeCalib.srv"
)

add_executable(hand_eye_calibration src/hand_eye_calib.cpp)
ament_target_dependencies(hand_eye_calibration "${CORE_DEPENDENCIES}" "${HAND_EYE_CALIB_DEPS}")
target_include_directories(hand_eye_calibration
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(command_interface src/command_interface.cpp)
ament_target_dependencies(command_interface ${CORE_DEPENDENCIES})
target_include_directories(command_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(hand_eye_calibration
  "${cpp_typesupport_target}"
  opencv_calib3d
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

target_link_libraries(command_interface
  "${cpp_typesupport_target}"
)

install(TARGETS
  hand_eye_calibration
  command_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
