cmake_minimum_required(VERSION 3.8)
project(lab8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set packages to be installed
set(PACKAGE_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  Eigen3
  angles
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  aruco_opencv_msgs
)

find_package(rosidl_default_generators REQUIRED)

# find dependencies
foreach(dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

# Create clangd compilation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commmands
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

add_executable(aruco_tf src/aruco_tf.cpp)
ament_target_dependencies(aruco_tf ${PACKAGE_DEPENDENCIES})
target_include_directories(aruco_tf
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
  aruco_tf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  calibration
  include
  DESTINATION share/${PROJECT_NAME}
)

rosidl_generate_interfaces("${PROJECT_NAME}"
"msg/Detection.msg"
DEPENDENCIES aruco_opencv_msgs
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

ament_package()
