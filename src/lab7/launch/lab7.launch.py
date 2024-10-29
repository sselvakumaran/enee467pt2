from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
  launch_description = LaunchDescription()

  table_number_arg = DeclareLaunchArgument(
    'table',
    default_value='1',
    description="Table number in the RAL you're working on."
  )
  launch_description.add_action(table_number_arg)

  table_folder = PythonExpression(["'table-' + '", LaunchConfiguration('table'), "'"])

  usb_cam_params_file = PathJoinSubstitution(
    [FindPackageShare("lab7"), 'config', table_folder, 'usb_cam_params.yaml']
  )

  usb_cam_node = Node(
    package='usb_cam',
    executable='usb_cam_node_exe',
    parameters=[usb_cam_params_file]
  )

  aruco_opencv_params_file = PathJoinSubstitution(
    [FindPackageShare("lab7"), 'config', 'aruco_opencv_params.yaml']
  )

  aruco_opencv_node = Node(
    package='aruco_opencv',
    executable='aruco_tracker_autostart',
    parameters=[aruco_opencv_params_file]
  )

  hand_eye_calib_node = Node(
    package='lab7',
    executable='hand_eye_calib',
    parameters=[
      {"workspace_dir": EnvironmentVariable('ROS_WS')},
      {"measurements_required": 15},
      {"marker_id": 5},
      {"robot_base_frame": 'base_link'},
      {"robot_gripper_frame": 'tool0'}
    ]
  )

  ur_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [FindPackageShare("ur_robot_driver"), "/launch", "/ur_control.launch.py"]
    ),
    launch_arguments={
      "ur_type": 'ur3e',
      "robot_ip": '192.168.77.22',
      "description_package": 'lab_description',
      "description_file": 'lab.urdf.xacro',
      "launch_rviz": 'true',
      "rviz_config_file": PathJoinSubstitution([FindPackageShare('lab7'), 'config', 'lab7.rviz'])
    }.items(),
  )

  hand_eye_calib_action = GroupAction(
    actions=[usb_cam_node, aruco_opencv_node, ur_driver, hand_eye_calib_node]
  )
  launch_description.add_action(hand_eye_calib_action)

  return launch_description
