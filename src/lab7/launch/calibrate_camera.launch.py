from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import PythonExpression, PathJoinSubstitution, LaunchConfiguration, FindExecutable
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  launch_description = LaunchDescription()

  table_number_arg = DeclareLaunchArgument(
    'table',
    default_value='1',
    description="Table number in the RAL you're working on.",
    choices=['1', '2', '3', '4', '5', '6']
  )
  launch_description.add_action(table_number_arg)

  table_name = PythonExpression(["'table-' + '", LaunchConfiguration('table'), "'"])

  table_folder_path = PathJoinSubstitution(
    [FindPackageShare("lab7"), 'config', table_name]
  )

  usb_cam_params_file = PathJoinSubstitution(
    [FindPackageShare("lab7"), 'config', table_name, 'usb_cam_params.yaml']
  )

  usb_cam_node = Node(
    package='usb_cam',
    executable='usb_cam_node_exe',
    parameters=[usb_cam_params_file]
  )
  launch_description.add_action(usb_cam_node)

  camera_calibration_action = Node(
    package='camera_calibration',
    executable='cameracalibrator',
    arguments=[
      '--size', '9x7',
      '--square', '0.0254'
    ],
    remappings=[
      ('image', '/image_raw')
    ]
  )
  launch_description.add_action(camera_calibration_action)

  # copy_calibration_file_action = ExecuteProcess(
  #   cmd=[FindExecutable(name='ros2'), ' run ', 'lab7 ' 'update_camera_info.sh ', table_folder_path],
  #   shell=True
  # )

  # calibration_completion_event = RegisterEventHandler(
  #   OnProcessExit(
  #     target_action=camera_calibration_action,
  #     on_exit=[copy_calibration_file_action]
  #   )
  # )
  # launch_description.add_action(calibration_completion_event)

  return launch_description
