from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  launch_description = LaunchDescription()

  camera_calibration_action = Node(
    package='camera_calibration',
    executable='cameracalibrator',
    arguments=[
      '--size', '7x9',
      '--square', '0.02'
    ]
  )
  launch_description.add_action(camera_calibration_action)

  return launch_description
