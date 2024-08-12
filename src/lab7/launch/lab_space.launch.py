import os
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  moveit_config = MoveItConfigsBuilder(
      "lab", package_name="lab_description")
      .robot_description(os.path.join(get_package_share_directory("lab_description"), "urdf/lab.urdf.xacro"))
      .robot_description_semantic(os.path.join(get_package_share_directory("lab_moveit_config"), "srdf/lab.srdf.xacro"))
      .planning_pipelines(pipelines=["ompl"])
      .trajectory_execution(os.path.join(get_package_share_directory("lab_simulation_gazebo"), "config/ur_controllers.yaml"))
      .to_dict()


  lab7_node = Node(
    package="lab7",
    executable="ur3e_mover",
    output="screen",
    parameters=[
      moveit_config,
    ]
  )

  nodes_to_launch = [lab7_node]

  return LaunchDescription(nodes_to_launch)
