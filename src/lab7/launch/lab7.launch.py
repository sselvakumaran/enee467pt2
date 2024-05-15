import os
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


# joint_limit_params = PathJoinSubstitution(
#     [FindPackageShare("lab_description"), "config", "ur3e", "joint_limits.yaml"]
# )
# kinematics_params = PathJoinSubstitution(
#     [FindPackageShare("lab_description"), "config", "ur3e", "default_kinematics.yaml"]
# )
# physical_params = PathJoinSubstitution(
#     [FindPackageShare("lab_description"), "config", "ur3e", "physical_parameters.yaml"]
# )
# visual_params = PathJoinSubstitution(
#     [FindPackageShare("lab_description"), "config", "ur3e", "visual_parameters.yaml"]
# )
# robot_description_content = Command(
#     [
#         PathJoinSubstitution([FindExecutable(name="xacro")]),
#         " ",
#         PathJoinSubstitution([FindPackageShare("lab_description"), "urdf", "lab.urdf.xacro"]),
#         " ",
#         "robot_ip:=xxx.yyy.zzz.www",
#         " ",
#         "joint_limit_params:=",
#         joint_limit_params,
#         " ",
#         "kinematics_params:=",
#         kinematics_params,
#         " ",
#         "physical_params:=",
#         physical_params,
#         " ",
#         "visual_params:=",
#         visual_params,
#         " ",
#         "safety_limits:=",
#         "true",
#         " ",
#         "safety_pos_margin:=",
#         "0.15",
#         " ",
#         "safety_k_position:=",
#         "20",
#         " ",
#         "name:=",
#         "ur",
#         " ",
#         "ur_type:=",
#         "ur3e",
#         " ",
#         "prefix:=",
#         '""',
#         " ",
#     ]
# )

# robot_description = {"robot_description": robot_description_content}

# # MoveIt Configuration
# robot_description_semantic_content = Command(
#     [
#         PathJoinSubstitution([FindExecutable(name="xacro")]),
#         " ",
#         PathJoinSubstitution([FindPackageShare("lab_moveit_config"), "srdf", "lab.srdf.xacro"]),
#         " ",
#         "name:=",
#         # Also ur_type parameter could be used but then the planning group names in yaml
#         # configs has to be updated!
#         "ur",
#         " ",
#         "prefix:=",
#         '""',
#         " ",
#     ]
# )
# robot_description_semantic = {
#     "robot_description_semantic": robot_description_semantic_content
# }

def generate_launch_description():
  # lab_gazebo_simulation_launch = IncludeLaunchDescription(
  #   PythonLaunchDescriptionSource(
  #     [FindPackageShare("lab_simulation_gazebo"), "/launch", "/lab_sim_moveit.launch.py"]
  #   ),
  #   launch_arguments={
  #     "ur_type": 'ur3e',
  #     "description_package": 'lab_description',
  #     "description_file": 'lab.urdf.xacro',
  #     "moveit_config_package": 'lab_moveit_config',
  #     "moveit_config_file": 'lab.srdf.xacro',
  #     "runtime_config_package": 'lab_simulation_gazebo',
  #     "launch_rviz": "false",
  #   }.items(),
  # )

  moveit_config = (
    MoveItConfigsBuilder(
      "lab", package_name="lab_description")
      .robot_description(os.path.join(get_package_share_directory("lab_description"), "urdf/lab.urdf.xacro"))
      .robot_description_semantic(os.path.join(get_package_share_directory("lab_moveit_config"), "srdf/lab.srdf.xacro"))
      .planning_pipelines(pipelines=["ompl"])
      .trajectory_execution(os.path.join(get_package_share_directory("lab_simulation_gazebo"), "config/ur_controllers.yaml"))
      .to_moveit_configs()
  )

  lab7_node = Node(
    package="lab7",
    executable="ur3e_mover",
    output="screen",
    parameters=[
      moveit_config.robot_description,
      moveit_config.robot_description_semantic,
      moveit_config.robot_description_kinematics,
      moveit_config.joint_limits,
      {"use_sim_time": True},
    ]
  )

  nodes_to_launch = [lab7_node]

  return LaunchDescription(nodes_to_launch)
