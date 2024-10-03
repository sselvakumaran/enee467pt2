from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
  launch_description = LaunchDescription()

  simulation_arg = DeclareLaunchArgument(
    'sim',
    default_value='true',
    description="Draw shapes in simulation or on the real arm"
  )
  launch_description.add_action(simulation_arg)

  simulation_state = LaunchConfiguration('sim')

  lab_gazebo_simulation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [FindPackageShare("lab_simulation_gazebo"), "/launch", "/lab_sim_moveit.launch.py"]
    ),
    launch_arguments={
      "ur_type": 'ur3e',
      "description_package": 'lab_description',
      "description_file": 'lab.urdf.xacro',
      "moveit_config_package": 'lab_moveit_config',
      "moveit_config_file": 'lab.srdf',
      "runtime_config_package": 'lab_simulation_gazebo',
      "controllers_file": 'lab_controllers.yaml',
    }.items(),
  )

  start_simulation_action = GroupAction(
    condition=IfCondition(PythonExpression(["'", simulation_state, "' == 'true'"])),
    actions=[lab_gazebo_simulation]
  )
  launch_description.add_action(start_simulation_action)

  ur_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [FindPackageShare("ur_robot_driver"), "/launch", "/ur_control.launch.py"]
    ),
    launch_arguments={
      "ur_type": 'ur3e',
      "robot_ip": '192.168.77.22',
      "description_package": 'lab_description',
      "description_file": 'lab.urdf.xacro',
      "launch_rviz": 'false',
    }.items(),
  )

  moveit_launch_action = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare("lab_moveit_config"), "/launch", "/lab_moveit.launch.py"]
    ),
    launch_arguments={
        "ur_type": 'ur3e',
        "safety_limits": 'true',
        "description_package": 'lab_description',
        "description_file": 'lab.urdf.xacro',
        "moveit_config_package": 'lab_moveit_config',
        "moveit_config_file": 'lab.srdf',
        "publish_robot_description_semantic": "True",
        "use_sim_time": "false",
        "launch_rviz": 'true',
        "use_fake_hardware": "false",  # to change moveit default controller to joint_trajectory_controller
    }.items(),
  )

  start_ur_driver_action = GroupAction(
    condition=IfCondition(PythonExpression(["'", simulation_state, "' == 'false'"])),
    actions=[ur_driver, moveit_launch_action]
  )
  launch_description.add_action(start_ur_driver_action)

  return launch_description
