

# Setup UR robot
```bash
# UR robot bringup
  ros2 launch ur_robot_driver ur3e.launch.py robot_ip:=192.168.77.22
```

# Moveit Launch
First ensure that the program is running on the robot, this can be verified by running the command as below

```bash
# Test robot controllers
  ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

If robot moves accordingly,
```bash
# To configure the robot for moveit support with RViz
  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```


# Sending Parameters into Launch file while running
```bash
# Examples to change default camera name
  ros2 launch lab8 extrinsic_calibration.launch.py camera_name_var:=logitech_webcam_640x480 show_output_var:=True
```


# Execute the calibration node
```bash
  ros2 run lab8 aruco_tf
```


# For moveit config, the lab scene has a table and robot mounted on it
# Check required params in ur3e_mover -> and link with official ur3e
# lab7.launch.py supposedly launches the scene of in lab
# go thoruhg launch_simulatioon_gazebo -> launches on table properly
# New launches fine, works fine just ensure that sim time false, fake harware false and control.launch is disabled -> real robot
# for gazebo i think flip the above
# but lab7.launch.py still fails
<!-- [ERROR] [launch]: Caught exception in launch (see debug for traceback): Caught multiple exceptions when trying to load file of format [py]:
 - XacroException: Undefined substitution argument name
 - InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown -->
