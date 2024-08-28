# Lab8 - Camera Calibration
## Introduction

The purpose of this experiment is primarily to “calibrate” the camera. In order to use the camera to measure the position of objects and to use that information as feedback for the control of the arm, it is necessary to know the pose of the camera in the World coordinate system. That pose is determined by 6 parameters which must be measured. As with many complicated measurements, it is unreasonable to believe that a single measurement is accurate enough. The standard approach is to repeat the measurements a reasonable number of times; determine a least-squares fit for the parameters; then check the fit by means of a separate set of measurements. If the result of the test is not good enough, make more (ideally more accurate) measurements.

Once you have calibrated the camera, it is interesting to check the accuracy of the largest square you found for the end effector in the previous lab. You only specified the corners. MoveIt determined the path and the trajectory. If you repeat them and observe the result with the camera, you can compare the path and trajectory as measured by the camera with that computed from the joint angles.

The first thing you have to do is make the first set of measurements for the calibration and record them. We supply the least squares code. Be sure to collect measurements from a broad range of positions, including some extreme ones.

## Getting Started
To get started with the lab, follow the steps and commands as listed below.

**The commands given in each step below are meant to be copied and pasted in the terminal**.

1. Remember to upload your work to GitHub using git commands in the host PC and not within the Docker container. The git repository onyl exists on the host PC and Docker has no access to it.

2. Clone the GitHub repository - Open a terminal and run the following command,
```bash
# Add command to git clone
  git clone git@github.com:ENEE467/lab-workspace.git
# Or
  git clone https://github.com/ENEE467/lab-workspace.git
# Command to get proper branch
  git checkout work-in-progress-2
```

3. Navigate into the cloned directory.
```bash
# Change directory
  cd lab-workspace
```

4. Open the project in Visual Studio Code to start editing.
```bash
# Open visual studio code
  code .
```

5. Reopen the project within a Docker container by followoing the steps below.

images

6. Setup the environment with the required dependencies. This might take a few minutes.
```bash
# Install using a bash script
  .devcontainer/setup.sh
```

7. Make sure that you are in the correct directory. The directory should be the GitHub directory. To verify the correct directory rn the following,
```bash 
# Check whether directory is correct
  ls
```
If the commands gives the output as,
```bash
.devcontianer .vscode src
```
The location is correct

8. Build the project to start working. 
```bash
# Build the ros2 project
  colcon build
```

9. Grant access of the ros2 project to the terminal.
```bash
# Source the directory
  source install/setup.sh
```


## Running Calibrate Camera Exercise on the UR3e Arm
### Startup the UR3e robot
```bash
# UR robot bringup
  ros2 launch ur_robot_driver ur3e.launch.py robot_ip:=192.168.77.22
```

### Additional Verification
First ensure that the program is running on the robot, this can be verified by running the command as below.
```bash
# Test robot controllers
  ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```
If robot moves the startup was successful.

### Launch Moveit2 for Pose Changing using RViz
```bash
# To configure the robot for moveit support with RViz
  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```

### Setup the Resources for Calibration Operation
```bash
# Examples to change default camera name
  ros2 launch lab8 extrinsic_calibration.launch.py camera_name_var:=logitech_webcam_640x480
```

To verify if everything is launched properly, run the following
```bash
  ros2 topic echo /aruco_detections
```
If continous stream of messages show up in the terminal, the robot has been setup and is ready to use for this lab exercise.
Quit the print by using "Ctrl + C".


# Execute the Calibration Program
```bash
  ros2 run lab8 aruco_tf
```
