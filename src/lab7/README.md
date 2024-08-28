# Lab7 - UR3e Motion using Moveit2
## Introduction
The purpose of this lab is to familiarize you with the UR3 robot and the tools we have for making it do useful and/or interesting things. Additionally, you will cause the end point of the robot arm to move in both a square and circle in a vertical plane and in a horizontal plane. The Horizontal Plane is the plane perpendicular to Z-Axis and the Vertical plane is the plane perpendicular to either X or Y axes. Next, you will cause the end point of the arm to move in the largest possible square that is not in the horizontal plane.

**NOTE**: Throughout the course you will **first create a successful simulation of the desired arm movement in Gazebo**. Only after getting this simulation approved by the lab staff will you implement it on the actual arm. This is a very important safety measure.

Throughout this part of the course, you will use a collection of tools that we have provided. These include Docker containers, ROS, Moveit2, and Gazebo. The basics of these tools are explained in the lectures. These tools are already loaded on the lab computers or on GitHub.

All the tools you need to do this lab are in a Docker container. Docker containers are built on images which are built from dockerfiles. Your system in the lab has the docker image with all the required tools such as ROS and Moveit! Installed. You just need to build a container using that image and work inside the container. To save the work, you will volume map a directory from the host pc to the docker container. Any changes you make in the mapped directory inside the container will reflect in the directory in the host pc. To learn more about docker visit [docker documentation](https://docs.docker.com/).

**Volume Map**: Docker Containers are destroyed when you exit the container, which means all the data will be lost. Volume mapping is used to save the important data before destroying the container. A directory from host pc is mapped to a directory in the container. Changes made in the mapped directory in the docker container is reflected in the host pc. Save the important data in the mapped directory inside the container.

## Docker and docker image

If you want to work on your own computer, install docker and portainer (optional) using [this page](https://github.com/ENRE467/Getting_Started/wiki/Installing-Docker-and-Portainer) and build a docker image using [this page](https://github.com/ENRE467/Getting_Started/wiki/Building-a-Docker-Image)


## Getting Started
To get started with the lab, follow the steps and commands as listed below.

**The commands given in each step below are meant to be copied and pasted in the terminal**.

1. Remember to upload your work to GitHub using git commands in the host PC and not within the Docker container. The git repository onyl exists on the host PC and Docker has no access to it.

2. Clone the GitHub repository - Open a terminal and run the following command,
```bash
# Add command to git clone
  git clone git@github.com:ENEE467/lab-workspace.git
# Command to change to proper branch
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


## Running the Project in Simulation
In this, the robot will be build and taskes perfomed in a simulated environment. There are two such environment, one is with with a physics engine (Gazebo) and the other is without (RViz)

### Launching the Robot on Gazebo and RViz
First we start by launching the robot in Gazebo and RViz. To ensure the robot is launched properly make sure that no error messages pop up. 

Gazebo launch is perfect if the robot is stable, standing on a desk.

RViz launch is perfect if there are no errors being shown in the left tab and the robot can be seen standing on the table.

```bash
# Command to launch the robot within Gazebo and RViz
  ros2 launch lab_simulation_gazebo lab_sim_moveit.launch.py ur_type:=ur3e description_package:=lab_description description_file:=lab.urdf.xacro moveit_config_package:=lab_moveit_config moveit_config_file:=lab.srdf.xacro runtime_config_package:=lab_simulation_gazebo launch_rviz:=false
```


### Running Custom Moveit Actions
An editable program file can be found in the directory "src/", the files "main.cpp" and "ur3e_mover.cpp" will be used to design custom actions for the robot to follow. "ur3e_mover.cpp" is a class which is implemented within "main.cpp".

To execute the custom operations a launch file is used to load the Moveit scene and initialize the parameters.
```bash
# Use the command below to setup and perform custom moveit tasks
  ros2 launch lab7 lab7_gazebo.launch.py
```

### Final Task
Once the robot is workign as intended in simulation, you an run the code on the real UR3e arm. Ask the Teaching Assistants for help.

## Running the Project on the UR3e Robot
Folow the stes in "Getting Started"
### Startup the UR3eRobot
```bash
# UR robot bringup
  ros2 launch ur_robot_driver ur3e.launch.py robot_ip:=192.168.77.22
```

### Verify the Startup
Run the follwoing commands,
```bash 
  ros2 topic list
```
to make sure that the connection was established successfuly and you have access to ROS topics.
If the output displays many names ending with / the robot startup is successful.

### Additional Verification
First ensure that the program is running on the robot, this can be verified by running the command as below.
```bash
# Test robot controllers
  ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

### Run the Execution Script on the UR3e Robot
Ask your TA to help launch the `ur3e_ros` program on the UR3e tablet. Note, at this point you are controlling the arm from your computer. The terminal with ROS drivers should print the following text `Robot connected to reverse interface. Ready to receive control commands.`

### Launch Moveit for Real UR3e Robot
```bash
# Command to setup Moveit on the real robot
  ros2 launch lab_simulation_gazebo lab_real_moveit.launch.py ur_type:=ur3e description_package:=lab_description description_file:=lab.urdf.xacro moveit_config_package:=lab_moveit_config moveit_config_file:=lab.srdf.xacro runtime_config_package:=lab_simulation_gazebo launch_rviz:=false
```

### Run Custom Moveit Script
```bash
# Use the command below to setup and perform custom moveit tasks
  ros2 launch lab7 lab7_real.launch.py
```
