# Required Extra Package Installations
```bash
  sudo apt-get install ros-humble-usb-cam
  sudo apt-get install ros-humble-aruco-opencv

```


# Setup UR robot
```bash
# UR robot bringup
  ros2 launch ur_robot_driver ur3e.launch.py robot_ip:=<Add the ip address>
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
