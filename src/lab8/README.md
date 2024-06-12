# Required Extra Package Installations
```bash
  sudo apt-get install ros-humble-usb-cam
  sudo apt-get install ros-humble-aruco-opencv

```

# Sending Parameters into Launch file while running
```bash
# Examples to change default camera name
  ros2 launch lab8 extrinsic_calibration.launch.py camera_name_var:=logitech_webcam_640x480
```