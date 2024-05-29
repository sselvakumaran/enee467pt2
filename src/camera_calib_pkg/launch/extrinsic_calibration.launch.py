import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindExecutable

def generate_launch_description():
    desc = LaunchDescription()

    show_output_var = LaunchConfiguration('show_output_var', default='false')
    show_output = DeclareLaunchArgument(
        name='show_output', default_value=show_output_var
    )

    launch_calibration_var = LaunchConfiguration('launch_calibration_var', default='false')
    launch_calibration = DeclareLaunchArgument(
        name='launch_calibration', default_value=launch_calibration_var
    )

    aruco_tracker_var = LaunchConfiguration('aruco_tracker_var', default='false')
    aruco_tracker = DeclareLaunchArgument(
        name='aruco_tracker', default_value=aruco_tracker_var
    )

    camera_name_var = LaunchConfiguration('camera_name_var', default='logitech_webcam')
    camera_name = DeclareLaunchArgument(
        name='camera_name', default_value=camera_name_var
    )

    camera_device_var = LaunchConfiguration('camera_device_var', default='/dev/video0')
    camera_device = DeclareLaunchArgument(
        name='camera_device', default_value=camera_device_var
    )


    desc.add_action(show_output)
    desc.add_action(launch_calibration)
    desc.add_action(aruco_tracker)
    desc.add_action(camera_device)
    desc.add_action(camera_name)

    package_dir = get_package_share_directory('camera_calib_pkg')
    yaml_params = os.path.join(
        package_dir, 'calibration/camera', f'{camera_name_var}.yaml'
    )

    start_usb_camera = Node(
        package='usb_cam',
        executable='usb_cam_node',
        name=camera_device_var,
        parameters=[
                {
                    'video_device': camera_device_var,
                    'image_width': '1920',
                    'image_height': '1080',
                    'pixel_format': 'yuyv',
                    'camera_frame_id': 'tracking_cam',
                    'camera_name': camera_name_var,
                    'io_map': 'mmap',
                    'camera_info_url': yaml_params,
                }
            ],
        output='screen',
    )

    desc.add_action(start_usb_camera)

    start_rectification = Node(
        package='image_proc',
        executable='image_proc_exe',
        name='rectify_image',
        namespace=camera_name_var,
    )

    desc.add_action(start_rectification)

    # Need to add aruco detection

    return desc