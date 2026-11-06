"""
ros2 launch stretch_ee_vel_control wrist_camera.launch.py [enable_depth:=false] [color_profile:=424x240x15]

Runs on the robot. Starts the gripper D405 through stretch_core's d405_basic.launch.py
with everything the policy doesn't need turned off (pointcloud, depth alignment, IMU),
so the stream stays light over WiFi. The laptop-side StretchHardwareEnv subscribes to

    /gripper_camera/color/image_rect_raw/compressed   (needs ros-<distro>-image-transport-plugins
                                                        on the robot; check with `ros2 topic list`)

via its hardware.extra_cameras config, e.g. ``extra_cameras: {wrist: <that topic>}``.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    d405 = os.path.join(get_package_share_directory('stretch_core'), 'launch', 'd405_basic.launch.py')
    color_profile = LaunchConfiguration('color_profile')
    return LaunchDescription([
        DeclareLaunchArgument('enable_depth', default_value='false'),
        DeclareLaunchArgument('color_profile', default_value='424x240x15',
                              description='WIDTHxHEIGHTxFPS of the color stream'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(d405),
            launch_arguments={
                'enable_depth': LaunchConfiguration('enable_depth'),
                # realsense_ros >= 4.55.1 reads rgb_camera.profile, older depth_module.color_profile
                'rgb_camera.profile': color_profile,
                'depth_module.color_profile': color_profile,
                'pointcloud.enable': 'false',
                'align_depth.enable': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
            }.items(),
        ),
    ])
