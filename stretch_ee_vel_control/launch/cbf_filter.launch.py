"""
ros2 launch stretch_ee_vel_control cbf_filter.launch.py [plane:=yz] [sim:=false] [rviz_config:=...]

Runs the CBF filter and RViz. With sim:=true it also brings up velocity_sim.launch.py
(robot_state_publisher + ee_velocity in sim) so everything runs on one machine; leave it
false when ee_velocity is running on the robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKG = "stretch_ee_vel_control"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("plane", default_value="yz", description="xy or yz"),
        DeclareLaunchArgument("reference_frame", default_value="base_link"),
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([FindPackageShare(PKG), "config", "stretch.rviz"])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(PKG), "launch", "velocity_sim.launch.py"])),
            launch_arguments={"sim": "true", "rviz": "false"}.items(),
            condition=IfCondition(LaunchConfiguration("sim")),
        ),

        Node(
            package=PKG, executable="cbf_filter",
            parameters=[{"plane": LaunchConfiguration("plane"),
                         "reference_frame": LaunchConfiguration("reference_frame")}],
            output="screen",
        ),

        Node(package="rviz2", executable="rviz2",
             arguments=["-d", LaunchConfiguration("rviz_config")]),
    ])
