"""
ros2 launch ./ee_velocity_sim.launch.py [sim:=true] [rviz:=true] [description_file:=...]

robot_state_publisher loads the URDF from stretch_description and publishes it on
/robot_description; the EE velocity node reads the model from there and publishes
/joint_states. Don't also run joint_state_publisher(_gui) or stretch_driver.
Expects stretch_ee_velocity_node.py in the same directory as this file.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    description_file = LaunchConfiguration("description_file")
    robot_description = ParameterValue(Command(["xacro ", description_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_base", default_value="false"),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("stretch_description"), "urdf", "stretch.urdf"])),

        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),

        Node(
            package="stretch_ee_vel_control",
            executable="ee_velocity",
            parameters=[
                {
                    "sim": LaunchConfiguration("sim"),
                    "use_base": LaunchConfiguration("use_base"),
                }
            ],
            output="screen",
        ),

        Node(package="rviz2", executable="rviz2",
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
