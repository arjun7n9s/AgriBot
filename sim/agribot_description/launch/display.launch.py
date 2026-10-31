"""Show the AgriBOT description in RViz with joint sliders.

    ros2 launch agribot_description display.launch.py
    ros2 launch agribot_description display.launch.py tool:=gripper mast_pin:=4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tool = LaunchConfiguration("tool")
    mast_pin = LaunchConfiguration("mast_pin")
    xacro_file = PathJoinSubstitution([FindPackageShare("agribot_description"), "urdf", "agribot.urdf.xacro"])
    description = ParameterValue(
        Command(["xacro ", xacro_file, " tool:=", tool, " mast_pin:=", mast_pin]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("tool", default_value="cutter", description="cutter or gripper"),
        DeclareLaunchArgument("mast_pin", default_value="2", description="SCARA mast position 0-4"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2"),
    ])
