"""AgriBOT tomato-row demo: the simulated robot plus the autonomy stack.

    ros2 launch agribot_gazebo demo.launch.py                     # world, robot, controllers, drive, perception
    ros2 launch agribot_gazebo demo.launch.py harvest:=true picks:=3
    ros2 launch agribot_gazebo demo.launch.py gui:=false rviz:=true

sim.launch.py (this package) brings up Gazebo and the robot; autonomy.launch.py
(agribot_autonomy) brings up the nodes that would also run on the real robot. With
capture_dir set, the film director and the camera-frame saver start too.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("agribot_gazebo")
    auto_share = get_package_share_directory("agribot_autonomy")
    capture_dir = LaunchConfiguration("capture_dir")
    filming = IfCondition(PythonExpression(["'", capture_dir, "' != ''"]))
    use_sim_time = {"use_sim_time": True}

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Open the Gazebo GUI"),
        DeclareLaunchArgument("rviz", default_value="false", description="Open RViz"),
        DeclareLaunchArgument("harvest", default_value="false", description="Run a harvest goal"),
        DeclareLaunchArgument("picks", default_value="2", description="Pick attempts in that goal"),
        DeclareLaunchArgument("mast_pin", default_value="3", description="SCARA mast height pin, 0-4"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(auto_share, "config", "autonomy.yaml"),
                              description="Parameters of the autonomy nodes"),
        DeclareLaunchArgument("capture_dir", default_value="", description="Save camera frames and direct the GUI camera"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, "launch", "sim.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gui"), "rviz": LaunchConfiguration("rviz"),
                              "mast_pin": LaunchConfiguration("mast_pin")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(auto_share, "launch", "autonomy.launch.py")),
            launch_arguments={"use_sim_time": "true", "params_file": LaunchConfiguration("params_file"),
                              "harvest": LaunchConfiguration("harvest"), "picks": LaunchConfiguration("picks")}.items()),
        Node(package="agribot_gazebo", executable="film_director.py", output="screen", condition=filming,
             parameters=[use_sim_time]),
        Node(package="agribot_gazebo", executable="capture_frames.py", output="screen", condition=filming,
             parameters=[use_sim_time, {"out_dir": capture_dir}]),
    ])
