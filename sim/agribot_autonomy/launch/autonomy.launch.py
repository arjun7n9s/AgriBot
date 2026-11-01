"""The autonomy stack on its own: drive, row follower, fruit detector and, optionally, the mission.

    ros2 launch agribot_autonomy autonomy.launch.py                       # drive, perception
    ros2 launch agribot_autonomy autonomy.launch.py harvest:=true picks:=3

It expects a robot (simulated or real) that provides /front_camera/image,
/arm_camera/{image,depth_image,camera_info}, /joint_states, /odom, /tf and
/robot_description, and controllers that take /wheel_controller/commands,
/steering_controller/commands, /arm_controller/joint_trajectory and
/tool_controller/commands. demo.launch.py in agribot_gazebo provides all of that.

With harvest:=true the mission server starts, and a client sends one Harvest goal
(it waits for the robot to be ready, so no start delay is needed).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_params_file(context, harvest):
    """Refuse to start on a missing or incomplete parameter file instead of silently using defaults."""
    import yaml
    path = LaunchConfiguration("params_file").perform(context)
    if not os.path.isfile(path):
        raise RuntimeError(f"params_file not found: {path}")
    try:
        with open(path, encoding="utf-8") as f:
            content = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        raise RuntimeError(f"params_file {path} is not valid YAML: {e}") from e
    needed = ["drive_node", "row_follower", "fruit_detector"] + (["harvest_mission"] if harvest else [])
    missing = [n for n in needed if "ros__parameters" not in (content.get(n) or {})]
    if missing:
        raise RuntimeError(f"params_file {path} has no ros__parameters section for: {', '.join(missing)}")


def launch_setup(context):
    harvest = LaunchConfiguration("harvest").perform(context).lower() in ("true", "1")
    check_params_file(context, harvest)
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    def node(executable):
        return Node(package="agribot_autonomy", executable=executable, output="screen",
                    parameters=[params_file, {"use_sim_time": use_sim_time}])

    actions = [node("drive_node"), node("row_follower"), node("fruit_detector")]
    if harvest:
        actions += [
            node("harvest_mission"),
            Node(package="agribot_autonomy", executable="harvest_client", output="screen",
                 parameters=[{"use_sim_time": use_sim_time}],
                 arguments=["--picks", LaunchConfiguration("picks"), "--distance", LaunchConfiguration("max_distance"),
                            "--wait", "120"])]
    return actions


def generate_launch_description():
    share = get_package_share_directory("agribot_autonomy")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use /clock"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "autonomy.yaml"),
                              description="Parameters of every autonomy node"),
        DeclareLaunchArgument("harvest", default_value="false", description="Start the mission and send one goal"),
        DeclareLaunchArgument("picks", default_value="2", description="Pick attempts in that goal"),
        DeclareLaunchArgument("max_distance", default_value="0.0", description="Metres of row, 0 = the mission default"),
        OpaqueFunction(function=launch_setup),
    ])
