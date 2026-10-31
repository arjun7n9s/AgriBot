"""AgriBOT tomato-row demo in Gazebo Fortress.

    ros2 launch agribot_gazebo demo.launch.py                 # world, robot, controllers, bridge, GUI
    ros2 launch agribot_gazebo demo.launch.py harvest:=true   # ... and run the harvest demo
    ros2 launch agribot_gazebo demo.launch.py gui:=false rviz:=true

Starts the Gazebo server with worlds/demo_row.sdf, spawns the robot from
agribot_gazebo.urdf.xacro, loads the controllers, bridges topics (config/bridge.yaml),
and runs the cmd_vel drive node. Optional: the Gazebo GUI, RViz, the harvest
demo and the camera-frame saver.
"""
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SPAWN = {"x": -1.6, "y": 1.0, "z": 0.02, "yaw": 0.0}


def generate_launch_description():
    share = get_package_share_directory("agribot_gazebo")
    world = os.path.join(share, "worlds", "demo_row.sdf")
    gui_config = os.path.join(share, "config", "gui.config")
    use_sim_time = {"use_sim_time": True}

    args = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("harvest", default_value="false"),
        DeclareLaunchArgument("capture_dir", default_value=""),
        DeclareLaunchArgument("mast_pin", default_value="3"),
        DeclareLaunchArgument("picks", default_value="2"),
    ]

    description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([FindPackageShare("agribot_gazebo"), "urdf", "agribot_gazebo.urdf.xacro"]),
        " tool:=cutter mast_pin:=", LaunchConfiguration("mast_pin")]), value_type=str)

    # gz_ros2_control registers itself for newer Gazebo (GZ_SIM_SYSTEM_PLUGIN_PATH);
    # Fortress reads IGN_GAZEBO_SYSTEM_PLUGIN_PATH.
    plugin_path = AppendEnvironmentVariable(
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH", os.path.join(get_package_prefix("gz_ros2_control"), "lib"))

    server = ExecuteProcess(cmd=["ign", "gazebo", "-s", "-r", "-v", "2", world], output="screen")
    gui = ExecuteProcess(cmd=["ign", "gazebo", "-g", "-v", "2", "--gui-config", gui_config],
                         output="screen", condition=IfCondition(LaunchConfiguration("gui")))

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": description}, use_sim_time], output="screen")
    def spawn_robot(context):
        # Spawn from an SDF with collision groups (agribot_gazebo/robot_sdf.py): the arm
        # passes through fruit on the trusses; the cradle and the crate still catch them.
        from agribot_gazebo import robot_sdf
        xacro_file = os.path.join(share, "urdf", "agribot_gazebo.urdf.xacro")
        urdf = subprocess.run(["xacro", xacro_file, "tool:=cutter",
                               f"mast_pin:={LaunchConfiguration('mast_pin').perform(context)}"],
                              capture_output=True, text=True, check=True).stdout
        path = os.path.join(tempfile.gettempdir(), "agribot_robot.sdf")
        counts = robot_sdf.build(urdf, path)
        print(f"[agribot] robot SDF {path}: {counts['0x03']} catching and {counts['0x01']} other collisions")
        return [Node(package="ros_gz_sim", executable="create", output="screen",
                     arguments=["-world", "demo_row", "-name", "agribot", "-file", path,
                                "-x", str(SPAWN["x"]), "-y", str(SPAWN["y"]), "-z", str(SPAWN["z"]),
                                "-Y", str(SPAWN["yaw"])])]

    spawn = OpaqueFunction(function=spawn_robot)
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml")}, use_sim_time])

    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"])])

    controllers = [spawner(c, 6.0 + 0.5 * i) for i, c in enumerate(
        ["joint_state_broadcaster", "wheel_controller", "steering_controller", "arm_controller",
         "tool_controller", "scout_controller"])]

    drive = Node(package="agribot_gazebo", executable="drive_node.py", output="screen", parameters=[use_sim_time])
    harvest = TimerAction(period=14.0, actions=[Node(
        package="agribot_gazebo", executable="harvest_demo.py", output="screen",
        parameters=[use_sim_time, {"spawn_x": SPAWN["x"], "spawn_y": SPAWN["y"],
                                   "targets_file": os.path.join(share, "worlds", "demo_row_targets.json"),
                                   "picks": LaunchConfiguration("picks"),
                                   "capture_dir": LaunchConfiguration("capture_dir")}],
        condition=IfCondition(LaunchConfiguration("harvest")))])
    capture = Node(package="agribot_gazebo", executable="capture_frames.py", output="screen",
                   parameters=[use_sim_time, {"out_dir": LaunchConfiguration("capture_dir")}],
                   condition=IfCondition(LaunchConfiguration("harvest")))
    rviz = Node(package="rviz2", executable="rviz2", output="screen", parameters=[use_sim_time],
                arguments=["-d", os.path.join(share, "rviz", "demo.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")))

    return LaunchDescription(args + [plugin_path, server, gui, rsp, spawn, bridge, drive, rviz, capture, harvest]
                             + controllers)
