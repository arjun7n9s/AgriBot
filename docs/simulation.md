# Gazebo harvest demo

Recorded 1 Nov 2026 on the Ubuntu laptop (RTX 4060, Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress 6.18).

The rover drives the furrow between two tomato beds and stops beside a ripe tomato. The SCARA arm brings the cutter's throat onto the fruit's pedicel and closes the blade, and the fruit drops into the padded cradle. The arm then carries it to the crate on the rover's deck and opens the trap door. Gazebo reads back each fruit's position afterwards, and a pick counts only if the fruit is inside the crate.

![First pick](media/agribot_pick.gif)

**Full run:** [media/agribot_demo.mp4](media/agribot_demo.mp4), 94 s at 1280×720, two picks.

| Drive | Cut | Drop |
| --- | --- | --- |
| ![](media/stills/drive.png) | ![](media/stills/cut.png) | ![](media/stills/drop.png) |

**What the robot's cameras published during the run** (ROS topics, saved by `capture_frames.py`):

| Front (navigation) | Arm RGB-D: colour | Arm RGB-D: depth | Scouting (pan) |
| --- | --- | --- | --- |
| ![](media/cameras/front_camera.png) | ![](media/cameras/arm_camera_rgb.png) | ![](media/cameras/arm_camera_depth.png) | ![](media/cameras/scout_camera.png) |

## Results

| | |
| --- | --- |
| Picks verified in the crate | **2 of 2** (fruit position read back from Gazebo) |
| Cutter on the pedicel | **0.1 mm** from target, measured: forward kinematics of the joint states plus odometry |
| Catch | the cut fruit lands in the cradle 1.8 cm below the cut point, as designed |
| Simulation speed | about 0.92 × real time (clock 458 Hz against 500 Hz) with the GUI and three cameras rendering |
| Topic rates | `/joint_states` 92 Hz, `/imu` 91 Hz, `/odom` 27 Hz, `/front_camera/image` 8.8 Hz, `/arm_camera/image` 4.6 Hz, `/arm_camera/depth_image` 4.5 Hz, `/scout_camera/image` 4.5 Hz |

The camera rates are below their configured 15 Hz (10 Hz for the scout) because rendering is the bottleneck on this GPU. The full snapshot (nodes, topics, controllers, rates, phases) is in [media/ros2/ros2_report.md](media/ros2/ros2_report.md).

## The ROS 2 system

![ROS 2 graph during a pick](media/ros2/ros2_graph.png)

The graph above is drawn from the live ROS graph by `tools/sim/ros_graph.py`, during a pick ([SVG](media/ros2/ros2_graph.svg)).

| Piece | What it does |
| --- | --- |
| `gz_ros2_control` + `controller_manager` | Runs the robot's joints in Gazebo through ros2_control |
| `wheel_controller` | Four wheel velocities (the stock board drives all four together) |
| `steering_controller` | One steering angle |
| `arm_controller` | JointTrajectoryController for J1, Z, J2, J3 |
| `tool_controller` | Blade and trap door |
| `scout_controller` | Scouting-camera pan |
| `joint_state_broadcaster` → `robot_state_publisher` | `/joint_states` → `/tf` |
| `ros_gz_bridge` | `/clock`, `/odom` (ground truth), `/imu`, all camera images and camera info |
| `drive_node` | `/cmd_vel` → wheel speeds and a steering angle (bicycle model, ±25°) |
| `harvest_demo` | The mission: drive, plan, cut, carry, drop, verify. Publishes `/harvest/phase` and `/harvest/count` |
| `capture_frames` | Saves the robot's camera images on each `/capture` message |

**Arm targeting:** each pose is solved by damped least squares on the robot's own URDF (`agribot_gazebo/urdf_fk.py`), with the cutter's mouth turned to face the plant. The node then compares forward kinematics from the measured joints with the target.

## Running it

On the Ubuntu laptop, after the setup in [assets.md](assets.md):

```bash
cd ~/agribot_ws && colcon build --symlink-install && source install/setup.bash
```

```bash
ros2 launch agribot_gazebo demo.launch.py harvest:=true
```

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `gui` | `true` | Open the Gazebo GUI |
| `rviz` | `false` | Open RViz with the robot model and camera images |
| `harvest` | `false` | Run the harvest mission |
| `picks` | `2` | How many fruit to pick |
| `mast_pin` | `3` | SCARA mast height, 0–4 |
| `capture_dir` | empty | Save camera frames and direct the GUI camera for filming |

To film, close or minimise other windows, then run:

```bash
tools/sim/ubuntu/film.sh ~/agribot_capture 2
```

It records the screen through GNOME's screencast service, cuts out the Gazebo 3D view, and writes the MP4, the GIF and one still per phase (`tools/sim/make_video.py`). It deletes the raw full-screen recording.

The world is generated: `python3 tools/sim/make_world.py` writes `worlds/demo_row.sdf`, its target list and the bridge config.

## Modelling choices and limitations

These are deliberate simplifications. Each one is a known gap between this demo and the field.

- **Targets come from the world file, not from vision.** The ripe-fruit detector is not in the loop yet. The node says so in its log and on `/harvest/phase`.
- **Fruit on the plants are static, and the robot passes through them.** They sit in collision group `0x04`, while the robot uses `0x01` and the cradle and crate use `0x03`. A real truss sways aside, but rigid welded fruit jammed the cradle in testing.
- **The cut is modelled.** When the blade closes, the node removes the fruit and spawns a free-falling copy at the same pose in group `0x02`, which the cradle and crate catch. Physics takes over from there: the fall into the cradle, the carry, and the drop into the crate.
- **Plant stems and stakes have no collision.** As rigid cylinders they stalled the arm, where a real stem bends.
- **Odometry is ground truth** (Gazebo's OdometryPublisher). Wheel encoders and the IMU would replace it on the robot.
- **Steering is one parallel-steer joint, and the wheelbase and track are assumed** (see [work-plan.md](work-plan.md)) until the car is measured.
- **The cutter is concept v0** ([scara-redesign.md](scara-redesign.md)), not a manufacturing design.

## Problems found and fixed on the way

| Symptom | Cause | Fix |
| --- | --- | --- |
| No controller manager | gz_ros2_control registers for newer Gazebo (`GZ_SIM_SYSTEM_PLUGIN_PATH`); Fortress reads `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` | The launch file sets both |
| SCARA Z axis frozen | DART locks a prismatic joint that starts exactly on its limit | `initial_value` 0.05 m in ros2_control; the measured limits are unchanged |
| Z axis would not lift | Estimated damping (20 N·s/m) and speed limit (0.05 m/s) were unrealistic | 2 N·s/m and 0.1 m/s (8 mm lead at about 750 rpm) |
| Fruit launched across the field | A stale `wrist_link__motor.stl` from an earlier link split put the gripper servo inside the wrist's collision hull | `extract_links.py` clears its output first |
| Fruit rolled out while carried | The soft lip stood 0.5 mm above the foam | The lip now stands 16 mm above the foam (design change) |
| Fruit collision poked into the cutter | The collision sphere used the fruit's horizontal radius | It now uses the vertical semi-axis, which matches the real top and bottom |
| Leaves and fruit rendered black | Blender omits `Kd` for textured materials; Gazebo multiplies the texture by black | The exporter adds `Kd 1 1 1`; QA fails on any textured material without `Kd` |
| Arm camera saw only its own plate | It was mounted above the top plate, looking down | It now sits on the side, at fruit height, turned 28° in (design change) |
| Trap door would swing up into the fruit | Hinge axis sign | The axis is now (0, −1, 0), so positive angles swing the floor down |
| Dropped fruit fell through the crate | The crate had no collision | Floor and four walls added |
