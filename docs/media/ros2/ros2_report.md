# ROS 2 snapshot: perception-driven harvest (1 Nov 2026)

Clean run on the Ubuntu laptop (RTX 4060, ROS 2 Humble, Gazebo Fortress 6.18), `picks:=3`. Nothing but this launch was running.

## Nodes

```
/arm_controller
/capture_frames
/controller_manager
/drive_node
/fruit_detector
/gz_ros2_control
/harvest_mission
/joint_state_broadcaster
/robot_state_publisher
/ros_gz_bridge
/row_follower
/scout_controller
/steering_controller
/tool_controller
/wheel_controller
```

## Controllers

```
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
wheel_controller        velocity_controllers/JointGroupVelocityController      active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
steering_controller     position_controllers/JointGroupPositionController      active
tool_controller         position_controllers/JointGroupPositionController      active
scout_controller        position_controllers/JointGroupPositionController      active
```

## Perception and mission topics

| Topic | Type | From → to |
| --- | --- | --- |
| `/front_camera/image` | `sensor_msgs/Image` | bridge → `row_follower` |
| `/row_follower/enable` | `std_msgs/Bool` | `harvest_mission` → `row_follower` |
| `/row_follower/state` | `std_msgs/Float32MultiArray` [near, far, confidence] | `row_follower` → (monitoring) |
| `/row_follower/debug_image` | `sensor_msgs/Image` | `row_follower` → `capture_frames`, RViz |
| `/cmd_vel` | `geometry_msgs/Twist` | `row_follower`, `harvest_mission` → `drive_node` |
| `/arm_camera/image`, `/arm_camera/depth_image`, `/arm_camera/camera_info` | `sensor_msgs` | bridge → `fruit_detector` |
| `/fruit/detections` | `geometry_msgs/PoseArray` (frame `base_footprint`) | `fruit_detector` → `harvest_mission` |
| `/fruit/markers` | `visualization_msgs/MarkerArray` | `fruit_detector` → RViz |
| `/fruit/debug_image` | `sensor_msgs/Image` | `fruit_detector` → `capture_frames`, RViz |
| `/harvest/phase`, `/harvest/count` | `std_msgs/String`, `std_msgs/Int32` | `harvest_mission` → (monitoring) |

## Rates (measured with `ros2 topic hz`, simulation running headless)

| Topic | Rate |
| --- | --- |
| `/clock` | 454.8 Hz (0.91 × real time) |
| `/joint_states` | 87.9 Hz |
| `/imu` | 89.4 Hz |
| `/odom` | 26.8 Hz |
| `/tf` | 18.2 Hz |
| `/front_camera/image` | 6.7 Hz |
| `/arm_camera/image` | 3.2 Hz |
| `/arm_camera/depth_image` | 6.1 Hz |
| `/scout_camera/image` | 4.6 Hz |
| `/row_follower/state` | 13.5 Hz |
| `/fruit/detections` | 13.8 Hz |

The row-follower and detector publish once per input image, yet they measure faster than the camera topics feeding them. So treat the camera figures as lower bounds. The likely cause is that `ros2 topic hz` (Python) cannot deserialise full images as fast as they arrive; the nodes' own output rates are the better estimate of what the cameras deliver.

## Harvest phases (from the `harvest_mission` log)

| Pick | Stop (fruit seen, rover frame, m) | Confirmed | Pedicel picked (world-file diagnostic) | Distance from estimate | Result |
| --- | --- | --- | --- | --- | --- |
| 1 | (0.108, −0.490, 0.831) | 24 of 24 frames, 2 fruit in reach | `plant_b0_r0_t1_f3`, half-ripe | 16 mm | in the crate |
| 2 | (0.102, −0.501, 0.785) | 24 of 24 frames, 1 fruit in reach | `plant_b0_r1_t1_f3`, ripe | 6 mm | in the crate |
| 3 | (0.103, −0.489, 0.835) | 24 of 24 frames, 2 fruit in reach | `plant_b0_r2_t1_f3`, half-ripe | 24 mm | in the crate |

Every pick: the throat reached the estimated pedicel to within 0.1 mm (forward kinematics of the measured joints). Final line of the log: `done: 3 of 3 picks verified in the crate`.

The world-file lookup is used only to say which fruit was cut and how far it was from the estimate; it never feeds the targeting.
