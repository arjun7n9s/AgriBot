# ROS 2 snapshot (2026-11-01T02:32:08Z)

## Nodes
/arm_controller
/controller_manager
/drive_node
/gz_ros2_control
/harvest_demo
/joint_state_broadcaster
/robot_state_publisher
/ros_gz_bridge
/rqt_gui_py_node_23864
/scout_controller
/steering_controller
/tool_controller
/wheel_controller

## Topics
/arm_camera/camera_info [sensor_msgs/msg/CameraInfo]
/arm_camera/depth_image [sensor_msgs/msg/Image]
/arm_camera/image [sensor_msgs/msg/Image]
/arm_controller/controller_state [control_msgs/msg/JointTrajectoryControllerState]
/arm_controller/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
/arm_controller/state [control_msgs/msg/JointTrajectoryControllerState]
/arm_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/capture [std_msgs/msg/String]
/clock [rosgraph_msgs/msg/Clock]
/cmd_vel [geometry_msgs/msg/Twist]
/dynamic_joint_states [control_msgs/msg/DynamicJointState]
/front_camera/camera_info [sensor_msgs/msg/CameraInfo]
/front_camera/image [sensor_msgs/msg/Image]
/harvest/count [std_msgs/msg/Int32]
/harvest/phase [std_msgs/msg/String]
/imu [sensor_msgs/msg/Imu]
/joint_state_broadcaster/transition_event [lifecycle_msgs/msg/TransitionEvent]
/joint_states [sensor_msgs/msg/JointState]
/odom [nav_msgs/msg/Odometry]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/robot_description [std_msgs/msg/String]
/rosout [rcl_interfaces/msg/Log]
/scout_camera/camera_info [sensor_msgs/msg/CameraInfo]
/scout_camera/image [sensor_msgs/msg/Image]
/scout_controller/commands [std_msgs/msg/Float64MultiArray]
/scout_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/steering_controller/commands [std_msgs/msg/Float64MultiArray]
/steering_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
/tool_controller/commands [std_msgs/msg/Float64MultiArray]
/tool_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]
/wheel_controller/commands [std_msgs/msg/Float64MultiArray]
/wheel_controller/transition_event [lifecycle_msgs/msg/TransitionEvent]

## Controllers
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
wheel_controller        velocity_controllers/JointGroupVelocityController      active
steering_controller     position_controllers/JointGroupPositionController      active
scout_controller        position_controllers/JointGroupPositionController      active
tool_controller         position_controllers/JointGroupPositionController      active

## Rates (measured with ros2 topic hz, sim running)
- `/clock`: 458.577 Hz
- `/joint_states`: 91.910 Hz
- `/odom`: 26.948 Hz
- `/imu`: 91.442 Hz
- `/tf`: 18.431 Hz
- `/front_camera/image`: 8.762 Hz
- `/scout_camera/image`: 4.488 Hz
- `/arm_camera/image`: 4.648 Hz
- `/arm_camera/depth_image`: 4.465 Hz

## Harvest phases
start: targets from the world file; detector not yet in the loop
drive: following the furrow to x = -0.02 m
approach: plant_b0_r0_t1_f1 (tomato_ripe), target from world ground truth
planned joints {'scara_j1': -0.665, 'scara_z': 0.191, 'scara_j2': 0.979, 'scara_j3': -1.885}
throat on the pedicel: planned error 0.0 mm, measured error 0.1 mm, joints {'scara_j1': -0.665, 'scara_z': 0.191, 'scara_j2': 0.979, 'scara_j3': -1.885}
cut: closing the blade across the throat
fruit after the cut: 1.8 cm from the cut point (in the cradle)
retreat and carry to the crate
drop: opening the trap door over the crate
