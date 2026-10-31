#!/bin/bash
# Stop a running demo. Patterns are exact program names so this never matches
# the shell that called it.
for p in "ign gazebo" "ros2 launch agribot_gazebo" "parameter_bridge" "robot_state_publisher" \
         "harvest_mission.py" "harvest_demo.py" "row_follower.py" "fruit_detector.py" "capture_frames.py"          "drive_node.py" "rviz2" "controller_manager/spawner"; do
  pgrep -f "$p" | grep -v -w -e $$ -e $PPID | xargs -r kill 2>/dev/null
done
sleep 2
pgrep -f "ign gazebo" | xargs -r kill -9 2>/dev/null
exit 0
