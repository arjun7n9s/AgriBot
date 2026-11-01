#!/bin/bash
# Stop a running demo, and anything left over from earlier runs: a stale node keeps
# publishing on the same topics and quietly changes the results.
# Patterns are exact program names so this never matches the shell that called it.
for p in "ign gazebo" "ros2 launch agribot_gazebo" "ros2 launch agribot_autonomy" "parameter_bridge" \
         "robot_state_publisher" "lib/agribot_autonomy/" "lib/agribot_gazebo/" \
         "sim_cut_model.py" "film_director.py" "capture_frames.py" "rviz2" "controller_manager/spawner"; do
  pgrep -f "$p" | grep -v -w -e $$ -e $PPID | xargs -r kill 2>/dev/null
done
sleep 2
pgrep -f "ign gazebo" | xargs -r kill -9 2>/dev/null
exit 0
