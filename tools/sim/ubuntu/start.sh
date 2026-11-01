#!/bin/bash
# Start the demo in the background: start.sh <log> [launch args...]
#   tools/sim/ubuntu/start.sh /tmp/demo.log harvest:=true
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/env.sh"
LOG=$1; shift
nohup ros2 launch agribot_gazebo demo.launch.py "$@" > "$LOG" 2>&1 &
echo "started pid $!"
