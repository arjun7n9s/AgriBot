#!/bin/bash
# Film the harvest demo: Gazebo GUI with the camera director, a GNOME screen
# recording, the robot's camera frames, then the published media.
#   tools/sim/ubuntu/film.sh [capture_dir] [picks]
# Close or minimise other windows first: the recording covers the whole screen,
# and only the Gazebo 3D view is cut out of it (tools/sim/make_video.py).
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-$HOME/agribot_capture}; PICKS=${2:-2}
export XDG_RUNTIME_DIR=/run/user/$(id -u) DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
rm -rf "$OUT"; mkdir -p "$OUT"
"$HERE/stop.sh"
"$HERE/start.sh" "$OUT/launch.log" gui:=true harvest:=true picks:=$PICKS capture_dir:=$OUT
sleep 10                                   # let the GUI open before recording
date +%s.%N > "$OUT/cast_start.txt"
python3 "$HERE/screencast.py" "$OUT/desktop.webm" 900 30 > "$OUT/screencast.log" 2>&1 &
CAST=$!
for i in $(seq 1 600); do                  # wait up to 10 min for the demo to finish
  grep -q "done:" "$OUT/launch.log" && break
  sleep 1
done
sleep 6                                    # hold the final shot
kill -INT $CAST; wait $CAST
"$HERE/stop.sh"
grep -E "pick [0-9]|done:" "$OUT/launch.log" | sed "s/\x1b\[[0-9;]*m//g" | sed -E "s/.*\[harvest_mission\]: //"
# The raw full-screen recording is deleted only once the cropped video exists.
if python3 "$HERE/../make_video.py" "$OUT" && [ "$(stat -c %s "$OUT/agribot_demo.mp4" 2>/dev/null || echo 0)" -gt 1000000 ]; then
  rm -f "$OUT/desktop.webm"
else
  echo "video step failed; kept $OUT/desktop.webm"
fi
