# Source this before running the demo on the Ubuntu laptop (ROS 2 Humble, Gazebo Fortress).
#   source tools/sim/ubuntu/env.sh
source /opt/ros/humble/setup.bash
source "${AGRIBOT_WS:-$HOME/agribot_ws}/install/setup.bash"
# Draw on the logged-in desktop (Wayland session with Xwayland) when run over SSH.
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)}
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export QT_QPA_PLATFORM=xcb
# Render on the NVIDIA GPU of a PRIME (hybrid graphics) laptop.
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
