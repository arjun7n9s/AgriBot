# AgriBOT: working notes for Claude

Read this first, then `README.md` and `docs/assets.md`.

## Rules

- **Git:** push to `github.com/arjun7n9s/AgriBot` (`main`) without asking again. Never add a `Co-Authored-By` trailer. Commit in logical chunks with descriptive messages.
- **Credit:** the SCARA base design is credited once, in `CREDITS.md` and the README "Credits" section. Keep that credit, and don't add extra mentions.
- **Generated files:** every asset comes from a script in `tools/`. Change the script and rebuild; never hand-edit a generated mesh, URDF or SDF.
- **Labelling numbers:** mark every number as measured, assumed or estimate. Don't guess unreadable rover-board markings; ask Tejesh (`preferences.md`).
- **Checks:** run `tools/qa/check_assets.py` and `tools/description/write_urdf.py` after any asset change. Both must pass before committing.

## State (31 Oct 2026)

**Done on Windows:**
- Rover, SCARA, cutter concept and field assets.
- `agribot_description` and `agribot_field` ROS 2 packages.
- Renders.
- Asset QA: 29 meshes, 0 warnings, 0 failures.

**Not yet run anywhere:** ROS 2 or Gazebo.

**Next, on Ubuntu 22.04 with ROS 2 Humble and Gazebo Fortress (`ros-humble-ros-gz`):**

1. **Build the packages.** Run `colcon build` on `sim/`, then `ros2 launch agribot_description display.launch.py`. Check the robot in RViz and that the joint sliders move it.
2. **Demo world.** Write a generator script for one row of about 12 plants. Spawn fruit from each plant's `fruit_sites.json`, attach each fruit at its knuckle with a DetachableJoint, and add a ground plane and sky. The full 108-plant field is too heavy for an 8 GB card.
3. **Gazebo extensions for the robot:**
   - `gz_ros2_control` for the wheels, steering, arm, blade and trap door
   - cameras (front, rear, arm RGB-D, scout)
   - an IMU
   - the label system for segmentation
4. **ROS 2 nodes:** drive, row follower, detector, fruit localizer, SCARA IK, pick action, mission. The plan is in `docs/ros2-demo-sim-plan.md`.

**Open questions for Tejesh:**
- the real wheelbase, track and steering linkage
- part masses
- whether the printed Arm 1 is the stock part
- PETG for Arm 2 and the cutter

## Environment notes

- **Windows (where the assets were built):**
  - Blender 5.1 at `C:/Program Files/Blender Foundation/Blender 5.1/blender.exe`
  - Python venv at `.venv` (`tools/requirements.txt`)
  - SolidWorks is available
- **Ubuntu:** `blender` and `python3 -m venv .venv`. The tools are portable.
- **Render GPU:** Blender renders use Cycles on the GPU (OptiX). Render times are 10 s to 2 min per image.
