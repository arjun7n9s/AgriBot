# AgriBOT: working notes for Claude

Read this first, then `README.md` and `docs/assets.md`.

## Rules

- **Git:** push to `github.com/arjun7n9s/AgriBot` (`main`) without asking again. Never add a `Co-Authored-By` trailer. Commit in logical chunks with descriptive messages.
- **Credit:** the SCARA base design is credited once, in `CREDITS.md` and the README "Credits" section. Keep that credit, and don't add extra mentions.
- **Generated files:** every asset comes from a script in `tools/`. Change the script and rebuild; never hand-edit a generated mesh, URDF or SDF.
- **Labelling numbers:** mark every number as measured, assumed or estimate. Don't guess unreadable rover-board markings; ask Tejesh (`preferences.md`).
- **Checks:** run `tools/qa/check_assets.py` and `tools/description/write_urdf.py` after any asset change. Both must pass before committing.

## State (1 Nov 2026)

**Running on the Ubuntu laptop (RTX 4060):**
- The Gazebo harvest demo works. See `docs/simulation.md`.
- 2 of 2 picks verified in the crate.
- The media in `docs/media` was filmed with `tools/sim/ubuntu/film.sh`.

**Next:**
1. **Ripe-fruit detector.** YOLO on the 4060, trained on real photos. Feed its detections plus depth into a fruit localiser in place of the world-file targets.
2. **Camera row-following** from `/front_camera/image`, in place of the ground-truth odometry line.
3. **Unreal** as the picture layer on the Windows laptop, if wanted.
4. **Real measurements from Tejesh:** wheelbase, track, steering linkage, masses, Arm 1 and PETG.

**Known simulation limitations** (by design, documented in `docs/simulation.md`):
- On-plant fruit are static and passable by the robot.
- The cut is a remove-and-respawn.
- Plant stems have no collision.
- Odometry is ground truth.

## Environment notes

- **Windows (where the assets were built):**
  - Blender 5.1 at `C:/Program Files/Blender Foundation/Blender 5.1/blender.exe`
  - Python venv at `.venv` (`tools/requirements.txt`)
  - SolidWorks is available
- **Ubuntu laptop:**
  - SSH from Windows through the `agribot-ubuntu` alias in `~/.ssh/config`, over this laptop's hotspot.
  - Workspace: `~/agribot_ws`, with `src/agribot` linking to `~/AgriBot/sim`.
  - Helpers: `tools/sim/ubuntu/` (`env.sh`, `start.sh`, `stop.sh`, `film.sh`).
- **Capture privacy:** never publish a full-desktop capture. Use only the Gazebo view crop (`make_video.py`), robot camera topics, or the graph drawn from ROS data. A full-desktop shot once caught personal chats.
- **Render GPU:** Blender renders use Cycles on the GPU (OptiX). Render times are 10 s to 2 min per image.
