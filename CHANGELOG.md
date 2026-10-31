# Changelog

## 0.1.1 (31 Oct 2026)

### Added

- **SCARA CAD and meshes:** the SCARA base CAD (unpacked) and its generated meshes are now in the repository, so the robot description is complete on a fresh clone.
- **Credit:** a CREDITS.md entry for the adapted base design.

## 0.1.0 (31 Oct 2026)

### Added

- **Project context:** plans, bills of materials, order lists, the control-board reverse-engineering notes and photos, the SCARA redesign, and the ROS 2 demo plan.
- **SCARA arm description:**
  - Split from the HowToMechatronics STEP into five links plus the stock gripper, in ROS frames.
  - Forward kinematics checked against the STEP to 0.002 mm.
  - Measured Z stroke: 249 mm.
  - Inertias estimated from mesh volumes.
- **Tomato cutter, concept v0:**
  - V mouth, 6 mm throat, hooked bypass blade, cradle and trap door.
  - RGB-D camera mounted on the cutter.
  - Blade and trap door are separate URDF joints.
- **Rover body:**
  - Stylised chassis at the measured size, with curved fenders, tread tyres, light bar and e-stop.
  - Front and rear camera pods, a scouting mast with pan head, and a harvest crate.
  - SCARA side mast with five pin heights.
- **Robot description:**
  - `agribot_description` ROS 2 package with xacro macros per subassembly and a top-level `agribot.urdf.xacro`.
  - xacro args: `tool:=cutter|gripper`, `mast_pin:=0..4`.
  - Expanded URDFs and an RViz display launch.
- **Tomato field:**
  - `agribot_field` ROS 2 package with Gazebo models and glTF copies.
  - Tomato fruit at three ripeness stages; origin at the knuckle, plus a cut-point frame.
  - Three staked plant variants, each with fruit-site lists.
  - Tileable raised mulch beds and a soil tile.
  - Field layout file and semantic labels.
- **Tooling:**
  - Asset generators: OpenCascade for the STEP, Blender for procedural assets and renders.
  - URDF generator with structural checks.
  - Minimal URDF FK/IK helper.
  - Renders in `docs/images`.

### Changed

- **SCARA mounting:** the SCARA base now runs along the rover's side. Pointing it outward put the base electronics box in the fruit zone.
- **SCARA Z stroke:** the measured 249 mm replaces the ~200 mm estimate in `docs/scara-redesign.md`.

### Not yet verified

- `colcon build`, RViz and Gazebo loading on Ubuntu 22.04 with ROS 2 Humble.
- Gazebo plugins: `ros2_control`, cameras, detachable joints, labels.
