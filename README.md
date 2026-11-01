# AgriBOT

An autonomous tomato-harvesting rover. It drives tomato rows, finds ripe fruit with its cameras, and harvests it with a SCARA arm on a converted ride-on chassis. The target is a functional field prototype on a budget of about ₹10 lakh.

![AgriBOT in a tomato field](docs/images/hero_field.png)

| | |
| --- | --- |
| ![Cutter closing on a ripe tomato](docs/images/hero_closeup.png) | ![Down the furrow](docs/images/field_overview.png) |

### Running in Gazebo with ROS 2

![Harvest demo in Gazebo](docs/media/agribot_pick.gif)

The rover follows the furrow with its front camera, finds red tomatoes with its arm's RGB-D camera, cuts each one off its pedicel, catches it and drops it in the crate. No target comes from the world file. Gazebo confirms each pick: three of three landed in the crate. Ripeness is not judged yet (a colour rule finds red fruit, half-ripe included); a learned model is next. Watch the [full run (MP4, 4 min, opens a player)](https://github.com/arjun7n9s/AgriBot/blob/main/docs/media/agribot_demo.mp4), or read [docs/simulation.md](docs/simulation.md) for measurements and limitations and [docs/ros-architecture.md](docs/ros-architecture.md) for how the ROS 2 side works.

## What's here

| Area | Status | Start with |
| --- | --- | --- |
| Project plan, purpose, open questions | Current | [docs/project-context.md](docs/project-context.md) |
| Simulation assets: rover, SCARA, cutter, tomato field | v0.1, built on Windows | [docs/assets.md](docs/assets.md) |
| Robot description (URDF/xacro, ROS 2 package) | Builds and runs on Ubuntu | [sim/agribot_description](sim/agribot_description) |
| Field models for Gazebo and Unreal | Running in Gazebo | [sim/agribot_field](sim/agribot_field) |
| Gazebo harvest demo (ROS 2 Humble, Fortress) | Working: camera row-following and fruit detection, 3 of 3 verified picks | [docs/simulation.md](docs/simulation.md), [sim/agribot_gazebo](sim/agribot_gazebo) |
| ROS 2 autonomy stack: nodes, interfaces, parameters, launch, 338 tests | Working; also runs without the simulator | [docs/ros-architecture.md](docs/ros-architecture.md), [sim/agribot_autonomy](sim/agribot_autonomy) |
| SCARA redesign and tomato cutter | Concept, waiting on bench tests | [docs/scara-redesign.md](docs/scara-redesign.md) |
| Simulation pathway | Plan | [docs/work-plan.md](docs/work-plan.md), [docs/ros2-demo-sim-plan.md](docs/ros2-demo-sim-plan.md) |
| Stock rover control board (reverse engineering) | In progress | [docs/rover-control-board.md](docs/rover-control-board.md), [docs/rover-logic-interface.md](docs/rover-logic-interface.md) |
| Bills of materials and orders | Current | [docs/bill-of-materials.md](docs/bill-of-materials.md), [docs/bill-of-materials-priced.md](docs/bill-of-materials-priced.md) |

`notes.md` tracks the open tasks (AB-1 to AB-8). `preferences.md` records the working rules agreed so far.

## Repository layout

```
docs/                     plans, designs, bills of materials, renders (docs/images)
sim/
  agribot_description/    ROS 2 package: URDF/xacro, meshes, frames, display launch
  agribot_field/          ROS 2 package: Gazebo models (tomatoes, plants, beds, soil) + glTF
  agribot_interfaces/     ROS 2 package: RowState, HarvestStatus, VerifyPick, the Harvest action
  agribot_autonomy/       ROS 2 package: row follower, fruit detector, drive node, harvest mission (no Gazebo)
  agribot_gazebo/         ROS 2 package: Gazebo world, ros2_control, cameras, bridge, cut model, filming
tools/
  scara/                  STEP -> URDF links (OpenCascade)
  blender/                procedural assets and renders (Blender 5.1)
  description/            URDF generator and a small FK/IK helper
media/                    board photos and models; SCARA base CAD (see CREDITS.md)
internal/                 working notes and photo cross-checks
```

## The robot in one paragraph

**Chassis:** a Chinese ride-on car body, 1.20 × 0.70 × 0.80 m on 0.38 m tyres. Its stock board drives four wheel motors and a steering motor, and that board is being reverse-engineered for autonomy.

**Arm:** a SCARA arm (L1 228 mm, L2 136.5 mm, 249 mm Z stroke) pins onto a side mast at one of five heights. It carries a V-capture bypass cutter that:
- slides a 6 mm throat onto the thin stem just above the fruit (the distal pedicel)
- slices it at the cheapest point (about 20–34 N)
- catches the fruit in a padded cradle
- drops it into the crate on the rover's deck

**Cameras:** front and rear RGB, an RGB-D camera on the cutter, and a scouting camera on a pan mast. They feed row navigation and a ripe-fruit detector.

## Getting started

Rebuild every asset on Windows (needs Blender 5.1 and Python 3.11). Full commands are in [docs/assets.md](docs/assets.md#rebuilding).

```bash
python -m venv .venv
.venv/Scripts/python -m pip install -r tools/requirements.txt
```

On Ubuntu 22.04 with ROS 2 Humble and Gazebo Fortress (`ros-humble-ros-gz`, `ros-humble-gz-ros2-control`), the packages build like any colcon workspace:

```bash
mkdir -p ~/agribot_ws/src && ln -s "$(pwd)/sim" ~/agribot_ws/src/agribot
cd ~/agribot_ws && colcon build && source install/setup.bash
ros2 launch agribot_gazebo demo.launch.py harvest:=true
```

## Credits

The SCARA base design is adapted from HowToMechatronics (see [CREDITS.md](CREDITS.md)). Its CAD is in `media/scara-arm/`.

## Licence

Apache-2.0 for everything authored here (see [LICENSE](LICENSE)). The adapted SCARA base CAD remains its original authors' work.
