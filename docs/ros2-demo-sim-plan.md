# ROS 2 demo simulation: tomato field harvest

Drafted 31 Oct 2026. This plan builds on [work-plan.md](work-plan.md) and does not replace it. The stack stays the same: Isaac Sim 5.0 on the RTX 5060 (8 GB VRAM, 32 GB RAM), ROS 2 Humble on Ubuntu 22.04, and the fruit model on the RTX 4060. What this plan adds is the presentation: a good-looking field, a rover that drives along a row and harvests ripe fruit, and a visible ROS 2 system behind it.

**Main aim: show ROS 2 working.** The field and the rover are the stage. The real subject is ROS 2: nodes on two machines, topics, an action, tf2, sim time, a launch file, and a recorded bag. Every visual choice below either serves that or stays cheap enough not to compete with it for the 8 GB card.

## What the audience sees

A video of about 90 seconds, plus a live run that can be repeated from one launch command.

| # | Shot | ROS 2 point it makes |
| --- | --- | --- |
| 1 | Golden-hour fly-over of the field. The camera settles on the rover at the head of a row. | None. This is the hook. |
| 2 | The rover drives into the row. A split screen opens: the Isaac view on the left, a dashboard on the right with the tf tree, odometry, and camera feeds. | `/clock`, `/tf`, `/odom`, `/joint_states` streaming from the sim |
| 3 | The scouting camera pans across the plants. Ripe fruit gets highlighted on the arm camera feed. | Image transport across the network to the 4060, `vision_msgs/Detection2DArray` coming back |
| 4 | The rover stops. Close-up: the arm swings out and the cradle settles under a red tomato. | `/harvest/pick` action: goal, feedback, result shown on screen |
| 5 | The cutter snips the stem, the fruit drops into the cradle, and the arm tips it into the crate on the rover. A counter ticks up. | Action result, `/harvest/count` |
| 6 | Cut to `rqt_graph`, with nodes grouped by machine (5060 and 4060). | Distributed ROS 2, one `ROS_DOMAIN_ID` |
| 7 | The rover carries on down the row. Wide shot, fade out. | Mission node chaining the actions |

## Rendering choice

Stay on Isaac Sim 5.0. The work-plan already chose it, and the ROS 2 bridge, the Articulation Controller, and the camera helpers are its parts. The look comes from art direction, not from scene size. Gazebo's renderer would undercut the visual goal. Unreal or Unity would add a second engine and a second ROS bridge.

- **Renderer:** RTX Real-Time with DLSS on Performance, as already measured on this card. Path tracing is not used for the live run.
- **Recording:** first try Isaac's Movie Capture. It renders the frames it steps, so the video can be smooth even if the live frame rate isn't. **To test:** whether the external ROS nodes stay in step during capture when they all run with `use_sim_time:=true`. If they don't, record the live run with OBS instead.
- **VRAM guard:** the work-plan rule still applies. `nvidia-smi` stays near the measured band of about 2.5 GB before the plants are loaded, and the build stops if it climbs toward the 7 GB renderer warning. Add props in the order below and measure after each step.

## Props

### Field

| Prop | Build | Notes |
| --- | --- | --- |
| Raised beds with plastic mulch | One modelled bed segment, repeated with instancing. Black or silver film over soil, with furrow lines. | Plastic mulch beds are typical of commercial tomato growing, and the shiny film catches the golden-hour light |
| Drip line | Thin tube along each bed | Cheap detail that reads as a real farm |
| Bamboo stakes and twine trellis | Instanced stakes, with the twine as a thin curve or texture card | Gives the row a vertical rhythm |
| Tomato plants | 3–4 plant variants, instanced with `PointInstancer`, random yaw and scale | Source: a CC0 or CC-BY model (check the licence before use), or model the variants in Blender. Leaves get alpha-cut cards, not modelled geometry. |
| Hero row | The row the rover drives: about 15–20 plants at full detail with harvestable fruit | The only row with physics fruit |
| Flanking rows | 2 rows each side, same plants, no physics fruit | Fills the side cameras |
| Far field | Large textured cards or a single low-poly merged mesh, faded with distance fog | The field looks endless without paying for it. The AgriGen authors note that plant count is what overloads consumer GPUs. |
| Horizon | HDRI dome (Poly Haven, CC0), golden hour or clear morning, with a tree line and haze | Most of the "elegant" look comes from the light |
| Ground plane | Physics ground under the beds, plus colliders on the bed tops | The rover needs something solid to drive on |

The real-row Gaussian splat in work-plan section 3 is **optional** here. A splat makes a photoreal backdrop but can't be lit to match, and it carries no labels or collision. For a controlled demo, modelled meshes are the main path. If Tejesh scans a row, the splat can go in as one background row.

### Fruit (the most important prop)

- **Geometry:** a slightly flattened sphere with a calyx (the green leafy cap) and a short stem.
- **Materials:** three photo-textured PBR materials (green, half-ripe, ripe) with a subtle highlight and a little subsurface scattering if the card allows it. This is also the camera-match test from work-plan section 9. The detector is trained only on real photos, so the closer the fruit looks to real, the more likely it fires on the sim image.
- **Physics:** each fruit is a rigid body with a sphere collider, held to its plant by a fixed joint at the stem.
- **Labels:** semantic class (`tomato_green`, `tomato_half`, `tomato_ripe`) and a unique id on every fruit.
- **Placement:** ripe fruit in the hero row hangs inside the arm's reach envelope (see "Open questions").

### Rover

- **Body:** a stylised shell modelled in Blender at the measured size (1.20 × 0.70 × 0.80 m, seat at 0.50 m, tyres 0.38 m × 0.15 m), with AgriBOT colours and a decal. The collision and joint skeleton stay the six links and five joints from work-plan section 1. The shell is visual only.
- **Steering:** a single parallel-steer joint, as in the work-plan, until the real linkage has been seen.
- **Sensor props matching the bill of materials:** front RGB camera, rear RGB camera, RGB-D camera on the arm, and the scouting camera on a pan mast. Only the arm camera publishes images at first (see the camera budget below). The others are modelled so the robot looks like the real build, and the pan mast still turns.
- **Harvest crate:** a plastic crate on the rear deck. Fruit lands in it with real physics.

### Arm and end effector

- **SCARA:** the HowToMechatronics meshes from `media/scara-arm/` in a two-tone PLA finish. Joints follow work-plan section 1: J1 and J2 revolute, Z prismatic.
- **End effector:** a **sim placeholder**, labelled as such until AB-4 settles the real design. It follows the harvester concept in the bill of materials (items 2.6–2.8): a soft cradle that slides under the fruit, a blade that cuts the stem, and a servo-driven trap door that drops the fruit into the crate.

## How the pick works in the sim (honestly)

The robot software never tells the world what to do. It only commands its own joints. The world reacts to geometry.

1. The arm places the cradle under the target fruit and the blade beside its stem.
2. The arm node closes the cutter joint.
3. A small Isaac-side script watches the cutter. When the cutter joint is closed **and** the blade is within a set distance (about 1 cm) of a stem joint, it disables that fruit's stem joint.
4. Gravity drops the fruit into the cradle. That's PhysX, not a scripted attach.
5. The arm swings over the crate and opens the trap door. The fruit falls into the crate.
6. A crate trigger volume counts arrivals and publishes `/harvest/count` (`std_msgs/UInt32`).

This keeps "cutting" honest. If the arm misses the stem, nothing is cut, and the action reports a failure.

## ROS 2 system

### Machines

- **RTX 5060 (Ubuntu 22.04):** Isaac Sim 5.0 using its internal Humble libraries, plus the robot nodes on the system's Humble.
- **RTX 4060:** the detector, plus the dashboard and RViz.
- **Network:** a wired Ethernet link between the two laptops. Wi-Fi multicast discovery is often unreliable. Fast DDS, the default, works over the cable with the same `ROS_DOMAIN_ID`.
- **Images:** cross the network compressed (`image_transport` republish to `compressed`), not raw.

### The boundary with Isaac

Only stock message types cross into Isaac, because the bridge runs Python 3.11 and custom interfaces would have to be built twice (work-plan section 4).

| Direction | Topic | Type | Isaac OmniGraph node |
| --- | --- | --- | --- |
| Isaac → ROS | `/clock` | `rosgraph_msgs/Clock` | ROS2 Publish Clock |
| Isaac → ROS | `/joint_states` | `sensor_msgs/JointState` | ROS2 Publish Joint State |
| Isaac → ROS | `/tf` | `tf2_msgs/TFMessage` | ROS2 Publish Transform Tree |
| Isaac → ROS | `/odom` | `nav_msgs/Odometry` | Isaac Compute Odometry + ROS2 Publish Odometry |
| Isaac → ROS | `/arm_cam/rgb`, `/arm_cam/depth`, `/arm_cam/camera_info` | `sensor_msgs/Image`, `CameraInfo` | ROS2 Camera Helper |
| Isaac → ROS | `/arm_cam/semantic` | `sensor_msgs/Image` | ROS2 Camera Helper (semantic). Used for ground truth and the fallback only. |
| Isaac → ROS | `/harvest/count` | `std_msgs/UInt32` | Crate trigger script |
| ROS → Isaac | `/joint_command` | `sensor_msgs/JointState` | ROS2 Subscribe Joint State → Articulation Controller |

### Nodes (external, system Humble)

| Package | Node | Does | Interfaces |
| --- | --- | --- | --- |
| `agribot_description` | `robot_state_publisher` | Publishes the URDF and the static tf | `/robot_description` |
| `agribot_drive` | `drive_controller` | Converts `/cmd_vel` (`Twist`) into 4 equal wheel speeds plus 1 steering angle | Subscribes `/cmd_vel`, publishes `/joint_command` |
| `agribot_drive` | `row_follower` | Pure-pursuit control along the row centreline, using `/odom` | Action `/follow_row` |
| `agribot_perception` | `tomato_detector` (on the 4060) | YOLO11-seg trained on real photos (work-plan section 5) | Subscribes `/arm_cam/rgb/compressed`, publishes `/tomato/detections` (`vision_msgs/Detection2DArray`) |
| `agribot_perception` | `fruit_localizer` | Detection + depth + `camera_info` → 3D fruit pose in `base_link` via tf2 | Publishes `/tomato/targets` (`geometry_msgs/PoseArray`) and RViz markers |
| `agribot_arm` | `scara_controller` | Closed-form two-link inverse kinematics plus Z; runs the cradle → cut → crate sequence | Action `/harvest/pick`, publishes `/joint_command` |
| `agribot_arm` | `scout_pan` | Sweeps the scouting camera mast | Publishes `/joint_command` |
| `agribot_mission` | `mission` | Drives the row, stops at a target, picks, and repeats | Action client for `/follow_row` and `/harvest/pick` |
| `agribot_bringup` | launch, params, dashboard layout, RViz config | One command: `ros2 launch agribot_bringup demo.launch.py` | — |

**Custom interfaces** (`agribot_interfaces`), used only between external nodes:
- `FollowRow.action`: goal is the row id and stop distance; feedback is distance along the row.
- `PickFruit.action`: goal is the target pose and fruit id; feedback is the phase (approach, cradle, cut, transfer, release); result is success or failure plus a reason.

**Two joint-command publishers.** `drive_controller`, `scara_controller` and `scout_pan` each publish only their own joint names on `/joint_command`. Check in step 4 below that the Articulation Controller accepts partial `JointState` messages from several publishers. If it doesn't, give each one its own topic and its own Articulation Controller node.

**Detector fallback, stated openly.** If the real-photo detector doesn't fire on a sim tomato, `fruit_localizer` uses the semantic image instead. The overlay is then labelled `GT`, and the log records which source was used. This is the work-plan's rule: never report a silent fallback as a detection.

### What the dashboard shows

**Foxglove** (`foxglove_bridge`), or **Lichtblick** (its open-source fork) if Foxglove's licence doesn't fit, on the 4060:
- Arm camera with mask overlays and confidence
- 3D panel: URDF, tf, fruit target markers, the row path
- Plots of joint positions versus commands
- `/harvest/pick` feedback phase
- Harvest counter

**RViz2** is the backup. **rqt_graph** gives the architecture shot.

**rosbag2** records every run (minus the raw images) so the run can be played back without the sim. That's a second ROS 2 point worth showing.

## VRAM and camera budget (RTX 5060)

| Item | Status |
| --- | --- |
| Base renderer, DLSS Performance | Measured at about 2.5 GB in the Qiita setup |
| Field: beds, instanced plants, far cards, HDRI | Measure after adding it |
| Arm RGB-D camera, 640 × 480 | One render product. The work-plan's one-camera rule. |
| Semantic output on the same camera | Measure. Remove it if it pushes VRAM out of the band. |
| Front and rear cameras publishing | **Not at first.** Add one at a time only if the band holds. |
| Scouting camera publishing | Visual only. It pans but doesn't publish. |

## Build order

Each phase ends with a check. Phases 1–4 are work-plan steps 1–9, with the visuals moved later on purpose: get everything working first, then make it look good.

1. **Rover and arm move under ROS.** Work-plan steps 1–7 plus 9 (description, drive, arm), on a flat grid with grey boxes.
   - Check: `ros2 topic echo /joint_states` shows all joints, and `/cmd_vel` and the arm command each move the robot.
2. **Pick loop on a test stand.** One staked plant, three fruit, the placeholder end effector, and the stem-cut script.
   - Check: `/harvest/pick` cuts a ripe fruit and lands it in the crate 5 times out of 5. A deliberately bad goal fails cleanly.
3. **Perception across two machines.** Detector on the 4060, localizer, fallback labelling.
   - Check: `rqt_graph` shows both machines, and target poses land within 2 cm of the true fruit pose.
4. **Mission.** `mission` chains row-following and picking along a 5-plant grey row.
   - Check: one launch command, no manual steps, and a rosbag of the run.
5. **Art pass.** Beds, mulch, stakes, plant variants, the hero row, flanking rows, far cards, HDRI, fog, the rover shell, materials. Measure VRAM after each one.
   - Check: the band holds and the phase 4 mission still passes unchanged.
6. **Dashboard and shots.** Foxglove layout, cinematic camera paths for shots 1 and 7, a Movie Capture test, then record and edit.
   - Check: a 90-second video and a live rerun on request.

## Open questions and blockers

- **AB-2 (Ubuntu 22.04 on the 5060) blocks everything.** The work-plan rule still holds: no sim scripts until Tejesh is on Ubuntu.
- **Arm reach versus fruit height.** The SCARA's reach and the arm's mounting height on the rover aren't recorded in this folder. Measure the J1–J2 and J2–tip distances from the STEP, choose a mount, then place the hero row's ripe fruit within that envelope. Staked tomato plants carry fruit higher than a 400 mm Z travel can reach from a fixed mount, so a real field will need a mast or riser. For the demo, say which one is shown.
- **Rover wheelbase, track and steering stop** are still the assumed values from work-plan section 1 (0.72 m, 0.55 m, ±25°). Replace them when the car is measured.
- **Steering linkage** is unknown, so the sim uses a single parallel-steer joint for now.
- **End effector** is a sim placeholder until AB-4.
- **Tomato plant asset:** the source and licence are not chosen yet.
- **Movie Capture** keeping sync with ROS is untested.
- **Multiple publishers on `/joint_command`** is untested (see the nodes section).
