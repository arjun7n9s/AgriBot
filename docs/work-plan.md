# Simulation build

The sim is the proof. Hardware stays parked. The stack that was measured stays: Isaac Sim 5.0 on the RTX 5060 (32 GB RAM), DLSS 4 set to Performance, one tomato row, the RTX 4060 training the fruit model. The cards do not share VRAM. This plan does not reopen that.

## The method

This is not a copy of the tomato-harvest papers. Those papers hand-label images, run a heavy detector, then search a path for a 5- or 6-axis arm. We do not.

1. One real row is scanned into a small splat. Each fruit is marked once in 3D: center, pedicel, and ripeness. That mark is the label for every camera view. Nobody draws a box on each frame. Fujinaga’s 2026 tomato paper did the box version of this in Unreal. We keep the free labels and add the pedicel, because a box cannot tell the arm where to cut.
2. The student model is a small detector with a keypoint head, distilled from a frozen vision foundation model on unlabeled row frames, then fine-tuned on a small labeled set. Khan et al., Scientific Reports, 3 July 2026, showed that distilling DINOv3 into YOLOv12n gets most of the accuracy from about 20% of the boxes. Full hand-labeling still wins. We spend the budget on our fruit, not on labeling thousands of frames. The 4060 runs this. YOLO11x-seg on Laboro is the baseline to beat, not the model we train.
3. Motion is closed form. Four wheels share one speed. Steering is one angle. The SCARA (two revolute joints and one vertical slide) has an analytic inverse. Ansari et al., Scientific Reports, 9 April 2026, used particle-swarm search because their arm has five joints and no closed form. Copying that search onto a SCARA wastes the mechanism. A search-based planner is only added if the analytic move misses.
4. The sim’s pass is geometric: the arm tip lands on the pedicel we placed. The model’s pass is a held-out set of real photos. A model that only scores well on the images it was labeled from has failed.

Their gripper (six soft fingers, a cutter, force sensors, an 80% lab pick) is a later mechanical design. It is not this sim. What we keep from that paper is the target: fruit center and pedicel, not a bounding box alone.

The Isaac Sim 5.0.0 HTML paths return 404 on 10 Oct 2026. The pages that opened for the importer, the robot wizard, the Ackermann tutorial, the ROS install, and the requirements are the 5.1.0 docs of the same tools. The simulator we run is still 5.0.0, because that is the version in the RTX 5060 laptop write-up. Where 5.0 and 5.1 differ, the step says so.

## 1. Where and how the rover and the SCARA get modeled

### Rover

There is no car CAD. "Exact" here means the joint count and the measurements of the car in front of us, in meters. Isaac's default unit is meters. A body copied from a picture, or from NVIDIA's Leatherback (wheelbase 0.32 m, track 0.24 m, wheel radius 0.052 m), fails this step.

Measured by Tejesh, 10 Oct 2026:

- Length 1.20 m
- Width 0.70 m, as taped. Not yet known whether that tape crossed the body or the outside of the tyres, so it is not the wheel track yet.
- Seat height 0.50 m from the ground
- Total height 0.80 m from the ground
- Tyre diameter 0.38 m, so radius 0.19 m. Tyre width 0.15 m

Catalogs for cars of this size publish length, width, and height. They do not publish axle spacing or steering angle. The two missing numbers are assumed from the tape, 10 Oct 2026, and can be replaced when the car is measured:

- Width 0.70 m is treated as outside-of-tyre to outside-of-tyre. Track, wheel-center to wheel-center, is 0.70 − 0.15 = 0.55 m.
- Wheelbase is 0.72 m. That is the 1.20 m length with one tyre radius of body past each axle (0.19 m front, 0.19 m rear).
- Steering stop is ±25°. Toy linkages stop well short of a real car’s lock.

The file is a URDF, then a USD. Six links:

- `base_link`: a box for the visual, a box for the collision.
- `steer_link`: one revolute joint off the chassis. This is the one steering motor.
- four wheel links: continuous joints. The two front wheels are children of `steer_link`, so one motor yaws both. The two rear wheels are children of `base_link`.

That is parallel steering. It matches one motor and a linkage we have not seen. The project record still says the linkage may be a bell-crank or a push-rod, and that neither is locked until it is seen. A second knuckle joint would invent a motor the car does not have.

Import with File > Import and `isaacsim.asset.importer.urdf`. For a mobile robot the importer page says: Moveable Base, velocity drive on the wheels, position drive on the steering joint, stiffness left at zero on a velocity drive. The output USD is the asset the stage references.

The nearby alternatives, and why they lose:

- The CAD converter. It is for Catia, SolidWorks, AutoCAD, and Creo files, and it produces visual meshes. There is no car file to feed it. On a robot, a CAD-converter USD is the wrong stop: in August 2025 a user opened one in Isaac 5 and could not rig it.
- The Robot Wizard. It is in beta, and the 5.1 page limits it to a robot already sitting on the stage, with few links. The same page says a URDF goes through File > Import. The wizard is the fallback if the importer drops the joints, not the authoring tool. Five joints is inside its stated scope, so it can repair a bad import. It cannot invent the measurements.
- Leatherback, Carter, or any Isaac wheeled asset used as our rover. Those are other mechanisms. Leatherback's graph drives two knuckle joints by name.

### SCARA

The CAD is the HowToMechatronics export in `media/scara-arm/`: `SCARA Robot 3D Model.STEP` (21.74 MB), `solidworks/SCARA Robot.SLDASM`, and the STL meshes. The public HowToMechatronics tutorial URL tried on 10 Oct 2026 returns 404, so the joint count comes from those files, not from that page.

The moving bodies, read off the part names:

- Base, fixed to the rover later.
- Arm 1, revolute, the J1 pulley and coupler.
- Arm 2, revolute, the J2 pulley and coupler.
- The Z carriage, prismatic. The D8 lead screw and nut are the transmission, not a second joint.
- The gripper, one revolute, the MG996R.

Belts, pulleys, bearings, bolts, and the Arduino cases are not rigid bodies. They are parented to the link they ride on, or left out of the physics file. A belt in PhysX is not how this arm moves. The tooth counts on the pulley filenames (110, 92, 90, and the 80-tooth mates) become a scale on that joint's command once the mating pair is read off the assembly. They are not simulated teeth.

"Exact" means those visual meshes, joint origins taken from the STEP, and one collision shape per link. It does not mean the 110-tooth pulley is a collision mesh. The URDF tutorial says collision meshes are often simpler than the visual ones, and the reference architecture lists convex hull, convex decomposition, and bounding box as the approximations. Wheels, on the rover, are the case PhysX wants as a cylinder. The arm links get a convex hull.

The path that a SolidWorks user marked solved on the NVIDIA forum in August 2025, for Isaac 5:

1. Collapse the assembly to one part per joint.
2. Put a reference point, axis, and frame on each joint, in ROS coordinates (x forward, y left, z up).
3. Assign a material so the inertia is not a guess from a hollow mesh.
4. Export URDF, open it in RViz, and only then import it.

If SolidWorks is not on the Ubuntu machine, the same URDF is written by hand against the STL files, with origins measured from the STEP. FreeCAD is a way to read STEP. No Isaac page I opened names FreeCAD, so that viewer is a local convenience, not part of the Isaac workflow. The Onshape importer exists and keeps a part structure, but mates are not supported and it would mean redrawing an arm we already have.

Import the arm with Static Base while it is tested alone. Self-collision stays off on this card. The Franka GUI tutorial turns it on; the Franka Python example turns it off. An 8 GB card is already under the published GPU floor, so the cheaper setting is the one we start with.

Arm 2 and the gripper are the unprinted parts and may still be redesigned. The sim uses the meshes that exist. A new Arm 2 or gripper replaces those two meshes and the joint that hangs on them. It does not rebuild the rover.

The CAD converter is the wrong finish for this arm, for the same reason as the August 2025 thread: the USD opens, and it has no joints.

Sources:

- [URDF import, Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/import_urdf.html) — Moveable Base, velocity drive for wheels, position drive for a steering joint. Collision meshes are the simplified ones. Static Base is the arm case.
- [Reference architecture, Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/introduction/reference_architecture.html) — assets become USD, units are meters, CAD converter is for visual conversion from SolidWorks and the other named CAD systems, URDF importer carries joints.
- [Robot Wizard, Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/robot_wizard.html) — beta, few links, robot must already be on the stage. A URDF uses File > Import.
- [SolidWorks into Isaac 5, NVIDIA forum, Aug 2025](https://forums.developer.nvidia.com/t/isaacsim-importing-robot-from-solidworks/343211) — CAD-converter USD could not be rigged. The solved path is one part per joint, reference frames, material, RViz, then import.
- [Ackermann tutorial, Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_ackermann_controller.html) — Leatherback's two knuckle joints and the 0.32 / 0.24 / 0.052 geometry. Those numbers are that asset's, not ours.

## 2. How those models get motion

PhysX runs the joints. The reference architecture says an articulation is the robot case, and that a scene with one robot should use the CPU solver. The GPU stays on the picture. A joint drive applies effort from stiffness and damping: position error times stiffness, plus velocity error times damping. The Gain Tuner is the tool that sets those two numbers, either directly or from a natural frequency and a damping ratio.

Drive type, from the importer page and the gain-tuning tutorial:

- Four wheel joints: velocity drive. Stiffness stays 0. Damping is raised until the wheel speed matches the command and the chassis does not shake.
- Steering joint: position drive.
- Arm 1, Arm 2, the Z prismatic, and the gripper: position drive.

The first thing that talks to those drives is the Articulation Controller. In the Action Graph it takes a position command or a velocity command, not both on the same joint, and it runs from On Playback Tick. Joint names are the URDF names. Angles in the controller are radians.

Before any ROS node exists, the rover check is four equal wheel velocities and one steering position, on Create > Environments > Flat Grid. Equal velocities, because the stock board drives the four wheel motors together. Per-wheel torque is a different car.

Differential drive loses. On the Isaac forum, NVIDIA's reply to a four-wheel forklift with swivels was that differential drive is for two driven wheels and no swivel. This car has four driven wheels and a steering motor.

The Leatherback Ackermann graph also waits. It is the documented ROS path for a car, and it is the right family once we know the linkage: an `AckermannDriveStamped` on `ackermann_cmd`, an Ackermann Controller node, then two Articulation Controllers, one for steering and one for the four wheel speeds. Using it now means commanding `Knuckle__Upright__Front_Left` and `Knuckle__Upright__Front_Right`, which we do not have, with Leatherback's wheelbase and track. After the linkage is measured, that graph is wired to our joint names. If the linkage is a rigid axle, the controller's two angles collapse back to the one steering joint we already have. If it is a trapezoid, the two angles are what the linkage produces from that one motor, and the URDF gains a constraint for the second knuckle. We do not guess which one today.

The arm, before ROS, is four position commands. A point in front of the rover becomes shoulder angle, elbow angle, and Z. Isaac's worked manipulator example is RMPFlow, and the Python sample is written for a Franka. MoveIt is supported through the ROS bridge, and cuMotion is the CUDA planner in Isaac ROS. Both are later. Neither page I opened gives a SCARA closed-form solution, so that geometry is ours: two-link planar IK plus the lead screw. The pass is the gripper frame against a marker, not a slider that moves.

Gains start from the UR10 tutorial's critically damped pair, natural frequency 0.5 and damping ratio 1.0, tested in the Gain Tuner with a step. If the arm sags, natural frequency goes up. If it rings, damping ratio goes up. The Franka import tutorial raises natural frequency when the arm oscillates. Gravity can be turned off inside the tuner to see the joint alone, then turned back on for the pass.

Sources:

- [Articulation Controller, Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/articulation_controller.html) — position, velocity, or effort, one method per joint, OmniGraph or Python.
- [Gain Tuner](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html) and [joint-drive tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/joint_tuning.html) — stiffness and damping, velocity drive with stiffness at zero.
- [Configure a manipulator](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/tutorial_configure_manipulator.html) — natural frequency 0.5, damping ratio 1.0, as the critically damped start.
- [Reference architecture](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/introduction/reference_architecture.html) — CPU solver unless the scene has hundreds of bodies. MoveIt is the bridge path for manipulation. cuMotion is named under Isaac Manipulator.
- [Differential drive is the wrong four-wheel model, NVIDIA forum](https://forums.developer.nvidia.com/t/asking-your-opinion-on-differencial-drive-for-isaac-sim-5-0-0/346497) — two driven wheels, no swivel. A swivel belongs on the Ackermann side.
- [Ackermann tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_ackermann_controller.html) — the graph we adopt after the linkage is known.

## 3. The farm

No tomato-row environment that I could open both exists and states that it runs in Isaac Sim on 8 GB.

What does exist:

- AgriGen, an Isaac Sim framework with ROS, procedural rows, and RTX rendering, presented as an ICRA 2026 workshop talk. The released crops are lemon, apple, cabbage, sugar beet, and grape. Tomato is not in the asset table. The authors say thousands of plants are intractable on consumer hardware, so they stream tiles. Their smallest measured GPU is an RTX 2080 Ti, about 22 FPS, and the frame rate did not care how large the world was because only the nearby tiles are loaded. There is no 8 GB number. A photoreal Isaac field of instanced plants is the picture this card already cannot host.
- LCAS `aoc_tomato_farm` is a real tomato glasshouse: rows, plants per row, spacing, fruit placed at random, Gazebo and Unity, ROS 2 Humble, Ubuntu 22.04. It does not load in Isaac Sim.
- The 2026 CMES geospatial pipeline builds Gazebo worlds and ships tomato models next to corn, cabbage, and sunflower. Also Gazebo. No Isaac scene, no 8 GB Isaac number.
- `dueiras/agriculture_bot` ran Isaac Sim 4.5 and ROS 2 Humble on an RTX 4070 with 8 GB and 32 GB RAM, using a Nova Carter in an agricultural scene. The stage is not in the repository. It is evidence that a small Isaac ag scene has fit on 8 GB, on 4.5, not a tomato row we can open in 5.0.
- The NuRec examples in the Isaac 5.1 docs are a cafe, a lab, a wormhole, and a lounge. Indoor. The NuRec reconstruction product wants more than 24 GB, and recommends more than 48 GB.

So the row is built, not downloaded.

The picture is a 3D Gaussian splat of one real tomato row, trained and exported the way NVIDIA's October 2025 Isaac Sim 5.0 tutorial describes:

1. Walk the row with a phone. JPEG, locked white balance, about 60% overlap. One row, not the field.
2. COLMAP, pinhole camera, on the 5060. This step wants the 32 GB of RAM. Isaac is closed.
3. 3DGUT, config `apps/colmap_3dgut_mcmc.yaml`, `export_usdz.enabled=true`, `export_usdz.apply_normalizing_transform=true`. The USDZ is what Isaac Sim 5.0 imports. A raw gsplat PLY is not that file.
4. File > Import into an empty stage. Add Create > Physics > Ground Plane and align it to the floor. The reconstruction has no collision. Point the NuRec volume's proxy at that ground mesh and turn on Matte Object so shadows have a surface.

3DGUT's README, opened 10 Oct 2026, supports CUDA 11.8, 12.4, 12.6, 12.8, and 13.0 as experimental, and it calls out CUDA 12.8 for Blackwell. The Qiita 5060 machine is CUDA 13.0, so the install is the 12.8 Blackwell path or the experimental 13.0 path. The October 2025 blog still says CUDA 11.8. The README is the later page. The README does not publish an 8 GB training figure. What is published, on a different trainer, is gsplat at 30k iterations: bonsai 2.29 GB, counter 2.23 GB, kitchen 3.17 GB, bicycle 10.58 GB, garden 9.88 GB. The 2.0 GB line in that table is four GPUs in one machine. A bicycle-sized splat does not fit. One row has to stay near the bonsai end of that table, and then survive the conversion to USDZ. In May 2026 a 1.17 GB 3DGUT USDZ imported into Isaac Sim 5.1 and a 2.7 GB one did not. NVIDIA's reply was to check VRAM, not to raise a format limit. The pass is a USDZ that displays, with `nvidia-smi` still in the band of the Qiita sample.

The splat is the camera picture. It is not a collider, and a separate test (Evercoast, static splats in Isaac) found splats absent from lidar. The ripe fruit the arm reaches is a small rigid mesh with a pose, sitting on the row, three colors for green, half-ripe, and ripe. Labels live on those meshes. The NVIDIA blog's proxy is the ground. Locahun's later note, written for native splats in Isaac Sim 6, says the same split: appearance from the splat, semantics and collision from a mesh. Isaac Sim 6 is not our version. The split is.

Mesh fallback, if the USDZ is black or the trainer will not install: one bed, the ground plane, a handful of low-poly plants, and the same three fruit meshes. No MDL glass, no second camera, no lidar. AgriGen's own lesson is that the plant count is what breaks a consumer GPU, so the fallback is one row of simple shapes. Those shapes will not look like a field. They are good enough to drive along and to give the arm a pose. They are not a training set.

Sources:

- [AgriGen, arXiv HTML](https://arxiv.org/html/2609.25725) and [project page](https://baj31415.github.io/agrigen/) — Isaac Sim, row crops, no tomato in the asset table, RTX 2080 Ti as the smallest measured GPU.
- [LCAS aoc_tomato_farm](https://github.com/LCAS/aoc_tomato_farm) — tomato rows in Gazebo and Unity, ROS 2 Humble.
- [CMES 2026, Gazebo crop worlds including tomato](https://www.techscience.com/CMES/v147n2/67506/html)
- [agriculture_bot](https://github.com/dueiras/agriculture_bot) — Isaac Sim 4.5, RTX 4070 8 GB, scene not shipped.
- [Reconstruct a scene for Isaac Sim 5.0, NVIDIA, 23 Oct 2025](https://developer.nvidia.com/blog/reconstruct-a-scene-in-nvidia-isaac-sim-using-only-a-smartphone/) — phone, COLMAP, 3DGUT USDZ, ground plane, proxy.
- [3DGUT / 3DGRUT README](https://github.com/nv-tlabs/3dgrut) — CUDA versions, including 12.8 for RTX 50. No 8 GB training number.
- [gsplat memory by scene](https://github.com/isl-org/gsplat/blob/main/docs/source/tests/eval.rst) — bonsai 2.29 GB versus bicycle 10.58 GB at 30k.
- [NuRec hardware](https://docs.nvidia.com/nurec/basics/hardware.html) — more than 24 GB.
- [NuRec scenes in Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_nurec.html) — indoor samples, collision plane added when the scene has none.
- [1.17 GB USDZ imported, 2.7 GB did not, May 2026](https://github.com/isaac-sim/IsaacSim/discussions/607)
- [Splats are not lidar returns](https://insights.evercoast.ai/p/integrating-4d-gaussian-splats-into)

## 4. The robotics system

ROS 2 Humble, on Ubuntu 22.04, with the Humble libraries shipped inside Isaac Sim 5.0.

That is the pair the install page loads by itself on Ubuntu 22.04, and Ubuntu 22.04 is the OS in the RTX 5060 write-up that measured this card. Isaac Sim's bridge is compatible with Humble and Jazzy only. Isaac speaks Python 3.11. System Humble is Python 3.10, so the external nodes stay on the system interpreter and the bridge uses the internal libraries. DDS carries the messages across that split. Custom messages, if we ever need one, have to be built twice, once for 3.11. Stock messages (`sensor_msgs/Image`, `sensor_msgs/JointState`, `geometry_msgs/Twist`, `ackermann_msgs`) do not.

Jazzy is the other distro on that page, and it is the recommended one on Ubuntu 24.04. On Isaac Sim 5.0 a user got navigation working with Humble in Docker and not with native Jazzy. The reply on that thread says the fix landed in Isaac Sim 5.1. We are not moving off 5.0 to pick up Jazzy. Jazzy on Ubuntu 22.04 is a source build, and the install page says that workspace has to be built in Docker.

Lyrical, released 22 May 2026 and supported to May 2031, is the current ROS LTS. It is not in the Isaac Sim 5.1 compatibility table. Humble is supported to May 2027. Jazzy to May 2029. Kilted ends December 2026. None of those dates puts Lyrical on this bridge.

The modern choice, in one sentence: ROS 2 Humble with Isaac Sim 5.0's internal Humble libraries, and the ROS 2 bridge Action Graph as the layer that moves joints.

Isaac ROS is the other name people reach for, and it is the wrong layer for this job. NVIDIA's description is perception, localization, mapping, and CUDA motion planning (cuVSLAM, cuMotion), aimed at Jetson as well as a workstation. cuVSLAM wants stereo. We have one camera. The current Isaac ROS line is tested with Isaac Sim 6.0; 5.0 and 5.1 are listed as legacy and still supported. Putting cuVSLAM or nvblox on the 5060 puts a second GPU stack next to a renderer that already sits at about 2.5 GB before the plants.

`ros2_control` is the layer that will matter on the real motor board, because the same controller YAML is what you swap onto hardware. It is not the first layer in Isaac Sim 5.0. The 5.1 doc path for a ros2_control tutorial returns 404. The tutorial that opened is on the current docs, which are the 6.x set, and it hosts a Controller Manager inside the process. The community sample that actually wires `ros2_control` to Isaac does it with topics, because Isaac's Python is its own environment. That sample's main branch is Isaac Sim 6.0.1 and Jazzy. The 5.x line is an older commit (`d03e96e` and earlier). We use it after the bridge already moves the joints, not before.

Layers, in the order they go on, each one subscribed to the sim:

1. `robot_state_publisher` from the URDF, and joint states out of the bridge.
2. A command into the rover's Articulation Controller: wheel velocities and one steering position. `Twist` is acceptable once a node converts it. `ackermann_cmd` replaces that conversion after the linkage is measured.
3. The same pattern for the four arm joints.
4. One RGB camera, ROS 2 Camera Helper, topic `rgb`. Depth is a second Camera Helper on `depth`, added only if the card is still in the small-scene band.
5. The detector, on the 4060, subscribed to `rgb`.
6. A ripe detection, or the fruit prim's pose when the detector does not fire on the sim picture, becomes the arm's target point.

Fast DDS is the bridge default. Cyclone DDS is supported and we do not need it.

Sources:

- [ROS 2 install, Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html) — Humble on Ubuntu 22.04, Jazzy on Ubuntu 24.04, Python 3.11 inside the bridge, DDS across the split.
- [Isaac Sim 5.0 native Jazzy navigation failed, fixed in 5.1](https://forums.developer.nvidia.com/t/isaac-sim-5-0-ros2-navigation-doesnt-work-with-ros2-jazzy-on-ubuntu-24-04-with-latest-updates/347957)
- [ROS 2 releases](https://docs.ros.org/en/jazzy/Releases.html) — Humble to May 2027, Jazzy to May 2029, Lyrical to May 2031.
- [Isaac ROS](https://developer.nvidia.com/isaac/ros) — perception and CUDA planning, not the joint drive.
- [Isaac ROS release notes](https://nvidia-isaac-ros.github.io/releases/index.html) — current testing is Isaac Sim 6.0; 5.0 and 5.1 remain supported as legacy.
- [ros2_control tutorial on the current Isaac docs](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/robot_control/tutorial_ros2_control.html) — in-process Controller Manager. This page is the 6.x docs. The 5.1 URL 404s.
- [topic_based ros2_control sample](https://github.com/hijimasa/isaac-ros2-control-sample) — main is Isaac Sim 6.0.1 and Jazzy; 5.x is `d03e96e` and earlier.
- [RGB camera graph](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera.html) — Camera Helper, topic `rgb`, depth on its own topic.

## 5. AI models

One model. A YOLO11x-seg segmenter, three classes, the Laboro names: green, half-ripened, fully ripened. The public training report that matches this card is `yolo11-tomato-segmentation`: YOLO11x-seg, 62M parameters, 640×640, batch 4, 100 epochs, AdamW, about 6 GB on an RTX 4060, 90.1% box mAP@0.5, 89.8% mask mAP@0.5, 28 ms. Laboro as that repo used it is 2,001 training images, 86 validation images, 898 instances, CC BY 4.0.

86 validation images is not a field trial. The training set is Laboro plus photographs we take of the same three stages, of the fruit this rover will see. A held-out set of those photographs is never shown to the video model below. The 90.1% figure is that repo's number on its split. Ours is whatever the held-out photos score.

What it must not be trained on: fruit that only exists as an Isaac color, AgriGen frames, or Gazebo frames. A classifier fit to a made-up fruit color will pass the sim and miss the row.

The exception is a real scanned row. Fujinaga, Ushiroji, and colleagues (Smart Agricultural Technology, 2026, [10.1016/j.atech.2026.101947](https://doi.org/10.1016/j.atech.2026.101947)) rebuilt a tomato scene from a harvesting robot’s own camera with 3D Gaussian splatting, placed that scene in Unreal Engine 5, randomized fruit position, ripeness, and light, and wrote YOLO boxes automatically from the known fruit poses. Transfer learning on that set detected tomatoes in real photos. That is the synthetic data we are allowed to add: a scan of a real row, fruit and light varied, labels from the 3D poses, then a held-out set of real photos. Their engine was Unreal. Ours stays Isaac for the wheels and the arm, because their paper does not drive a rover or a SCARA. The full ScienceDirect PDF was not open when this was checked. The method is from the abstract and the Osaka Metropolitan University account of the same paper.

Extra frames are allowed from 8-bit LTX-Video, image-to-video, starting from one of those real photos. The Q8 fork says a 720×480×121 clip in under a minute on an RTX 4060 laptop with 8 GB, and the run command is `--low_vram` with `--input_image_path`. Lightricks' own README repeats the 4060 line. Frame count has to be a multiple of 8, plus 1. A person looks at the clip before any frame is labeled. Generated frames stay a minority of the set. If the color of a fruit drifts, the frame is thrown out. The Q8 author also says 8 GB is enough to fine-tune the 2B model. We are not fine-tuning it.

The 4060 recipe, one GPU, Isaac not running:

- `yolo11x-seg.pt` as the start, COCO weights, not a random init.
- `imgsz=640`, `batch=4`, `epochs=100`, `device=0`, `amp=True` (Ultralytics default), `workers=2`, `cache=False`.
- The tomato repo's recommended RAM is 16 GB. This laptop's RAM is not the 5060's 32 GB, and it has not been measured here, so the dataset stays off the RAM cache.
- If the 4060 reports out of memory, Ultralytics' own instruction is to cut the batch. `batch=-1` profiles a single GPU and aims at about 60% of VRAM. Use it once to find a batch, then lock the integer. The next model down is YOLO11s-seg, still on this GPU.
- No second process on the card during the run. No DDP. A data-parallel launch keeps a full copy of the model on each GPU, and these two laptops are not one machine.

Inference of this x-seg model stays on the 4060, subscribed to the sim image. The 28 ms figure is that repo's, on the 4060. A TensorRT export is a later speed pass, after the weights exist. Ultralytics' TensorRT number for YOLO11n (1.5 ms on a T4) is a different, much smaller model. It is not a reason to put YOLO11x-seg inside the Isaac process.

Sources:

- [YOLO11x-seg on Laboro, RTX 4060, about 6 GB](https://github.com/Fonyuy45/yolo11-tomato-segmentation)
- [Ultralytics training tips](https://docs.ultralytics.com/guides/model-training-tips/) — on one GPU, `batch=-1` targets about 60% of memory. Multi-GPU needs an explicit global batch.
- [LTX-VideoQ8](https://github.com/KONAKONA666/LTX-Video) — 720×480×121 in under a minute on an RTX 4060 laptop, 8 GB, image-to-video, `--low_vram`.
- [Lightricks LTX-Video](https://github.com/Lightricks/LTX-Video) — the same 4060 claim.
- [Replicator, in the Isaac reference architecture](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/introduction/reference_architecture.html) — synthetic COCO and KITTI labels exist. They are not the training set.

## 6. How a session runs

Two laptops, two processes, no shared framebuffer.

The 5060 is the simulator. Isaac Sim 5.0.0, the stage, one robot, one row, one camera. DLSS is set in the Render Settings tab, under Ray Tracing, NVIDIA DLSS, mode Performance. On the Qiita RTX 5060 laptop (8 GB GDDR7, 32 GB RAM, Ubuntu 22.04, driver 580.95.05, CUDA 13.0) the RoboParty sample read 2767 MiB and 100% GPU at 41 W with DLSS on Auto, and 2510 MiB, 53% GPU, 27 W with DLSS on Performance. Startup settled near 25 seconds. The compatibility check still painted VRAM red. That warning matches the requirements page: the published minimum is an RTX 4080 and 16 GB, and under 16 GB "may be insufficient" above 16 megapixels and for large sensor suites. The red line is expected. A crash, or a climb toward 8 GB, is the failure.

The 4060 is the trainer and, later, the detector. Isaac is not opened on it. The fruit photos, Laboro, and the LTX clips live on that machine.

Memory order on the 5060, every time a splat is built or rebuilt:

1. Close Isaac. `nvidia-smi` shows the card idle.
2. COLMAP, then 3DGUT, then the USDZ. One of those GPU jobs at a time.
3. Close the trainer. Idle card again.
4. Open Isaac, set DLSS to Performance, import the USDZ, then the robot.

The May 2026 USDZ thread is the reason for that order. A file that would not display was explained as the card running out of memory. Training and Isaac on the same 8 GB card is how that happens.

During a live mission the 5060 publishes `rgb` (and `depth` only if step 4 of the ROS list passed). The 4060 runs the segmenter and publishes the detections. The arm target is computed on the 5060 from that message, because the joint commands have to hit the Articulation Controller in-process. The image crosses the network. The install page already says DDS does not care that the two Pythons differ.

What is not open in a session: a path-traced forest, a second RTX camera, RTX lidar, NuRec's reconstruction app, Cosmos, Isaac Lab, cuVSLAM, and a training job on the 5060.

Sources:

- [Isaac Sim 5.0.0 on an RTX 5060 laptop, 4 Jan 2026](https://qiita.com/ebiten_tabetai/items/24447cd1974b2b587d33) — DLSS Performance, 2510 MiB, 53% GPU. Compatibility check VRAM red.
- [Requirements, Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) — 32 GB RAM minimum, 16 GB VRAM minimum, under 16 GB called out for heavy frames and many sensors.
- [USDZ import and VRAM, May 2026](https://github.com/isaac-sim/IsaacSim/discussions/607)

## 7. Sequential build

Each step starts only after the previous one passes. A fail stops the line. No simulation scripts in this plan. The checks are measurements, the Isaac GUI, and ROS command-line tools.

**1. Rover numbers.** Tape the car. Write wheelbase, track, wheel radius, body box, steering stop, in meters.

Pass: five numbers, from this car. Fail: any figure taken from the Leatherback table (0.32 m, 0.24 m, 0.052 m) or from an unmeasured photo.

**2. Rover URDF, imported.** Six links, five joints, boxes and cylinders, Moveable Base, velocity drives on the wheels, position drive on the steer joint.

Pass: the stage shows five joints and the wheels clear the body. Fail: one mesh and no joints, or a CAD-converter USD that will not take a drive. That failure is the August 2025 thread.

**3. Rover motion, flat grid, no ROS.** Equal velocity on the four wheels. One position on the steer joint. CPU physics solver.

Pass: forward, back, steer left, steer right, the chassis stays on the plane, and a shake does not grow. Fail: the model only skid-steers, or two knuckle joints move, or it falls through the floor.

**4. SCARA URDF.** One rigid body per DOF: base, arm 1, arm 2, Z carriage, gripper. Joint origins from the STEP. Convex hulls. Belts are a scale, if the mating pulleys are readable, or no scale yet. RViz or the importer's collision view.

Pass: four joints, meshes in meters, no belt as its own body. Fail: the full STEP dropped in and rigid, which is the CAD-converter outcome on the forum.

**5. Arm motion, fixed base, no rover, no ROS.** Position drives. Gain Tuner from natural frequency 0.5 and damping ratio 1.0. Each joint to a command and back. Gripper open and shut.

Pass: each joint holds the command with gravity on. Fail: a joint that sags and stays sagged, or an oscillation that grows. Retune before going on. Do not add the rover to a shaking arm.

**6. A point in front of the arm.** Marker on the ground plane. Shoulder, elbow, and Z from the two-link geometry. Gripper frame against the marker, then back to the start pose.

Pass: the gripper frame meets the marker within 2 cm. That tolerance is ours. No Isaac page I opened states one for this arm. Fail: the joints move and nobody measured the gripper frame.

**7. Arm mounted.** Fixed joint from the rover chassis to the arm base. One articulation. Repeat step 3 and step 6 in the same stage.

Pass: the rover still drives and the arm still hits the marker. Fail: the arm left on the world frame, or a second physics scene.

**8. The row.** Isaac closed, splat built, Isaac opened, DLSS Performance, USDZ imported, ground plane aligned, three fruit meshes placed. If the splat fails, the mesh row, and the failure is written down.

Pass: `nvidia-smi` stays near the Qiita band (about 2.5 GB before the splat, and not climbing at the 7 GB renderer warning), the camera view shows the row, the rover drives along it, each fruit has a pose. Fail: a black stage (the 2.7 GB USDZ case), a bicycle-sized splat, or a path-traced multi-row field.

**9. ROS, one layer at a time.** Internal Humble libraries. Isaac launched from a terminal that has not sourced the system ROS Python.

- Description. Pass: `ros2 topic echo` on joint states shows the five rover joints and the four arm joints while the GUI moves them. Fail: empty names, or a Python 3.10 library clash on startup. The install page says that clash is what you get from sourcing system ROS into Isaac's terminal.
- Drive. Pass: a ROS command produces the same forward, back, and steer as step 3. Fail: the topic publishes and the wheels do not move.
- Arm. Pass: a ROS command repeats the marker reach. Fail: the GUI sliders still work and the topic does not.
- Camera. Pass: `rqt_image_view` on `rgb` shows the row. Depth is optional and fails closed if the card leaves the band. Fail: no image, or a second RTX sensor added to make the first one "more real."
- Detector, on the 4060, trained with the recipe in section 5. Pass: a held-out real photo of a ripe tomato is segmented as fully ripened, and the training folder contains no Isaac frames. The node also subscribes to `rgb`. Whether it fires on the sim image is recorded, not papered over. A mesh tomato that the real-photo model ignores is a failed camera match, not a failed arm. Fail: the only score is on synthetic frames.
- Target. Pass: when the detector fires, the arm moves to that fruit. When it does not fire, the fruit prim's pose still commands the arm, and the log says which one happened. Fail: a silent fallback that gets reported as a detection.

**10. One mission.** From a ROS launch on the 5060, detector process on the 4060: drive the row, stop at a ripe fruit, move the arm to it.

Pass: all three, with `nvidia-smi` on the 5060 still in the small-scene band and the 4060 doing the segmentation. Fail: both jobs on one card, a second GPU joined by DDP, or the arm moving to a coordinate that was typed in and never tied to a fruit.
