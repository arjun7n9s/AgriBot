# AgriBOT

Autonomous tomato-field rover aimed at a functional field prototype, with a project budget around ₹10 lakh.

## Purpose

One integrated farming system that:

- Travels tomato rows on its own
- Sees plants and fruit, judges which tomatoes are ripe, and localizes a target relative to the robot
- Positions a harvester, plucks the fruit, and carries the produce
- Does this without continuous human control

Soil testing is already developed separately. It belongs to the wider agricultural system, not to the rover work still ahead.

## Current plan

### SCARA arm

The harvester is a SCARA arm. The 3D files are in `media/scara-arm/`. Most of those parts are printed. Arm 2 and the gripper are unprinted and still to be redesigned. Assembly of the printed parts starts when the screws and attachments arrive.

### Rover base

The base is a Chinese kids’ mall ride-on car with a load capacity above 100 kg. It still needs to be modified for the field.

Stock hardware:

- Four motors, one at each wheel
- A fifth motor for steering. The linkage is not confirmed; it may be a direct bell-crank or a push-rod. Do not lock either one until it is seen.
- One board that includes the motor driver and, as far as we can tell, the radio receiver
- A handheld controller

That board can run all five motors, but only from the stock controller. There are no obvious open signal pins for a microcontroller, and the board cannot simply be reprogrammed. Autonomy depends on reverse engineering it.

### Still to come from Tejesh

- What each remaining header short does (forward, steering, `ROCK`, `S`, and `F`’s mate)
- How Arm 2 and the gripper should change

## Current state

Most SCARA printed parts are done. The end effector and the second arm link that connects to it are unprinted and waiting on a redesign. The old purchase list is the already-bought set: [Already procured](already-procured.md). The SCARA 3D files are in `media/scara-arm/`.

## What the platform combines

- Rugged locomotion on uneven field ground
- Autonomous row navigation
- Machine vision for ripeness, detection, and localization
- SCARA positioning and an end effector
- Produce handling and storage on the robot
- Embedded motor and control electronics
- The existing soil-testing capability

## Earlier design snapshot

Superseded for the rover base and the arm. Kept so the old numbers are not mistaken for the current plan.

- Chassis: rigid rectangle, about 1200 × 800 mm. Four independent suspension assemblies were dropped.
- Wheels: pneumatic, for uneven ground.
- Drivetrain: simplified toward 2WD. Motors considered included MY1016Z/MY1016ZL PMDC units and higher-torque BLDC or hub motors, including a 48 V, 500 W, roughly 200 rpm hub motor rated around 40 N·m.
- Harvester: a square or frame mechanism for X-Y positioning, about 500 mm of Z travel, and an articulated, largely 3D-printed arm. Discussed geometry included about 240 mm and 280 mm arm links, a 220 mm rotating base, a roughly 300 × 300 mm bottom plate, and an overall mechanism height around 600 mm.
- Structure: square steel tube such as 30 × 30 × 2.6 mm and 40 × 40 × 4 mm, with the SolidWorks model repeatedly lightened so the chassis stays practical.

## Simulation

The sim is the proof, hardware parked, 10 Oct 2026. Order is rover, then SCARA, then one tomato row, then ROS one layer at a time. See [work pathway](work-plan.md). The RTX 5060 (32 GB RAM) runs Isaac Sim 5.0 with DLSS 4 on Performance. The RTX 4060 trains the ripe-fruit detector. The cards do not share VRAM. The row is a small Gaussian splat or simple meshes, not a path-traced field. No sim scripts until Tejesh is on Ubuntu.

The robot in the sim is the known chassis (four wheel motors, one steering joint) and the SCARA. Commands follow the five-pin model: a–b power, c–d forward, c–e back. Steering is added when that short is known. One RTX camera publishes a real RGB image, and a depth image, on ROS.

## Working assumptions

- Design and build work stays in the cloud, because development moves between Windows and Ubuntu.
- The current plan above is the authority. The earlier snapshot is not.
