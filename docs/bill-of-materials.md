# Bill of materials

## 1. Mobile base

| S. No. | Component | Specification | Qty | Unit | Application |
| --- | --- | --- | --- | --- | --- |
| 1.1 | Chassis frame | Metal rod frame, about 1.2 m long | 1 | No. | Carries the arm |
| 1.2 | Body and seat | Moulded plastic body, covers, and seat | 1 | Set | Bodywork |
| 1.3 | Wheel and tyre | Tyre diameter 0.38 m, width 0.15 m | 4 | Nos. | Ground contact |
| 1.4 | Steering linkage | Linkage from the steering motor to the front wheels | 1 | No. | Steers the rover |
| 1.5 | Arm mounting plate | Metal plate from the arm base to the rod frame | 1 | No. | Fixes the arm to the chassis |
| 1.6 | Chassis reinforcement | Reinforcement of the rod frame at the arm mount | 1 | Set | Carries arm load on the frame |

## 2. Arm structure

| S. No. | Component | Specification | Qty | Unit | Application |
| --- | --- | --- | --- | --- | --- |
| 2.1 | 3 kg PLA | Filament for printing the SCARA links and the gripper | 1 | No. | Printed arm |
| 2.2 | Guide rod | Smooth rod, 10 mm × 400 mm | 4 | Nos. | Vertical slide |
| 2.3 | Lead screw | 8 mm × 400 mm | 1 | No. | Raises and lowers the slide |
| 2.4 | Bearing set | Linear bearings for the guide rods, radial bearings, and thrust bearings for the rotating base | 1 | Set | Supports the joints and the slide |
| 2.5 | Transmission set | Timing belts, toothed pulleys, idlers, and a shaft coupling from the motor to the lead screw | 1 | Set | Couples the motors to the joints |
| 2.6 | Stem-cutting blade assembly | Blade assembly for cutting the fruit stem | 1 | No. | Cuts the stem |
| 2.7 | Cutter actuator and linkage | Actuator and linkage for the blade | 1 | Set | Drives the cutter |
| 2.8 | Soft fruit-catching cradle | Soft cradle under the fruit | 1 | No. | Catches the fruit as it is cut |
| 2.9 | TPU/silicone protective lining | Soft lining for fruit-contact surfaces | 1 | Set | Protects the fruit |
| 2.10 | Cutter guard and mounting bracket | Guard and bracket for the cutter | 1 | Set | Covers the blade and mounts it |
| 2.11 | Cutter home/position sensor | Sensor for cutter home and position | 1 | No. | Reports cutter position |

## 3. Electronics

| S. No. | Component | Specification | Qty | Unit | Application |
| --- | --- | --- | --- | --- | --- |
| 3.1 | Wheel motor | Geared DC motor. Motor about 25,000 rpm, reduced for low speed and high torque | 4 | Nos. | Drive the wheels |
| 3.2 | Steering motor | Geared DC motor of the same class, with a higher gear ratio | 1 | No. | Turns the front wheels |
| 3.3 | Wheel motor driver | High-current driver for four motors, with a built-in wireless receiver | 1 | No. | Drives the wheel motors from the handset |
| 3.4 | RF transmitter | Handheld wireless transmitter matched to the wheel-motor driver | 1 | No. | Manual driving |
| 3.5 | Steering motor driver | High-current DC motor driver | 1 | No. | Drives the steering motor |
| 3.6 | Stepper motor | NEMA 17, 4.2 kg·cm | 2 | Nos. | Arm joint and base rotation |
| 3.7 | Stepper motor | NEMA 17, 7 kg·cm | 2 | Nos. | Arm joint and vertical screw |
| 3.8 | Stepper driver | 3 A class | 4 | Nos. | Rated current for the stepper motors |
| 3.9 | Stepper driver | 1 A class | 4 | Nos. | Lighter stepper drive |
| 3.10 | Stepper driver | Higher-current microstepping driver | 1 | No. | Fine position on one 7 kg·cm motor |
| 3.11 | Gripper servo | Hobby servo | 2 | Nos. | Open and close the gripper |
| 3.12 | Battery | 12 V, 7.6 Ah | 1 | No. | Wheel motors and steering motor |
| 3.13 | Battery | 24 V, 10 Ah | 1 | No. | Steppers, servos, control boards, and the onboard computer |
| 3.14 | Charger | Matched to the 24 V battery | 1 | No. | Recharges the arm and computer battery |
| 3.15 | DC-DC converter — logic | 24 V to 5 V | 1 | No. | Microcontrollers and relays |
| 3.16 | DC-DC converter — compute | 24 V to the onboard-computer input. Output set when that computer is chosen | 1 | No. | Onboard computer |
| 3.17 | DC-DC converter — servos | 24 V to 5–6 V | 1 | No. | Gripper servos |
| 3.18 | Fuses and fuse holders | Fuses and holders on each battery feed | 1 | Set | Protects both battery feeds |
| 3.19 | Limit switches | Roller lever, for the arm joints and both ends of the lead screw | 4 | Nos. | End of travel and joint home |
| 3.20 | Emergency stop | Latching switch on both battery feeds | 1 | No. | Kills wheel, steering, and arm power |
| 3.21 | Control switches | Power disconnect and other panel switches | 1 | Set | Isolates power and selects mode |
| 3.22 | Battery monitoring | Voltage and current sensing; battery-management interface where supported | 1 | Set | Monitors both batteries |
| 3.23 | Cooling hardware | Heatsinks, fans and ventilation for components that require them | 1 | Set | Cools drivers and the onboard computer |

## 4. Control and wiring

| S. No. | Component | Specification | Qty | Unit | Application |
| --- | --- | --- | --- | --- | --- |
| 4.1 | Low-level microcontrollers | ESP32-S3 or equivalent | 2 | Nos. | Actuator commands, limit switches and sensor I/O |
| 4.2 | Relay modules | 4-channel, opto-isolated | 1 | No. | Chassis contact closures for the wheel driver |
| 4.3 | Wiring harness | Signal and power harness for the rover and the arm | 1 | Set | Power and signal runs |
| 4.4 | Connectors and terminals | Connectors and terminals matched to the harness | 1 | Set | Joints in the wiring |
| 4.5 | Cable sleeves and drag-chain wiring | Sleeves and drag chain along the moving arm | 1 | Set | Protects wiring on the moving joints |

## 5. Sensing and onboard compute

| S. No. | Component | Specification | Qty | Unit | Application |
| --- | --- | --- | --- | --- | --- |
| 5.1 | Front navigation camera | USB RGB camera; preferably global shutter, suitable for outdoor crop-row detection | 1 | No. | Sees the row ahead |
| 5.2 | Rear camera | USB RGB camera for reversing and rear obstacle awareness | 1 | No. | Reversing and rear obstacles |
| 5.3 | Primary arm camera | Short-range RGB-D camera for tomato detection, depth estimation and picking alignment | 1 | No. | Fruit image, depth, and pick alignment |
| 5.4 | Rotating scouting camera | RGB camera on a motorised pan mechanism for scanning neighbouring plants | 1 | No. | Scans neighbouring plants |
| 5.5 | Onboard computer | NVIDIA Jetson platform (development kit), selected according to model performance, memory, camera bandwidth and power requirements | 1 | No. | Runs the cameras and the fruit model on the rover |
| 5.6 | Storage | Compatible NVMe SSD or other supported storage | 1 | No. | OS, models, datasets and logs |
| 5.7 | Powered USB hub | Sufficient USB ports, bandwidth and power for the selected cameras | 1 | No. | Connects the cameras to the onboard computer |
| 5.8 | Camera mounting brackets | Rigid, adjustable mounts for the front, rear and arm cameras | 1 | Set | Holds the fixed cameras |
| 5.9 | Pan mechanism | Geared motor, driver, rotating platform and position feedback for the scouting camera | 1 | Set | Pans the scouting camera |
| 5.10 | IMU | Accelerometer and gyroscope | 1 | No. | Attitude and motion estimation |
| 5.11 | Wheel encoders | Feedback for wheel/axle speed and odometry, matched to the drivetrain | 1 | Set | Wheel speed and odometry |
| 5.12 | Steering-angle sensor | Measures actual steering position | 1 | No. | Steering position |
| 5.13 | Lighting | Diffuse LED illumination for close-range tomato inspection | 1 | Set | Lights the fruit under variable outdoor light |
| 5.14 | Protective camera housings | Dust and splash protection for field use, without obstructing the optics | 1 | Set | Protects the cameras |

## 6. Miscellaneous

| S. No. | Component | Specification | Qty | Unit | Application |
| --- | --- | --- | --- | --- | --- |
| 6.1 | Mechanical fastener kit | Spare metric bolts, nuts, washers, locking nuts and screws | 1 | Set | Assembly and spares |
| 6.2 | Threadlocker | Medium-strength threadlocker for vibration-prone joints | 1 | No. | Locks vibration-prone joints |
| 6.3 | Cable ties and mounting bases | UV-resistant ties and adhesive or mechanical cable mounts | 1 | Set | Dresses and fixes cables |
| 6.4 | Heat-shrink tubing | Assorted diameters for insulation and strain relief | 1 | Set | Insulates joints |
| 6.5 | Electrical insulation materials | Insulating sleeves, electrical tape and terminal covers | 1 | Set | Insulates terminals |
| 6.6 | Lubricants and grease | Lubricant for bearings, lead screws and moving joints | 1 | Set | Lubricates moving parts |
| 6.7 | Cleaning supplies | Soft brushes, lint-free cloths and lens-cleaning materials | 1 | Set | Cleans the cameras and the machine |
| 6.8 | Weatherproofing materials | Gaskets, seals and enclosure sealant for dust and splash protection | 1 | Set | Seals enclosures |
| 6.9 | Fabrication consumables | Welding consumables, cutting discs and drilling consumables, as required | 1 | Set | Frame fabrication |
| 6.10 | Spare electrical components | Spare fuses, terminals and connectors matching the final design | 1 | Set | Field spares |
| 6.11 | Spare sensors and switches | Spare limit switch and other critical low-cost sensors, selected after commissioning | 1 | Set | Field spares |
| 6.12 | Hand tools | Spanners, hex keys, screwdrivers, pliers and suitable adjustment tools | 1 | Set | Assembly and adjustment |
| 6.13 | Crimping and wiring tools | Crimpers, wire stripper and ferrule tools for the selected connectors | 1 | Set | Builds the harness |

## 7. Optional upgrades and future enhancements

Not part of the current build.

| S. No. | Component | Purpose / justification | Priority |
| --- | --- | --- | --- |
| 7.1 | RTK-GNSS receiver and antenna | Centimetre-level positioning in suitable outdoor conditions; useful for repeatable field-row navigation | Future |
| 7.2 | 2D or 3D LiDAR | Improved obstacle detection and mapping around plants, people and equipment | Future |
| 7.3 | Additional ultrasonic or ToF sensors | Low-cost proximity sensing for close obstacles and blind spots | Optional |
| 7.4 | Rear or side obstacle sensors | Additional coverage during reversing and tight manoeuvres | Optional |
| 7.5 | Environmental sensors | Temperature, humidity and other field-condition logging | Optional |
| 7.6 | Soil moisture sensor | Monitors soil moisture to support field-condition analysis | Optional |
| 7.7 | Cellular connectivity module | Remote status monitoring and telemetry where Wi-Fi is unavailable | Optional |
| 7.8 | Telemetry and fleet-monitoring dashboard | Logs battery status, navigation, picking success and system faults | Future |
| 7.9 | Additional work lighting | Improves visibility in shaded crop rows or low-light operation | Optional |
