# Still to procure — for the funding request

Drafted 14 Nov 2026 from [bill-of-materials-priced.md](bill-of-materials-priced.md), [already-procured.md](already-procured.md), the OnlyScrews, Robocraze, Robokits, Robu and Amazon orders, and section 7 of [arm-v2-design.md](arm-v2-design.md).

Everything that has already been bought or comes with the ride-on base is left out (listed at the end). Prices are the bill-of-materials figures or the arm v2 estimates: rough Indian retail, rounded, **all [E] estimates**, not quotes. Where the source gave a range, the upper end is used. Where a ₹ figure comes from a real order, it is marked [M].

## 1. Compute and vision

| # | Item | Spec | Qty | ₹ [E] |
| --- | --- | --- | --- | --- |
| 1.1 | Onboard computer | NVIDIA Jetson dev kit (Orin class), runs the cameras and the fruit detector | 1 | 45,000 |
| 1.2 | Storage | NVMe SSD for OS, models, datasets, logs | 1 | 4,000 |
| 1.3 | RGB-D camera | Short-range depth camera for fruit localisation (see question 1) | 1 | 35,000 |
| 1.4 | Front navigation camera | USB RGB, global shutter, for crop-row following | 1 | 6,500 |
| 1.5 | Rear camera | USB RGB, reversing and rear obstacles | 1 | 2,500 |
| 1.6 | Scouting camera | RGB, scans neighbouring plants | 1 | 3,000 |
| 1.7 | Pan mechanism | Geared motor, driver, platform, position feedback for 1.6 | 1 set | 3,500 |
| 1.8 | Powered USB hub | Ports, bandwidth and power for the cameras | 1 | 2,000 |
| 1.9 | Camera brackets | Rigid adjustable mounts | 1 set | 1,500 |
| 1.10 | Camera housings | Dust and splash protection | 1 set | 2,500 |
| 1.11 | Lighting | Diffuse LED for close-range fruit inspection | 1 set | 800 |
|  | **Subtotal** |  |  | **1,06,800** |

## 2. Navigation sensors

| # | Item | Spec | Qty | ₹ [E] |
| --- | --- | --- | --- | --- |
| 2.1 | IMU | Accelerometer and gyroscope | 1 | 1,500 |
| 2.2 | Wheel encoders | Wheel or axle speed for odometry | 1 set | 1,200 |
| 2.3 | Steering-angle sensor | Actual steering position | 1 | 600 |
|  | **Subtotal** |  |  | **3,300** |

## 3. Power and safety

| # | Item | Spec | Qty | ₹ [E] |
| --- | --- | --- | --- | --- |
| 3.1 | Battery | 24 V, 10 Ah, for the arm, electronics and Jetson | 1 | 6,000 |
| 3.2 | Charger | Matched to 3.1 | 1 | 1,200 |
| 3.3 | DC-DC converter for the Jetson | 24 V to the Jetson input | 1 | 800 |
| 3.4 | Fuses and holders | On each battery feed | 1 set | 400 |
| 3.5 | Emergency stop | Latching, cuts both battery feeds | 1 | 350 |
| 3.6 | Panel switches | Power disconnect, mode select | 1 set | 400 |
| 3.7 | Battery monitoring | Voltage and current sensing | 1 set | 1,500 |
| 3.8 | Cooling | Heatsinks and fans for drivers and the Jetson | 1 set | 1,000 |
|  | **Subtotal** |  |  | **11,650** |

## 4. Rover conversion

| # | Item | Spec | Qty | ₹ [E] |
| --- | --- | --- | --- | --- |
| 4.1 | Chassis reinforcement | Rod frame at the arm mount | 1 set | 2,000 |
| 4.2 | Arm mounting plate | Arm base to rod frame | 1 | 1,500 |
| 4.3 | Steering linkage parts | Depends on Tejesh confirming the stock linkage | 1 | 1,000 |
| 4.4 | Control-board interface | Optocouplers or PhotoMOS relays to drive the stock board from the ESP32 ([rover-logic-interface.md](rover-logic-interface.md)) | 1 set | 500 |
| 4.5 | Fabrication consumables | Welding, cutting discs, drill bits | 1 set | 1,500 |
| 4.6 | Weatherproofing | Gaskets, seals, sealant | 1 set | 800 |
|  | **Subtotal** |  |  | **7,300** |

## 5. SCARA arm (stock HTM build)

| # | Item | Spec | Qty | ₹ [E] |
| --- | --- | --- | --- | --- |
| 5.1 | Pancake NEMA 17 | L24 class, for J3 in the stock design (none on hand) | 1 | 1,500 |
| 5.2 | 51107 thrust bearing | Stock design needs 4, 3 on hand | 1 | 200 (₹197 [M] on Amazon) |
| 5.3 | PETG filament | Field-sun parts: Arm 2, couplers, tool | 2 kg | 2,000 |
| 5.4 | Heat-set inserts and M4 screws | M4 × 8 inserts ×20, M4 × 20 ×5, M4 × 12 ×4 | 1 lot | 300 |
|  | **Subtotal** |  |  | **4,000** |

## 6. End effector (suction draw-in and guillotine)

The cup, fitting, 8 mm rod, LM8UU pair, blades and tool camera are already bought.

| # | Item | Spec | Qty | ₹ [E] |
| --- | --- | --- | --- | --- |
| 6.1 | Vacuum pump | 12 V mini diaphragm, 370 type, about −60 kPa | 1 | 700 |
| 6.2 | Solenoid valve | 12 V mini 3/2, vents the cup | 1 | 450 |
| 6.3 | Vacuum pressure sensor | XGZP6847A, −100–0 kPa, analog | 1 | 800 |
| 6.4 | Cutter servo | DS3225MG (the MG996R is under the 40 N target) | 1 | 1,800 |
| 6.5 | MOSFET module | IRF520 class, 2 channels, pump and valve | 1 | 150 |
| 6.6 | Small hardware | M4 × 16 shoulder bolts ×2, 6 × 2 mm magnets ×4, M3 grub and self-tappers, spare blades | 1 lot | 400 |
|  | **Subtotal** |  |  | **4,300** |

## 7. Wiring

| # | Item | Spec | Qty | ₹ [E] |
| --- | --- | --- | --- | --- |
| 7.1 | Wiring harness | Signal and power, rover and arm (silicone wire already bought from Robu) | 1 set | 3,000 |
| 7.2 | Connectors and terminals | Matched to the harness | 1 set | 1,000 |
| 7.3 | Drag chain and sleeves | Along the moving arm | 1 set | 1,500 |
| 7.4 | Crimping tools | Crimper, stripper, ferrule tool | 1 set | 1,800 |
|  | **Subtotal** |  |  | **7,300** |

## 8. Consumables and spares

| # | Item | Qty | ₹ [E] |
| --- | --- | --- | --- |
| 8.1 | Threadlocker, lubricants, insulation, cleaning supplies | 1 lot | 1,250 |
| 8.2 | Spare fuses, terminals, connectors | 1 lot | 800 |
| 8.3 | Spare limit switch and small sensors | 1 lot | 500 |
|  | **Subtotal** |  | **2,550** |

## Total

| Section | ₹ [E] |
| --- | --- |
| 1. Compute and vision | 1,06,800 |
| 2. Navigation sensors | 3,300 |
| 3. Power and safety | 11,650 |
| 4. Rover conversion | 7,300 |
| 5. SCARA arm | 4,000 |
| 6. End effector | 4,300 |
| 7. Wiring | 7,300 |
| 8. Consumables and spares | 2,550 |
| **Still to procure** | **₹1,47,200** |

The Jetson and the RGB-D camera are ₹80,000 of this, about 54 %.

## Optional, not in the total

From section 7 of the priced bill: RTK-GNSS (₹25,000), 2D LiDAR (₹15,000), telemetry dashboard (₹8,000), cellular module (₹3,500), obstacle, ToF, environmental and soil sensors and extra work lights (₹6,000). Together ₹57,500 [E].

## Left out because we already have it

- **Arm:** 10 mm rods ×4, lead screw, LM10LUU ×5, 608RS ×5, thrust bearings, 6806-ZZ and 6807-ZZ, NEMA 17 ×4 (2 × 4.2 kg·cm, 2 × 7 kg·cm), DM542, TB6600 ×4, A4988 ×4, MG996R ×2, GT2 pulleys and belts, 5–8 mm coupling, PLA.
- **Electronics:** ESP32-S3, limit switches, 4-channel relay board, BTS7960 (steering), XL4015 and 12 A buck converters, XT60 pairs, silicone wire, heat-shrink.
- **Hardware and tools:** the OnlyScrews fastener, spanner, Allen key, plier and drill-bit order.
- **Ride-on base:** body, wheels, the four wheel motors, the steering motor, the stock driver board and the handset.

## Questions before sending

1. **RGB-D camera (₹35,000).** The arm v2 tool now carries a USB board camera, so the RGB-D is no longer needed on the tool. The simulation still localises fruit with an arm RGB-D camera. Keep it as the main depth sensor, or drop it and save ₹35k?
2. **12 V battery and tyres.** The priced bill has a 12 V 7.6 Ah battery (₹1,400) and four tyres (₹3,200). Left out here on the assumption the ride-on came with them. If the field needs bigger or pneumatic tyres, add them back.
3. **Servo and logic converters.** Left out on the assumption the XL4015 and the 12 A buck cover the 5 V logic and the servo rail. If not, add about ₹650.
4. **Store orders.** The OnlyScrews, Robocraze, Robokits and Robu orders are treated as placed. If any were not, their totals go back in (about ₹1.9k, ₹1.7k, ₹2.0k and ₹1.5k [M], in that order).
