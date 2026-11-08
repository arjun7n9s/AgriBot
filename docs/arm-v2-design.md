# SCARA arm v2 and the tomato cutter

First design 4 Nov 2026. **Reworked 8 Nov 2026 for printing:** every made part is now printed in PETG on the Anycubic Kobra S1 (250 × 250 × 250 mm bed). There are no aluminium or laser-cut parts. Only the cutter uses bought extras: blades, sensors, a camera and a stronger servo.

This document replaces sections 2 and 3 of [scara-redesign.md](scara-redesign.md) (Arm 2 and the cutter concept). That document's literature figures and its height plan (section 4) still apply.

**Status:**
- Already built (printed in PLA): the base, the 4 rods and the lead screw, the Z carriage, and the top cap with its stepper and limit switch.
- Arm 1, Arm 2, J2, J3 and the cutter: CAD complete, collision-checked and checked against the print bed. Nothing is printed or bench tested yet.
- The simulation still uses the 0.4.0 arm (L2 136.5 mm, cutter concept v0).

**Files:**
- CAD script: `tools/cad/build_arm_v2.py`
- Shared numbers: `tools/cad/arm_v2_geometry.py`
- Calculations: `tools/cad/arm_v2_calcs.py`, which writes `cad/arm-v2/calcs.json`
- Collision and bed check: `tools/cad/check_arm_v2.py`
- Output, in [cad/arm-v2](../cad/arm-v2):
  - `stl/`: every printed part, ready to slice
  - `step/`: the same parts as STEP
  - `agribot_arm_v2.step`: the whole arm in the stock STEP frame, so it overlays the stock base
  - `end_effector.step`: the tool alone
  - `parts.csv`: part list with print settings and mass estimates

Every number below is tagged: **[M]** measured (stock STEP, a datasheet, or a cited paper), **[A]** assumed (a design choice or an unchecked value), **[E]** estimate (worked out; replace it with a measurement).

![Arm v2](images/arm_v2.png)

## 1. What stays and what changes

**Already built, not touched:**
- The base and the J1 drive (20:1 through 20T→80T and 22T→110T).
- The Z column: 4 × Ø10 rods, 8 mm lead screw, 249 mm stroke [M].
- The Z carriage (the stock "Z-axis Mount Platform"), with the J2 motor and its 20T pulley.
- The top cap with the Z motor and the top limit switch.

**New, all printed:**
- **Arm 1.** A tray and a lid. It bolts to the carriage's existing face with the stock 4 × M5 pattern: ±25.5 mm apart, at z 245.1 and 263.1, 62 mm out from the J1 axis [M].
- **The J2 joint and Arm 2.** L2 = 200 mm [A]. Arm 2 is a body and a cover.
- **The J3 joint**, with a standard tool flange.
- **The cutter.**

L1 stays at 228 mm [M]. The carriage fixes where the J2 motor sits. The two J2 belt stages (400 and 300 mm belts) only fit at the stock centre distances: 148.1 and 89.9 mm, which give 398.7 and 300.2 mm loops [M]. So the J2 drive keeps its stock geometry, including the stock 80T/23T compound pulley. Print that one from the stock STL.

### Material: PETG for every new part

| | PLA | PETG | ABS |
| --- | --- | --- | --- |
| Softens at (glass transition, filament datasheets) | 55–60 °C | about 80 °C | about 100 °C |
| Warping, shrinkage | Low | Low | High; bearing seats and pulley teeth come out undersize |
| Layer bonding | Good | Very good | Good in an enclosure |
| Verdict | Already used for the base; keep it there | **Use for the arm and the cutter** | Fallback for any part that softens in the sun |

Dark parts in field sun can pass 55 °C, so new load-bearing parts should not be PLA. PETG holds its size best, which matters for bearing seats and belt teeth. The Kobra S1 is enclosed, so ABS also works if a part ever needs more heat resistance. The PLA base and carriage are already built. Watch them in the sun, and reprint them in PETG if they creep.

**Filament:** the printed parts weigh about 0.9 kg [E]. Buy 2 kg of PETG so there is enough for test rings and reprints.

## 2. Where every part on hand goes

Taken from the procured list and the OnlyScrews, Robocraze, Robokits, Robu and Amazon orders.

| On hand | Qty | Used for | Spare |
| --- | --- | --- | --- |
| NEMA17 7 kg·cm + DM542 | 1 | J1 (largest inertia) | 0 |
| NEMA17 7 kg·cm (premium) | 1 | J2, TB6600 | 0 |
| NEMA17 4.2 kg·cm (economy) | 2 | Z (TB6600); J3 (TB6600, under Arm 2) | 0 |
| TB6600 drivers / A4988 | 4 / 4 | TB6600 for Z, J2, J3 and one spare; A4988 for bench work | 1 / 4 |
| MG996R | 2 | Basket door; the cutter until a DS3225MG arrives | 0 |
| 51108 thrust, 40×60×13 | 3 | J1 (stock) ×2 | 1 |
| 51107 thrust, 35×52×12 | 3 | J2 ×2 | 1 |
| 6806-ZZ, 30×42×7 | 2 | J2 ×1, J3 ×1 | 0 |
| 6807-ZZ, 35×47×7 | 2 | J1 (stock) ×1, J3 ×1 | 0 |
| 608RS | 5 | J1 idler ×2, J2 compound-pulley axle ×2, Z screw ×1 | 0 |
| GT2 belts 200 / 300 / 400 | 1 / 2 / 2 | J1: 200 + 300. J2: 400 + 300. J3: 400 | 0 |
| GT2 20T pulley, 5 mm bore | 6 | J1, J2, J3 motors | 3 |
| GT2 toothless idler, 5 mm bore | 6 | Belt tensioners in Arm 1 ×2 (the stock arm had none) | 4 |
| Roller micro switches | 10 | Z top, J1, J2 and J3 home, cutter feeler | 5 |
| M4 × 40 | 25 | Arm 1 lid into the tray columns ×6 | 19 |
| M4 × 30 | 25 | Arm 2 cover into the body columns ×5 | 20 |
| M4 × 20 | 15 | J2 hub ×6, J2 spindle flange ×6, J2 pulley ×3 (buy 5 more: section 7) | 0 |
| M4 nuts | 80 | Every M4 above that isn't in a heat-set insert | |
| M5 × 25 / M5 × 35 | 15 / 10 | Arm 1 to the carriage ×4 / idler axles ×2 | |
| M8 × 45 | 2 | Axle for the J2 compound pulley | 1 |
| M3 × 20 + nyloc | 100 / 30 | Link pins ×2, fork pivot ×1, door hinge ×1 | |
| Compression springs OD 4 / OD 6 | 4 / 4 | Feeler fork bias / blade pivot preload | |
| 12 A buck, XL4015, ESP32-S3 | 1 each | Servo rail at 6.0 V; 5 V logic; arm controller | |

The stock design needed four 51107s and a pancake motor for J3. You have three 51107s and no pancake motor. v2 builds J3 from the spare 6806 and 6807, and hangs a standard NEMA17 under Arm 2.

## 3. Reach: why L2 = 200 mm, and where the plants are

**Assumed plant line: 305 mm from the J1 axis [A].** This is the project's field layout, also used in the Gazebo world ([assets.md](assets.md)):
- The rover is 0.70 m wide [M] and drives a 0.90 m furrow.
- The plant line is 0.30 m from the bed centre, with plants every 0.45 m.
- The SCARA base sits on the side mast, so J1 is 0.395 m from the rover centreline. The plant line is then about 0.30 m from J1.

Measure this on the first field visit and update `PLANT_LINE` in `arm_v2_calcs.py`.

The tool approaches square to the row, so the wrist sits a fixed distance behind the cut point: 88 mm for v2, 64 mm for the stock gripper. J2 is limited to ±150°. That limit, not the link lengths, sets the inner edge of the workspace. The fruit zone for one rover stop is 450 mm along the row. Across the row it runs from 150 mm on the near side of the plant line to 100 mm beyond it [A].

| J1 axis to plant line | Stock (L2 136.5) | v2 (L2 200) |
| --- | --- | --- |
| 255 mm | 86% | 85% |
| **305 mm (assumed)** | 91% | **94%** |
| 355 mm | 78% | 100% |
| 405 mm | 59% | 94% |

![Reach](images/arm_v2_reach.png)

At the assumed 305 mm, any L2 from 180 to 228 mm scores within 1%. 200 mm is kept because it still covers 94% if the rover ends up driving 100 mm further from the plants. Arm 2 is 278 mm overall, so it is printed diagonally on the bed (section 6).

## 4. The links

### Construction

Both links are **printed boxes**: a part with the walls and one face, plus a flat lid or cover, tied together by M4 bolts in printed columns. Each column has a side slot near its end for a trapped M4 nut. The belts run inside, which also keeps leaves out of them.

| | Arm 1 | Arm 2 |
| --- | --- | --- |
| Length between joints | 228.0 [M] | 200.0 [A] |
| Parts | A1-01 tray (8 mm floor, 3 mm walls, root wall, idler boss), A1-02 lid (5 mm), A1-03 J2 hub | A2-01 body (6 mm top, 3 mm walls), A2-02 cover (5 mm, with the J3 housing) |
| Overall height | 49 mm (z 224–273) | 36 mm (z 160–196) |
| Inside | J2 stage 1 and stage 2 belts, stock 80T/23T pulley on 2 × 608 in a boss in the floor, two tensioner idlers on slots | J3 belt: 20T on the J3 motor → 90T on J3 |
| Closing | 6 × M4 × 40 down through the lid into the columns | 5 × M4 × 30 up through the cover into the columns |
| Fixing | Root wall on the carriage face with 4 × M5 × 25, nuts inside | J2 spindle flange on the top with 6 × M4 × 20, nuts inside |
| Mass, printed + bought parts | 0.68 kg [E] | 0.70 kg [E] |

### Is printed stiff enough? [E]

Printed PETG (E about 1.8 GPa [A]) is about 40 times less stiff than aluminium. The box sections make up for it:

| | Value |
| --- | --- |
| Arm 1 section, bending / torsion | 478,000 / 658,000 mm⁴ |
| Arm 2 section, bending / torsion | 203,000 / 331,000 mm⁴ |
| Sag at the cut point, arm stretched straight (tool + 250 g fruit) | 0.25 mm |
| Same, with Arm 2 square to Arm 1 (Arm 1 twists 0.05°) | 0.46 mm |
| Moment at the carriage face | 5.5 N·m |
| Pull on each upper M5 bolt into the carriage | 54 N |

That is well inside what the vision can hit (the sim was 6–24 mm off). The sums treat the links as one solid box. The bolted lids, the bearing fits and the PLA carriage will add compliance, so measure the real sag after assembly. The carriage takes 54 N per upper bolt in printed PLA. Use washers on both sides, and check for creep after a hot day.

### J2 joint

The stack is the stock arrangement (two 51107s plus one 6806), now printed. The spindle goes in from below. Its diameters step down going up (78 → 35 → 30 mm), so it passes every bearing. From the bottom up:

| z (STEP frame) | Part |
| --- | --- |
| 196–204 | J2 spindle flange (Ø78), bolted to the Arm 2 top |
| 204–216 | 51107, lower |
| 216–224 | J2 hub (Ø78), bolted under the Arm 1 floor; 6806 inside at 216–223 |
| 228–240 | 51107, upper, in a Ø52.6 × 4 mm counterbore in the Arm 1 floor |
| 240–254.4 | 92T pulley (printed). Its Ø35 spigot centres the upper thrust. 4 × M4 × 20 go into heat-set inserts in the spindle top and clamp the stack. |

The spindle is hollow (Ø16), so the cables from Arm 2 and the tool run up through the joint and out through the Ø32 hole in the Arm 1 lid.

### J3 joint and the tool flange

- **Bearings:** the 6807 sits below and the 6806 above, in a boss under the Arm 2 cover. A 3 mm web between them only touches the outer rings. The bearings must go this way round: the spindle enters from below (flange, then Ø35, then Ø30), so the larger bore has to be at the bottom. The first v2 design had them the other way, which could not be assembled.
- **Load path:** the tool hangs on the 90T pulley. The pulley bears on the 6806 inner ring, and the 6806 sits on the web. Upward loads go from the spindle collar into the 6807 and then the web. Add a 0.1–0.2 mm shim if there is play.
- **Tool flange:** Ø60 × 8 mm, with 4 × M4 heat-set inserts on a 48 mm PCD, a Ø20 spigot and a Ø4 dowel at 45°. Any future tool bolts on here.
- **Cables:** a Ø14 bore through the spindle and the pulley into Arm 2.

### J3 motor placement (no new belt)

A 400 mm belt on 20T→90T needs a 143.3 mm centre distance [M]. So the J3 motor hangs under the Arm 2 cover, 143.3 mm from J3 (56.7 mm from J2). Its slots give ±4 mm of belt tension. Because it sits close to J2, it adds little to J2's inertia.

The ratio stays 4.5:1. That gives the economy NEMA17 about 1.8 N·m at J3 [E], far more than the tool needs.

### Joint limits (from the CAD collision check)

- **J2: ±150°.** No contact between Arm 1, Arm 2 and the tool anywhere in this range.
- **J3: −105° to +85°.** Past this, the camera or its bracket meets the underside of Arm 2. Put hard stops 5° beyond the soft limits. The cutter only needs to face away from Arm 2, and this range covers that.

### Loads and resolution [E]

| | Value | Margin |
| --- | --- | --- |
| Bending moment at J2 (Arm 2 + tool + 250 g fruit) | 2.3 N·m | 51107 pair: very large |
| J1 inertia (whole arm, fruit, full reach) | 0.25 kg·m² | |
| J1 torque for 3 rad/s² | 0.74 N·m | 5.9 N·m available (0.69 N·m × 20 × 0.85, then halved) |
| Z load (arm + tool + fruit + carriage share) | 25 N | 0.11 N·m on an 8 mm-lead screw against 0.41 N·m: ×3.7 |
| Tip movement per 1/16 microstep | J1: 0.04 mm at full reach. J2: 0.02 mm | |

**The Z axis is back-drivable with an 8 mm lead [A].** If the Z motor loses power, the arm drops. Keep the motor enabled, and before a power-down drive Z to its lowest position. Check the lead on your screw: 2 mm or 8 mm.

## 5. The end effector

![Cutter](images/end_effector.png)

### Decisions

| Question | Options weighed | Choice and reason |
| --- | --- | --- |
| How to separate the fruit | Pull or twist at the knuckle; rotating disc; scissor pair; hot wire; bypass shear | **Bypass shear across a slot.** Pulling needs clamping near the 11 N bruise limit, and only reached 72.9% success in bending mode [M]. A disc cuts anything it touches. A hot wire is slow and power-hungry. With a slot, only the stem that is in the 8 mm slot can be cut. |
| Blades | Custom laser-cut steel; bought pruner head; **standard snap-off cutter blades** | **Two pieces of an 18 mm snap-off blade, 3 segments each.** One is fixed on the deck as the shear edge; the other is clamped in the printed lever. They're cheap, hardened, sharp and sold everywhere. A blunt one is replaced by snapping a fresh piece. Custom steel would need a laser cutter and heat treatment. |
| Holding the blades | Glue; drilled holes (hardened steel won't drill); pockets | **Pockets.** The fixed blade sits flush in a 0.5 mm pocket in the deck, clamped by a printed keeper strip. The moving blade sits in a pocket on the lever's underside, held by two 6 × 2 mm magnets. In use it is trapped against the deck. |
| Actuator | Servo; stepper and screw; solenoid | **Standard-size servo through a 4-bar.** It holds position, drives straight from the ESP32 with PWM, and needs no limit switch. The **DS3225MG (25 kg·cm) is recommended**. The MG996R on hand fits the same pocket and still clears the force target. |
| Approach | From below; from the side | **From the side, at calyx height.** The links stay out of the canopy. The V mouth catches a pedicel up to ±25 mm off the aim. |
| Getting the cut height right | Trust vision; detect the pedicel; touch the fruit | **Touch the fruit.** After the pedicel is in the slot, Z lowers the deck slowly until a feeler fork under the deck meets the fruit's shoulders. The cut is then a fixed ~10 mm above the shoulders, whatever the camera thought. This corrects the 6–24 mm pedicel error from the sim mechanically. |
| Holding the cut fruit | Fingers; suction; basket | **Basket with a padded door.** The fruit falls a few centimetres onto 10 mm EVA foam. A 10 mm EVA lip keeps it in while the arm backs out. |
| Into the crate | Trap door over a funnel; carry it to the crate; fabric sleeve | **Trap door over a padded funnel** on the rover, with a fabric sock down into the crate. |

### 5.1 How one pick goes

1. **Plan.** The camera (D405 class, 7 cm minimum range [M datasheet]) finds the fruit 15–35 cm away. The planner picks a J3 yaw so that the open mouth faces a side with no neighbouring fruit.
2. **Pre-pose.** The throat is aimed 60 mm short of the pedicel along the approach line. The deck is aimed at the estimated calyx height + 15 mm [A].
3. **Advance** at 30 mm/s until the IR beam across the throat breaks: the pedicel is in the slot. If 80 mm of travel goes by with no break, back out and re-detect.
4. **Touch-off.** Lower Z at 5 mm/s until the feeler releases its switch (at most 40 mm).
5. **Cut.** Swing the blade quickly to −8°. Then cross the pedicel slowly (−6° to +12° in about 1.5 s, roughly 4 mm/s at the stem), as Weng et al. 2024 recommend [M]. The raked edge pulls the pedicel into the end of the slot rather than pushing it out.
6. **Check.** The TCRT5000 in the rear wall should see the fruit within 0.5 s.
7. **Retreat.** Open the blade and back out along the approach line, accelerating at ≤ 0.5 m/s².
8. **Unload.** Move over the funnel, open the door to 75° for 0.6 s, close it, and check that the basket reads empty.

Cycle time is about 8–10 s per fruit [E], mostly arm travel to the funnel.

### 5.2 Cutter mechanics

| | Value | Tag |
| --- | --- | --- |
| Deck | 6 mm PETG; the slot and V are cut in it | A |
| Slot width, length | 8 mm, 12 mm, then a 60° V to a 50 mm mouth | A |
| Blades | 18 mm snap-off, 0.5 mm, 12.5 mm segments, score lines at about 60°; 3 segments = 37.5 mm edge | A: check your blades against `BLADE` in `arm_v2_geometry.py` |
| Fixed blade | Edge along the +y side of the slot, 0.6 mm proud of the printed slot wall, front corner where the V starts. Its ends lean back, so no edge sticks into the V. | A |
| Distal pedicel diameter, cut force | 3–5 mm; 19.6 N at 3 mm, 26.7 N at 3.5 mm, 33.0 N at 4 mm | M (Weng 2024) |
| Design force | ≥ 40 N at the stem | A |
| Blade-lever pivot to stem | 20 mm; M4 bolt into a heat-set insert in the deck, with the OD 6 spring under the head for preload | A |
| 4-bar (horn, link, lever arm) | 14 mm, 40 mm, 30 mm; servo travel 118° for 50° of blade | A, from the search in `arm_v2_calcs.py` |
| Lowest force ratio while cutting | 2.33 | E |
| Force at the stem while cutting, MG996R (0.5 N·m usable) | ≥ 58 N | E |
| Same, DS3225MG (1.2 N·m usable) | ≥ 140 N | E |
| Edge rake | 15° toward the slot end | A |

![Cutter force](images/cutter_force.png)

The MG996R clears the 40 N target by about 1.5×. Clone MG996Rs vary, so fit the DS3225MG if bench test 1 shows stalls. Run both servos from the 12 A buck at 6.0 V, never from the logic rail: an MG996R can draw more than 2 A when stalled.

**Nothing may stand proud of the deck top inside the lever's sweep.** The lever and the moving blade slide flat on the deck and on the fixed blade. So every screw near the lever comes from below: the flange bolts (M4 × 12 into the flange inserts), the servo bracket and the basket (M3 self-tapping). The keeper, camera bracket and IR posts sit outside the sweep and are screwed from above.

### 5.3 Sensors

| Signal | Part | Note |
| --- | --- | --- |
| Pedicel in the throat | 38 kHz modulated IR LED + TSOP38238 receiver in two posts 46 mm each side of the throat | Use modulated IR. A plain phototransistor is blinded by sunlight. The posts sit outside the blade's sweep. |
| Deck has touched the fruit | Feeler fork (printed) on an M3 pivot; its tail holds a roller micro switch closed | At rest the switch is pressed; contact releases it. A broken wire then reads as "touched", which stops the descent: fail-safe. Bias it with the OD 4 spring. |
| Fruit in the basket | TCRT5000 behind a window in the rear wall | It also confirms the basket is empty after a drop. |
| Blade open | Servo position command | Optional: a magnet on the lever and a hall switch, if a stalled servo needs catching. |

### 5.4 Basket, fruit size and drop height

- **Inside:** 95 × 95 mm. The front is open. The door pad sits 118 mm below the deck top [A].
- **Fruit up to 85 mm wide and 75 mm tall clears the 10 mm lip on the way in [A].**
- **Drop after the cut:** about 108 mm minus the fruit's height. The pad is 118 mm below the deck top, and the fork touches the fruit about 10 mm below it.
  - Typical 60–80 mm Indian table fruit (50–65 mm tall [E]): about 45–60 mm.
  - A 40 mm fruit: about 70–80 mm.

On 10 mm EVA foam that is below the bruising energy for typical fruit [E]. Bench test 3 checks it. The side walls have windows to save weight; the lower 35 mm is solid so the fruit can't roll out.

### 5.5 Unloading

- **Drop pose:** the door opens downward on a hinge along one side, so the fruit rolls off one edge into a padded funnel (mouth about 200 mm).
- **Funnel position:** fixed to the SCARA base, so it moves with the mast pins and the drop pose is the same joint set at every pin height. It sits inboard of J1 (over the rover deck), 250 mm from J1, outside the 114 mm inner limit set by J2.
- **Into the crate:** a fabric sock (a decelerator tube, as on apple platforms) runs from the funnel into the crate.
- **Low pins:** at pins 0 and 1 the SCARA base sits at or below the 0.50 m deck. The crate then has to ride lower, on a side tray. This is open item 3.

### 5.6 Parts and mass

| Id | Part | Material and how |
| --- | --- | --- |
| T-01 | Deck | PETG, printed top face down, 6 walls, 50% gyroid |
| T-02 | Fixed-blade keeper | PETG |
| T-03 | Blade-lever | PETG, 100%, printed with the blade pocket up; press in 2 × 6 × 2 mm magnets |
| T-04 | Link | PETG, 100% |
| T-05 | Cutter servo bracket | PETG |
| T-06 | Basket | PETG, 4 walls |
| T-07 / T-08 / T-09 | Door / pad / lip | PETG / 10 mm EVA / 10 mm EVA strip |
| T-10 | Door servo bracket | PETG |
| T-11 / T-12 | Feeler fork, hinge blocks | PETG |
| T-13 | IR posts | PETG |
| T-14 | Camera bracket | PETG |
| BUY-14 / BUY-15 | Fixed and moving blades | 18 mm snap-off blade, 3 segments each |

**Tool mass, including two servos and the camera: about 445 g [E].** The arm loads above include it.

## 6. Printing it

All parts fit the Kobra S1 bed with a 5 mm margin; `check_arm_v2.py` checks this.

| Part | Footprint, mm | Orientation | Settings |
| --- | --- | --- | --- |
| A1-01 Arm 1 tray | 210 × 88 × 44 | Floor down | 5 walls, 35% gyroid |
| A1-02 Arm 1 lid | 210 × 88 × 5 | Flat | 5 walls, 35% gyroid |
| A2-01 Arm 2 body | 240 × 196 × 31 (diagonal, 36°) | Top face down | 5 walls, 35% gyroid |
| A2-02 Arm 2 cover | 240 × 196 × 17 (diagonal, 36°) | Inner face down, J3 boss up | 5 walls, 40% gyroid |
| A1-03 J2 hub, J2-01 and J3-01 spindles | ≤ 78 × 78 × 44 | Flange down | 100%: these carry the joint loads |
| J2-02, J3-02 pulleys | ≤ 61 × 61 × 26 | Teeth up | 100%. **Print a 10 mm test ring of each first and check the belt seats.** The GT2 teeth are an approximation. |
| Cutter parts | ≤ 184 × 115 × 6 (deck); basket 104 × 111 × 129 | See section 5.6 | |

Diagonal placements put the part only 5 mm from the bed corners. Use a brim, and check the slicer preview before printing.

**Fits:** bearing seats are drawn at nominal +0.03 mm (6806: Ø42.06, 6807: Ø47.06, 608: Ø22.06). Printers differ, so print one bearing-seat test ring first and scale the hole compensation in the slicer until the bearing presses in by hand. **Heat-set inserts:** M4 × 8 in Ø5.6 holes. Press them in with a soldering iron at about 230 °C.

Assembly order:
1. Arm 1 tray onto the carriage face (4 × M5 × 25, washers both sides).
2. Belt 1 through the root window and around the motor pulley.
3. The two 608s into the idler boss, the M8 axle from below, and the stock compound pulley on top.
4. 6806 into the J2 hub; hub under the floor (6 × M4 × 20).
5. The J2 stack: lower thrust onto the spindle; spindle up through the hub, 6806 and upper thrust; pulley on top (4 × M4 × 20 into the inserts).
6. Belt 2, both idlers, Arm 1 lid (6 × M4 × 40). Tension both idlers.
7. Arm 2 body onto the J2 flange (6 × M4 × 20, nuts inside).
8. Into the cover: 6807 from below, 6806 from above. J3 spindle up from below, J3 pulley on top. J3 motor under the cover, belt on, cover onto the body (5 × M4 × 30). Tension with the motor slots.
9. The tool on the flange (4 × M4 × 12 from below the deck).

## 7. To buy

For the arm:

| Item | Qty | Why |
| --- | --- | --- |
| PETG filament | 2 kg | All new parts, test rings, reprints |
| M4 brass heat-set inserts, 8 mm long | 20 | J2 spindle ×4, J3 spindle ×8, blade pivot ×1, spares |
| M4 × 20 socket or pan head | 5 | The J2/J3 stacks need 20; 15 on hand |
| M4 × 12 | 4 | Deck to tool flange |

For the cutter ("push the limits" extras):

| Item | Qty | Why |
| --- | --- | --- |
| 18 mm snap-off cutter blades (Olfa LB or similar), pack | 1 | Fixed and moving blades, spares |
| DS3225MG servo (25 kg·cm, metal gear) | 1 | 2.4× the cut force of the MG996R |
| Neodymium magnets 6 × 2 mm | 4 | Hold the moving blade in the lever |
| TSOP38238 + 940 nm IR LED | 1 + 1 | Pedicel beam, sunlight-proof |
| TCRT5000 module | 1 | Fruit in the basket |
| Intel RealSense D405 (or another RGB-D camera with ≤ 10 cm minimum range) | 1 | Bill item 5.3 |
| 10 mm EVA foam sheet | 1 | Door pad, lip, funnel lining |
| M3 self-tapping screws, 8–10 mm | 20 | Deck screws (no heads above the deck in the sweep) |
| M4 × 16 | 1 | Blade pivot |

## 8. Bench tests before the field

1. **Cut force on your variety** (as in scara-redesign.md test 1). Then cut 20 pedicels in the real cutter. Pass: 20 of 20 cut, no stall.
2. **Touch-off.** Lower the deck onto 10 fruit on a staked plant at 5 mm/s. Measure the stub left after the cut. Target: under 10 mm (FAO about 1 cm [M]).
3. **Drop.** Drop 10 ripe fruit from the measured drop height onto the pad. Check for bruising after 24 h.
4. **Capture tolerance.** Approach a pedicel 0, 10, 20 and 25 mm off the aim. Pass: it enters the slot every time.
5. **IR in sunlight.** The beam must break on a 3 mm pedicel in direct sun.
6. **Blade pair.** With the preload spring fitted, the moving blade must cut paper along the whole slot, with no gap between the blades.
7. **Arm sag.** Hang 700 g on the tool flange with the arm straight, and measure the drop at the flange. The estimate is about 0.3 mm. Over 2 mm means a loose joint.

## 9. Open items

1. **The field number:** the real distance from the rover's side to the plant line (305 mm from J1 is assumed).
2. **Z lead screw:** 2 mm or 8 mm lead? An 8 mm lead needs a park routine.
3. **Crate position for low mast pins** (section 5.5).
4. **Economy motor length:** 40 mm is assumed. Measure it and update `NEMA17` in `arm_v2_geometry.py`.
5. **Stock compound pulley:** check its bore and height on the printed part. The boss assumes Ø8 and 246.2–265.0 mm in the STEP frame.
6. **Blade size:** check the snap-off blades you buy against `BLADE` (width, segment pitch, score angle) before printing the deck and lever.
7. **Simulation:** update the URDF to L2 = 200 mm and the v2 cutter geometry, then re-run the Gazebo harvest.

**Not checked:** no FEA. The stiffness figures treat each link as one solid box. The masses use typical weights for bought parts. Fits are nominal. The GT2 tooth form needs the test print.
