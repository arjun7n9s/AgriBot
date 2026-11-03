# SCARA redesign: Arm 2 and the tomato cutter

> **Superseded for Arm 2 and the cutter** by [arm-v2-design.md](arm-v2-design.md) (4 Nov 2026). The literature figures, the height plan (section 4) and the questions for Tejesh (section 7) still apply.

Drafted 31 Oct 2026 for AB-4. Already printed: the base, the Z axis and Arm 1. Still to design: Arm 2 and the end effector.

The geometry below was measured from `media/scara-arm/.../solidworks/SCARA Robot.SLDASM` (component positions; the joint axes are vertical). The tomato figures come from a literature pass. Each is marked **[M]** (measured in the cited source) or **[E]** (an estimate). Spot-check the sources before quoting them in a funding document.

## 1. The arm as it stands

| Item | Value | How it was found |
| --- | --- | --- |
| L1, J1 axis → J2 axis | **228.0 mm** | Assembly: J1 at x 42.5, J2 at x 270.5 |
| L2, J2 axis → J3 (wrist) axis | **136.5 mm** | Assembly: J2 (270.5, 0) → J3 (372.8, 90.3) |
| Horizontal reach | 364.5 mm, with a dead zone inside a 91.5 mm radius | L1 ± L2 |
| J3 drive | Pancake NEMA 17 (L24) at J2 → GT2 belt inside Arm 2 → 90-tooth pulley at J3 | Assembly |
| J3 belt check | Calculated loop 398.4 mm against the stock 400 mm belt | Belt formula; this confirms the measurements |
| J2 drive | NEMA 17 on the Z carriage → two-stage belt (80-tooth intermediate) inside Arm 1 → 92-tooth pulley | Assembly |
| Wrist bearings | 2 × thrust 35×52×12 and 1 × radial 30×42×7 | Assembly. The order list's 51107 and 6806-ZZ are these sizes. |
| Z axis | 4 × Ø10 × 400 mm rods, 8 mm lead screw, clamps at heights 41 and 411 mm | STEP part clearances: **about 249 mm** (166 mm down to the base cover, 83 mm up to the top limit switch). Confirm on the printed carriage. See [assets.md](assets.md). |
| Stock gripper | About 150–200 g including the MG996R | STL volumes at 100% PLA [E] |

Arm 1 keeps the J2 belt drive. Leaving it untouched means the printed work stays valid.

## 2. Recommendation: keep Arm 1 and make a new Arm 2

Redesigning both links gains little. The rover already moves along the row, so reach in that direction is free. Height is the real gap (section 4), and no link length fixes that. A new Arm 1 would also mean redesigning the J2 two-stage belt.

### Choosing L2

Arm 2 carries the J3 belt, so L2 has to suit a closed GT2 belt you can actually buy. The motor pulley sits about 6 mm off the J2 axis, so the belt centre distance is L2 + 6 mm.

| L2 | J3 belt (6 mm GT2, closed) | Reach | Dead-zone radius | Tip deflection vs stock, same section |
| --- | --- | --- | --- | --- |
| 136.5 (stock) | 400 | 364 | 92 | 1.0× |
| ~182 | 488 | 410 | 46 | 2.4× |
| **~188** | **500** | **416** | **40** | **2.6×** |
| ~214 | 552 | 442 | 14 | 3.9× |
| ~238 | 600 | 466 | 10 | 5.3× |

**Start from L2 ≈ 188 mm with a 500 mm belt.** Use 182 mm with a 488 mm belt if that is what the shop stocks. Either way, add a slotted motor plate with about ±3 mm of tension travel. That gives about 14% more reach and cuts the dead zone by more than half. The extra deflection is won back with a stiffer section. Beyond about 214 mm, deflection and inertia rise quickly for little gain. Lock L2 only after the field numbers in section 7 are in.

### Arm 2 details

- **Section:** keep the 78 mm-deep box with its cover, add internal ribs, and print 4 perimeters with about 40% gyroid infill. Taper the outer end so it slips through foliage.
- **Tool flange at J3:** 4 × M3 heat-set inserts on a 30 mm circle, a 20 mm locating spigot, and a key for orientation. New cutters then bolt on without reprinting Arm 2.
- **J3 range:** ±150°. Route the cables in a service loop with spiral wrap (bill item 4.5), and add a micro-switch home on J3.
- **Reuse:** the existing J3 bearing stack, pancake motor and 90-tooth pulley carry over unchanged.
- **Material: use PETG, not PLA, for Arm 2 and the cutter.** PLA starts softening at about 55–60 °C, and dark parts in an Indian field in direct sun can get that hot. The bill of materials lists 3 kg of PLA, so changing material is a decision for Tejesh. That's why it is flagged here and not changed in the bill.

## 3. The end effector: V-capture bypass cutter with a catch cradle

### What it has to do

| Requirement | Value | Source |
| --- | --- | --- |
| Cut the distal pedicel (between the knuckle and the calyx) | 3–5 mm diameter. Cut force 19.6 N at 3 mm, 26.7 N at 3.5 mm, 33.0 N at 4 mm. Proximal pedicel 62 N. | [M] Weng et al., *Agronomy* 2024, 14:2274 |
| Cut technique | Slicing cut at 63–78°, slow (2–4 mm/s). Faster cuts made the pedicel tilt away. | [M] Weng 2024 |
| Stub length | Shortest possible. Stubs puncture other fruit. FAO allows up to about 1 cm. | [M] FAO handling manual; UNL G1746 |
| Fruit size | 40–85 mm diameter, up to about 250 g. Indian table varieties are often 80–100 g. | [M] AGMARK, NHB grades, IIHR (Arka Rakshak) |
| Handling limit | Squeeze under about 11 N. Free drop onto a hard surface under about 3–4 cm (onset of impact bruising 60–80 mJ). | [M] Zhou (IJABE); Van Zeebroeck 2007; [E] drop height |
| Blade force design target | **At least 40 N at the cut point** | [E] covers 5 mm distal pedicels with margin |
| Tool mass | Under about 320 g | [E] budget below |

Cutting is chosen over pull-and-snap. Snapping at the knuckle takes 13–72 N of bending or pulling, depending on the stem. The fruit has to be clamped to do that, which is when damage happens (Weng 2024). Bending-mode picking reached only 72.9% success (Liu et al., cited in Weng 2024).

### Concept

```
SIDE VIEW (approaching the fruit from the right)

                pedicel
                   |
   flange  ________|______________     <- top plate: V-guide and bypass blade (steel)
   to J3 =|________|____V_________|       the throat sits just above the calyx
          |  spine |   (calyx)        <- open front: the fruit enters sideways
          |        |    .--.
          |        |   (    )  fruit
          |        |    '--'
          |________|_____________     <- cradle floor: 10 mm soft foam,
                    \___ trap door       servo 2 opens it over the crate

TOP VIEW OF THE TOP PLATE

             wide V mouth (about 40 mm) faces the plant
        \                      /
         \                    /
          \______    ________/
                 |  |  <- 6 mm throat: the distal pedicel fits, the 5–8 mm knuckle does not
   pivot o-------hook blade--->   sweeps across the throat (bypass, slicing angle)
```

1. **Approach.** The camera gives the fruit pose and size. J3 turns the open mouth toward the fruit, along the side with no neighbouring fruit. The arm sets the top plate about 5 mm above the calyx and moves in horizontally. The pedicel slides into the V, and the fruit slides into the open front of the cradle.
2. **Confirm.** An IR break-beam across the throat confirms a stem is there. With no beam break, the tool does not cut and backs out.
3. **Cut.** Servo 1 swings a hooked bypass blade across the throat, slowly, ramping the PWM setpoint (see "Cut speed" below). The hook pulls the pedicel into the corner against a fixed steel edge, so it can't slide out. The edge is angled for a slicing cut in the 63–78° band.
4. **Catch.** The fruit drops onto the soft floor. The floor-to-top-plate gap is set for the field's typical fruit (section 7), so the drop stays under about 3 cm for typical fruit. A TCRT5000-class reflective sensor in the floor confirms the catch.
5. **Unload.** The arm moves over the crate funnel on the rover. Servo 2 opens the trap door, closes it again, and the arm returns.

### Why this shape

- **Self-locating cut.** The 6 mm throat accepts the distal pedicel but stops at the thicker knuckle. The cut therefore lands on the cheapest stem section (33 N at 4 mm against 62 N proximal) and leaves a short stub.
- **The blade guard is built in.** The moving blade hides behind the fixed plate when open, and it only crosses a 6 mm slot. That covers bill items 2.10 (guard) and 2.11 (home sensor).
- **The tool nose reaches into the canopy, the links don't.** The mouth sits about 60 mm ahead of the J3 axis, so Arm 2 stays mostly outside the plant.
- **One fruit at a time.** The 6 mm throat admits one pedicel. Neighbours in a truss are handled by choosing the approach angle with J3.

### Cutter drive and force check

- **Blade:** a replaceable stainless scalpel or utility-knife insert, clamped in a printed carrier. The fixed bypass edge is 1 mm stainless sheet. PLA or PETG can't hold an edge.
- **Linkage:** MG996R horn at 15 mm → link → blade arm at 25 mm from the blade pivot. The stem sits about 12 mm from the pivot. The force at the stem is about T × (25/15) / 0.012 ≈ **139 × T** (N, with T in N·m).
- **Margin:** the MG996R stalls at about 0.9–1.1 N·m (datasheet range; check yours). At 45% of stall (about 0.45 N·m) the cut force is about **62 N**, against the 40 N target. Lay the linkage out so it nears toggle at the end of the stroke, where the cut finishes, so force rises exactly where it's needed.
- **Cut speed:** command the servo in small steps so the blade crosses the throat in about 1–2 s. A servo left to slew at full speed is the fast cut that tilts the stem away.
- **Power:** run the servos from the 5–6 V servo converter (bill item 3.17), not from the logic rail. One MG996R can draw over 2 A when stalled.

### Mass budget [E]

| Part | g |
| --- | --- |
| 2 × MG996R | 110 |
| PETG body, top plate carrier, cradle | 90 |
| Blade, fixed steel edge, pins | 25 |
| Foam and TPU lining | 20 |
| Sensors and wiring | 20 |
| RGB-D camera on the tool, if mounted here | ~60 |
| **Total** | **~325** |

On the camera: the tool camera needs a short minimum depth range. Some RGB-D cameras can't see closer than about 25–30 cm. Check the minimum range in the chosen camera's datasheet before fixing bill item 5.3.

### Alternatives considered

| Option | Why not first |
| --- | --- |
| Pull or twist and snap at the knuckle | Needs clamping force near the damage limit, and only 72.9% success in bending mode |
| Suction cup plus scissors (Ling 2019, Park 2023) | Needs a vacuum pump, valve and hose on the arm. Reported success was 80.6–87.5% at 15–30 s, without beating a passive cradle on cost. |
| Saw cup taking the whole truss (AHPPEBot 2024) | Harvests whole trusses. Indian table tomatoes are usually picked fruit by fruit. |
| A fabric sleeve from the tool down to the crate (no trap door) | Faster unloading, but a hanging sleeve snags in the canopy. Worth trying once the basic tool works. |

## 4. The real gap: height

The fruit band on staked Indian plants is roughly **20–150 cm** above the ground [E]. Measured fruit heights for Indian fields were not found. Plant height is 55–170 cm [M], and stakes are 1–2 m [M]. The arm's own Z axis covers about 25 cm. No link redesign fixes that.

- **Prototype:** mount the SCARA base on a mast beside the rover body with pinned height positions every ~18 cm. This is now modelled: `scara_mast` in [assets.md](assets.md#rover), five pins, base at 0.18–0.90 m. Harvest a row in several passes at different heights. Cheap, rigid, and it keeps the printed Z axis.
- **Funded upgrade:** a motorised lift stage under the SCARA, as a second, coarse Z axis. Add it to the bill as a new line when chosen.
- **Not recommended:** stretching the printed Z axis to 1 m+ on Ø10 rods. Long thin rods bend, and it throws away the printed base.

## 5. Bench tests before committing the CAD

1. **Pedicel cut force on your own variety.** Clip 20 pedicels at the distal section. Cut them with the chosen blade on a lever, pulling with a spring scale. Record the diameter and the force. This decides the linkage ratio.
2. **Fruit size.** Measure the diameter and height of 20 ripe fruit. This decides the floor-to-top-plate gap, then the cradle width.
3. **Knuckle stop.** Check that a 6 mm slot passes the distal pedicel and stops at the knuckle on your variety.
4. **Drop.** Drop ripe fruit onto the chosen foam from 2, 3 and 5 cm. Check for bruising after 24 h.
5. **MG996R torque.** Hang a weight on a 20 mm arm at 6 V and find the torque it holds without stalling.

## 6. Build order

1. Bench tests 1–4 (no CAD needed).
2. Cutter head on its own: top plate, blade, servo, break-beam. Hand-held test on real plants.
3. Cradle and trap door added to the head. Test catch and unload by hand.
4. New Arm 2 with the tool flange, PETG.
5. Mount on the arm. Pick tests on a staked plant at the bench.
6. Mast on the rover.

## 7. Needed from Tejesh

1. **Row spacing in the target field, where the rover drives, and the distance from the rover's side to the plant line.** This locks L2 and the mast position. Indian spacing is usually 75–90 cm between rows [M], and the rover is 70 cm wide.
2. **The lowest and highest ripe fruit in that field.** This sets the mast positions.
3. **Bench tests 1–3.**
4. **Is the printed Arm 1 the stock 228 mm part?** Was anything changed?
5. **Is PETG OK for Arm 2 and the cutter?**

## Sources (from the literature pass)

- Weng et al., *Agronomy* 2024, 14:2274, doi 10.3390/agronomy14102274: pedicel diameters, cut, tensile and bending forces, cut angle and speed
- Kondo et al. 2010, *EAEF* 3(1):20: peduncle diameter and cut force; truss-harvest failure modes
- Ling et al. 2019, *Robotics and Autonomous Systems* 114:134: vacuum plus scissors, 87.5%
- Panasonic US12349627B2 and the 2018 Panasonic story: ring pull-and-snap, keeps the calyx
- AHPPEBot, arXiv 2405.06959: saw-cup truss harvesting, 86.7%
- FAO handling manual; UNL Extension G1746: stem stubs
- AGMARK / DMI grades; NHB weight grades; IIHR Arka Rakshak: fruit size
- Van Zeebroeck 2007: impact bruising energy
- TNAU agritech, Agropedia, NHB via Vikaspedia: Indian spacing and staking
