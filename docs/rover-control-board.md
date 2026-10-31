# Rover control board: HH-8130K-RXT2

This is the stock motor-and-relay board from AgriBOT's ride-on base. Everything here comes from Tejesh's twelve routing photos. The older photo set was used only for three markings that the routing photos can't resolve, and those are flagged where they appear. These are phone photos, so a marking, pad, trace, or pin that is not clearly readable is written as **unknown** and has a question at the end.

Deliverables:

- 3D model: `/cursor/stores/bc-d1b8e747-87e9-4a20-9cfe-d9339460fb08/media/rover-board.glb`
- Rendered preview: `/cursor/stores/bc-d1b8e747-87e9-4a20-9cfe-d9339460fb08/media/rover-board-3d-preview.png`

## Photos opened

All twelve files in `media/motor-controller/routing/` were opened in order before anything was written.

| File | What it shows |
| --- | --- |
| `routing-01.png` | Component side from almost straight above, board rotated a quarter turn. Shows all heatsinks, the relay face, the yellow F1 disc, and the lower-edge tabs. |
| `routing-02.png` | Component side backlit, low angle from the blade-tab edge. Shows the silhouetted traces along the 7-way-header edge, the meander trace, the SOP-16, `16M`, `C8`, `C10`, `C11`, `F1`, and the `RR` / `RL` pads. |
| `routing-03.png` | Same backlit edge, lower and closer. Shows the 7-way header, the F1 disc standing on its leads, and traces around the two SOP-8s. |
| `routing-04.png` | Same backlit edge, slightly higher. Shows the heatsink tops against the relay, and the meander trace. |
| `routing-05.png` | Component side, dark, backlit from behind, rotated a quarter turn. Shows the meander trace and right-edge traces in silhouette, plus the `RR` / `RL` labels. |
| `routing-06.png` | Same orientation with more front light. Shows the full layout plus the backlit right-edge copper. |
| `routing-07.png` | Oblique view from the relay side, backlit. Shows heatsink fin counts, the J1 link, and C16. |
| `routing-08.png` | Component side, rotated 180°, front-lit. This is the sharpest relay print and the opposite-side view of the blade tabs. |
| `routing-09.png` | Near-repeat of 07. Gives the overhead fin profiles of all three heatsinks. |
| `routing-10.png` | Component side, square-on and front-lit. This is the master top view: rectified to millimetres and used as the model's top texture. |
| `routing-11.png` | Solder side, square-on. |
| `routing-12.png` | Solder side, square-on, sharper. Rectified and used for every through-hole position and as the model's bottom texture. |

Support only (older set in `media/motor-controller/`): the `CSC2111FL` top mark on the two SOP-8s, `2A104J` on C16, and `16.000 MHz` on the crystal. None of these three is legible in the routing photos.

## Frame, scale, and what was measured

All positions use the component side as seen in `routing-10.png`. The origin is the top-left corner. X runs right and Y runs down, both in millimetres. The 7-way header sits at the top-left, and the blade tabs run along the bottom edge.

- **Outline:** square, 75.0 × 75.0 mm (±1.5 mm). The scale is set by the rectified board corners. It checks out against the 7-way header's 2.50 mm pin pitch and the TO-220 lead pitch (measured 2.4–2.6 mm against a nominal 2.54 mm).
- **Thickness:** 1.6 mm is assumed, not measured.
- **Layers:** two copper layers are visible. Backlight passes through the board with only outer-layer traces in silhouette.
- **Solder mask:** blue on both faces.
- **Through-hole positions:** taken from solder-joint centroids in the rectified `routing-12.png`. Top-view positions of tall parts are shifted by camera parallax, so they were not used.
- **SMD package positions:** taken from the rectified `routing-10.png`, where parallax is under 0.5 mm. An orthographic overlay of the model on the photo lines up for the SOP-16, both SOP-8s, IC4, D4, and the crystal.
- **Heights:** these are estimates and should be read as ±3 mm. The relay is fixed at 15.3 mm, which is assumed rather than measured: it is the standard height for a 19.0 × 15.5 mm relay body. Other heights come from how far each part leans in the rectified `routing-10.png`, scaled to the relay. Heatsink H1 is clearly taller than H2 and H3 in `routing-04.png` and `routing-07.png`.

## Markings

**Readable in the routing photos:**

| Marking | Where | Photos |
| --- | --- | --- |
| `HH-8130K-RXT2` | Silkscreen on both faces | 01, 02, 08, 10, 11, 12 |
| `YONGNENG` | Relay top | 08 and 10. The fifth letter reads N in both full-resolution crops. Earlier readings of `YONGSENG` / "Yongsheng" are superseded. |
| `YX306H-S-112G`, `16A 250VAC`, `16A 30VDC` | Relay top | 08, 10 |
| `D1RG2…` | Relay lot code | 08, 10. The last two or three characters are blurred; `D1RG261` is only a candidate. |
| `JK` | Silkscreen just below the relay | 10. The glossy relay face also shows a reflection of it. |
| `+/−` | Lower-edge blade group 1 (left) | 08, 10 |
| `F1`, `Q3`, `Q4`, `C14`, `C15`, `C16`, `C17`, `J1`, `IC4`, `D4`, `D5`, `R20`, `R21`, `C13`, `R11`, `RR`, `RL`, `16M` | Silkscreen | 02, 05, 06, 08, 10 |

**Unknown or disputed:**

- **Lower-edge labels at blade groups 2 and 3: unknown.** Each shows `F`, then `/`, then a third character whose top is covered by glare and a solder fillet. `F/B` is the closest candidate, but it is not confirmed. This document records them only as `F/?` until a sharper photo shows the third character.
- **2-way header label (lower-right): unknown.** It reads as either `R/L` or `RA`, because the slash and the L merge in every photo.
- **Footprint left of `RR`:** the label looks like `J3`. Whether a part is fitted there is unknown.
- **Unknown part markings:**
  - the SOP-16
  - IC4
  - D4
  - all four TO-220s, whose faces are hidden by heatsinks or screws
  - the F1 disc
  - every electrolytic's value
- **Relay coil voltage:** not legible.
- **7-way header pin labels:** not resolved in the routing photos. Tejesh read them on the board. See “Confirmed on the bench” below. The photos are not the source for those seven names.

## Component side layout

H1, H2, and H3 and the blade-group numbers are names used in this note. They are not printed on the board. Sizes are W × D × H in mm, and "est." marks a height estimate.

| Item | Silkscreen | Position (mm) | Size / detail |
| --- | --- | --- | --- |
| 7-way white header, top entry | none read | Body X 0–19.7, Y 1.3–7.05. Pins at Y 4.0, X 2.25 → 17.25. | Pins on 2.5 mm pitch; ~7 high (est.) |
| Yellow radial disc | `F1` | Leads (2.25, 12.0) and (2.25, 17.25) | Ø ~8 × 2.5 thick, standing on edge, top ~10 (est.). Slight tilt not modelled. |
| Heatsink H1 (comb) | none | Spine X 7.3–9.3, Y 19.7–51.7 | 6 plates toward the left edge (X ~0.3). End plates continue to X 18.0 to form a channel for Q3/Q4. ~20 high (est.) |
| TO-220 | `Q3` | Leads X 11.6; Y 27.4, 30.0, 32.4 | Tab screwed (brass screw) to the H1 spine |
| TO-220 | `Q4` | Leads X 11.6; Y 39.4, 41.6, 44.3 | Same as Q3 |
| Heatsink H2 (U-channel) | none | Spine X 21.5–23.5, Y 24.3–47.7 | 6 plates toward +X (to X ~32.7). End plates continue to X 16.7. ~17 high (est.) |
| TO-220 in H2 | unknown | Leads X 19.1; Y 33.0, 36.0, 38.6 | Screwed to the H2 spine |
| Heatsink H3 (U-channel) | none | Spine X 39.5–41.5, Y 24.3–47.7 | Same profile as H2. Plates reach X ~49, touching the relay. |
| TO-220 in H3 | unknown | Leads X 36.9; Y 33.7, 36.2, 38.7 | Screwed to the H3 spine |
| Black sealed relay | `JK` | Body X 49.05–64.55, Y 23.9–42.9 | 15.5 × 19.0 × 15.3 (height assumed). Five pins: (50.5, 26.7), (62.2, 27.4), (50.9, 38.8), (62.2, 38.2), (56.5, 40.6). |
| Electrolytic | `C14` | Centre (6.4, 60.0) | Ø ~8, ~12 high (est.) |
| Electrolytic | `C15` | Centre (6.2, 68.6) | Ø ~8, ~12 high (est.) |
| Electrolytic | `C17` | Centre (67.9, 36.8) | Ø ~5, ~7 high (est.) |
| Electrolytic | not read | Centre (40.2, 16.4), between `R1` and `V1` | Ø ~6, ~7 high (est.) |
| SMD can-type electrolytic | not read | Centre (51.3, 17.6), left of `C7` | Ø ~4. No through-holes on the solder side. |
| Green film capacitor | `C16` | Centre (41.7, 50.6) | ~8 × 3.5 × 10 (est.). `2A104J` from the older close-ups. |
| Bare wire link | `J1` | Holes ~(21.6, 50.9) and (31.9, 50.8) | Lies about 1 mm above the board |
| Blade group 1 | `+/−` | Tab A (blade along Y): legs (23.25, 58.5) and (23.25, 63.4). Tab B (blade along X): legs (16.1, 69.8) and (21.4, 69.8). | Upright 6.3 mm-class tabs with a centre hole and a side tongue, ~8 high (est.) |
| Blade group 2 | unknown (`F/?`) | Tab A legs (35.25, 58.5) / (35.25, 63.4). Tab B legs (31.5, 69.8) / (36.75, 69.8). | Same tab type |
| Blade group 3 | unknown (`F/?`) | Tab A legs (49.9, 58.5) / (49.9, 63.4). Tab B legs (46.5, 69.8) / (51.4, 69.8). | Same tab type |
| 2-way white header, top entry | unknown (`R/L` or `RA`) | Body X 62.6–70.9, Y 66.1–71.5 | Pins ~4.1 apart at (64.65, 68.6) and (68.8, 68.6). ~8 high (est.) |
| SOP-16 | not read | Centre (61.1, 10.25), long axis along X | Marking unknown |
| Two SOP-8s, end to end | not read | Centres (24.9, 14.1) and (24.9, 19.1), long axis along Y | `CSC2111FL` from the older close-ups |
| SOP-8 | `IC4` | Centre (68.4, 47.5), long axis along X | Marking unknown |
| Metal-lid crystal | `16M` beside it | X 66.6–68.9, Y 11.1–14.7 | `16.000 MHz` from the older close-ups |
| Black SMD body | `D4` | X 66.0–71.4, Y 56.8–60.0 | Package unknown |
| Empty 2-pad footprints | `RR`, `RL` | X ~67.5–71.5, Y ~26.5–29.5 | No parts fitted; pads bare |
| Footprint | `J3`(?) | X ~66–67, Y ~26.5–29.5 | Fitted or not: unknown |
| Diode footprint | `D5` | X ~3–10, Y ~53–55 | Fitted or not: unknown |

Many small resistors and capacitors are visible but are not listed one by one. They appear in the model only through the photo texture.

## Copper routing that is actually visible

No nets are claimed. Where a trace passes under a heatsink, the relay, or a shadow, its route is unknown.

**Component side** (backlit 02–07 and 09; front-lit 01, 08, 10):

- **Meander trace:** a wide copper trace runs in a meander along the 7-way-header edge, about X 21–58, Y 0–4, between the board edge and the `HH-8130K-RXT2` text. Its right end heads toward the `C8` / SOP-16 corner. A shape like this is often a printed radio antenna, but nothing in the photos confirms that.
- **Top-strip traces:** groups of narrow traces link the 7-way header, the two SOP-8s, and the SOP-16 area along the top strip. Where they dive under heatsink H2 and the relay, they can't be followed.

**Solder side** (11, 12):

- **Tinned areas under the blade tabs:** each blade tab sits on a wide, solder-coated copper area.
  - Group 1's two tabs each have their own area.
  - Groups 2 and 3 share areas: the two blade-along-Y tabs sit on one continuous area (X ~32–53, Y ~56–66), and the two blade-along-X tabs sit on another (X ~29–54, Y ~67.6–72).
  - If the copper under that solder is continuous, groups 2 and 3 are wired in parallel. That has not been measured.
- **Tinned strips at H2 and H3:** a solder-coated strip runs along X through the middle lead of the H2 and H3 TO-220s to each heatsink's solder pin (X ~14.4–23.8 and ~31.3–42.1, at Y ~36).
- **Q3 / Q4 and relay:** Q3 and Q4 sit on tinned pads, and the relay's five pins sit in wider copper.
- **Narrow parallel traces:**
  - A bundle of 4–5 traces leaves the 7-way header's pads and runs under the H1 area (X ~7–10, Y ~8–36) toward the TO-220 pads.
  - A second set runs along X at Y ~10–16 into a dense via field around the SOP-16 and crystal (X ~63–74, Y ~5–18).
  - One long trace follows the left edge (X ~3–5, Y ~18–70).
  - None of these endpoints is resolved.

## The 3D model

`rover-board.glb` is in metres, Y-up, with the board centred on the origin and the component side up. It has 61 named meshes and 16 PBR materials. Three JPEG textures are embedded:

- the rectified `routing-10.png` on the component face
- the rectified `routing-12.png` on the solder face
- the photographed relay top

Copper and silkscreen are therefore exactly as photographed; no trace or label was redrawn.

**Included:** every part in the layout table except the small passives and the empty footprints, plus 54 solder joints on the solder side at their measured positions.

**Mesh names carry the uncertainty:**

- `Blade_group2_label_unknown_tab_A`
- `Header_2way_label_unknown_RL_or_RA`
- `TO220_in_H2_designator_unknown`
- `SOP16_top_right_marking_unknown`

**Left out on purpose:**

- relay pins under the body
- TO-220 markings
- anything under the heatsinks or relay that no photo shows
- small passives, which exist only in the texture
- F1's tilt

**Known artefact:** tall parts lean toward the 7-way-header edge in `routing-10.png`. The top texture therefore carries smeared ghosts of the heatsinks and relay just above each real part. They were left in rather than painted over, because painting over them would invent board surface the photo doesn't show.

The preview PNG has three panels: the component side from the blade-tab edge, a second view from the 7-way-header corner (for F1, the SOP-8 pair, and the SOP-16), and the solder side. Red callouts mark unknown labels or markings.

## Priority

Make this board programmable from our own logic. The 3D model is not the work.

## Confirmed on the bench

Tejesh read the 7-pin header and tried it with the circuit connected. This is not from the photos.

Pin order, as read: `+` `V` `ROCK` `−` `B` `F` `S`

- Shorting `−` to `B` makes the motors run backwards.
- Shorting `+` to `V` turns the circuit on. It is not a remote switch. Once it is on, control can be code or manual.

Not established: what `V` is electrically, which of `−` and the pin marked `B` is the shared motion pin, and which third pin is forward. `ROCK` is recorded as Tejesh spelled it.

## Working model

Tejesh, 9 Oct 2026. Treat the header as five command pins, a b c d e:

- Short a to b: circuit on.
- Short c to d: forward.
- Short c to e: backward.

Bench fit so far: a to b is `+` to `V`, already confirmed. c to e is `−` to the pin marked `B`, already confirmed as reverse. Which of those two is c is unknown. d is whichever other header pin, shorted to c, makes the wheels go forward. That short has not been made yet.

The letter b here is not the pin marked `B`.

## What this does not establish

- No full pinout. The 7-pin names and two tested shorts are in “Confirmed on the bench.” `ROCK`, `F`, and `S` are still untested. The 2-pin header is still unread.
- No net list.
- No motor-to-terminal mapping.
- No battery voltage.
- No proof of where the handset's radio receiver is. The meander trace and the 16 MHz crystal are suggestive, not proof.
- The board still can't be commanded from our own microcontroller. That needs the answers below plus continuity measurements.

## Need from Tejesh

1. **Labels at blade groups 2 and 3.**
   - Why it matters: these tabs likely carry motor current. The label is the only on-board hint of what each one drives.
   - Settles it: one square-on macro of each label, with diffuse light from the side so the solder fillet doesn't flare. Or just say what is printed there.
2. **2-way header label (`R/L` or `RA`).**
   - Why it matters: it tells us whether this is a left/right steering connection or something else.
   - Settles it: a square-on macro of the silkscreen under the header, or a note of what it plugs into in the car.
3. **Which pin is c, and which pin is d.**
   - Why it matters: forward is c shorted to d. Reverse is already `−` to the pin marked `B`, so one of those two is c and the other is e.
   - Settles it: with `+` held to `V`, short `−` to `ROCK`, then to `F`, then to `S`. Then short the pin marked `B` to those same three, one at a time. The pair that drives forward names c and d.
4. **The board installed in the car.**
   - Why it matters: it shows which blade tab and header goes to the battery, each drive motor, and the steering motor.
   - Settles it: photos of the mounted board with every connector attached, following each wire to its far end.
5. **Battery voltage.**
   - Why it matters: it sets the tab ratings and the safe test range for every measurement that follows.
   - Settles it: a photo of the battery label, or a meter reading across the `+/−` tabs.
6. **Whether groups 2 and 3 are joined.**
   - Why it matters: the shared tinned areas on the solder side suggest the two groups are wired in parallel. If true, two motors share one output.
   - Settles it: continuity readings between matching tabs of groups 2 and 3, and between each tab and the nearest TO-220 or relay pin.
7. **Markings on the four TO-220s, and whether their tabs are insulated from the heatsinks.**
    - Why it matters: the part numbers say whether these are MOSFETs, transistors, or regulators. The solder-side strip from each H2/H3 TO-220's middle lead to the heatsink pin suggests those heatsinks are tied to that lead. Treat them as live until measured.
   - Settles it: a photo of each TO-220's printed face, which may mean loosening a screw, plus a continuity check from each heatsink to the TO-220's middle lead.
8. **SOP-16, IC4, and D4 markings.**
   - Why it matters: the SOP-16 is the likely controller or receiver chip, and IC4 may drive the relay. Both decide where a microcontroller could take over.
   - Settles it: a sharp macro of each with light at a low angle across the chip.
9. **Relay coil voltage and full lot code.**
   - Why it matters: we need the coil voltage to drive or monitor the relay directly.
   - Settles it: a square-on macro of the relay's printed face, with no glare on the lower text lines.
10. **`RR`, `RL`, and `J3`.**
    - Why it matters: `RR` and `RL` are empty two-pad footprints next to the relay. They could be factory test points or option pads, which would make them useful injection points.
    - Settles it: an angled macro of the three footprints, or any knowledge of the board's variants.
11. **The meander trace and the radio receiver.**
    - Why it matters: if this board holds the handset receiver, the handset protocol can be decoded here rather than on a separate module.
    - Settles it: confirm there is no separate receiver module in the car, and a backlit photo of the full meander, including both of its ends.
12. **Copper hidden under the heatsinks and the relay.**
    - Why it matters: the routes between the SOP-16 area and the TO-220s pass under these parts.
    - Settles it: backlit photos taken through the board from the solder side with the light behind the relay and heatsinks, or continuity readings between the SOP-16 pins and each TO-220 lead.
13. **Real part heights and board thickness.**
    - Why it matters: model heights are estimates (±3 mm), and the relay height is assumed.
    - Settles it: caliper readings for the board thickness, the relay, each heatsink, C14/C15, the blade tabs, and both headers.
14. **Capacitor values and the F1 rating.**
    - Why it matters: C14/C15 voltage ratings bound the supply voltage, and the F1 rating bounds the logic or supply current.
    - Settles it: a photo of each electrolytic's sleeve text, and of the F1 disc's face.
15. **Whether D5 is fitted.**
    - Why it matters: its two pads and diode symbol are visible, but not whether a part sits on them. If a part is fitted, D5 is in the supply path near C14/C15.
    - Settles it: an angled macro of the D5 footprint.
16. **The handset remote's other side.**
    - Why it matters: its radio chip and antenna would confirm the link that the board must be receiving.
    - Settles it: a sharp photo of the remote PCB's unphotographed side.
