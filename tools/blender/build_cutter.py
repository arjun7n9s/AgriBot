"""Build the V-capture bypass cutter (end effector concept v0) for the sim.

Follows docs/scara-redesign.md, section 3. This is a simulation and visual
model, not a manufacturing design: wall thicknesses, fasteners and the servo
linkage are indicative.

Frame: origin at the centre of the tool-flange face (where it meets the J3
flange), x forward toward the fruit, z up. The tool hangs below (negative z).

Output (sim/agribot_description/meshes/cutter/):
    visual/cutter_body   .glb .obj   everything that does not move
    visual/cutter_blade  .glb .obj   hooked blade and carrier, in the blade-pivot frame
    visual/cutter_door   .glb .obj   trap door, foam pad and lip, in the hinge frame
    cutter_frames.json               joint origins and key points for the URDF
    docs/images/cutter_preview.png

    blender -b --factory-startup --python tools/blender/build_cutter.py
"""
import json
import math
import pathlib
import shutil
import sys

import bpy

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import agri_common as ac  # noqa: E402

OUT = ac.REPO / "sim/agribot_description/meshes/cutter"


def mm(*v):
    return tuple(x / 1000 for x in v)


def mmp(points):
    return [mm(*p) for p in points]


# ---- design parameters (mm) -------------------------------------------------
P = dict(
    flange_r=20, flange_t=5,
    plate_top=-48, plate_t=4,            # top plate: z from -52 to -48
    mouth_x=118, mouth_half=20,          # 40 mm V mouth
    throat_x0=57, throat_x1=74, throat_half=3,   # 6 mm throat
    pivot=(48, 14),                      # blade pivot
    blade_open_deg=15, blade_stroke_deg=70,
    floor_top=-137, floor_t=3,           # cradle floor (trap door) top face
    foam_t=8,
    cradle_half=42, wall_t=3, wall_top=-98,
    hinge=(98, -140),                    # trap-door hinge (x, z), axis along y
    # Side-mounted RGB-D camera outside the left wall, at fruit height, turned in
    # toward the cradle: it sees the target ahead and the fruit once caught. (A
    # first version above the top plate looked straight down onto the plate.)
    camera=(-2, -70, -84), camera_pitch_deg=5, camera_yaw_deg=28,
)


def body_parts():
    bone, green, steel, black = ac.pal("pla_bone"), ac.pal("pla_green"), ac.pal("steel"), ac.pal("motor_black")
    parts = []

    # Tool flange with locating spigot.
    parts.append(ac.cylinder("flange", mm(P["flange_r"])[0], mm(P["flange_t"])[0],
                             loc=mm(0, 0, -P["flange_t"] / 2), mat=green, bevel=0.0008))
    parts.append(ac.cylinder("spigot", 0.010, 0.003, loc=mm(0, 0, 1.5), mat=green, bevel=0.0004))

    # Spine: vertical plate behind the cradle, tapering up into the flange.
    spine = [(-22, -3), (22, -3), (40, -32), (45, -150), (-45, -150), (-40, -32)]
    parts.append(ac.plate("spine", mmp(spine), mm(8)[0], z=mm(-16)[0], mat=green,
                          bevel=0.0012, plane="YZ"))

    # Top plate with the V mouth and the throat slot.
    t0, t1, th = P["throat_x0"], P["throat_x1"], P["throat_half"]
    mx, mh = P["mouth_x"], P["mouth_half"]
    slot_end = ac.arc(t0 + th, 0, th, -90, -270, 10)     # round end of the throat
    outline = ([(-16, -42), (96, -42), (mx, -24), (mx, -mh), (t1, -th)]
               + slot_end + [(t1, th), (mx, mh), (mx, 24), (96, 42), (-16, 42)])
    z0 = P["plate_top"] - P["plate_t"]
    parts.append(ac.plate("top_plate", mmp(outline), mm(P["plate_t"])[0], z=mm(z0)[0],
                          mat=bone, bevel=0.0007))

    # Fixed steel bypass edge around the throat.
    insert = ([(50, -12), (82, -12), (82, -th), (t1, -th)] + slot_end
              + [(t1, th), (82, th), (82, 12), (50, 12)])
    parts.append(ac.plate("fixed_edge", mmp(insert), 0.001, z=mm(P["plate_top"])[0],
                          mat=steel, bevel=0.0002))

    # Blade pivot pin.
    px, py = P["pivot"]
    parts.append(ac.cylinder("pivot_pin", 0.0016, 0.011, loc=mm(px, py, P["plate_top"] + 3),
                             mat=steel, verts=24))

    # Servo 1 (MG996R), mounted upside down with the horn at plate level.
    parts.append(ac.box("servo1", mm(40.7, 19.7, 37), loc=mm(26.3, 29.8, P["plate_top"] + 4 + 18.5),
                        mat=black, bevel=0.0012))
    parts.append(ac.box("servo1_tabs", mm(54, 19.7, 2.5), loc=mm(26.3, 29.8, P["plate_top"] + 4 + 9),
                        mat=black, bevel=0.0005))
    parts.append(ac.cylinder("servo1_horn", 0.0045, 0.003, loc=mm(16, 29.8, P["plate_top"] + 2.5),
                             mat=ac.pal("pla_bone"), verts=24))
    horn_tip = (16 + 15 * math.cos(math.radians(-80)), 29.8 + 15 * math.sin(math.radians(-80)))
    pin = (30, 6)
    dx, dy = pin[0] - horn_tip[0], pin[1] - horn_tip[1]
    length = math.hypot(dx, dy)
    parts.append(ac.cylinder("link_rod", 0.0011, length / 1000,
                             loc=mm((horn_tip[0] + pin[0]) / 2, (horn_tip[1] + pin[1]) / 2, P["plate_top"] + 2.5),
                             rot=(0, math.radians(90), math.atan2(dy, dx)), mat=steel, verts=12))

    # Cradle side walls with a rounded front corner.
    wall = ([(-8, -140), (100, -140), (100, -110)] + ac.arc(88, -110, 12, 0, 90, 8)[1:]
            + [(-8, P["wall_top"])])
    for side, y in (("right", P["cradle_half"]), ("left", -P["cradle_half"] - P["wall_t"])):
        parts.append(ac.plate(f"wall_{side}", mmp(wall), mm(P["wall_t"])[0], z=mm(y)[0],
                              mat=bone, bevel=0.0008, plane="XZ"))

    # Hinge barrel and trap-door servo behind the spine.
    hx, hz = P["hinge"]
    parts.append(ac.cylinder("hinge_barrel", 0.0025, mm(2 * P["cradle_half"])[0],
                             loc=mm(hx, 0, hz), rot=(math.radians(90), 0, 0), mat=steel, verts=24))
    parts.append(ac.box("servo2", mm(19.7, 40.7, 37), loc=mm(-26, 0, -128), mat=black, bevel=0.0012))

    # IR break-beam pair either side of the throat, under the plate.
    for y in (9, -9):
        parts.append(ac.box(f"ir_{'tx' if y > 0 else 'rx'}", mm(6, 5, 7),
                            loc=mm(70, y, P["plate_top"] - P["plate_t"] - 3.5),
                            mat=ac.pal("switch"), bevel=0.0006))

    # Arm RGB-D camera on a side bracket (see P["camera"]).
    cx, cy, cz = P["camera"]
    pitch, yaw = math.radians(P["camera_pitch_deg"]), math.radians(P["camera_yaw_deg"])
    parts.append(ac.box("camera_bracket", mm(30, 26, 10), loc=mm(cx - 4, (cy + -45) / 2 - 2, cz - 26),
                        mat=green, bevel=0.0008))
    cam = ac.box("arm_camera", mm(23, 42, 42), loc=mm(cx, cy, cz), rot=(0, pitch, yaw),
                 mat=black, bevel=0.0025, segments=4)
    parts.append(cam)
    fwd = (math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch))
    side = (-math.sin(yaw), math.cos(yaw), 0.0)
    for i, d in enumerate((-12, 0, 12)):
        lp = [c + 11.6 * f + d * s for c, f, s in zip((cx, cy, cz), fwd, side)]
        parts.append(ac.cylinder(f"lens_{i}", 0.0042 if d else 0.0032, 0.0014, loc=mm(*lp),
                                 rot=(0, math.radians(90) + pitch, yaw), mat=ac.pal("lens"), verts=32))
    return parts


def blade_parts():
    """Blade and carrier in the pivot frame, blade pointing along +x (open
    pose is applied by the joint origin)."""
    steel, bone = ac.pal("blade"), ac.pal("pla_bone")
    hub = ac.arc(0, 0, 5, 90, 270, 10)
    blade = hub + [(10, -4), (18, -2.6), (24, -2.6), (28.5, -4.8), (30.5, -3), (30.5, 1.5),
                   (24, 4), (12, 5)]
    z = P["plate_top"] + 1          # rides on the fixed edge
    b = ac.plate("blade", mmp(blade), 0.0008, z=0, mat=steel, bevel=0.0001)
    lever = ac.arc(0, 0, 6, 70, 290, 10) + [(-19.5 - 3, -3.1 - 3), (-19.5 + 2, -3.1 - 4),
                                             (6, -3), (6, 3)]
    c = ac.plate("blade_carrier", mmp(lever), 0.003, z=0.0008, mat=bone, bevel=0.0005)
    for o in (b, c):
        o.location.z = 0     # geometry already relative to the pivot plane
    return [b, c], z


def door_parts():
    """Trap door (cradle floor), foam pad and soft lip, in the hinge frame."""
    bone, foam, tpu = ac.pal("pla_bone"), ac.pal("foam"), ac.pal("tpu_orange")
    hx, hz = P["hinge"]
    half = P["cradle_half"] - 0.5
    floor = ac.box("door", mm(104, 2 * half, P["floor_t"]),
                   loc=mm(-6 + 52 - hx, 0, P["floor_top"] - P["floor_t"] / 2 - hz), mat=bone,
                   bevel=0.0008)
    pad = ac.box("foam_pad", mm(98, 2 * half - 4, P["foam_t"]),
                 loc=mm(-3 + 49 - hx, 0, P["floor_top"] + P["foam_t"] / 2 - hz), mat=foam,
                 bevel=0.002, segments=4)
    # Soft TPU lip: stands 16 mm above the foam so a round fruit cannot roll out of
    # the open front while the arm moves (the fruit enters over it before the cut).
    lip = ac.box("soft_lip", mm(4, 2 * half, P["foam_t"] + 16), loc=mm(100 - hx, 0, P["floor_top"] + (P["foam_t"] + 16) / 2 - hz),
                 mat=tpu, bevel=0.0015, segments=4)
    return [floor, pad, lip]


def main():
    if OUT.exists():
        shutil.rmtree(OUT)

    ac.reset_scene()
    body = body_parts()
    ac.export_all(body, OUT / "visual/cutter_body")

    ac.reset_scene()
    blade, blade_z = blade_parts()
    ac.export_all(blade, OUT / "visual/cutter_blade")

    ac.reset_scene()
    door = door_parts()
    ac.export_all(door, OUT / "visual/cutter_door")

    px, py = P["pivot"]
    cx, cy, cz = P["camera"]
    frames = {
        "units": "metres, radians; frame = tool-flange face, x toward the fruit, z up",
        "blade_joint": {"xyz": mm(px, py, blade_z), "rpy": [0, 0, math.radians(P["blade_open_deg"])],
                        "axis": [0, 0, 1], "limit": [-math.radians(P["blade_stroke_deg"]), 0.0]},
        "door_joint": {"xyz": mm(P["hinge"][0], 0, P["hinge"][1]), "rpy": [0, 0, 0],
                       "axis": [0, -1, 0], "limit": [0.0, math.radians(75)]},   # +angle swings the floor down
        "cut_point": mm((P["throat_x0"] + P["throat_x1"]) / 2, 0, P["plate_top"] - P["plate_t"] / 2),
        "camera_link": {"xyz": mm(*[c + 11.6 * f for c, f in zip(
                            (cx, cy, cz),
                            (math.cos(math.radians(P["camera_yaw_deg"])) * math.cos(math.radians(P["camera_pitch_deg"])),
                             math.sin(math.radians(P["camera_yaw_deg"])) * math.cos(math.radians(P["camera_pitch_deg"])),
                             -math.sin(math.radians(P["camera_pitch_deg"]))))]),
                        "rpy": [0, math.radians(P["camera_pitch_deg"]), math.radians(P["camera_yaw_deg"])]},
        "cradle": {"floor_top_z": mm(P["floor_top"] + P["foam_t"])[0],
                   "inner_width": mm(2 * P["cradle_half"])[0],
                   "gap_plate_to_foam": mm(P["plate_top"] - P["plate_t"] - P["floor_top"] - P["foam_t"])[0]},
        "collision_boxes": {
            "top_plate": {"size": mm(134, 84, 4), "xyz": mm(51, 0, -50)},
            "spine": {"size": mm(8, 90, 147), "xyz": mm(-12, 0, -76.5)},
            "wall_right": {"size": mm(108, 3, 42), "xyz": mm(46, 43.5, -119)},
            "wall_left": {"size": mm(108, 3, 42), "xyz": mm(46, -43.5, -119)},
            "servo1": {"size": mm(40.7, 19.7, 37), "xyz": mm(26.3, 29.8, -25.5)},
            "camera": {"size": mm(30, 46, 42), "xyz": mm(cx, cy, cz)},
            "door": {"size": mm(104, 83, 11), "xyz": mm(-52, 0, 6.5)},
            "lip": {"size": mm(4, 83, 24), "xyz": mm(2, 0, 15)},
        },
    }
    (OUT / "cutter_frames.json").write_text(json.dumps(frames, indent=2))

    # Preview: blade half closed, door shut, with a stand-in fruit in the cradle.
    ac.reset_scene()
    objs = body_parts()
    bl, _ = blade_parts()
    for o in bl:
        o.location = mm(px, py, blade_z)
        o.rotation_euler = (0, 0, math.radians(P["blade_open_deg"] - 25))
    dr = door_parts()
    for o in dr:
        o.location = mm(P["hinge"][0], 0, P["hinge"][1])
    fruit = ac.sphere("stand_in_fruit", 0.031, loc=mm(52, 0, P["floor_top"] + P["foam_t"] + 30),
                      mat=ac.material("stand_in_red", "#C9302C", roughness=0.25, coat=0.4))
    fruit.scale.z = 0.88
    ac.studio_render(objs + bl + dr + [fruit], ac.REPO / "docs/images/cutter_preview.png",
                     view=(1.0, -1.15, 0.55), lens=85, samples=128)
    ac.log("cutter done")


if __name__ == "__main__":
    main()
