"""Build the AgriBOT rover visual meshes (stylised ride-on chassis).

There is no CAD for the ride-on car, so this is a clean stylised body at the
measured size (docs/work-plan.md, section 1): length 1.20 m, width 0.70 m over
the tyres, height 0.80 m, seat 0.50 m, tyres 0.38 m x 0.15 m. Wheelbase 0.72 m
and track 0.55 m are the assumed values from that section. Physics collision
uses URDF primitives, not these meshes.

Frame: base_link sits midway between the axles at axle height (0.19 m above
the ground), x forward, z up.

Output (sim/agribot_description/meshes/rover/visual/):
    rover_body   .glb .obj   shell, deck, bumpers, light bar, e-stop, camera pods
    wheel        .glb .obj   tyre and rim, wheel frame (spin axis = y)
    scout_mast   .glb .obj   fixed mast for the scouting camera
    scout_head   .glb .obj   pan head with its camera (pan axis = z)
    scara_mast   .glb .obj   side mast with pinned height positions for the SCARA
    harvest_crate.glb .obj   crate on the rear deck
    rover_dims.json          key dimensions and mount points for the URDF
docs/images/rover_preview.png

    blender -b --factory-startup --python tools/blender/build_rover.py
"""
import json
import math
import pathlib
import shutil
import sys

import bpy

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import agri_common as ac  # noqa: E402

OUT = ac.REPO / "sim/agribot_description/meshes/rover"

D = dict(
    length=1.20, width=0.70, height=0.80, seat=0.50,
    tyre_d=0.38, tyre_w=0.15, wheelbase=0.72, track=0.55,
)
R = D["tyre_d"] / 2          # axle height above ground
G = -R                       # ground z in base_link
MAST_H = 0.95                # SCARA mast rail length
MAST_LIFT = 0.08             # rail bottom above the ground
MAST_PINS = [0.10 + 0.18 * i for i in range(5)]   # pin holes above the rail bottom


def body_parts():
    white, green, dark = ac.pal("rover_body"), ac.pal("rover_accent"), ac.pal("rover_dark")
    parts = []
    L, W = D["length"], D["width"]
    body_w = D["track"] - 0.04          # body sits between the tyres
    deck_z = D["seat"] + G              # deck top level (seat height)

    # Lower tub: rounded box from just above the ground to the deck.
    tub_h = deck_z - (G + 0.12)
    tub = ac.box("tub", (L - 0.06, body_w, tub_h), loc=(0, 0, G + 0.12 + tub_h / 2),
                 mat=white, bevel=0.06, segments=6)
    parts.append(tub)

    # Wheel arches: cut half-cylinders out of the tub at each axle.
    for x in (D["wheelbase"] / 2, -D["wheelbase"] / 2):
        cut = ac.cylinder("arch_cut", R + 0.03, W + 0.2, loc=(x, 0, 0),
                          rot=(math.radians(90), 0, 0), verts=64)
        ac.boolean(tub, cut)
        ac.shade(tub, 30)

    # Hood: sloped front cover rising toward the dash.
    hood_profile = [(0.60, deck_z - 0.02), (0.60, deck_z + 0.02), (0.22, deck_z + 0.17),
                    (0.10, deck_z + 0.17), (0.10, deck_z - 0.02)]
    hood = ac.plate("hood", hood_profile, body_w - 0.04, z=-(body_w - 0.04) / 2,
                    mat=green, bevel=0.025, segments=5, plane="XZ")
    parts.append(hood)

    # Curved fenders following each tyre (accent colour).
    for x in (D["wheelbase"] / 2, -D["wheelbase"] / 2):
        for side in (1, -1):
            outer = ac.arc(x, 0, R + 0.055, 15, 165, 24)
            inner = ac.arc(x, 0, R + 0.030, 165, 15, 24)
            y0 = side * D["track"] / 2 - (D["tyre_w"] + 0.03) / 2
            parts.append(ac.plate("fender", outer + inner, D["tyre_w"] + 0.03, z=y0,
                                  mat=green, bevel=0.006, segments=3, plane="XZ"))

    # Flat equipment deck where the seat was (the SCARA mast and crate mount here).
    parts.append(ac.box("deck", (0.62, body_w - 0.02, 0.025), loc=(-0.24, 0, deck_z + 0.0125),
                        mat=dark, bevel=0.006))

    # Bumpers front and rear.
    for x, name in ((L / 2 - 0.01, "bumper_front"), (-L / 2 + 0.01, "bumper_rear")):
        parts.append(ac.box(name, (0.06, body_w + 0.06, 0.07), loc=(x, 0, G + 0.24),
                            mat=dark, bevel=0.02, segments=4))

    # Roll hoop behind the dash with an LED light bar.
    for side in (1, -1):
        parts.append(ac.cylinder("hoop_post", 0.014, 0.30,
                                 loc=(0.05, side * (body_w / 2 - 0.03), deck_z + 0.17 + 0.15),
                                 mat=dark, verts=24))
    parts.append(ac.cylinder("hoop_bar", 0.014, body_w - 0.06, loc=(0.05, 0, deck_z + 0.47),
                             rot=(math.radians(90), 0, 0), mat=dark, verts=24))
    parts.append(ac.box("light_bar", (0.05, body_w - 0.16, 0.035), loc=(0.07, 0, deck_z + 0.50),
                        mat=dark, bevel=0.008))
    parts.append(ac.box("light_lens", (0.006, body_w - 0.18, 0.02), loc=(0.096, 0, deck_z + 0.50),
                        mat=ac.material("led_white", "#FFF7E0", roughness=0.1, emission="#FFF2CC")))

    # Front and rear camera pods.
    front_cam = (0.58, 0, deck_z + 0.05)
    rear_cam = (-0.585, 0, deck_z - 0.02)
    for name, loc, yaw in (("front_cam", front_cam, 0), ("rear_cam", rear_cam, math.pi)):
        parts.append(ac.box(name, (0.04, 0.10, 0.045), loc=loc, rot=(0, 0, yaw), mat=dark,
                            bevel=0.008))
        lx = loc[0] + math.cos(yaw) * 0.021
        parts.append(ac.cylinder(name + "_lens", 0.012, 0.004, loc=(lx, 0, loc[2]),
                                 rot=(0, math.radians(90), yaw), mat=ac.pal("lens"), verts=32))

    # E-stop on the dash.
    parts.append(ac.cylinder("estop_base", 0.025, 0.02, loc=(0.16, -0.18, deck_z + 0.18),
                             mat=ac.material("estop_yellow", "#F2C230", roughness=0.4), verts=32))
    parts.append(ac.cylinder("estop_button", 0.019, 0.02, loc=(0.16, -0.18, deck_z + 0.20),
                             mat=ac.material("estop_red", "#C62828", roughness=0.35), verts=32,
                             bevel=0.004))

    # Side skirt stripe (accent).
    for side in (1, -1):
        parts.append(ac.box("stripe", (L - 0.40, 0.004, 0.03),
                            loc=(-0.02, side * (body_w / 2 + 0.001), G + 0.30), mat=green,
                            bevel=0.001))
    dims = {"deck_z": deck_z, "front_camera": front_cam, "rear_camera": rear_cam,
            "body_width": body_w}
    return parts, dims


def wheel_parts():
    tyre_m, rim_m = ac.pal("tyre"), ac.pal("rim")
    w, r = D["tyre_w"], R
    bpy.ops.mesh.primitive_torus_add(major_radius=r - 0.045, minor_radius=0.045,
                                     major_segments=72, minor_segments=24)
    tyre = bpy.context.active_object
    tyre.scale = (1, 1, w / 0.09)
    ac.apply_transform(tyre)
    tyre.rotation_euler = (math.radians(90), 0, 0)
    ac.apply_transform(tyre)
    tyre.name = "tyre"
    ac.assign(tyre, tyre_m)
    ac.shade(tyre, None)
    # Tread blocks around the circumference.
    parts = [tyre]
    for i in range(36):
        a = 2 * math.pi * i / 36
        for s in (-1, 1):
            blk = ac.box("tread", (0.03, w * 0.36, 0.012),
                         loc=((r - 0.003) * math.cos(a), s * w * 0.2, (r - 0.003) * math.sin(a)),
                         rot=(0, -a + math.pi / 2, 0), mat=tyre_m, bevel=0.003)
            blk.rotation_euler = (0, math.pi / 2 - a + (0.12 if s > 0 else -0.12), 0)
            parts.append(blk)
    tread = ac.join(parts[1:], "tread")
    rim = ac.cylinder("rim", r - 0.075, w * 0.7, rot=(math.radians(90), 0, 0), mat=rim_m,
                      bevel=0.008, verts=64)
    hub = ac.cylinder("hub", 0.035, w * 0.78, rot=(math.radians(90), 0, 0),
                      mat=ac.pal("rover_accent"), bevel=0.006, verts=32)
    spokes = []
    for i in range(5):
        a = 2 * math.pi * i / 5
        spokes.append(ac.box("spoke", (0.028, w * 0.72, r - 0.11),
                             loc=(math.sin(a) * (r - 0.11) / 2, 0, math.cos(a) * (r - 0.11) / 2),
                             rot=(0, a, 0), mat=rim_m, bevel=0.005))
    return [tyre, tread, rim, hub] + spokes


def mast_parts(dims):
    dark, green = ac.pal("rover_dark"), ac.pal("rover_accent")
    # Scouting camera mast at the rear-left corner.
    mast = [ac.cylinder("scout_pole", 0.018, 0.50, loc=(0, 0, 0.25), mat=dark, verts=32),
            ac.cylinder("scout_foot", 0.05, 0.012, loc=(0, 0, 0.006), mat=dark, verts=32)]
    head = [ac.cylinder("pan_base", 0.035, 0.03, loc=(0, 0, 0.015), mat=green, verts=48, bevel=0.004),
            ac.box("scout_cam", (0.05, 0.09, 0.05), loc=(0.01, 0, 0.06), mat=dark, bevel=0.01),
            ac.cylinder("scout_lens", 0.014, 0.006, loc=(0.037, 0, 0.06),
                        rot=(0, math.radians(90), 0), mat=ac.pal("lens"), verts=32)]
    # SCARA side mast: vertical 80 x 40 extrusion bolted to the body side with two
    # brackets, pin holes every 0.18 m. Frame origin = bottom of the rail.
    rail = ac.box("rail", (0.08, 0.04, MAST_H), loc=(0, 0, MAST_H / 2), mat=ac.pal("steel_dark"),
                  bevel=0.003)
    for z in MAST_PINS:
        hole = ac.cylinder("pin_hole", 0.007, 0.06, loc=(0, 0, z),
                           rot=(math.radians(90), 0, 0), verts=24)
        ac.boolean(rail, hole)
    ac.shade(rail, 30)
    cap = ac.box("mast_cap", (0.09, 0.05, 0.012), loc=(0, 0, MAST_H + 0.006), mat=dark, bevel=0.003)
    brackets = [ac.box("mast_bracket", (0.10, 0.05, 0.03), loc=(0, 0.045, z), mat=dark, bevel=0.004)
                for z in (0.10, 0.30)]
    return mast, head, [rail, cap] + brackets


def crate_parts():
    blue = ac.material("crate_blue", "#2D6FA3", roughness=0.55)
    shell = ac.box("crate", (0.40, 0.30, 0.20), loc=(0, 0, 0.10), mat=blue, bevel=0.008)
    inner = ac.box("crate_in", (0.384, 0.284, 0.20), loc=(0, 0, 0.11))
    ac.boolean(shell, inner)
    for i in range(6):
        slot = ac.box("slot", (0.04, 0.4, 0.07), loc=(-0.13 + 0.052 * i, 0, 0.12))
        ac.boolean(shell, slot)
    ac.shade(shell, 30)
    return [shell]


def main():
    if OUT.exists():
        shutil.rmtree(OUT)

    ac.reset_scene()
    body, dims = body_parts()
    ac.export_all(body, OUT / "visual/rover_body")

    ac.reset_scene()
    ac.export_all(wheel_parts(), OUT / "visual/wheel")

    ac.reset_scene()
    mast, head, scara_mast = mast_parts(dims)
    ac.export_all(mast, OUT / "visual/scout_mast")
    ac.reset_scene()
    _, head, _ = mast_parts(dims)
    ac.export_all(head, OUT / "visual/scout_head")
    ac.reset_scene()
    _, _, scara_mast = mast_parts(dims)
    ac.export_all(scara_mast, OUT / "visual/scara_mast")

    ac.reset_scene()
    ac.export_all(crate_parts(), OUT / "visual/harvest_crate")

    deck = dims["deck_z"]
    out = {
        "units": "metres; base_link midway between axles at axle height, x forward, z up",
        "measured": {k: D[k] for k in ("length", "width", "height", "seat", "tyre_d", "tyre_w")},
        "assumed": {"wheelbase": D["wheelbase"], "track": D["track"], "steer_limit_deg": 25},
        "wheel_radius": R, "deck_z": deck, "ground_z": G,
        "mounts": {
            "front_camera": dims["front_camera"],
            "rear_camera": dims["rear_camera"],
            "scout_mast": [-0.50, 0.20, deck + 0.025],
            "scara_mast": [-0.12, -(dims["body_width"] / 2 + 0.045), G + MAST_LIFT],
            "harvest_crate": [-0.36, 0.02, deck + 0.025],
            "imu": [0.0, 0.0, deck - 0.05],
        },
        "scara_mast_pin_heights_above_ground": [round(MAST_LIFT + z, 3) for z in MAST_PINS],
        "collision": {
            "body": {"size": [D["length"] - 0.06, dims["body_width"], deck - (G + 0.12) + 0.17],
                     "xyz": [0, 0, (G + 0.12 + deck + 0.17) / 2]},
        },
    }
    (OUT / "rover_dims.json").write_text(json.dumps(out, indent=2))

    # Preview: assembled rover (no arm), three-quarter front view.
    ac.reset_scene()
    objs, _ = body_parts()
    for x, y, steer in ((D["wheelbase"] / 2, D["track"] / 2, 0.25), (D["wheelbase"] / 2, -D["track"] / 2, 0.25),
                        (-D["wheelbase"] / 2, D["track"] / 2, 0), (-D["wheelbase"] / 2, -D["track"] / 2, 0)):
        for o in wheel_parts():
            o.location = (x, y, 0)
            o.rotation_euler = (0, 0, steer)
            objs.append(o)
    m, h, s = mast_parts(dims)
    for o in m:
        o.location = out["mounts"]["scout_mast"]
    for o in h:
        o.location = (out["mounts"]["scout_mast"][0], out["mounts"]["scout_mast"][1],
                      out["mounts"]["scout_mast"][2] + 0.50)
        o.rotation_euler = (0, 0, math.radians(-35))
    for o in s:
        o.location = out["mounts"]["scara_mast"]
    for o in crate_parts():
        o.location = out["mounts"]["harvest_crate"]
        objs.append(o)
    objs += m + h + s
    ac.studio_render(objs, ac.REPO / "docs/images/rover_preview.png",
                     view=(1.0, -0.95, 0.5), lens=55, samples=96)
    ac.log("rover done")


if __name__ == "__main__":
    main()
