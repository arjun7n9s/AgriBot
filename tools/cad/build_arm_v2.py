"""Build the CAD for SCARA arm v2 (new Arm 1, Arm 2, J2/J3 joints) and the tomato cutter.

Run with the CAD venv (build123d), from the repo root:

    .venv-cad/Scripts/python tools/cad/build_arm_v2.py

Writes cad/arm-v2/:
    agribot_arm_v2.step     whole arm in the stock STEP frame (overlays the stock base)
    end_effector.step       the tool alone, tool frame
    step/<part>.step        each made part, in its own frame
    stl/<part>.stl          printed parts
    dxf/<part>.dxf          laser/waterjet profiles of the plates
    parts.csv               part list with material, process, quantity and mass
    preview/*.glb           meshes for the Blender renders (tools/cad/render_arm_v2.py)

Design notes, numbers and sources: docs/arm-v2-design.md. Shared numbers: arm_v2_geometry.py.
"""
import csv
import math
import sys
from pathlib import Path

from build123d import (Align, Axis, BuildSketch, Circle, Color, Compound, Cylinder, Box, Location,
                       Locations, Plane, Polygon, Pos, Rectangle, Rot, SlotCenterToCenter, export_gltf,
                       export_step, export_stl, extrude, make_hull, Part, ExportDXF, Unit, section)

sys.path.insert(0, str(Path(__file__).parent))
import arm_v2_geometry as G  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "cad" / "arm-v2"

DENSITY = {"6061": 2.70, "SS": 7.95, "steel": 7.85, "PETG": 1.27 * 0.55, "TPU": 1.21 * 0.5,
           "foam": 0.05, "bought": None}
COLOR = {"6061": Color(0.72, 0.74, 0.77), "SS": Color(0.82, 0.83, 0.85), "steel": Color(0.55, 0.57, 0.6),
         "PETG": Color(0.15, 0.45, 0.25), "TPU": Color(0.1, 0.1, 0.1), "foam": Color(0.9, 0.85, 0.6),
         "bought": Color(0.2, 0.2, 0.22), "belt": Color(0.05, 0.05, 0.05), "brass": Color(0.8, 0.65, 0.3)}


# ============================================================================ helpers
def cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def box(x0, x1, y0, y1, z0, z1):
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN))


def ring(r_in, r_out, z0, z1, x=0.0, y=0.0):
    return cyl(r_out, z0, z1, x, y) - cyl(r_in, z0 - 1, z1 + 1, x, y)


def hull_face(circles, polys=()):
    """2D hull of circles [(x, y, r)] and point lists."""
    with BuildSketch() as s:
        for x, y, r in circles:
            with Locations((x, y)):
                Circle(r)
        for pts in polys:
            Polygon(*pts, align=None)
        make_hull()
    return s.sketch


def plate(face, z0, z1, holes=(), slots=()):
    """Extrude a 2D face between z0 and z1 and cut holes [(x, y, d)] and slots [(x0, y0, x1, y1, w)]."""
    p = Pos(0, 0, z0) * extrude(face, z1 - z0)
    for x, y, d in holes:
        p -= cyl(d / 2, z0 - 1, z1 + 1, x, y)
    for x0, y0, x1, y1, w in slots:
        ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
        ln = math.hypot(x1 - x0, y1 - y0)
        with BuildSketch() as s:
            SlotCenterToCenter(ln, w)
        p -= Pos((x0 + x1) / 2, (y0 + y1) / 2, z0 - 1) * Rot(0, 0, ang) * extrude(s.sketch, z1 - z0 + 2)
    return p


def polar(n, r, start=0.0):
    return [(r * math.cos(math.radians(start + i * 360 / n)), r * math.sin(math.radians(start + i * 360 / n)))
            for i in range(n)]


def gt2_profile(teeth):
    """Approximate GT2 (2 mm) pulley profile: OD = PD - 0.508, 0.75 mm deep round-bottom grooves.
    Print a test ring and check the belt seats before printing the full pulley."""
    r_od = G.gt2_pitch_radius(teeth) - 0.254
    rg, depth, w_top = 0.555, 0.75, 0.68
    rc = r_od - depth + rg
    pts = []
    step = 2 * math.pi / teeth
    for i in range(teeth):
        a0 = i * step

        def at(r, t):
            return (r * math.cos(a0) - t * math.sin(a0), r * math.sin(a0) + t * math.cos(a0))
        pts.append(at(r_od, -w_top))
        for k in range(0, 9):                            # round bottom, centre (rc, 0)
            phi = math.pi * k / 8
            pts.append(at(rc - rg * math.sin(phi), -rg * math.cos(phi)))
        pts.append(at(r_od, w_top))
        # land up to the next groove
        for k in range(1, 3):
            a = a0 + step * k / 3
            pts.append((r_od * math.cos(a), r_od * math.sin(a)))
    return Polygon(*pts, align=None)


def gt2_pulley(teeth, z_teeth_mid, hub=None, bore=0.0, flange=0.9, belt_w=6.0):
    """Toothed band of belt_w centred at z_teeth_mid, with flanges; hub = (r, z0, z1) list."""
    z0, z1 = z_teeth_mid - belt_w / 2, z_teeth_mid + belt_w / 2
    p = Pos(0, 0, z0) * extrude(gt2_profile(teeth), belt_w)
    r_f = G.gt2_pitch_radius(teeth) + 1.0
    p += cyl(r_f, z0 - flange, z0) + cyl(r_f, z1, z1 + flange)
    for r, a, b in (hub or []):
        p += cyl(r, a, b)
    if bore:
        p -= cyl(bore / 2, z0 - 50, z1 + 50)
    return p


def bearing(d, D, z0, z1, x=0.0, y=0.0):
    return ring(d / 2, D / 2, z0, z1, x, y)


def thrust_bearing(d, D, z0, z1):
    return ring(d / 2, D / 2, z0, z1)


# ============================================================================ Arm 1 (arm1 frame)
A1_SPACERS = [(88.0, 32.0), (88.0, -32.0), (160.0, 32.0), (160.0, -32.0), (205.0, 36.0), (205.0, -36.0),
              (266.0, 0.0)]
IDLER1 = (100.0, 25.3)       # belt 1 tensioner, slot along y
IDLER2 = (185.0, 23.5)       # belt 2 tensioner
ROOT_M4 = [(68.0, 36.0), (68.0, -36.0), (68.0, 10.0), (68.0, -10.0)]


def arm1_outline():
    x0 = G.CARRIAGE_FACE_X + 0.5
    return hull_face([(160.0, 32.0, 8.5), (160.0, -32.0, 8.5), (G.L1, 0.0, 44.0)],
                     polys=[[(x0, -40), (x0 + 10, -40), (x0 + 10, 40), (x0, 40)]])


def arm1_bottom_plate():
    holes = [(x, y, 4.5) for x, y in A1_SPACERS] + [(x, y, 4.5) for x, y in ROOT_M4]
    holes += [(G.IDLER_X, 0.0, 30.2)] + [(G.IDLER_X + x, y, 3.4) for x, y in polar(3, 18.0, 90)]
    holes += [(G.L1, 0.0, 54.0)] + [(G.L1 + x, y, 4.5) for x, y in polar(6, 34.0, 30)]
    slots = [(IDLER2[0], IDLER2[1] - 3, IDLER2[0], IDLER2[1] + 3, 5.3)]
    return plate(arm1_outline(), *G.A1_BOT, holes=holes, slots=slots)


def arm1_top_plate():
    holes = [(x, y, 4.5) for x, y in A1_SPACERS] + [(x, y, 4.5) for x, y in ROOT_M4[:2]]
    holes += [(G.IDLER_X, 0.0, 20.0), (G.L1, 0.0, 24.0), (112.0, -22.0, 16.0)]
    slots = [(IDLER1[0], IDLER1[1] - 3, IDLER1[0], IDLER1[1] + 3, 5.3)]
    return plate(arm1_outline(), *G.A1_TOP, holes=holes, slots=slots)


def arm1_root_flange():
    """U-block bolted to the carriage face with the stock 4 x M5; belt 1 passes through the U."""
    x0, x1 = G.CARRIAGE_FACE_X, G.CARRIAGE_FACE_X + 12.0
    z0, z1 = G.A1_BOT[1], G.A1_TOP[0]
    p = box(x0, x1, -40, 40, z0, z1) - box(x0 - 1, x1 + 1, -20, 20, 245.0, z1 + 1)
    for y in (-G.CARRIAGE_M5_Y, G.CARRIAGE_M5_Y):
        for z in G.CARRIAGE_M5_Z:
            p -= Pos(x0 - 1, y, z) * Rot(0, 90, 0) * Cylinder(2.75, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for x, y in ROOT_M4:
        p -= cyl(1.65, z0 - 1, z0 + 10, x, y)          # M4 tapped from below
    for x, y in ROOT_M4[:2]:
        p -= cyl(1.65, z1 - 10, z1 + 1, x, y)          # M4 tapped from above
    return p


def j2_hub():
    """Turned 6061. Carries the 6806 and both 51107 housing washers; bolted under the bottom plate."""
    z0, z1 = G.J2_HUB
    p = cyl(39.0, z0, z1) - cyl(21.0, z0 - 1, G.J2_RADIAL[1]) - cyl(19.0, G.J2_RADIAL[1] - 0.1, z1 + 1)
    for x, y in polar(6, 34.0, 30):
        p -= cyl(1.65, z1 - 8, z1 + 1, x, y)
    return Pos(G.L1, 0, 0) * p


def idler_cup():
    """Turned 6061 cup under the bottom plate: two 608 for the stock 80T/23T compound pulley axle."""
    z_f0 = G.A1_BOT[0] - 3.0
    p = cyl(20.0, z_f0, G.A1_BOT[0]) + cyl(15.0, G.A1_BOT[0], 244.6)
    p -= cyl(11.0, 230.0, 244.7) + cyl(6.0, z_f0 - 1, 231)
    for x, y in polar(3, 18.0, 90):
        p -= cyl(1.65, z_f0 - 1, G.A1_BOT[0] + 0.01, x, y)
    return Pos(G.IDLER_X, 0, 0) * p


def compound_pulley_stock():
    """Stock printed part 'GT2 Pulley - 23 - 80 teeth' (reused, not rebuilt); envelope only."""
    z0, z1 = G.COMPOUND_PULLEY_Z
    p = cyl(8.7, z0, 255.6) + cyl(26.8, 255.6, z1) - cyl(4.0, z0 - 1, z1 + 1)
    return Pos(G.IDLER_X, 0, 0) * p


def toothless_idler(x, y, zmid):
    """On-hand 'GT2 toothless driven pulley, 5 mm bore' envelope (assumed 18 mm flanges)."""
    p = cyl(6.5, zmid - 3.5, zmid + 3.5) + cyl(9.0, zmid - 4.3, zmid - 3.5) + cyl(9.0, zmid + 3.5, zmid + 4.3)
    return Pos(x, y, 0) * (p - cyl(2.5, zmid - 10, zmid + 10))


def spacers(points, z0, z1, od=8.0):
    out = None
    for x, y in points:
        s = ring(2.2, od / 2, z0, z1, x, y)
        out = s if out is None else out + s
    return out


# ============================================================================ J2 joint (arm1 frame at J2)
def j2_spindle():
    """Turned 6061. Flange bolts to the Arm 2 top plate; hollow for cables."""
    f0, f1 = G.J2_FLANGE
    p = cyl(39.0, f0, f1) + cyl(17.5, f1, G.J2_THRUST_LO[1]) + cyl(15.0, G.J2_THRUST_LO[1], G.J2_PULLEY[0])
    p -= cyl(8.0, f0 - 1, G.J2_PULLEY[0] + 1)
    for x, y in polar(6, 32.0, 0):
        p -= cyl(2.25, f0 - 1, f1 + 1, x, y)
    for x, y in polar(4, 11.0, 45):
        p -= cyl(1.65, G.J2_PULLEY[0] - 10, G.J2_PULLEY[0] + 1, x, y)
    return Pos(G.L1, 0, 0) * p


def j2_pulley():
    """Printed PETG, 92T. Spigot centres the upper 51107; 4 x M4 into the spindle top."""
    z0 = G.J2_PULLEY[0]
    p = gt2_pulley(G.J2_PULLEY_TEETH, G.BELT2_Z, hub=[(24.0, z0, G.BELT2_Z - 3.0)])
    p += ring(15.1, 17.4, G.J2_THRUST_HI[0] + 0.5, z0)
    p -= cyl(8.0, z0 - 20, z0 + 30)
    for x, y in polar(4, 11.0, 45):
        p -= cyl(2.25, z0 - 1, z0 + 30, x, y)
    return Pos(G.L1, 0, 0) * p


# ============================================================================ Arm 2 (arm2 frame)
A2_SPACERS = [(-36.0, 0.0), (100.0, 32.0), (100.0, -32.0), (160.0, 30.0), (160.0, -30.0)]


def arm2_outline():
    return hull_face([(0.0, 0.0, 42.0), (G.L2, 0.0, 34.0)])


def arm2_top_plate():
    holes = [(x, y, 4.5) for x, y in A2_SPACERS] + [(x, y, 4.5) for x, y in polar(6, 32.0, 0)]
    holes += [(0.0, 0.0, 16.0), (G.L2, 0.0, 16.0)]
    return plate(arm2_outline(), *G.A2_TOP, holes=holes)


def arm2_bottom_plate():
    m = G.J3_MOTOR_X
    holes = [(x, y, 4.5) for x, y in A2_SPACERS] + [(G.L2, 0.0, 50.0)]
    holes += [(G.L2 + x, y, 4.5) for x, y in polar(6, 28.0, 0)]
    holes += [(30.0, 18.0, 10.0)]                                           # motor cable up into the arm
    slots = [(m - 4, 0, m + 4, 0, 23.0)]
    h = G.NEMA17["hole_sq"] / 2
    slots += [(m + sx * h - 4, sy * h, m + sx * h + 4, sy * h, 3.4) for sx in (-1, 1) for sy in (-1, 1)]
    return plate(arm2_outline(), *G.A2_BOT, holes=holes, slots=slots)


def j3_hub():
    z0, z1 = G.J3_HUB
    p = cyl(32.0, z0, z1)
    p -= cyl(21.0, z0 - 1, G.J3_RADIAL_LO[1])                 # 6806 seat
    p -= cyl(19.5, G.J3_RADIAL_LO[1] - 0.1, 156.0)            # web: bears on the 6806 outer ring only
    p -= cyl(20.75, 156.0, G.J3_RADIAL_HI[0])                 # web: bears on the 6807 outer ring only
    p -= cyl(23.5, G.J3_RADIAL_HI[0], z1 + 1)                 # 6807 seat
    for x, y in polar(6, 28.0, 0):
        p -= cyl(1.65, z1 - 8, z1 + 1, x, y)
    return Pos(G.L2, 0, 0) * p


def j3_spindle():
    """Turned 6061: tool flange (4 x M4 on 48 PCD, 20 mm spigot, 4 mm dowel), hollow for tool cables."""
    f0, f1 = G.J3_FLANGE
    p = cyl(30.0, f0, f1) + cyl(10.0, f0 - 3.0, f0) + cyl(16.5, f1, G.J3_HUB[0])
    p += cyl(15.0, G.J3_HUB[0], G.J3_RADIAL_LO[1]) + cyl(17.5, G.J3_RADIAL_LO[1], 171.0)
    p -= cyl(7.0, f0 - 5, 172)
    for x, y in polar(4, 24.0, 0):
        p -= cyl(1.65, f0 - 1, f0 + 6, x, y)
    p -= cyl(2.0, f0 - 1, f0 + 4, 20 * math.cos(math.radians(45)), 20 * math.sin(math.radians(45)))
    for x, y in polar(4, 12.0, 45):
        p -= cyl(1.65, 171 - 8, 172, x, y)
    return Pos(G.L2, 0, 0) * p


def j3_pulley():
    p = gt2_pulley(G.J3_PULLEY_TEETH, G.J3_BELT_Z, hub=[(19.0, G.J3_RADIAL_HI[1], 171.0), (24.0, 171.0, G.J3_BELT_Z - 3)])
    p -= cyl(17.55, G.J3_RADIAL_HI[1] - 1, 171.0) + cyl(7.0, 170, 200)
    for x, y in polar(4, 12.0, 45):
        p -= cyl(2.25, 171, 200, x, y)
    return Pos(G.L2, 0, 0) * p


def nema17(x, y, face_z, down=True, length=None):
    n = G.NEMA17
    ln = length or n["length"]
    s = n["side"] / 2
    if down:
        body = box(x - s, x + s, y - s, y + s, face_z - ln, face_z)
        body += cyl(n["boss_d"] / 2, face_z, face_z + n["boss_h"], x, y)
        body += cyl(n["shaft_d"] / 2, face_z, face_z + n["shaft_len"], x, y)
    else:
        body = box(x - s, x + s, y - s, y + s, face_z, face_z + ln)
        body += cyl(n["shaft_d"] / 2, face_z - n["shaft_len"], face_z, x, y)
    return body


def gt2_20t(x, y, zmid):
    p = gt2_pulley(20, zmid, hub=[(8.0, zmid - 3.9 - 7.0, zmid - 3.9)], bore=5.0)
    return Pos(x, y, 0) * p


# ============================================================================ belts (simple envelopes)
def belt(c1, r1, c2, r2, zmid, w=6.0, t=1.4):
    """Closed belt around two pulleys at pitch radii r1, r2, as a solid band."""
    def band(rr):
        return hull_face([(c1[0], c1[1], rr(r1)), (c2[0], c2[1], rr(r2))])
    outer = Pos(0, 0, zmid - w / 2) * extrude(band(lambda r: r + t / 2), w)
    inner = Pos(0, 0, zmid - w / 2 - 1) * extrude(band(lambda r: max(r - t / 2, 0.5)), w + 2)
    return outer - inner


# ============================================================================ end effector (tool frame)
def v_outline(offset=0.0):
    """Slot + V opening, as a polygon open toward +x (beyond the tool), grown by `offset`."""
    hw = G.SLOT_W / 2 + offset
    xr = G.THROAT_X - offset
    x_v = G.THROAT_X + G.SLOT_LEN
    t = math.tan(math.radians(G.V_HALF_ANGLE))
    x_far = 170.0
    pts = [(x_v, hw), (x_far, hw + (x_far - x_v) * t), (x_far, -hw - (x_far - x_v) * t), (x_v, -hw)]
    with BuildSketch() as s:
        Polygon(*pts, align=None)
        with Locations((G.THROAT_X, 0)):
            Rectangle(2 * G.SLOT_LEN + 0.2, 2 * hw, align=(Align.MIN, Align.CENTER))
        with Locations((G.THROAT_X, 0)):
            Circle(hw)
    return s.sketch, xr


def deck():
    """6061-T6, 3 mm. Bolts under the J3 flange; carries everything."""
    t = math.tan(math.radians(G.V_HALF_ANGLE))
    x_mouth = G.THROAT_X + G.SLOT_LEN + (G.MOUTH_W / 2 - G.SLOT_W / 2) / t
    outline = hull_face([(0.0, 0.0, 36.0), (x_mouth + 4, 31.0, 6.0), (x_mouth + 4, -31.0, 6.0)],
                        polys=[[(10, -62), (46, -62), (46, -53), (10, -53)],
                               [(36, 53), (120, 53), (120, -53), (36, -53)]])
    vcut, _ = v_outline(offset=3.0)
    p = Pos(0, 0, -G.DECK_T) * extrude(outline, G.DECK_T)
    p -= Pos(0, 0, -G.DECK_T - 1) * extrude(vcut, G.DECK_T + 2)
    p -= cyl(10.1, -5, 1)                                                     # flange spigot
    for x, y in polar(4, 24.0, 0):
        p -= cyl(2.25, -5, 1, x, y)
    p -= cyl(2.0, -5, 1, 20 * math.cos(math.radians(45)), 20 * math.sin(math.radians(45)))
    p -= cyl(7.0, -5, 1, *G.SERVO_SHAFT)                                      # cutter servo boss
    p -= cyl(2.25, -5, 1, *G.PIVOT)                                           # blade pivot
    for x, y in VPLATE_SCREWS + BASKET_SCREWS + CAMERA_SCREWS + POST_SCREWS:
        p -= cyl(1.7, -5, 1, x, y)
    for x, y in SERVO_BRACKET_SCREWS:
        p -= cyl(1.7, -5, 1, x, y)
    p -= cyl(4.0, -5, 1, 30.0, 30.0)                                          # tool cables down
    return p


VPLATE_SCREWS = [(70.0, 22.0), (70.0, -22.0), (110.0, 22.0), (110.0, -22.0), (132.0, 33.0), (132.0, -33.0)]
BASKET_SCREWS = [(48.0, 50.5), (48.0, -50.5), (100.0, 50.5), (100.0, -50.5), (41.2, 25.0), (41.2, -25.0)]
CAMERA_SCREWS = [(46.0, 22.0), (52.0, 22.0)]
POST_SCREWS = [(88.0, 38.0), (88.0, -38.0)]
SERVO_BRACKET_SCREWS = [(20.0, -61.0), (36.6, -61.0), (20.0, -2.3), (36.6, -2.3)]


def v_plate():
    """SS304/410, 1.5 mm, laser cut. Its slot edges are the shear edges; the mouth funnels the pedicel."""
    t = math.tan(math.radians(G.V_HALF_ANGLE))
    x_mouth = G.THROAT_X + G.SLOT_LEN + (G.MOUTH_W / 2 - G.SLOT_W / 2) / t
    outline = hull_face([(x_mouth + 4, 31.0, 6.0), (x_mouth + 4, -31.0, 6.0), (62.0, 26.0, 4.0), (62.0, -26.0, 4.0)])
    vcut, _ = v_outline()
    p = extrude(outline, G.VPLATE_T) - Pos(0, 0, -1) * extrude(vcut, G.VPLATE_T + 2)
    for x, y in VPLATE_SCREWS + POST_SCREWS:
        p -= cyl(1.7, -1, 3, x, y)
    p -= cyl(2.25, -1, 3, *G.PIVOT)
    return p


def blade_lever_2d():
    """Blade-lever at angle 0, pivot at the origin (lever frame)."""
    a = math.radians(G.EDGE_SLICE)
    d = (math.cos(a), math.sin(a))
    n = (math.sin(a), -math.cos(a))          # toward the back (-y) side of the edge
    e0 = (G.EDGE_R, 0.0)
    pts = [(e0[0] + s * d[0], e0[1] + s * d[1]) for s in (-12.0, 20.0)]
    back = [(x + 11 * n[0], y + 11 * n[1]) for x, y in pts]
    blade = hull_face([(0, 0, 7.0)], polys=[[pts[0], pts[1], back[1], back[0]]])
    ph = math.radians(G.LEVER_IN_PHASE)
    q = (G.LEVER_IN_R * math.cos(ph), G.LEVER_IN_R * math.sin(ph))
    arm = hull_face([(0, 0, 6.0), (q[0], q[1], 5.0)])
    return blade, arm, q


def blade_lever():
    blade, arm, q = blade_lever_2d()
    z0 = G.VPLATE_T
    p = Pos(0, 0, z0) * extrude(blade, G.LEVER_T) + Pos(0, 0, z0) * extrude(arm, G.LEVER_T)
    p -= cyl(2.05, z0 - 1, z0 + 5) + cyl(1.6, z0 - 1, z0 + 5, *q)
    return p


def lever_at(theta):
    return Pos(G.PIVOT[0], G.PIVOT[1], 0) * Rot(0, 0, theta) * blade_lever()


def link_at(theta):
    alpha, q = G.servo_angle_for_lever(theta)
    h = (G.SERVO_SHAFT[0] + G.HORN_R * math.cos(math.radians(alpha)),
         G.SERVO_SHAFT[1] + G.HORN_R * math.sin(math.radians(alpha)))
    z0 = G.VPLATE_T + G.LEVER_T + 1.0
    face = hull_face([(h[0], h[1], 4.0), (q[0], q[1], 4.0)])
    p = Pos(0, 0, z0) * extrude(face, 2.0)
    p -= cyl(1.6, z0 - 1, z0 + 3, *h) + cyl(1.6, z0 - 1, z0 + 3, *q)
    horn = Pos(0, 0, z0 - 2.5) * extrude(hull_face([(G.SERVO_SHAFT[0], G.SERVO_SHAFT[1], 5.0), (h[0], h[1], 3.5)]), 2.0)
    horn += cyl(4.5, SERVO_TOP_Z + 5.5, z0 - 2.5, *G.SERVO_SHAFT)
    return p, horn


SERVO_TOP_Z = -G.DECK_T - 0.5     # cutter servo output face, just under the deck


def servo_body(x, y, top_z, yaw=0.0, flip=False):
    """Standard-size servo (MG996R / DS3225), output shaft at (x, y), output face at top_z, body below."""
    s = G.SERVO
    off = s["l"] / 2 - s["shaft_off"]
    b = box(-s["l"] / 2, s["l"] / 2, -s["w"] / 2, s["w"] / 2, -s["h"], 0)
    b += box(-s["tab_l"] / 2, s["tab_l"] / 2, -s["w"] / 2, s["w"] / 2, -s["h"] + s["tab_z"], -s["h"] + s["tab_z"] + s["tab_t"])
    b = Pos(off, 0, 0) * b
    b += cyl(6.0, 0, 3.0) + cyl(3.0, 3.0, 5.5)
    if flip:
        b = Rot(180, 0, 0) * b
    return Pos(x, y, top_z) * Rot(0, 0, yaw) * b


def cutter_servo_bracket():
    """Printed PETG. Hangs the cutter servo under the deck by its tabs."""
    s = G.SERVO
    x, y = G.SERVO_SHAFT
    z_tab = SERVO_TOP_Z - s["h"] + s["tab_z"]
    cy = y + (s["l"] / 2 - s["shaft_off"])
    outer = box(x - 11.35, x + 11.3, cy - 31.5, cy + 31.5, z_tab - 4.0, -G.DECK_T)
    inner = box(x - s["w"] / 2 - 0.3, x + s["w"] / 2 + 0.3, cy - s["l"] / 2 - 0.3, cy + s["l"] / 2 + 0.3, z_tab - 5, 0)
    tabs = box(x - s["w"] / 2 - 0.3, x + s["w"] / 2 + 0.3, cy - s["tab_l"] / 2 - 0.3, cy + s["tab_l"] / 2 + 0.3,
               z_tab, z_tab + s["tab_t"] + 0.3)
    p = outer - inner - tabs
    p -= box(x - 9, x + 9, cy - 26, cy + 26, z_tab + s["tab_t"] + 0.3, -G.DECK_T - 2.5)
    for sx, sy in SERVO_BRACKET_SCREWS:
        p -= cyl(1.25, -G.DECK_T - 10, -G.DECK_T + 1, sx, sy)
    return p


def basket():
    """Printed PETG. Rear wall, two side walls and a top flange; front and floor open."""
    x0, x1 = G.BASKET_IN_X
    w, hw = G.BASKET_WALL, G.BASKET_IN_HALF_W
    zb = G.PAD_TOP_Z - 10.0
    zt = -G.DECK_T
    p = box(x0 - w, x1 + 6, -hw - w, hw + w, zb, zt) - box(x0, x1 + 10, -hw, hw, zb - 1, zt + 1)
    p += box(x0 - w, x1 + 6, hw, hw + 8, zt - 3, zt) + box(x0 - w, x1 + 6, -hw - 8, -hw, zt - 3, zt)
    p += box(x0 - w, x0 + 6, -hw, hw, zt - 3, zt)
    # lightening windows in the side walls (keep the lower 35 mm solid to hold the fruit)
    for xa, xb in ((x0 + 10, x0 + 40), (x0 + 50, x0 + 80)):
        p -= box(xa, xb, -hw - 5, hw + 5, -90.0, -20.0)
    for x, y in BASKET_SCREWS:
        p -= cyl(1.6, zt - 6, zt + 1, x, y)
    # hinge knuckles for the door along +y bottom edge
    for xk in (x0 + 4, x1 - 4):
        p += Pos(xk, G.DOOR_HINGE[0], G.DOOR_HINGE[1]) * Rot(0, 90, 0) * Cylinder(4.0, 8.0)
    p -= Pos(x0 - 10, G.DOOR_HINGE[0], G.DOOR_HINGE[1]) * Rot(0, 90, 0) * Cylinder(1.6, 200, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(x0 - 10, G.DOOR_HINGE[0], G.DOOR_HINGE[1]) * Rot(0, 90, 0) * Cylinder(
        7.5, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))                 # door servo spline
    # fruit sensor window in the rear wall
    p -= box(x0 - w - 1, x0 + 1, -6, 6, -100, -92)
    return p


def door(open_deg=0.0):
    """Printed PETG door plate with a TPU lip and 10 mm foam; hinge along x at +y bottom edge."""
    x0, x1 = G.BASKET_IN_X
    hw = G.BASKET_IN_HALF_W - 0.5
    yh, zh = G.DOOR_HINGE
    plate_top = G.PAD_TOP_Z - 10.0
    d = box(x0 + 0.5, x1 - 0.5, -hw, hw - 4, plate_top - 3.0, plate_top)
    d += box(x0 + 0.5, x1 - 0.5, hw - 6, yh, zh, plate_top)               # web to the hinge
    d += Pos(x0 + 0.5, yh, zh) * Rot(0, 90, 0) * Cylinder(2.9, x1 - x0 - 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    pad = box(x0 + 2, x1 - 7.0, -hw + 2, hw - 6, plate_top, G.PAD_TOP_Z)
    lip = box(x1 - 6.5, x1 - 0.5, -hw, hw - 4, plate_top, G.LIP_TOP_Z)
    spine_hole = Pos(x0 - 5, yh, zh) * Rot(0, 90, 0) * Cylinder(1.7, 200, align=(Align.CENTER, Align.CENTER, Align.MIN))
    d -= spine_hole
    for xk in (x0 + 4, x1 - 4):
        d -= box(xk - 4.5, xk + 4.5, yh - 5, yh + 5, zh - 5, zh + 5)
    rot = Pos(0, yh, zh) * Rot(open_deg, 0, 0) * Pos(0, -yh, -zh)
    return rot * d, rot * pad, rot * lip


def door_servo_bracket():
    """Printed PETG. Holds the door servo behind the rear wall, shaft on the hinge axis."""
    x0 = G.BASKET_IN_X[0] - G.BASKET_WALL
    yh, zh = G.DOOR_HINGE
    s = G.SERVO
    off = s["l"] / 2 - s["shaft_off"]
    zc = zh + off                                       # body centre height
    za, zb = zc - s["tab_l"] / 2 - 2.0, zc + s["tab_l"] / 2 + 2.0
    xb = x0 - 2.0 - s["h"] - 1.5
    p = box(xb, x0, yh - 12.5, yh + 12.5, za, zb)
    p -= box(xb + 1.5, x0 - 2.0 + 0.01, yh - s["w"] / 2 - 0.3, yh + s["w"] / 2 + 0.3, za + 1.5, zb - 1.5)
    p -= box(xb - 1, xb + 2, yh - 7, yh + 7, za + 6, zb - 6)                     # cable exit
    p -= box(x0 - 3, x0 + 1, yh - 7, yh + 7, zh - 7, zh + 7)                     # spline window
    p += box(x0 - 6, x0, yh - 12.5, yh - 6.0, zb, -G.DECK_T)                     # strut up to the deck
    return p


def door_servo():
    x0 = G.BASKET_IN_X[0] - G.BASKET_WALL
    yh, zh = G.DOOR_HINGE
    s = G.SERVO
    # output face toward +x at x0 - 2, shaft on (yh, zh), body long axis vertical (up)
    b = servo_body(0, 0, 0)
    return Pos(x0 - 2.0, yh, zh) * Rot(0, 90, 0) * Rot(0, 0, 180) * b


FORK_PIVOT = (56.0, -G.DECK_T - 4.0)


def feeler_fork(lift=0.0):
    """Printed PETG fork under the deck. The fruit shoulders lift it; its tail presses a micro switch."""
    xp, zp = FORK_PIVOT
    za, zt = -G.DECK_T - 3.0, -G.DECK_T - 0.5                                  # arms 0.5 mm under the deck
    p = box(49.5, 60.0, -20.0, 20.0, za - 4.5, zt)                             # bridge around the pivot
    for y0, y1 in ((9.0, 17.0), (-17.0, -9.0)):
        p += box(60.0, 112.0, y0, y1, za, za + 2.5)                            # arms
    p += box(49.5, 53.5, -22.0, -20.0, -20.0, za - 4.5)                       # tail drop
    p += box(49.5, 53.5, -36.0, -20.0, -20.0, -16.0)                           # tail tab under the switch
    p -= Pos(xp, 0, zp) * Rot(90, 0, 0) * Cylinder(1.7, 60)
    return Pos(xp, 0, zp) * Rot(0, -lift, 0) * Pos(-xp, 0, -zp) * p


def fork_hinge_blocks():
    xp, zp = FORK_PIVOT
    out = None
    for s in (-1, 1):
        b = box(xp - 4, xp + 4, s * 26.5 - 2.5, s * 26.5 + 2.5, zp - 5, -G.DECK_T)
        b -= Pos(xp, 0, zp) * Rot(90, 0, 0) * Cylinder(1.6, 60)
        out = b if out is None else out + b
    return out


def micro_switch(x_act, y, z_top):
    """On-hand roller-lever micro switch (about 20 x 6.5 x 10 body); actuator at x_act, body toward +x."""
    return (box(x_act - 2, x_act + 18, y - 3.25, y + 3.25, z_top - 10, z_top)
            + box(x_act - 1.5, x_act + 1.5, y - 2, y + 2, z_top - 13, z_top - 10))


def ir_posts():
    """Printed posts each side of the throat: 3 mm IR LED one side, phototransistor the other."""
    out = None
    for y in (-38.0, 38.0):
        p = box(84.0, 92.0, y - 3.0, y + 3.0, 0.0, 14.0)
        p -= Pos(88.0, y, 10.0) * Rot(90, 0, 0) * Cylinder(1.6, 10)
        p -= cyl(1.6, 0, 6, 88.0, y)
        out = p if out is None else out + p
    return out


def camera_and_bracket():
    c = G.CAMERA
    x_face, y_c, z_c = 72.0, 5.0, 36.0
    cam = box(-c["d"], 0, -c["w"] / 2, c["w"] / 2, -c["h"] / 2, c["h"] / 2)
    cam = Pos(x_face, y_c, z_c) * Rot(0, c["tilt"], 0) * cam
    br = box(38.0, 56.0, 14.0, 30.0, 0.0, 4.0) + box(38.0, 42.0, 14.0, 30.0, 0.0, 24.0)
    br += Pos(x_face, y_c, z_c) * Rot(0, c["tilt"], 0) * box(-c["d"] - 3.0, -c["d"], 0.0, c["w"] / 2 + 4.0,
                                                              -c["h"] / 2, c["h"] / 2)
    for x, y in CAMERA_SCREWS:
        br -= cyl(1.7, -1, 6, x, y)
    return cam, br


def fruit_sensor():
    x0 = G.BASKET_IN_X[0] - G.BASKET_WALL
    return box(x0 - 6.0, x0 - 2.4, -10, 10, -104, -88)


def flange_stub():
    """Reference only: the J3 flange underside the deck bolts to (tool frame)."""
    return cyl(30.0, 0, 6.0)


# ============================================================================ assembly
PARTS = []   # (id, name, solid in its frame, frame, material, process, qty, note)


def add(pid, name, solid, frame, material, process, note="", qty=1, export=True):
    PARTS.append(dict(id=pid, name=name, solid=solid, frame=frame, material=material, process=process,
                      qty=qty, note=note, export=export))
    return solid


def build_parts(lever_theta=G.LEVER_OPEN, door_open=0.0):
    PARTS.clear()
    # Arm 1
    add("A1-01", "Arm 1 top plate", arm1_top_plate(), "arm1", "6061", "laser/waterjet 4 mm, drill")
    add("A1-02", "Arm 1 bottom plate", arm1_bottom_plate(), "arm1", "6061", "laser/waterjet 4 mm, drill")
    add("A1-03", "Arm 1 root flange (U-block)", arm1_root_flange(), "arm1", "6061", "mill, tap 6 x M4")
    add("A1-04", "J2 hub", j2_hub(), "arm1", "6061", "turn, drill and tap 6 x M4")
    add("A1-05", "Idler cup (2 x 608)", idler_cup(), "arm1", "6061", "turn")
    add("A1-06", "Arm 1 spacer tube 36 mm", spacers(A1_SPACERS, G.A1_BOT[1], G.A1_TOP[0]), "arm1", "6061",
        "8/4.4 tube cut to 36.0 (or print PETG)", qty=len(A1_SPACERS), export=False)
    add("J2-01", "J2 spindle", j2_spindle(), "arm1", "6061", "turn, drill and tap")
    add("J2-02", "J2 pulley 92T", j2_pulley(), "arm1", "PETG", "print, 100% infill near the spigot")
    add("STK-1", "Stock 80T/23T compound pulley (reused)", compound_pulley_stock(), "arm1", "PETG",
        "stock part, already printed", export=False)
    add("BUY-01", "51107 thrust bearing x2 (J2)",
        Pos(G.L1, 0, 0) * (thrust_bearing(35, 52, *G.J2_THRUST_LO) + thrust_bearing(35, 52, *G.J2_THRUST_HI)),
        "arm1", "bought", "on hand", qty=2, export=False)
    add("BUY-02", "6806-ZZ (J2)", Pos(G.L1, 0, 0) * bearing(30, 42, *G.J2_RADIAL), "arm1", "bought", "on hand", export=False)
    add("BUY-03", "608 x2 (idler axle)", Pos(G.IDLER_X, 0, 0) * (bearing(8, 22, 230.0, 237.0) + bearing(8, 22, 237.6, 244.6)),
        "arm1", "bought", "on hand", qty=2, export=False)
    add("BUY-04", "Toothless idler x2 (belt tensioners)", toothless_idler(*IDLER1, G.BELT1_Z) + toothless_idler(*IDLER2, G.BELT2_Z),
        "arm1", "bought", "on hand", qty=2, export=False)
    r20, r80 = G.gt2_pitch_radius(20), G.gt2_pitch_radius(80)
    r23, r92 = G.gt2_pitch_radius(23), G.gt2_pitch_radius(92)
    add("BELT-1", "GT2 400 belt (J2 stage 1)", belt((G.J2_MOTOR_X, 0), r20, (G.IDLER_X, 0), r80, G.BELT1_Z), "arm1",
        "belt", "on hand", export=False)
    add("BELT-2", "GT2 300 belt (J2 stage 2)", belt((G.IDLER_X, 0), r23, (G.L1, 0), r92, G.BELT2_Z), "arm1",
        "belt", "on hand", export=False)
    # Arm 2
    add("A2-01", "Arm 2 top plate", arm2_top_plate(), "arm2", "6061", "laser/waterjet 4 mm, drill")
    add("A2-02", "Arm 2 bottom plate", arm2_bottom_plate(), "arm2", "6061", "laser/waterjet 3 mm, drill")
    add("A2-03", "Arm 2 spacer tube 25 mm", spacers(A2_SPACERS, G.A2_BOT[1], G.A2_TOP[0]), "arm2", "6061",
        "8/4.4 tube cut to 25.0 (or print PETG)", qty=len(A2_SPACERS), export=False)
    add("A2-04", "J3 hub", j3_hub(), "arm2", "6061", "turn, drill and tap 6 x M4")
    add("J3-01", "J3 spindle and tool flange", j3_spindle(), "arm2", "6061", "turn, drill and tap")
    add("J3-02", "J3 pulley 90T", j3_pulley(), "arm2", "PETG", "print")
    add("BUY-05", "6806-ZZ + 6807-ZZ (J3)", Pos(G.L2, 0, 0) * (bearing(30, 42, *G.J3_RADIAL_LO) + bearing(35, 47, *G.J3_RADIAL_HI)),
        "arm2", "bought", "on hand", export=False)
    add("BUY-06", "NEMA17 4.2 kg.cm (J3)", nema17(G.J3_MOTOR_X, 0, G.A2_BOT[0]), "arm2", "bought", "on hand", export=False)
    add("BUY-07", "GT2 20T pulley (J3 motor)", gt2_20t(G.J3_MOTOR_X, 0, G.J3_BELT_Z), "arm2", "bought", "on hand", export=False)
    add("BELT-3", "GT2 400 belt (J3)", belt((G.J3_MOTOR_X, 0), r20, (G.L2, 0), G.gt2_pitch_radius(90), G.J3_BELT_Z), "arm2",
        "belt", "on hand", export=False)
    # End effector
    add("T-01", "Cutter deck", deck(), "tool", "6061", "laser/waterjet 3 mm, drill")
    add("T-02", "V-plate (shear edge)", v_plate(), "tool", "SS", "laser 1.5 mm SS410 (or SS304), deburr slot edges square")
    add("T-03", "Blade-lever", lever_at(lever_theta), "tool", "steel", "laser 2 mm EN42/SS420, harden, grind 25 deg bevel")
    lk, horn = link_at(lever_theta)
    add("T-04", "Link 40 mm", lk, "tool", "SS", "laser 2 mm SS304 or 3 mm 6061")
    add("BUY-08", "Servo horn (stock single arm)", horn, "tool", "bought", "comes with the servo", export=False)
    add("BUY-09", "Cutter servo MG996R / DS3225MG", servo_body(*G.SERVO_SHAFT, SERVO_TOP_Z, yaw=90.0), "tool", "bought",
        "MG996R on hand; DS3225MG recommended", export=False)
    add("T-05", "Cutter servo bracket", cutter_servo_bracket(), "tool", "PETG", "print")
    add("T-06", "Basket", basket(), "tool", "PETG", "print, 3 walls")
    d, pad, lip = door(door_open)
    add("T-07", "Door", d, "tool", "PETG", "print")
    add("T-08", "Door pad 10 mm", pad, "tool", "foam", "cut from 10 mm EVA or PU foam", export=False)
    add("T-09", "Door lip", lip, "tool", "TPU", "print TPU 95A")
    add("T-10", "Door servo bracket", door_servo_bracket(), "tool", "PETG", "print")
    add("BUY-10", "Door servo MG996R", door_servo(), "tool", "bought", "on hand", export=False)
    add("T-11", "Feeler fork", feeler_fork(), "tool", "PETG", "print")
    add("T-12", "Fork hinge blocks", fork_hinge_blocks(), "tool", "PETG", "print", qty=2)
    add("BUY-11", "Roller micro switch (feeler)", micro_switch(51.5, -32.75, -G.DECK_T), "tool", "bought", "on hand", export=False)
    add("T-13", "IR posts", ir_posts(), "tool", "PETG", "print", qty=2)
    cam, br = camera_and_bracket()
    add("T-14", "Camera bracket", br, "tool", "PETG", "print")
    add("BUY-12", "RGB-D camera (D405 class)", cam, "tool", "bought", "bill item 5.3", export=False)
    add("BUY-13", "Fruit sensor TCRT5000", fruit_sensor(), "tool", "bought", "buy", export=False)


def frame_transform(frame, j1=0.0, j2=-40.0, j3=35.0):
    """Place a frame in the STEP frame at joint angles (deg). j1 is added to the stock Arm 1 yaw."""
    yaw1 = G.ARM1_YAW + j1
    t1 = Pos(*G.J1_XY, 0) * Rot(0, 0, yaw1)
    if frame == "arm1":
        return t1
    t2 = t1 * Pos(G.L1, 0, 0) * Rot(0, 0, j2)
    if frame == "arm2":
        return t2
    return t2 * Pos(G.L2, 0, G.TOOL_Z0) * Rot(0, 0, j3)


def mass_g(p):
    rho = DENSITY.get(p["material"])
    return None if rho is None else p["solid"].volume / 1000.0 * rho


def assembly(j1=0.0, j2=-40.0, j3=35.0, frames=("arm1", "arm2", "tool")):
    kids = []
    for p in PARTS:
        if p["frame"] not in frames:
            continue
        s = frame_transform(p["frame"], j1, j2, j3) * p["solid"] if frames != ("tool",) else p["solid"]
        s.label = f'{p["id"]} {p["name"]}'
        s.color = COLOR.get(p["material"], COLOR["bought"])
        kids.append(s)
    return Compound(children=kids, label="agribot_arm_v2")


def manufacturing_frame(p):
    s = p["solid"]
    bb = s.bounding_box()
    return Pos(0, 0, -bb.min.Z) * s


def write_outputs():
    for sub in ("step", "stl", "dxf", "preview"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)
    rows = []
    for p in PARTS:
        m = mass_g(p)
        rows.append([p["id"], p["name"], p["material"], p["process"], p["qty"],
                     "" if m is None else f"{m:.0f}", p["note"]])
        if not p["export"]:
            continue
        stem = p["id"].lower()
        solid = manufacturing_frame(p)
        export_step(solid, str(OUT / "step" / f"{stem}.step"))
        if p["material"] in ("PETG", "TPU"):
            export_stl(solid, str(OUT / "stl" / f"{stem}.stl"), tolerance=0.02, angular_tolerance=0.2)
        if "laser" in p["process"] or "waterjet" in p["process"]:
            bb = solid.bounding_box()
            zmid = (bb.min.Z + bb.max.Z) / 2
            sec = Pos(0, 0, -zmid) * section(solid, Plane.XY.offset(zmid))
            dxf = ExportDXF(unit=Unit.MM)
            dxf.add_layer("cut")
            dxf.add_shape(sec, layer="cut")
            dxf.write(str(OUT / "dxf" / f"{stem}.dxf"))
    with open(OUT / "parts.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["id", "name", "material", "process", "qty", "mass_g_total_est", "note"])
        w.writerows(rows)
    export_step(assembly(), str(OUT / "agribot_arm_v2.step"))
    export_step(assembly(frames=("tool",)), str(OUT / "end_effector.step"))
    export_gltf(assembly(), str(OUT / "preview" / "arm_v2.glb"), binary=True, linear_deflection=0.08, angular_deflection=0.3)
    export_gltf(assembly(frames=("tool",)), str(OUT / "preview" / "end_effector.glb"), binary=True,
                linear_deflection=0.05, angular_deflection=0.25)
    return rows


def main():
    build_parts()
    rows = write_outputs()
    for r in rows:
        print(" | ".join(str(x) for x in r[:6]))


if __name__ == "__main__":
    main()
