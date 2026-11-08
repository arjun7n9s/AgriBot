"""Build the CAD for SCARA arm v2 (printed Arm 1, Arm 2, J2/J3 joints) and the tomato cutter.

Every made part is printed (PETG) on a 250 x 250 x 250 mm bed (Anycubic Kobra S1). Bought parts are the
bearings, belts, motors and fasteners on hand, plus the cutter extras (blades, sensors, camera, inserts).

Run with the CAD venv (build123d), from the repo root:

    .venv-cad/Scripts/python tools/cad/build_arm_v2.py

Writes cad/arm-v2/:
    agribot_arm_v2.step     whole arm in the stock STEP frame (overlays the stock base)
    end_effector.step       the tool alone, tool frame
    step/<part>.step        each printed part, in its own frame
    stl/<part>.stl          each printed part, ready to slice
    parts.csv               part list with material, process, quantity and mass
    preview/*.glb           meshes for the Blender renders (tools/cad/render_arm_v2.py)

Design notes, numbers and sources: docs/arm-v2-design.md. Shared numbers: arm_v2_geometry.py.
"""
import csv
import math
import shutil
import sys
from pathlib import Path

from build123d import (Align, BuildSketch, Circle, Color, Compound, Cylinder, Box, Locations, Plane, Polygon, Pos,
                       Rectangle, RegularPolygon, Rot, SlotCenterToCenter, Sphere, export_gltf, export_step, export_stl,
                       extrude, make_hull, offset)

sys.path.insert(0, str(Path(__file__).parent))
import arm_v2_geometry as G  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "cad" / "arm-v2"

# PETG 1.27 g/cm3; printed parts are about 60% solid at 4-6 walls and 30-40% gyroid [assumed]
DENSITY = {"PETG": 1.27 * 0.6, "PETG100": 1.27, "steel": 7.85, "foam": 0.05, "bought": None, "fruit": None}
COLOR = {"PETG": Color(0.15, 0.45, 0.25), "PETG100": Color(0.15, 0.45, 0.25), "steel": Color(0.82, 0.83, 0.85),
         "foam": Color(0.9, 0.85, 0.6), "bought": Color(0.2, 0.2, 0.22), "belt": Color(0.05, 0.05, 0.05),
         "fruit": Color(0.85, 0.12, 0.08)}
PRINTED = ("PETG", "PETG100")


# ============================================================================ helpers
def cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
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


def poly_face(pts):
    with BuildSketch() as s:
        Polygon(*pts, align=None)
    return s.sketch


def slot_cut(x0, y0, x1, y1, w, z0, z1):
    ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
    with BuildSketch() as s:
        SlotCenterToCenter(max(math.hypot(x1 - x0, y1 - y0), 0.01), w)
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Rot(0, 0, ang) * extrude(s.sketch, z1 - z0)


def plate(face, z0, z1, holes=(), slots=()):
    """Extrude a 2D face between z0 and z1 and cut holes [(x, y, d)] and slots [(x0, y0, x1, y1, w)]."""
    p = Pos(0, 0, z0) * extrude(face, z1 - z0)
    for x, y, d in holes:
        p -= cyl(d / 2, z0 - 1, z1 + 1, x, y)
    for x0, y0, x1, y1, w in slots:
        p -= slot_cut(x0, y0, x1, y1, w, z0 - 1, z1 + 1)
    return p


def polar(n, r, start=0.0):
    return [(r * math.cos(math.radians(start + i * 360 / n)), r * math.sin(math.radians(start + i * 360 / n)))
            for i in range(n)]


def hex_prism(x, y, z0, h, af):
    with BuildSketch() as s:
        RegularPolygon(af / math.sqrt(3) + 0.15, 6)
    return Pos(x, y, z0) * extrude(s.sketch, h)


def nut_trap(x, y, z0, toward):
    """M4 hex nut pocket at (x, y) from z0, with a side slot so the nut slides in from `toward`."""
    af, t = G.NUT_M4["af"], G.NUT_M4["t"] + 0.3
    p = hex_prism(x, y, z0, t, af)
    ang = math.degrees(math.atan2(toward[1], toward[0]))
    p += Pos(x, y, z0) * Rot(0, 0, ang) * Pos(0, -(af + 0.3) / 2, 0) * Box(
        12.0, af + 0.3, t, align=(Align.MIN, Align.MIN, Align.MIN))
    return p


def inward(x, y):
    """Unit vector from a column toward the inside of the link (the nut goes in from there)."""
    return (0.0, -1.0 if y > 0 else 1.0) if abs(y) > 1 else (1.0, 0.0)


def columns(points, z0, z1, outline, r=5.0):
    """Printed bolt columns, trimmed to the link outline."""
    out = None
    for x, y in points:
        c = cyl(r, z0, z1, x, y)
        out = c if out is None else out + c
    return out & (Pos(0, 0, z0) * extrude(outline, z1 - z0))


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
        for k in range(1, 3):                            # land up to the next groove
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


# ============================================================================ Arm 1 (arm1 frame)
A1_COLS = [(68.0, 32.0), (68.0, -32.0), (120.0, 32.0), (120.0, -32.0), (205.0, 36.0), (205.0, -36.0)]
IDLER1 = (100.0, 25.3)       # belt 1 tensioner, slot along y, hangs from the lid
IDLER2 = (185.0, 23.5)       # belt 2 tensioner, slot along y, stands on the floor
J2_HUB_BOLTS = [(G.L1 + x, y) for x, y in polar(6, 34.0, 30)]
ROOT_X0 = G.CARRIAGE_FACE_X + 0.5
ROOT_X1 = ROOT_X0 + 12.0


def arm1_outline():
    return hull_face([(160.0, 32.0, 8.5), (160.0, -32.0, 8.5), (G.L1, 0.0, 44.0)],
                     polys=[[(ROOT_X0, -40), (ROOT_X0 + 10, -40), (ROOT_X0 + 10, 40), (ROOT_X0, 40)]])


def arm1_tray():
    """Printed PETG, floor down. Floor, walls, root wall (4 x M5 to the carriage), bolt columns, idler boss."""
    f0, f1 = G.A1_FLOOR
    top = G.A1_LID[0]
    out = arm1_outline()
    solid = Pos(0, 0, f0) * extrude(out, top - f0)
    p = solid - Pos(0, 0, f1) * extrude(offset(out, -G.WALL), top - f1 + 1)
    p += box(ROOT_X0, ROOT_X1, -40, 40, f1, top) & solid
    p -= box(ROOT_X0 - 1, ROOT_X1 + 1, -20, 20, 245.0, top + 1)                 # belt 1 window
    for y in (-G.CARRIAGE_M5_Y, G.CARRIAGE_M5_Y):
        for z in G.CARRIAGE_M5_Z:
            p -= Pos(ROOT_X0 - 2, y, z) * Rot(0, 90, 0) * Cylinder(2.75, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p += columns(A1_COLS, f1, top, out)
    for x, y in A1_COLS:
        p -= cyl(2.2, f1 + 1.0, top + 1, x, y)
        p -= nut_trap(x, y, f1 + 3.0, inward(x, y))
    # idler boss: two 608 for the stock 80T/23T compound pulley axle (M8 x 45, head below, turns with the pulley)
    p += cyl(15.0, f1, 244.6, G.IDLER_X, 0)
    p -= cyl(11.03, 230.0, 245.0, G.IDLER_X, 0) + cyl(8.5, f0 - 1, 230.01, G.IDLER_X, 0)
    # J2: hub bolts under the floor; the upper 51107 sits in a 4 mm counterbore
    p -= cyl(19.25, f0 - 1, f1 + 1, G.L1, 0) + cyl(26.3, G.J2_THRUST_HI[0], f1 + 1, G.L1, 0)
    for x, y in J2_HUB_BOLTS:
        p -= cyl(2.25, f0 - 1, f1 + 1, x, y)
    p -= slot_cut(IDLER2[0], IDLER2[1] - 3, IDLER2[0], IDLER2[1] + 3, 5.3, f0 - 1, f1 + 1)
    return p


def arm1_lid():
    """Printed PETG, 5 mm. M4 x 40 down into nuts trapped in the columns."""
    holes = [(x, y, 4.5) for x, y in A1_COLS] + [(G.IDLER_X, 0.0, 20.0), (G.L1, 0.0, 32.0)]
    slots = [(IDLER1[0], IDLER1[1] - 3, IDLER1[0], IDLER1[1] + 3, 5.3)]
    return plate(arm1_outline(), *G.A1_LID, holes=holes, slots=slots)


def j2_hub():
    """Printed PETG. 6806 seat; bolted under the Arm 1 floor with 6 x M4 x 20, nuts in hex pockets."""
    z0, z1 = G.J2_HUB
    p = cyl(39.0, z0, z1) - cyl(21.03, z0 - 1, G.J2_RADIAL[1]) - cyl(19.25, G.J2_RADIAL[1] - 0.1, z1 + 1)
    for x, y in polar(6, 34.0, 30):
        p -= cyl(2.25, z0 - 1, z1 + 1, x, y)
        p -= hex_prism(x, y, z0 - 0.1, 3.6, G.NUT_M4["af"])
    return Pos(G.L1, 0, 0) * p


def compound_pulley_stock():
    """Stock printed part 'GT2 Pulley - 23 - 80 teeth' (reused, not rebuilt); envelope only."""
    z0, z1 = G.COMPOUND_PULLEY_Z
    p = cyl(8.7, z0, 255.6) + cyl(26.8, 255.6, z1) - cyl(4.0, z0 - 1, z1 + 1)
    return Pos(G.IDLER_X, 0, 0) * p


def toothless_idler(x, y, zmid):
    """On-hand 'GT2 toothless driven pulley, 5 mm bore' envelope (assumed 18 mm flanges)."""
    p = cyl(6.5, zmid - 3.5, zmid + 3.5) + cyl(9.0, zmid - 4.3, zmid - 3.5) + cyl(9.0, zmid + 3.5, zmid + 4.3)
    return Pos(x, y, 0) * (p - cyl(2.5, zmid - 10, zmid + 10))


# ============================================================================ J2 joint (arm1 frame at J2)
def j2_spindle():
    """Printed PETG, 100% infill, flange down. Hollow for cables. 4 x M4 heat-set inserts in the top."""
    f0, f1 = G.J2_FLANGE
    top = G.J2_PULLEY[0]
    p = cyl(39.0, f0, f1) + cyl(17.5, f1, G.J2_THRUST_LO[1]) + cyl(15.0, G.J2_THRUST_LO[1], top)
    p -= cyl(8.0, f0 - 1, top + 1)
    for x, y in polar(6, 32.0, 30):
        p -= cyl(2.25, f0 - 1, f1 + 1, x, y)
    for x, y in polar(4, 11.5, 45):
        p -= cyl(G.INSERT_M4["hole"] / 2, top - G.INSERT_M4["depth"], top + 1, x, y)
    return Pos(G.L1, 0, 0) * p


def j2_pulley():
    """Printed PETG, 92T. Spigot centres the upper 51107; 4 x M4 x 20 into the spindle inserts."""
    z0 = G.J2_PULLEY[0]
    p = gt2_pulley(G.J2_PULLEY_TEETH, G.BELT2_Z, hub=[(24.0, z0, G.BELT2_Z - 3.0)])
    p += ring(15.1, 17.4, G.J2_THRUST_HI[0] + 0.5, z0)
    p -= cyl(8.0, z0 - 20, z0 + 30)
    for x, y in polar(4, 11.5, 45):
        p -= cyl(2.25, z0 - 1, z0 + 30, x, y)
    return Pos(G.L1, 0, 0) * p


# ============================================================================ Arm 2 (arm2 frame)
A2_COLS = [(-36.0, 0.0), (100.0, 32.0), (100.0, -32.0), (165.0, 31.0), (165.0, -31.0)]


def arm2_outline():
    return hull_face([(0.0, 0.0, 42.0), (G.L2, 0.0, 36.0)])


def arm2_body():
    """Printed PETG, top face down on the bed. Top plate (J2 flange bolts), walls, bolt columns."""
    c1 = G.A2_COVER[1]
    t0, t1 = G.A2_TOP
    out = arm2_outline()
    p = Pos(0, 0, c1) * extrude(out, t1 - c1) - Pos(0, 0, c1 - 1) * extrude(offset(out, -G.WALL), t0 - c1 + 1)
    p += columns(A2_COLS, c1, t0, out)
    for x, y in A2_COLS:
        p -= cyl(2.2, c1 - 1, t0 - 0.8, x, y)
        p -= nut_trap(x, y, t0 - 6.5, inward(x, y))
    for x, y in polar(6, 32.0, 30):
        p -= cyl(2.25, t0 - 1, t1 + 1, x, y)
    p -= cyl(8.0, t0 - 1, t1 + 1)                                               # cables up into J2
    return p


def arm2_cover():
    """Printed PETG, inner face down on the bed. Carries the J3 bearings (6807 below, 6806 above) and the J3 motor.
    M4 x 30 up into nuts trapped in the body columns."""
    c0, c1 = G.A2_COVER
    m, j = G.J3_MOTOR_X, G.L2
    p = Pos(0, 0, c0) * extrude(arm2_outline(), c1 - c0)
    p += cyl(32.0, G.J3_HUB[0], c0 + 0.01, j, 0)
    for x, y in A2_COLS:
        p -= cyl(2.25, c0 - 1, c1 + 1, x, y)
    p -= cyl(23.53, G.J3_RADIAL_LO[0] - 1, G.J3_RADIAL_LO[1], j, 0)            # 6807 seat, open below
    p -= cyl(21.0, G.J3_WEB[0] - 0.01, G.J3_WEB[0] + 1.5, j, 0)                # web: 6807 outer ring only
    p -= cyl(19.5, G.J3_WEB[0] + 1.49, G.J3_WEB[1] + 0.01, j, 0)               # web: 6806 outer ring only
    p -= cyl(21.03, G.J3_RADIAL_HI[0], c1 + 1, j, 0)                           # 6806 seat, open above
    p -= cyl(5.0, c0 - 1, c1 + 1, 30.0, 18.0)                                   # motor cable up into the arm
    p -= slot_cut(m - 4, 0, m + 4, 0, 23.0, c0 - 1, c1 + 1)
    h = G.NEMA17["hole_sq"] / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            p -= slot_cut(m + sx * h - 4, sy * h, m + sx * h + 4, sy * h, 3.4, c0 - 1, c1 + 1)
    return p


def j3_spindle():
    """Printed PETG, 100% infill, flange down. Tool flange: 4 x M4 heat-set inserts on a 48 PCD, 20 mm spigot,
    4 mm dowel. Hollow for the tool cables. 4 x M4 inserts in the top for the pulley."""
    f0, f1 = G.J3_FLANGE
    top = G.J3_SPINDLE_TOP
    p = cyl(30.0, f0, f1) + cyl(10.0, f0 - 3.0, f0) + cyl(19.3, f1, G.J3_RADIAL_LO[0])
    p += cyl(17.5, G.J3_RADIAL_LO[0], G.J3_RADIAL_LO[1]) + cyl(15.0, G.J3_RADIAL_LO[1], top)
    p -= cyl(7.0, f0 - 5, top + 1)
    for x, y in polar(4, 24.0, 45):
        p -= cyl(G.INSERT_M4["hole"] / 2, f0 - 1, f1 + 0.01, x, y)
    p -= cyl(2.0, f0 - 1, f0 + 4, 20.0, 0.0)
    for x, y in polar(4, 11.0, 45):
        p -= cyl(G.INSERT_M4["hole"] / 2, top - G.INSERT_M4["depth"], top + 1, x, y)
    return Pos(G.L2, 0, 0) * p


def j3_pulley():
    """Printed PETG, 90T. Bears on the 6806 inner ring; 4 x M4 x 20 into the spindle inserts."""
    c1 = G.A2_COVER[1]
    top = G.J3_SPINDLE_TOP
    band0 = G.J3_BELT_Z - 3.0 - 0.9
    p = gt2_pulley(G.J3_PULLEY_TEETH, G.J3_BELT_Z, hub=[(19.0, c1 + 1.0, band0), (19.0, G.J3_BELT_Z + 3.9, 185.0)])
    p += ring(15.2, 16.8, c1, c1 + 1.0)
    p -= cyl(15.15, c1 - 1, top) + cyl(7.0, top - 1, 190)
    for x, y in polar(4, 11.0, 45):
        p -= cyl(2.25, top - 1, 190, x, y)
    return Pos(G.L2, 0, 0) * p


def nema17(x, y, face_z, length=None):
    n = G.NEMA17
    ln = length or n["length"]
    s = n["side"] / 2
    body = box(x - s, x + s, y - s, y + s, face_z - ln, face_z)
    body += cyl(n["boss_d"] / 2, face_z, face_z + n["boss_h"], x, y)
    body += cyl(n["shaft_d"] / 2, face_z, face_z + n["shaft_len"], x, y)
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


# ============================================================================ end effector v3 (tool frame)
# A suction cup draws the fruit out of the cluster and up under the hood; the pedicel threads the hood slot; a
# utility blade in a sliding carrier cuts it against a stainless edge just above the hood. Two servos, one vacuum
# line. See docs/arm-v2-design.md section 5.
CH_X = (-50.0, 76.0)
CH_Y = 56.0
CH_Z = (-6.0, 0.0)
HANGERS = [(62.0, 74.0, 16.0, 22.0), (50.0, 60.0, -46.0, -40.0)]       # x0, x1, y0, y1: chassis down to the hood
HOUSING = dict(x=(-44.0, 24.0), y=13.0, z=(-70.0, -50.0))              # LM8UU housing under the chassis
BEARINGS_X = [(-40.0, -16.0), (-4.0, 20.0)]
FLANGE_BOLTS = polar(4, 24.0, 45)                                      # M4 x 12 up into the J3 flange inserts
STRIP_SCREWS = [(80.0, 23.0), (126.0, 23.0)]
SLIDE_FACE_Z = -43.5          # slide servo (shaft down): output face
CUT_FACE_Z = -21.0            # cutter servo (shaft up): output face; its tabs sit on the hood extension
SLIDE_ARM_Z = (-58.0, -54.0)
SLIDE_LINK_Z = (-53.0, -49.5)
SLIDE_HORN_Z = (-49.0, -46.5)
CARRIER_Z = (G.HOOD_TOP + G.SS_STRIP_T, G.HOOD_TOP + G.SS_STRIP_T + 6.0)   # rides on the steel strip and the land
CUT_HORN_Z = (-15.5, -13.0)
CUT_LINK_Z = (-12.5, -9.5)


def servo_case(x, y, z_face, up=True, yaw=0.0):
    """Standard servo (MG996R / DS3225MG): output shaft at (x, y), output face at z_face. Body runs toward local +x."""
    s = G.SERVO
    off = s["l"] / 2 - s["shaft_off"]
    b = box(-s["l"] / 2, s["l"] / 2, -s["w"] / 2, s["w"] / 2, -s["h"], 0)
    b += box(-s["tab_l"] / 2, s["tab_l"] / 2, -s["w"] / 2, s["w"] / 2, -s["h"] + s["tab_z"], -s["h"] + s["tab_z"] + s["tab_t"])
    b = Pos(off, 0, 0) * b
    b += cyl(6.0, 0, 3.0) + cyl(3.0, 3.0, 5.5)
    if not up:
        b = Rot(180, 0, 0) * b
    return Pos(x, y, z_face) * Rot(0, 0, yaw) * b


def along_x(r, x0, x1, y=0.0, z=0.0):
    return Pos(x0, y, z) * Rot(0, 90, 0) * Cylinder(r, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))


# ---------------------------------------------------------------------------- slide
def slide_state(s):
    """Cup face x, crank pin (x, y) and tab x for slide command s (0 retracted, 1 extended)."""
    th = math.radians(180.0 * (1.0 - s))
    sx, sy = G.SLIDE_SERVO
    pin = (sx + G.SLIDE_CRANK_R * math.cos(th), sy + G.SLIDE_CRANK_R * math.sin(th))
    cf = G.cup_face(s)
    return cf, pin, cf - G.TAB_TO_CUP


def cup_and_fitting(cf):
    z = G.CUP_Z
    p = along_x(G.CUP["d"] / 2, cf - 3, cf, z=z) + along_x(10.5, cf - 9, cf - 3, z=z)
    p += along_x(12.0, cf - 13, cf - 9, z=z) + along_x(6.0, cf - G.CUP["h"], cf - 13, z=z)
    p += along_x(G.CUP_FITTING["d"] / 2, cf - G.CUP["h"] - G.CUP_FITTING["l"], cf - G.CUP["h"], z=z)
    return p


def cup_holder(cf):
    """Printed PETG, 100%. Clamps the rod end, holds the cup fitting, and carries the slide arm and its pin."""
    x0, x1 = cf - 41.0, cf - 29.0
    z = G.CUP_Z
    p = box(x0, x1, -10, 10, z - 10, z + 10)
    p += box(x0, x1, -10, G.SLIDE_SERVO[1] + 4, *SLIDE_ARM_Z)
    p -= along_x(4.1, x0 - 1, x0 + 10, z=z)                                   # rod, 10 mm deep (M3 grub screw)
    p -= along_x(2.6, x1 - 8, x1 + 1, z=z)                                    # M5 cup stem: nut inside a slot
    p -= box(x1 - 7, x1 - 3, -4.1, 4.1, z - 10, z + 0.1)                     # M5 nut slot, open below
    p -= cyl(1.6, SLIDE_ARM_Z[0] - 1, SLIDE_ARM_Z[1] + 1, cf - G.TAB_TO_CUP, G.SLIDE_SERVO[1])
    p -= cyl(1.6, z + 2, z + 11, x0 + 6, 0)                                    # grub screw onto the rod
    return p


def rod_and_bearings(cf):
    x_front = cf - 31.0
    rod = along_x(G.ROD["d"] / 2, x_front - G.ROD["l"], x_front, z=G.CUP_Z)
    brg = None
    for a, b in BEARINGS_X:
        r = along_x(G.LM8UU["d"] / 2, a, b, z=G.CUP_Z) - along_x(4.0, a - 1, b + 1, z=G.CUP_Z)
        brg = r if brg is None else brg + r
    return rod, brg


def slide_drive(s):
    """Stock horn (crank, bought) and the printed link from the crank pin to the slide arm."""
    cf, pin, tab = slide_state(s)
    sx, sy = G.SLIDE_SERVO
    horn = Pos(0, 0, SLIDE_HORN_Z[0]) * extrude(hull_face([(sx, sy, 5.0), (pin[0], pin[1], 3.5)]),
                                                SLIDE_HORN_Z[1] - SLIDE_HORN_Z[0])
    z0, z1 = SLIDE_LINK_Z
    link = Pos(0, 0, z0) * extrude(hull_face([(pin[0], pin[1], 4.0), (tab, sy, 4.0)]), z1 - z0)
    link -= cyl(1.6, z0 - 1, z1 + 1, *pin) + cyl(1.6, z0 - 1, z1 + 1, tab, sy)
    return horn, link


def slide_servo():
    return servo_case(*G.SLIDE_SERVO, SLIDE_FACE_Z, up=False, yaw=180.0)


def slide_servo_cradle():
    """Printed PETG. A ledge under the slide servo's tabs and two side walls up to the chassis."""
    s = G.SERVO
    sx, sy = G.SLIDE_SERVO
    off = s["l"] / 2 - s["shaft_off"]
    cx = sx - off                                         # body centre (body runs toward -x)
    tab_bot = SLIDE_FACE_Z + s["h"] - s["tab_z"] - s["tab_t"]
    led = box(cx - s["tab_l"] / 2 - 3, cx + s["tab_l"] / 2 + 3, sy - s["w"] / 2 - 3, sy + s["w"] / 2 + 3, tab_bot - 4, tab_bot)
    led -= box(cx - s["l"] / 2 - 0.3, cx + s["l"] / 2 + 0.3, sy - s["w"] / 2 - 0.3, sy + s["w"] / 2 + 0.3, tab_bot - 5, tab_bot + 1)
    for dx in (-1, 1):
        led -= cyl(1.25, tab_bot - 5, tab_bot + 1, cx + dx * (s["l"] / 2 + 4.5), sy)
    walls = None
    for dy in (-1, 1):
        w = box(cx - s["tab_l"] / 2 - 3, cx + s["tab_l"] / 2 + 3, sy + dy * (s["w"] / 2 + 0.3),
                sy + dy * (s["w"] / 2 + 5.3), tab_bot - 4, CH_Z[0])
        walls = w if walls is None else walls + w
    p = led + walls
    for x, y in CRADLE_SCREWS:
        p -= cyl(1.25, CH_Z[0] - 10, CH_Z[0] + 1, x, y)                    # M3 self-tap from the chassis
    return p


# ---------------------------------------------------------------------------- cutter
def blade_pts(edge_y):
    """Utility blade outline (x, y) for the edge at edge_y (at BLADE_X_MID). Edge first, then back."""
    a = math.radians(G.BLADE_RAKE)
    d = (math.cos(a), math.sin(a))
    n = (math.sin(a), -math.cos(a))                       # toward the blade back (-y)
    m = (G.BLADE_X_MID, edge_y)
    e, bk, h = G.UTILITY_BLADE["edge"] / 2, G.UTILITY_BLADE["back"] / 2, G.UTILITY_BLADE["h"]
    return [(m[0] - e * d[0], m[1] - e * d[1]), (m[0] + e * d[0], m[1] + e * d[1]),
            (m[0] + bk * d[0] + h * n[0], m[1] + bk * d[1] + h * n[1]),
            (m[0] - bk * d[0] + h * n[0], m[1] - bk * d[1] + h * n[1])]


def blade(c):
    z0 = CARRIER_Z[0]
    return Pos(0, 0, z0) * extrude(poly_face(blade_pts(G.blade_y(c))), G.UTILITY_BLADE["t"])


def carrier(c):
    """Printed PETG, 100%. Holds the blade (pocket underneath, 2 x 6x2 magnets); slides across the slot on two
    M4 shoulder bolts with OD 6 springs that press it flat on the steel strip."""
    y = G.blade_y(c)
    dy = y - G.BLADE_Y_OPEN
    a = math.radians(G.BLADE_RAKE)
    n = (math.sin(a), -math.cos(a))
    pts = blade_pts(y)
    k0, k1 = G.BLADE_EXPOSED, G.UTILITY_BLADE["h"] + 2.0
    back = [(pts[0][0] + k0 * n[0], pts[0][1] + k0 * n[1]), (pts[1][0] + k0 * n[0], pts[1][1] + k0 * n[1])]
    z0, z1 = CARRIER_Z
    outline = hull_face([], polys=[back + [(148.0, -53.0 + dy), (60.0, -53.0 + dy), (60.0, -22.0 + dy), (148.0, -22.0 + dy)]])
    p = Pos(0, 0, z0) * extrude(outline, z1 - z0)
    p -= Pos(0, 0, z0 - 0.1) * extrude(poly_face(pts), G.UTILITY_BLADE["t"] + 0.1)
    for f in (0.3, 0.7):                                                    # magnets over the blade body
        mx = pts[0][0] + f * (pts[1][0] - pts[0][0]) + 10 * n[0]
        my = pts[0][1] + f * (pts[1][1] - pts[0][1]) + 10 * n[1]
        p -= cyl(3.1, z0 + G.UTILITY_BLADE["t"] - 0.01, z0 + G.UTILITY_BLADE["t"] + 2.2, mx, my)
    for px, py in G.CARRIER_PINS:                                            # slots over the fixed pins
        p -= slot_cut(px, py + dy, px, py + dy - 24.0, 4.4, z0 - 1, z1 + 1)
    post = (G.CUT_SERVO[0], y + G.CARRIER_PIN_DY)
    p += cyl(4.0, z1, CUT_LINK_Z[0] - 0.5, *post)
    p -= cyl(1.6, z0 + 1, CUT_LINK_Z[1] + 1, *post)
    return p


def cut_drive(c):
    th = math.radians(180.0 * (1.0 - c))
    sx, sy = G.CUT_SERVO
    pin = (sx + G.CUT_CRANK_R * math.sin(th), sy + G.CUT_CRANK_R * math.cos(th))
    post = (sx, G.blade_y(c) + G.CARRIER_PIN_DY)
    horn = Pos(0, 0, CUT_HORN_Z[0]) * extrude(hull_face([(sx, sy, 5.0), (pin[0], pin[1], 3.5)]), CUT_HORN_Z[1] - CUT_HORN_Z[0])
    z0, z1 = CUT_LINK_Z
    link = Pos(0, 0, z0) * extrude(hull_face([(pin[0], pin[1], 4.0), (post[0], post[1], 4.0)]), z1 - z0)
    link -= cyl(1.6, z0 - 1, z1 + 1, *pin) + cyl(1.6, z0 - 1, z1 + 1, *post)
    return horn, link


def cut_servo():
    return servo_case(*G.CUT_SERVO, CUT_FACE_Z, up=True, yaw=0.0)


def shear_strip():
    """Stainless strip cut from the ruler on hand. The ruler's factory edge is the shear edge, on the slot side."""
    x0, x1 = G.STRIP_X
    p = box(x0, x1, G.SHEAR_EDGE_Y, G.SHEAR_EDGE_Y + 22.0, G.HOOD_TOP, G.HOOD_TOP + G.SS_STRIP_T)
    for x, y in STRIP_SCREWS:
        p -= cyl(1.65, G.HOOD_TOP - 1, G.HOOD_TOP + 2, x, y)
    return p


# ---------------------------------------------------------------------------- hood and chassis
def slot_and_v():
    """Slot (with the +y wall set 0.6 mm back under the steel edge) and the V mouth, as a 2D face."""
    w = G.SLOT_W / 2
    x0, x1 = G.SLOT_X
    xf = G.HOOD_X[1]
    with BuildSketch() as s:
        with Locations((x0 + w, 0)):
            Circle(w)
        Polygon((x0 + w, -w), (x1, -w), (x1, w + 0.6), (x0 + w, w + 0.6), align=None)
        Polygon((x1, -w), (xf + 1, -G.MOUTH_W / 2), (xf + 1, G.MOUTH_W / 2), (x1, w), align=None)
    return s.sketch


def hood():
    """Printed PETG, bottom face down. The fruit is drawn up under it; its slot takes the pedicel. A 1 mm land on
    the -y side carries the blade; the steel strip on the +y side is the shear edge. The front lip is chamfered
    underneath so a fruit sitting a little high is pushed down, not caught."""
    z0, z1 = G.HOOD_BOT, G.HOOD_TOP
    x0, x1 = G.HOOD_X
    y0, y1 = G.HOOD_Y
    p = box(x0, x1, y0, y1, z0, z1) + box(48.0, x0 + 1, y0, -36.0, z0, z1) + box(88.0, 137.0, -78.0, y0 + 1, z0, z1)
    p += box(64.0, 146.0, y0, -G.SLOT_W / 2 - 0.6, z1, z1 + G.SS_STRIP_T)                 # blade land
    p -= Pos(0, 0, z0 - 1) * extrude(slot_and_v(), z1 - z0 + G.SS_STRIP_T + 2)
    with BuildSketch(Plane.XZ) as s:                                                       # front chamfer underneath
        Polygon((x1 - 10, z0 - 0.01), (x1 + 1, z0 - 0.01), (x1 + 1, z0 + 3), align=None)
    p -= extrude(s.sketch, 200, both=True)
    s_ = G.SERVO
    cx = G.CUT_SERVO[0] + s_["l"] / 2 - s_["shaft_off"]
    p -= box(cx - s_["l"] / 2 - 0.3, cx + s_["l"] / 2 + 0.3, G.CUT_SERVO[1] - s_["w"] / 2 - 0.3,
             G.CUT_SERVO[1] + s_["w"] / 2 + 0.3, z0 - 1, z1 + 1)
    for dx in (-1, 1):
        p -= cyl(1.25, z0 - 1, z1 + 1, cx + dx * (s_["l"] / 2 + 4.5), G.CUT_SERVO[1])   # servo tab screws
    for x, y in G.CARRIER_PINS:
        p -= cyl(G.INSERT_M4["hole"] / 2, z0 - 1, z1 + 2, x, y)                          # M4 inserts from below
    for x, y in STRIP_SCREWS:
        p -= cyl(1.25, z0 + 1, z1 + 1, x, y)                                               # M3 self-tap, blind
    for x0_, x1_, y0_, y1_ in HANGERS:
        p -= cyl(1.7, z0 - 1, z1 + 1, (x0_ + x1_) / 2, (y0_ + y1_) / 2)
    return p


def chassis():
    """Printed PETG, top face down on the bed. Bolts under the J3 flange; carries the hood hangers, the LM8UU
    housing for the cup rod, and the camera bracket."""
    z0, z1 = CH_Z
    p = box(CH_X[0], CH_X[1], -CH_Y, CH_Y, z0, z1)
    p -= cyl(10.1, z0 - 1, z1 + 1)                                         # flange spigot; hose and wires pass here
    for x, y in FLANGE_BOLTS:
        p -= cyl(2.25, z0 - 1, z1 + 1, x, y) + cyl(4.2, z0 - 1, z0 + 2.0, x, y)
    p -= cyl(2.0, z1 - 4, z1 + 1, 20.0, 0.0)                               # dowel
    for x0, x1, y0, y1 in HANGERS:
        h = box(x0, x1, y0, y1, G.HOOD_TOP, z0 + 0.01)
        h -= cyl(1.25, G.HOOD_TOP - 1, G.HOOD_TOP + 10, (x0 + x1) / 2, (y0 + y1) / 2)
        p += h
    hx, hy, hz = HOUSING["x"], HOUSING["y"], HOUSING["z"]
    p += box(hx[0], hx[1], -6.0, 6.0, hz[1] - 0.01, z0 + 0.01) + box(hx[0], hx[1], -hy, hy, hz[0], hz[1])
    p -= along_x(4.6, hx[0] - 1, hx[1] + 1, z=G.CUP_Z)
    p -= along_x(G.LM8UU["d"] / 2 + 0.03, hx[0] - 1, BEARINGS_X[0][1], z=G.CUP_Z)
    p -= along_x(G.LM8UU["d"] / 2 + 0.03, BEARINGS_X[1][0], hx[1] + 1, z=G.CUP_Z)
    for x, y in CRADLE_SCREWS:
        p -= cyl(1.7, z0 - 1, z1 + 1, x, y)                                # slide cradle screws
    for x, y in CAMERA_SCREWS:
        p -= cyl(1.25, z1 - 5, z1 + 1, x, y)
    return p


CAMERA_SCREWS = [(42.0, -12.0), (42.0, 12.0)]
_CX = G.SLIDE_SERVO[0] - (G.SERVO["l"] / 2 - G.SERVO["shaft_off"])
CRADLE_SCREWS = [(_CX + dx, G.SLIDE_SERVO[1] + dy * (G.SERVO["w"] / 2 + 2.8)) for dx in (-20.0, 20.0) for dy in (-1, 1)]
CAMERA_POSE = (58.0, 0.0, 20.0)


def camera_and_bracket():
    c = G.CAMERA
    place = Pos(*CAMERA_POSE) * Rot(0, c["tilt"], 0)
    cam = place * (box(-c["d"], 0, -c["w"] / 2, c["w"] / 2, -c["h"] / 2, c["h"] / 2)
                   + along_x(c["lens_d"] / 2, 0, c["lens_l"]))
    br = box(36.0, 60.0, -18.0, 18.0, 0.0, 4.0)
    br += place * box(-c["d"] - 3.0, -c["d"], -c["w"] / 2 - 2, c["w"] / 2 + 2, -c["h"] / 2 - 10, c["h"] / 2)
    for x, y in CAMERA_SCREWS:
        br -= cyl(1.7, -1, 6, x, y)
    for dy in (-12.0, 12.0):                                               # M2 board screws
        br -= place * along_x(1.1, -c["d"] - 4, -c["d"] + 1, y=dy, z=0)
    return cam, br


def slide_switch():
    """Roller micro switch (on hand) on the housing; the slide arm presses it when fully retracted."""
    hx1 = HOUSING["x"][1]
    return box(hx1 - 20, hx1, 13.5, 20.0, -62.0, -52.0) + box(hx1, G.CUP_FACE_IN - 41.0 + 0.6, 14.0, 18.0,
                                                               SLIDE_ARM_Z[0] + 0.5, SLIDE_ARM_Z[1] - 0.5)


def ref_fruit(r, slide=0.0):
    """Reference fruit of radius r held on the cup and seated on the hood (not a part)."""
    zc = G.HOOD_BOT - r
    cf = G.cup_face(slide)
    xc = cf + math.sqrt(r * r - (G.CUP_Z - zc) ** 2)
    return Pos(xc, 0, zc) * Sphere(r)


def flange_stub():
    """Reference only: the J3 tool flange the chassis bolts to (tool frame)."""
    return cyl(30.0, 0, G.J3_FLANGE[1] - G.J3_FLANGE[0])


def build_tool(slide=0.0, cut=0.0, fruit=32.0):
    """Tool parts at slide command `slide` and cutter command `cut` (0..1). Returns the list it added to PARTS."""
    start = len(PARTS)
    cf, _, _ = slide_state(slide)
    add("T-01", "Tool chassis and LM8UU housing", chassis(), "tool", "PETG", "print top face down, 6 walls, 40% gyroid")
    add("T-02", "Hood", hood(), "tool", "PETG", "print bottom face down, 6 walls, 50% gyroid")
    add("T-03", "Blade carrier", carrier(cut), "tool", "PETG100", "print pocket up, 100%; 2 x 6x2 magnets")
    hornc, linkc = cut_drive(cut)
    add("T-04", "Cutter link 30 mm", linkc, "tool", "PETG100", "print flat, 100%")
    add("T-05", "Cup holder and slide arm", cup_holder(cf), "tool", "PETG100", "print flat, 100%")
    hs, ls = slide_drive(slide)
    add("T-06", "Slide link 70 mm", ls, "tool", "PETG100", "print flat, 100%")
    add("T-07", "Slide servo cradle", slide_servo_cradle(), "tool", "PETG", "print ledge down")
    cam, br = camera_and_bracket()
    add("T-08", "Camera bracket", br, "tool", "PETG", "print base down")
    add("T-09", "Shear strip, 1 mm stainless", shear_strip(), "tool", "steel",
        "cut from the stainless ruler on hand, factory edge on the slot side; drill 2 x 3.2")
    add("BUY-20", "Utility blade (trapezoid, 61 mm)", blade(cut), "tool", "steel", "buy a pack", export=False)
    add("BUY-21", "Suction cup 25 mm bellows + M5 fitting", cup_and_fitting(cf), "tool", "bought", "buy", export=False)
    rod, brg = rod_and_bearings(cf)
    add("BUY-22", "8 mm smooth rod, 160 mm", rod, "tool", "steel", "buy, cut to length", export=False)
    add("BUY-23", "LM8UU x2", brg, "tool", "bought", "buy", qty=2, export=False)
    add("BUY-24", "Slide servo MG996R", slide_servo(), "tool", "bought", "on hand", export=False)
    add("BUY-25", "Cutter servo DS3225MG", cut_servo(), "tool", "bought", "buy (MG996R fits the same cutout)", export=False)
    add("BUY-26", "Servo horns (stock single arm) x2", hs + hornc, "tool", "bought", "come with the servos", export=False)
    add("BUY-27", "Slide-home micro switch", slide_switch(), "tool", "bought", "on hand", export=False)
    add("BUY-28", "USB board camera, 32 mm", cam, "tool", "bought", "buy", export=False)
    if fruit:
        add("REF-1", f"Reference fruit, {2 * fruit:.0f} mm", ref_fruit(fruit, slide), "tool", "fruit", "not a part",
            export=False)
    return PARTS[start:]


# ============================================================================ assembly
PARTS = []


def add(pid, name, solid, frame, material, process, note="", qty=1, export=True):
    PARTS.append(dict(id=pid, name=name, solid=solid, frame=frame, material=material, process=process,
                      qty=qty, note=note, export=export))
    return solid


def build_parts(slide=0.0, cut=0.0, fruit=32.0):
    PARTS.clear()
    # Arm 1
    add("A1-01", "Arm 1 tray", arm1_tray(), "arm1", "PETG", "print floor down, 5 walls, 35% gyroid")
    add("A1-02", "Arm 1 lid", arm1_lid(), "arm1", "PETG", "print flat, 5 walls, 35% gyroid")
    add("A1-03", "J2 hub", j2_hub(), "arm1", "PETG100", "print flat, 100%")
    add("J2-01", "J2 spindle", j2_spindle(), "arm1", "PETG100", "print flange down, 100%; 4 x M4 heat-set inserts")
    add("J2-02", "J2 pulley 92T", j2_pulley(), "arm1", "PETG100", "print teeth up, 100%; test ring first")
    add("STK-1", "Stock 80T/23T compound pulley (reused)", compound_pulley_stock(), "arm1", "bought",
        "stock part, print from the stock STL", export=False)
    add("BUY-01", "51107 thrust bearing x2 (J2)",
        Pos(G.L1, 0, 0) * (bearing(35, 52, *G.J2_THRUST_LO) + bearing(35, 52, *G.J2_THRUST_HI)),
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
    add("A2-01", "Arm 2 body", arm2_body(), "arm2", "PETG", "print top face down, 5 walls, 35% gyroid; diagonal on the bed")
    add("A2-02", "Arm 2 cover and J3 housing", arm2_cover(), "arm2", "PETG",
        "print inner face down, 5 walls, 40% gyroid; diagonal on the bed")
    add("J3-01", "J3 spindle and tool flange", j3_spindle(), "arm2", "PETG100",
        "print flange down, 100%; 8 x M4 heat-set inserts")
    add("J3-02", "J3 pulley 90T", j3_pulley(), "arm2", "PETG100", "print teeth up, 100%; test ring first")
    add("BUY-05", "6807-ZZ + 6806-ZZ (J3)", Pos(G.L2, 0, 0) * (bearing(35, 47, *G.J3_RADIAL_LO) + bearing(30, 42, *G.J3_RADIAL_HI)),
        "arm2", "bought", "on hand", export=False)
    add("BUY-06", "NEMA17 4.2 kg.cm (J3)", nema17(G.J3_MOTOR_X, 0, G.A2_COVER[0]), "arm2", "bought", "on hand", export=False)
    add("BUY-07", "GT2 20T pulley (J3 motor)", gt2_20t(G.J3_MOTOR_X, 0, G.J3_BELT_Z), "arm2", "bought", "on hand", export=False)
    add("BELT-3", "GT2 400 belt (J3)", belt((G.J3_MOTOR_X, 0), r20, (G.L2, 0), G.gt2_pitch_radius(90), G.J3_BELT_Z), "arm2",
        "belt", "on hand", export=False)
    # End effector
    build_tool(slide, cut, fruit)


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
    return Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * s


def bed_fit(solid):
    """Smallest yaw (deg) at which the part's footprint fits the bed, or None. Returns (yaw, x, y, z)."""
    bx, by, bz = (v - 2 * G.BED_MARGIN for v in G.BED)
    for yaw in range(0, 91, 3):
        bb = (Rot(0, 0, yaw) * solid).bounding_box()
        sx, sy, sz = bb.size.X, bb.size.Y, bb.size.Z
        if sx <= bx and sy <= by and sz <= bz:
            return yaw, sx, sy, sz
    return None


def write_outputs():
    for sub in ("step", "stl", "dxf"):
        shutil.rmtree(OUT / sub, ignore_errors=True)
    for sub in ("step", "stl", "preview"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)
    rows = []
    for p in PARTS:
        m = mass_g(p)
        rows.append([p["id"], p["name"], p["material"], p["process"], p["qty"],
                     "" if m is None else f"{m * p['qty']:.0f}", p["note"]])
        if not p["export"]:
            continue
        stem = p["id"].lower()
        solid = manufacturing_frame(p)
        export_step(solid, str(OUT / "step" / f"{stem}.step"))
        if p["material"] in PRINTED:
            export_stl(solid, str(OUT / "stl" / f"{stem}.stl"), tolerance=0.02, angular_tolerance=0.2)
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
