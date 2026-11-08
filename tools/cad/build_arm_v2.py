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

from build123d import (Align, BuildSketch, Circle, Color, Compound, Cylinder, Box, Locations, Polygon, Pos,
                       Rectangle, RegularPolygon, Rot, SlotCenterToCenter, export_gltf, export_step, export_stl,
                       extrude, make_hull, offset)

sys.path.insert(0, str(Path(__file__).parent))
import arm_v2_geometry as G  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "cad" / "arm-v2"

# PETG 1.27 g/cm3; printed parts are about 60% solid at 4-6 walls and 30-40% gyroid [assumed]
DENSITY = {"PETG": 1.27 * 0.6, "PETG100": 1.27, "steel": 7.85, "foam": 0.05, "bought": None}
COLOR = {"PETG": Color(0.15, 0.45, 0.25), "PETG100": Color(0.15, 0.45, 0.25), "steel": Color(0.82, 0.83, 0.85),
         "foam": Color(0.9, 0.85, 0.6), "bought": Color(0.2, 0.2, 0.22), "belt": Color(0.05, 0.05, 0.05)}
PRINTED = ("PETG", "PETG100")


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
    for x, y in polar(4, 24.0, 0):
        p -= cyl(G.INSERT_M4["hole"] / 2, f0 - 1, f1 + 0.01, x, y)
    p -= cyl(2.0, f0 - 1, f0 + 4, 20 * math.cos(math.radians(45)), 20 * math.sin(math.radians(45)))
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


# ============================================================================ end effector (tool frame)
def x_mouth():
    t = math.tan(math.radians(G.V_HALF_ANGLE))
    return G.THROAT_X + G.SLOT_LEN + (G.MOUTH_W / 2 - G.SLOT_W / 2) / t


def v_outline(offset_=0.0):
    """Slot + V opening, as a polygon open toward +x (beyond the tool), grown by `offset_`."""
    hw = G.SLOT_W / 2 + offset_
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
    return s.sketch


def blade_pts(p0, d, n, length):
    """3-segment snap-off blade piece: edge from p0 along d, back edge w along n, ends slanted at the score angle.
    The back edge is shifted toward -d, so both ends lean back from the edge."""
    w = G.BLADE["w"]
    sh = w / math.tan(math.radians(G.BLADE["score_deg"]))
    e0 = p0
    e1 = (p0[0] + length * d[0], p0[1] + length * d[1])
    b1 = (e1[0] + w * n[0] - sh * d[0], e1[1] + w * n[1] - sh * d[1])
    b0 = (e0[0] + w * n[0] - sh * d[0], e0[1] + w * n[1] - sh * d[1])
    return [e0, e1, b1, b0]


BLADE_LEN = G.BLADE["seg"] * G.BLADE["n"]


def fixed_blade_pts():
    """Fixed blade on the deck: edge along the +y side of the slot, front corner where the V starts."""
    x1 = G.THROAT_X + G.SLOT_LEN
    return blade_pts((x1 - BLADE_LEN, G.FIXED_EDGE_Y), (1.0, 0.0), (0.0, 1.0), BLADE_LEN)


def lever_frame_vectors():
    a = math.radians(G.EDGE_SLICE)
    d = (math.cos(a), math.sin(a))
    n = (math.sin(a), -math.cos(a))          # toward the back of the edge
    return d, n


def moving_blade_pts():
    d, n = lever_frame_vectors()
    s0, _ = G.EDGE_S
    p0 = (G.EDGE_R + s0 * d[0], s0 * d[1])
    return blade_pts(p0, d, n, BLADE_LEN)


KEEPER_SCREWS = [(62.0, 25.5), (90.0, 25.5)]
BASKET_SCREWS = [(48.0, 50.5), (48.0, -50.5), (100.0, 50.5), (100.0, -50.5), (41.2, 25.0), (41.2, -25.0)]
CAMERA_SCREWS = [(40.0, 22.0), (48.0, 22.0)]
POST_Y = 46.0
POST_SCREWS = [(88.0, POST_Y), (88.0, -POST_Y)]
SERVO_BRACKET_SCREWS = [(20.0, -61.0), (36.6, -61.0), (20.0, -2.3), (36.6, -2.3)]
SERVO_TOP_Z = -2.5      # cutter servo case top, in a pocket under the deck; its output boss passes the deck


def servo_footprint():
    s = G.SERVO
    x, y = G.SERVO_SHAFT
    cy = y + (s["l"] / 2 - s["shaft_off"])
    return x, cy


def deck():
    """Printed PETG, 6 mm, top face down on the bed. Carries everything; its V and slot guide the pedicel.
    No screw heads stand above the top face inside the blade-lever's sweep: screws come from below."""
    xm = x_mouth()
    t = G.DECK_T
    outline = hull_face([(0.0, 0.0, 36.0), (xm + 4, 31.0, 6.0), (xm + 4, -31.0, 6.0)],
                        polys=[[(10, -62), (46, -62), (46, -53), (10, -53)],
                               [(36, 53), (120, 53), (120, -53), (36, -53)]])
    p = Pos(0, 0, -t) * extrude(outline, t)
    p -= Pos(0, 0, -t - 1) * extrude(v_outline(), t + 2)
    # under the fixed blade the slot edge steps back 0.6 mm, so the steel edge is the shear edge
    p -= box(G.THROAT_X - 4.0, G.THROAT_X + G.SLOT_LEN + 0.5, 0.0, G.FIXED_EDGE_Y + 0.6, -t - 1, 1)
    p -= Pos(0, 0, -G.BLADE["t"]) * extrude(poly_face(fixed_blade_pts()), G.BLADE["t"] + 1)   # fixed blade pocket
    p -= cyl(10.1, -t - 1, 1)                                                  # flange spigot
    for x, y in polar(4, 24.0, 0):
        p -= cyl(2.25, -t - 1, 1, x, y) + cyl(4.2, -t - 1, -t + 2.0, x, y)     # M4 x 12 up into the flange inserts
    p -= cyl(2.0, -t - 1, 1, 20 * math.cos(math.radians(45)), 20 * math.sin(math.radians(45)))
    p -= cyl(G.INSERT_M4["hole"] / 2 - 0.4, -t - 1, 1, *G.PIVOT)              # blade pivot: M4 heat-set from below
    p -= cyl(6.5, -t - 1, 1, *G.SERVO_SHAFT)                                  # cutter servo output boss
    sx, sy = servo_footprint()
    p -= box(sx - 10.2, sx + 10.2, sy - 20.7, sy + 20.7, -t - 1, SERVO_TOP_Z)   # servo case pocket
    for x, y in KEEPER_SCREWS + CAMERA_SCREWS + POST_SCREWS:
        p -= cyl(1.25, -t + 1.0, 1, x, y)                                     # M3 self-tap from above, blind
    for x, y in BASKET_SCREWS + SERVO_BRACKET_SCREWS:
        p -= cyl(1.25, -t - 1, -1.0, x, y)                                    # M3 self-tap from below, blind
    p -= cyl(4.0, -t - 1, 1, 30.0, 30.0)                                      # tool cables down
    return p


def fixed_blade():
    return Pos(0, 0, -G.BLADE["t"]) * extrude(poly_face(fixed_blade_pts()), G.BLADE["t"])


def keeper():
    """Printed PETG strip that clamps the fixed blade's back half to the deck (2 x M3 self-tap)."""
    p = box(58.0, 94.0, 12.5, 29.0, 0.0, 3.0)
    for x, y in KEEPER_SCREWS:
        p -= cyl(1.7, -1, 4, x, y)
    return p


def blade_lever_parts():
    """Printed blade-lever at angle 0, pivot at the origin (lever frame), and the blade in its underside pocket."""
    d, n = lever_frame_vectors()
    pts = moving_blade_pts()
    e0, e1, b1, b0 = pts
    ex = G.BLADE_EXPOSED

    def mv(p, k):
        return (p[0] + k * n[0], p[1] + k * n[1])
    backing = [mv(e0, ex), mv(e1, ex), mv(b1, 2.0), mv(b0, 2.0)]
    body = hull_face([(0, 0, 6.5)], polys=[backing])
    ph = math.radians(G.LEVER_IN_PHASE)
    q = (G.LEVER_IN_R * math.cos(ph), G.LEVER_IN_R * math.sin(ph))
    arm = hull_face([(0, 0, 6.0), (q[0], q[1], 5.0)])
    lever = extrude(body, G.LEVER_T) + extrude(arm, G.LEVER_T)
    blade_face = poly_face(pts)
    lever -= Pos(0, 0, -0.1) * extrude(blade_face, G.BLADE["t"] + 0.1)        # blade pocket on the underside
    mid = [((e0[0] + b0[0]) / 2 * 0.5 + (e1[0] + b1[0]) / 2 * 0.5 + (k - 0.5) * 16 * d[0],
            (e0[1] + b0[1]) / 2 * 0.5 + (e1[1] + b1[1]) / 2 * 0.5 + (k - 0.5) * 16 * d[1]) for k in (0, 1)]
    for x, y in mid:
        lever -= cyl(3.1, G.BLADE["t"] - 0.01, G.BLADE["t"] + 2.2, x, y)       # 6 x 2 magnets hold the blade
    lever -= cyl(2.1, -1, G.LEVER_T + 1) + cyl(1.6, -1, G.LEVER_T + 1, *q)
    blade = extrude(blade_face, G.BLADE["t"])
    return lever, blade, q


def lever_at(theta):
    lever, blade, _ = blade_lever_parts()
    t = Pos(G.PIVOT[0], G.PIVOT[1], 0) * Rot(0, 0, theta)
    return t * lever, t * blade


LINK_Z = (G.LEVER_T + 1.0, G.LEVER_T + 5.0)
HORN_Z = (3.5, 6.0)


def link_at(theta):
    alpha, q = G.servo_angle_for_lever(theta)
    h = (G.SERVO_SHAFT[0] + G.HORN_R * math.cos(math.radians(alpha)),
         G.SERVO_SHAFT[1] + G.HORN_R * math.sin(math.radians(alpha)))
    z0, z1 = LINK_Z
    p = Pos(0, 0, z0) * extrude(hull_face([(h[0], h[1], 4.0), (q[0], q[1], 4.0)]), z1 - z0)
    p -= cyl(1.6, z0 - 1, z1 + 1, *h) + cyl(1.6, z0 - 1, z1 + 1, *q)
    horn = Pos(0, 0, HORN_Z[0]) * extrude(hull_face([(G.SERVO_SHAFT[0], G.SERVO_SHAFT[1], 5.0), (h[0], h[1], 3.5)]),
                                          HORN_Z[1] - HORN_Z[0])
    horn += cyl(4.5, SERVO_TOP_Z + 3.0, HORN_Z[0], *G.SERVO_SHAFT)
    return p, horn


def servo_body(x, y, top_z, yaw=0.0):
    """Standard-size servo (MG996R / DS3225), output shaft at (x, y), case top at top_z, body below."""
    s = G.SERVO
    off = s["l"] / 2 - s["shaft_off"]
    b = box(-s["l"] / 2, s["l"] / 2, -s["w"] / 2, s["w"] / 2, -s["h"], 0)
    b += box(-s["tab_l"] / 2, s["tab_l"] / 2, -s["w"] / 2, s["w"] / 2, -s["h"] + s["tab_z"], -s["h"] + s["tab_z"] + s["tab_t"])
    b = Pos(off, 0, 0) * b
    b += cyl(6.0, 0, 3.0) + cyl(3.0, 3.0, 5.5)
    return Pos(x, y, top_z) * Rot(0, 0, yaw) * b


def cutter_servo_bracket():
    """Printed PETG. Hangs the cutter servo under the deck by its tabs; 4 x M3 self-tap up into the deck."""
    s = G.SERVO
    x, cy = servo_footprint()
    z_tab = SERVO_TOP_Z - s["h"] + s["tab_z"]
    outer = box(x - 11.35, x + 11.3, cy - 31.5, cy + 31.5, z_tab - 4.0, -G.DECK_T)
    inner = box(x - s["w"] / 2 - 0.3, x + s["w"] / 2 + 0.3, cy - s["l"] / 2 - 0.3, cy + s["l"] / 2 + 0.3, z_tab - 5, 0)
    tabs = box(x - s["w"] / 2 - 0.3, x + s["w"] / 2 + 0.3, cy - s["tab_l"] / 2 - 0.3, cy + s["tab_l"] / 2 + 0.3,
               z_tab, z_tab + s["tab_t"] + 0.3)
    p = outer - inner - tabs
    p -= box(x - 9, x + 9, cy - 26, cy + 26, z_tab + s["tab_t"] + 0.3, -G.DECK_T - 2.5)
    for sx, sy in SERVO_BRACKET_SCREWS:
        p -= cyl(1.7, z_tab - 10, -G.DECK_T + 1, sx, sy)
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
    for xa, xb in ((x0 + 10, x0 + 40), (x0 + 50, x0 + 80)):     # side windows; lower 35 mm solid
        p -= box(xa, xb, -hw - 5, hw + 5, -90.0, -20.0)
    for x, y in BASKET_SCREWS:
        p -= cyl(1.7, zt - 6, zt + 1, x, y)
    for xk in (x0 + 4, x1 - 4):                                  # hinge knuckles for the door
        p += Pos(xk, G.DOOR_HINGE[0], G.DOOR_HINGE[1]) * Rot(0, 90, 0) * Cylinder(4.0, 8.0)
    p -= Pos(x0 - 10, G.DOOR_HINGE[0], G.DOOR_HINGE[1]) * Rot(0, 90, 0) * Cylinder(
        1.6, 200, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(x0 - 10, G.DOOR_HINGE[0], G.DOOR_HINGE[1]) * Rot(0, 90, 0) * Cylinder(
        7.5, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))                 # door servo spline
    p -= box(x0 - w - 1, x0 + 1, -6, 6, -100, -92)                              # fruit sensor window
    return p


def door(open_deg=0.0):
    """Printed PETG door plate, 10 mm EVA pad, 10 mm EVA lip strip glued on the front edge."""
    x0, x1 = G.BASKET_IN_X
    hw = G.BASKET_IN_HALF_W - 0.5
    yh, zh = G.DOOR_HINGE
    plate_top = G.PAD_TOP_Z - 10.0
    d = box(x0 + 0.5, x1 - 0.5, -hw, hw - 4, plate_top - 3.0, plate_top)
    d += box(x0 + 0.5, x1 - 0.5, hw - 6, yh, zh, plate_top)
    d += Pos(x0 + 0.5, yh, zh) * Rot(0, 90, 0) * Cylinder(2.9, x1 - x0 - 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    pad = box(x0 + 2, x1 - 7.0, -hw + 2, hw - 6, plate_top, G.PAD_TOP_Z)
    lip = box(x1 - 6.5, x1 - 0.5, -hw, hw - 4, plate_top, G.LIP_TOP_Z)
    d -= Pos(x0 - 5, yh, zh) * Rot(0, 90, 0) * Cylinder(1.7, 200, align=(Align.CENTER, Align.CENTER, Align.MIN))
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
    zc = zh + off
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
    b = servo_body(0, 0, 0)
    return Pos(x0 - 2.0, yh, zh) * Rot(0, 90, 0) * Rot(0, 0, 180) * b


FORK_PIVOT = (56.0, -G.DECK_T - 4.0)


def feeler_fork(lift=0.0):
    """Printed PETG fork under the deck. The fruit shoulders lift it; its tail presses a micro switch."""
    xp, zp = FORK_PIVOT
    za, zt = -G.DECK_T - 3.0, -G.DECK_T - 0.5
    p = box(49.5, 60.0, -20.0, 20.0, za - 4.5, zt)
    for y0, y1 in ((9.0, 17.0), (-17.0, -9.0)):
        p += box(60.0, 112.0, y0, y1, za, za + 2.5)
    p += box(49.5, 53.5, -22.0, -20.0, -23.0, za - 4.5)
    p += box(49.5, 53.5, -36.0, -20.0, -23.0, -19.0)
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
    """Printed posts each side of the throat: 38 kHz IR LED one side, TSOP38238 the other."""
    out = None
    for y in (-POST_Y, POST_Y):
        p = box(84.0, 92.0, y - 3.0, y + 3.0, 0.0, 14.0)
        p -= Pos(88.0, y, 10.0) * Rot(90, 0, 0) * Cylinder(1.6, 10)
        p -= cyl(1.7, -1, 6, 88.0, y)
        out = p if out is None else out + p
    return out


def camera_and_bracket():
    c = G.CAMERA
    x_face, y_c, z_c = 72.0, 5.0, 36.0
    place = Pos(x_face, y_c, z_c) * Rot(0, c["tilt"], 0)
    cam = place * box(-c["d"], 0, -c["w"] / 2, c["w"] / 2, -c["h"] / 2, c["h"] / 2)
    br = box(36.0, 52.0, 14.0, 32.0, 0.0, 4.0) + box(36.0, 52.0, 27.5, 32.0, 0.0, 30.0)
    br += place * box(-c["d"] - 3.0, -c["d"], 0.0, c["w"] / 2 + 6.0, -c["h"] / 2, c["h"] / 2)
    br += place * box(-c["d"] - 3.0, 0.0, c["w"] / 2 + 0.3, c["w"] / 2 + 6.0, -c["h"] / 2, c["h"] / 2)
    for x, y in CAMERA_SCREWS:
        br -= cyl(1.7, -1, 6, x, y)
    return cam, br


def fruit_sensor():
    x0 = G.BASKET_IN_X[0] - G.BASKET_WALL
    return box(x0 - 6.0, x0 - 2.4, -10, 10, -104, -88)


def flange_stub():
    """Reference only: the J3 tool flange the deck bolts to (tool frame)."""
    return cyl(30.0, 0, G.J3_FLANGE[1] - G.J3_FLANGE[0])


# ============================================================================ assembly
PARTS = []


def add(pid, name, solid, frame, material, process, note="", qty=1, export=True):
    PARTS.append(dict(id=pid, name=name, solid=solid, frame=frame, material=material, process=process,
                      qty=qty, note=note, export=export))
    return solid


def build_parts(lever_theta=G.LEVER_OPEN, door_open=0.0):
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
    add("T-01", "Cutter deck", deck(), "tool", "PETG", "print top face down, 6 walls, 50% gyroid")
    add("T-02", "Fixed-blade keeper", keeper(), "tool", "PETG", "print flat")
    add("BUY-14", "Fixed blade: 18 mm snap-off blade, 3 segments", fixed_blade(), "tool", "steel", "buy, snap to length",
        export=False)
    lever, mblade = lever_at(lever_theta)
    add("T-03", "Blade-lever", lever, "tool", "PETG100", "print blade pocket up, 100%; 2 x 6x2 magnets")
    add("BUY-15", "Moving blade: 18 mm snap-off blade, 3 segments", mblade, "tool", "steel", "buy, snap to length",
        export=False)
    lk, horn = link_at(lever_theta)
    add("T-04", "Link 40 mm", lk, "tool", "PETG100", "print flat, 100%")
    add("BUY-08", "Servo horn (stock single arm)", horn, "tool", "bought", "comes with the servo", export=False)
    add("BUY-09", "Cutter servo DS3225MG (or MG996R on hand)", servo_body(*G.SERVO_SHAFT, SERVO_TOP_Z, yaw=90.0), "tool",
        "bought", "DS3225MG recommended; MG996R fits the same pocket", export=False)
    add("T-05", "Cutter servo bracket", cutter_servo_bracket(), "tool", "PETG", "print")
    add("T-06", "Basket", basket(), "tool", "PETG", "print rear wall down, 4 walls")
    d, pad, lip = door(door_open)
    add("T-07", "Door", d, "tool", "PETG", "print")
    add("T-08", "Door pad 10 mm", pad, "tool", "foam", "cut from 10 mm EVA sheet", export=False)
    add("T-09", "Door lip 10 mm", lip, "tool", "foam", "cut from 10 mm EVA sheet, glue on", export=False)
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
