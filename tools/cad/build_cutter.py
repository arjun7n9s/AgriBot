"""Build the CAD for the stem cutter (cutter v1) on the stock HowToMechatronics SCARA.

A bought pruner blade set (hook blade fixed, moving blade driven) is closed by one servo through a
four-bar: servo horn -> printed link -> printed lever pinned to the moving blade. The tool bolts under
the stock J3 coupler. Numbers and tags: cutter_geometry.py. Forces: cutter_calcs.py. Checks: check_cutter.py.

    .venv-cad/Scripts/python tools/cad/build_cutter.py

Writes cad/cutter/:
    cutter_open.step / cutter_closed.step   the whole tool, tool frame, blades open / closed
    cutter_on_stock_arm.step                the tool under the stock J3 coupler, stock STEP frame (open)
    step/<part>.step, stl/<part>.stl        each printed part, in the tool frame (as assembled)
    parts.csv                               part list with material, quantity and mass
"""
import csv
import math
import shutil
import sys
from pathlib import Path

from build123d import (Align, BuildSketch, Box, Circle, Color, Compound, Cylinder, Locations, Polygon, Pos, Rot,
                       Sphere, export_gltf, export_step, export_stl, extrude, make_hull, offset)

sys.path.insert(0, str(Path(__file__).parent))
import cutter_calcs as C  # noqa: E402
import cutter_geometry as G  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "cad" / "cutter"

# PETG 1.27 g/cm3; printed parts about 60% solid at 5-6 walls and 35-40% gyroid, 100% for the lever and link (assumed)
DENSITY = {"PETG": 1.27 * 0.6, "PETG100": 1.27, "steel": 7.85}
# Preview colours, matched to looks in render_cutter.py
COLOR = {"petg": Color(0.15, 0.45, 0.25), "stainless": Color(0.82, 0.83, 0.85), "steel": Color(0.55, 0.57, 0.60),
         "bought": Color(0.2, 0.2, 0.22), "fruit": Color(0.85, 0.12, 0.08), "stem": Color(0.30, 0.48, 0.18)}

# Stock assembly placement of the tool frame (measured from the stock STEP): J3 axis, coupler underside, and
# the Arm 2 direction (J2 -> J3) so the tool points straight out along Arm 2.
J3_STEP = (367.6, -19.5, 152.6)
J2_STEP = (260.9, 65.6)
ARM2_YAW = math.degrees(math.atan2(J3_STEP[1] - J2_STEP[1], J3_STEP[0] - J2_STEP[0]))


# ============================================================================ helpers
def cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN))


def hull_face(circles, pts=()):
    with BuildSketch() as s:
        for x, y, r in circles:
            with Locations((x, y)):
                Circle(r)
        if pts:
            Polygon(*pts, align=None)
        make_hull()
    return s.sketch


def poly_face(pts):
    with BuildSketch() as s:
        Polygon(*pts, align=None)
    return s.sketch


def slab(face, z0, z1):
    return Pos(0, 0, z0) * extrude(face, z1 - z0)


def along(p0, p1, r, z0=None):
    """Cylinder of radius r from 3D point p0 to p1."""
    dx, dy, dz = (p1[i] - p0[i] for i in range(3))
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.atan2(math.hypot(dx, dy), dz))
    return Pos(*p0) * Rot(0, 0, yaw) * Rot(0, pitch, 0) * Cylinder(r, length, align=(Align.CENTER, Align.CENTER, Align.MIN))


def rot_about(pt, centre, deg):
    a = math.radians(deg)
    x, y = pt[0] - centre[0], pt[1] - centre[1]
    return centre[0] + x * math.cos(a) - y * math.sin(a), centre[1] + x * math.sin(a) + y * math.cos(a)


def union(parts):
    out = None
    for p in parts:
        out = p if out is None else out + p
    return out


# ============================================================================ the linkage state
def state(theta):
    """Lever pin q and horn pin h (tool xy) and horn angle at blade rotation theta (deg)."""
    q = C.lever_pin(theta)
    h = C.horn_pin(q)
    psi = math.degrees(math.atan2(h[1] - G.SERVO_SHAFT[1], h[0] - G.SERVO_SHAFT[0]))
    return q, h, psi


def drive_dowel_xy(theta):
    return G.photo_to_tool_xy(G.DRIVE_HOLE, G.CLOSE_SIGN * theta)


# ============================================================================ bought parts
def fixed_blade():
    pts = [G.photo_to_tool_xy(p) for p in G.FIXED_OUTLINE]
    b = slab(poly_face(pts), *G.FIXED_Z)
    tail = G.photo_to_tool_xy(G.TAIL_HOLE)
    b -= cyl(G.PIVOT_HOLE_D / 2, G.FIXED_Z[0] - 1, G.FIXED_Z[1] + 1, *G.PIVOT)
    b -= cyl(G.TAIL_HOLE_D / 2, G.FIXED_Z[0] - 1, G.FIXED_Z[1] + 1, *tail)
    return b


def moving_blade(theta):
    rot = G.CLOSE_SIGN * theta
    pts = [G.photo_to_tool_xy(p, rot) for p in G.MOVING_OUTLINE]
    b = slab(poly_face(pts), *G.MOVING_Z)
    b -= cyl(G.PIVOT_HOLE_D / 2, G.MOVING_Z[0] - 1, G.MOVING_Z[1] + 1, *G.PIVOT)
    b -= cyl(G.DRIVE_HOLE_D / 2, G.MOVING_Z[0] - 1, G.MOVING_Z[1] + 1, *drive_dowel_xy(theta))
    return b


def pivot_hardware():
    """Steel bush; M8 button-head bolt from below (low under the blades, where the fruit hangs); washer, spring
    washer and nyloc on top of the deck."""
    px, py = G.PIVOT
    top = G.DECK_Z + G.DECK_T
    bot = G.LEVER_Z[0]
    w, sw, nut, head = G.WASHER_T, G.SPRING_WASHER_T, G.NUT_H, G.BUTTON_HEAD
    return {
        "bush": cyl(G.BUSH["od"] / 2, G.FIXED_Z[1] - G.BUSH["len"], G.FIXED_Z[1], px, py)
        - cyl(G.BUSH["id"] / 2, bot - 1, top + 1, px, py),
        "washer_bot": cyl(8.0, bot - w, bot, px, py) - cyl(4.3, bot - 5, bot + 1, px, py),
        "bolt_head": cyl(head["d"] / 2, bot - w - head["h"], bot - w, px, py),
        "bolt_shank": cyl(4.0, bot - w, top + w + sw + nut + 1.0, px, py),
        "washer_top": cyl(8.0, top, top + w, px, py) - cyl(4.3, top - 1, top + 5, px, py),
        "spring_washer": cyl(7.4, top + w, top + w + sw, px, py) - cyl(4.2, top, top + 9, px, py),
        "nut": cyl(7.5, top + w + sw, top + w + sw + nut, px, py) - cyl(4.0, top, top + 30, px, py),
    }


def tail_dowel():
    x, y = G.photo_to_tool_xy(G.TAIL_HOLE)
    return cyl(G.TAIL_HOLE_D / 2 - 0.05, G.FIXED_Z[0] + 0.2, G.FIXED_Z[0] + 0.2 + TAIL_DOWEL_L, x, y)


TAIL_DOWEL_L = 10.0       # Ø5 x 10 dowel: through the deck (7) and 2.8 into the hook blade's tail hole
DRIVE_DOWEL_L = 8.0       # Ø5 x 8 dowel: through the lever (6 thick, 0.8 short of its underside) and 2.8 into the blade


def drive_dowel(theta):
    x, y = drive_dowel_xy(theta)
    z1 = G.MOVING_Z[1] - 0.2
    return cyl(G.DRIVE_HOLE_D / 2 - 0.05, z1 - DRIVE_DOWEL_L, z1, x, y)


def servo_case():
    """MG996R/DS3225MG case, shaft pointing down at SERVO_SHAFT, body pointing away from the pivot."""
    s = G.SERVO
    off = s["l"] / 2 - s["shaft_off"]
    f = G.SERVO_FACE_Z
    body = box(off - s["l"] / 2, off + s["l"] / 2, -s["w"] / 2, s["w"] / 2, f, f + s["h"])
    tab_z0 = f + s["h"] - s["tab_z"] - s["tab_t"]
    body += box(off - s["tab_l"] / 2, off + s["tab_l"] / 2, -s["w"] / 2, s["w"] / 2, tab_z0, tab_z0 + s["tab_t"])
    body += cyl(6.0, f - 1.5, f) + cyl(2.9, G.HORN_Z[1], f - 1.5)          # output boss and spline
    return Pos(*G.SERVO_SHAFT, 0) * Rot(0, 0, C.servo_yaw(G.SERVO_SHAFT)) * body


def servo_tabs_z():
    s = G.SERVO
    tab_z0 = G.SERVO_FACE_Z + s["h"] - s["tab_z"] - s["tab_t"]
    return tab_z0, tab_z0 + s["tab_t"]


def horn(psi):
    """Stock single-arm horn: hub on the spline, arm out to the link pin at HORN_R."""
    hub = cyl(4.5, G.HORN_Z[0], G.SERVO_FACE_Z - 1.6)
    arm = slab(hull_face([(0, 0, 5.0), (G.HORN_R + 0.0, 0, 3.2)]), *G.HORN_Z)
    h = hub + arm - cyl(2.95, G.HORN_Z[1] - 0.5, G.SERVO_FACE_Z, 0, 0)               # spline bore
    h -= cyl(1.6, G.HORN_Z[0] - 1, G.HORN_Z[1] + 1, G.HORN_R, 0)                     # stock hole drilled to 3.2 mm
    return Pos(*G.SERVO_SHAFT, 0) * Rot(0, 0, psi) * h


def pin_hardware(theta):
    """M3 bolts and nylocs at the two link pins."""
    q, h, _ = state(theta)
    parts = []
    # horn -> link: head on the horn, nyloc under the link
    parts.append(cyl(2.75, G.HORN_Z[1], G.HORN_Z[1] + 2.0, *h))
    parts.append(cyl(1.5, G.LINK_Z[0] - 2.4, G.HORN_Z[1], *h))
    parts.append(cyl(3.2, G.LINK_Z[0] - 2.4, G.LINK_Z[0], *h))
    # link -> lever: head on the link, nyloc under the lever
    parts.append(cyl(2.75, G.LINK_Z[1], G.LINK_Z[1] + 2.0, *q))
    parts.append(cyl(1.5, G.LEVER_Z[0] - 2.4, G.LINK_Z[1], *q))
    parts.append(cyl(3.2, G.LEVER_Z[0] - 2.4, G.LEVER_Z[0], *q))
    return union(parts)


def ir_pair():
    a, b = beam_ends()
    led = along(beam_point(a, -6.0), a, G.IR_D / 2)
    rx = along(beam_point(b, -6.0), b, G.IR_D / 2)
    return led, rx


# ============================================================================ printed parts
def lever(theta):
    q, _, _ = state(theta)
    d = drive_dowel_xy(theta)
    f = hull_face([(G.PIVOT[0], G.PIVOT[1], 12.0), (d[0], d[1], 6.0), (q[0], q[1], 5.5)])
    p = slab(f, *G.LEVER_Z)
    p -= cyl(G.BUSH["od"] / 2 + 0.1, G.LEVER_Z[0] - 1, G.LEVER_Z[1] + 1, *G.PIVOT)
    p -= cyl(G.DRIVE_HOLE_D / 2 - 0.05, G.LEVER_Z[0] - 1, G.LEVER_Z[1] + 1, *d)     # press fit for the dowel
    p -= cyl(1.6, G.LEVER_Z[0] - 1, G.LEVER_Z[1] + 1, *q)
    return p


def link(theta):
    q, h, _ = state(theta)
    p = slab(hull_face([(h[0], h[1], 4.5), (q[0], q[1], 4.5)]), *G.LINK_Z)
    p -= cyl(1.6, G.LINK_Z[0] - 1, G.LINK_Z[1] + 1, *h)
    p -= cyl(1.6, G.LINK_Z[0] - 1, G.LINK_Z[1] + 1, *q)
    return p


TOWER_WALL = 2.6
PLATE_R = G.SKIRT_ID / 2 + G.SKIRT_W              # 42.2
BODY_TOP_Z = -G.PLATE_T
FLANGE_T = 5.0                                     # body top flange under the plate
INSERT_HOLE_D = 4.0                                # M3 x 5.7 heat-set insert (in the plate)
BODY_SCREW_R = 36.0


def servo_rect(grow, length=None):
    """Servo footprint (case or tabs) grown by `grow`, as tool-frame points."""
    s = G.SERVO
    off = s["l"] / 2 - s["shaft_off"]
    ln = s["l"] if length is None else length
    yaw = math.radians(C.servo_yaw(G.SERVO_SHAFT))
    pts = []
    for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        lu = off + u * (ln / 2 + grow)
        lv = v * (s["w"] / 2 + grow)
        pts.append((G.SERVO_SHAFT[0] + lu * math.cos(yaw) - lv * math.sin(yaw),
                    G.SERVO_SHAFT[1] + lu * math.sin(yaw) + lv * math.cos(yaw)))
    return pts


def tower_face():
    pts = servo_rect(TOWER_WALL + 0.3, G.SERVO["tab_l"])
    return offset(poly_face(pts), 0.01)


def jaw_polygon():
    """Open jaw region: hook cutting edge (crotch -> tip), then the open moving edge back to the crotch."""
    hook = [p for p in reversed(G.FIXED_EDGE) if not C.inside(p, G.MOVING_OUTLINE)]      # crotch -> tip
    mov = [p for p in reversed(G.MOVING_EDGE) if not C.inside(p, G.FIXED_OUTLINE)]       # tip -> crotch
    return [G.photo_to_tool_xy(p) for p in hook + mov]


def beam_point(p, dist):
    """Point `dist` along the beam from p, toward the stem."""
    sx, sy = G.STEM_XY
    dx, dy = sx - p[0], sy - p[1]
    n = math.hypot(dx, dy)
    return (p[0] + dist * dx / n, p[1] + dist * dy / n, p[2])


def beam_dir():
    """Beam across the throat, square to the pivot -> stem line."""
    vx, vy = G.STEM_XY[0] - G.PIVOT[0], G.STEM_XY[1] - G.PIVOT[1]
    n = math.hypot(vx, vy)
    return -vy / n, vx / n


def beam_ends():
    ux, uy = beam_dir()
    half = 12.0                    # LED and receiver faces sit 12 mm either side of the stem, in the guide arms
    sx, sy = G.STEM_XY
    return (sx + half * ux, sy + half * uy, G.BEAM_Z), (sx - half * ux, sy - half * uy, G.BEAM_Z)


def guide_arms_face():
    jaw = poly_face(jaw_polygon())
    band = offset(jaw, G.GUIDE_CLEAR + 6.0) - offset(jaw, G.GUIDE_CLEAR)
    # open the front: drop everything beyond the line through the two tips
    hook_tip = G.photo_to_tool_xy(G.FIXED_EDGE[0])
    mov_tip = G.photo_to_tool_xy(G.MOVING_EDGE[-1])
    mx, my = (hook_tip[0] + mov_tip[0]) / 2, (hook_tip[1] + mov_tip[1]) / 2
    nx, ny = mx - G.PIVOT[0], my - G.PIVOT[1]
    n = math.hypot(nx, ny)
    nx, ny = nx / n, ny / n
    tx, ty = -ny, nx
    far = 200.0
    cut = poly_face([(mx + tx * far - nx * 4, my + ty * far - ny * 4), (mx - tx * far - nx * 4, my - ty * far - ny * 4),
                     (mx - tx * far + nx * far, my - ty * far + ny * far), (mx + tx * far + nx * far, my + ty * far + ny * far)])
    return band - cut


def deck_face():
    tail = G.photo_to_tool_xy(G.TAIL_HOLE)
    tw = servo_rect(TOWER_WALL + 0.3, G.SERVO["tab_l"])
    front = sorted(tw, key=lambda p: -(p[0] * (G.PIVOT[0] - G.SERVO_SHAFT[0]) + p[1] * (G.PIVOT[1] - G.SERVO_SHAFT[1])))[:2]
    return hull_face([(G.PIVOT[0], G.PIVOT[1], 15.0), (tail[0], tail[1], 7.5)], pts=front)


def body():
    z0 = G.DECK_Z
    tower = slab(tower_face(), z0, BODY_TOP_Z)
    deck = slab(deck_face(), z0, z0 + G.DECK_T)
    arms = slab(guide_arms_face(), z0, z0 + G.DECK_T)
    # spine on top of the deck, tower -> pivot, sloping down toward the pivot (prints without support, deck down)
    sx, sy = G.SERVO_SHAFT
    px, py = G.PIVOT
    ux, uy = px - sx, py - sy
    n = math.hypot(ux, uy)
    ux, uy = ux / n, uy / n
    start = (sx + ux * 12.0, sy + uy * 12.0)
    end = (px - ux * 13.0, py - uy * 13.0)
    L = math.hypot(end[0] - start[0], end[1] - start[1])
    with BuildSketch() as s:
        Polygon((0, 0), (L, 0), (L, 4.0), (0, BODY_TOP_Z - FLANGE_T - (z0 + G.DECK_T)), align=None)
    spine = Pos(start[0], start[1], z0 + G.DECK_T) * Rot(0, 0, math.degrees(math.atan2(uy, ux))) * Rot(90, 0, 0) * \
        Pos(0, 0, -3.0) * extrude(s.sketch, 6.0)
    flange = slab(hull_face([(0, 0, PLATE_R - 0.2)]), BODY_TOP_Z - FLANGE_T, BODY_TOP_Z)
    b = tower + deck + arms + spine + flange

    # servo: tab slot from the top down to the tab ledge, body pocket to the output face, then shaft and horn clearance
    tz0, _ = servo_tabs_z()
    b -= slab(poly_face(servo_rect(0.3, G.SERVO["tab_l"])), tz0, BODY_TOP_Z + 1)
    b -= slab(poly_face(servo_rect(0.3)), G.SERVO_FACE_Z - 0.5, BODY_TOP_Z + 1)
    sweep = [hull_face([(G.SERVO_SHAFT[0], G.SERVO_SHAFT[1], 7.5), (h[0], h[1], 6.0)])
             for h in (state(t)[1] for t in range(0, int(G.CLOSED_DEG) + 1, 2))]
    b -= slab(union(sweep), z0 - 1, G.SERVO_FACE_Z)                      # horn, its pin head and the spline boss
    links = [hull_face([(h[0], h[1], 5.5), (q[0], q[1], 5.5)])
             for q, h, _ in (state(t) for t in range(0, int(G.CLOSED_DEG) + 1, 2))]
    b -= slab(union(links), z0 - 1, z0 + 1.0)                            # 1.5 mm clear over the link's path
    # servo screw holes in the ledge (self-tapping, as supplied with the servo), 48 mm apart along the case
    s = G.SERVO
    off = s["l"] / 2 - s["shaft_off"]
    yaw = math.radians(C.servo_yaw(G.SERVO_SHAFT))
    for u in (off - 24.0, off + 24.0):
        for v in (-5.0, 5.0):
            x = G.SERVO_SHAFT[0] + u * math.cos(yaw) - v * math.sin(yaw)
            y = G.SERVO_SHAFT[1] + u * math.sin(yaw) + v * math.cos(yaw)
            b -= cyl(1.0, tz0 - 8, tz0 + 1, x, y)
    # clearance for the link-lever pin head as it swings
    heads = [cyl(4.0, z0 - 1, G.LINK_Z[1] + 2.6, *state(t)[0]) for t in range(0, int(G.CLOSED_DEG) + 1, 2)]
    b -= union(heads)
    # pivot bolt, tail dowel (press), cable hole into the coupler bore
    b -= cyl(6.0, BODY_TOP_Z - FLANGE_T - 1, BODY_TOP_Z + 1)
    b -= cyl(4.2, z0 - 1, BODY_TOP_Z + 1, *G.PIVOT)
    tail = G.photo_to_tool_xy(G.TAIL_HOLE)
    b -= cyl(G.TAIL_HOLE_D / 2 - 0.05, z0 - 1, z0 + G.DECK_T + 1, *tail)
    # IR beam holes in the arms
    a, c = beam_ends()
    b -= along(beam_point(a, -10.0), beam_point(a, 9.0), G.IR_D / 2 + 0.1)
    b -= along(beam_point(c, -10.0), beam_point(c, 9.0), G.IR_D / 2 + 0.1)
    # body screws up into heat-set inserts in the plate
    for x, y in body_screw_points():
        b -= cyl(1.7, BODY_TOP_Z - FLANGE_T - 1, BODY_TOP_Z + 1, x, y)
    # flange bolt heads (counterbored in the plate) are covered: nothing to cut. Spare M3 holes for a future catcher:
    for x, y in catcher_points():
        b -= cyl(1.25, z0 - 1, z0 + 10.0, x, y)
    return b


def body_screw_points():
    """4 M3 screws through the body flange into the plate, clear of the tower (key access from below)."""
    tower = tower_face()
    deck = deck_face()
    picks = []
    for k in range(72):
        a = math.radians(k * 5.0)
        x, y = BODY_SCREW_R * math.cos(a), BODY_SCREW_R * math.sin(a)
        probe = hull_face([(x, y, 4.0)])
        if (probe & tower).area > 0.01 or (probe & deck).area > 0.01:
            continue
        picks.append((k * 5.0, x, y))
    # spread four of them as evenly as possible
    best = None
    for i in range(len(picks)):
        for j in range(i + 1, len(picks)):
            for k in range(j + 1, len(picks)):
                for m in range(k + 1, len(picks)):
                    angs = sorted(p[0] for p in (picks[i], picks[j], picks[k], picks[m]))
                    gaps = [(angs[(n + 1) % 4] - angs[n]) % 360 for n in range(4)]
                    score = min(gaps)
                    if best is None or score > best[0]:
                        best = (score, [(p[1], p[2]) for p in (picks[i], picks[j], picks[k], picks[m])])
    assert best and best[0] >= 40, "no room for four body screws clear of the tower"
    return best[1]


def catcher_points():
    """Two spare M3 holes in the deck top (pilot 2.5 mm) for a future fruit catcher."""
    tail = G.photo_to_tool_xy(G.TAIL_HOLE)
    sx, sy = G.SERVO_SHAFT
    return [((tail[0] + sx) / 2 + 8.0, (tail[1] + sy) / 2 + 8.0), ((tail[0] + sx) / 2 + 8.0, (tail[1] + sy) / 2 - 8.0)]


def plate():
    p = cyl(PLATE_R, -G.PLATE_T, 0.0)
    p += cyl(PLATE_R, 0.0, G.SKIRT_H) - cyl(G.SKIRT_ID / 2, -0.1, G.SKIRT_H + 1)       # skirt round the coupler flange
    p -= cyl(G.CABLE_BORE_D / 2 + 1.0, -G.PLATE_T - 1, 1)                                 # cables up into the J3 bore
    p -= cyl(G.PULLEY_SCREW_RECESS_D / 2, -1.0, 0.1)                                      # relief for the pulley screw heads
    for a in flange_bolt_angles():
        x, y = G.FLANGE_PCD / 2 * math.cos(math.radians(a)), G.FLANGE_PCD / 2 * math.sin(math.radians(a))
        p -= cyl(2.2, -G.PLATE_T - 1, 1, x, y)
        p -= cyl(4.2, -G.PLATE_T - 1, -G.PLATE_T + 4.2, x, y)                             # head counterbore from below
    for x, y in body_screw_points():
        p -= cyl(INSERT_HOLE_D / 2, -G.PLATE_T - 1, -G.PLATE_T + 6.5, x, y)
    return p


def flange_bolt_angles():
    """Five of the ten coupler holes, 72 deg apart (even load)."""
    return [G.FLANGE_HOLE_ANGLES[k] for k in range(0, 10, 2)]


# ============================================================================ assembly
PRINTED = {
    "C-01": ("Tool plate (bolts under the J3 coupler)", "PETG", plate),
    "C-02": ("Body: servo tower, blade deck, stem guide arms", "PETG", body),
    "C-03": ("Blade lever", "PETG100", lambda: lever(0.0)),
    "C-04": ("Link, horn to lever", "PETG100", lambda: link(0.0)),
}


def bought(theta):
    out = {"BUY-30a hook blade": fixed_blade(), "BUY-30b moving blade": moving_blade(theta),
           "BUY-33 drive dowel": drive_dowel(theta), "BUY-34 tail dowel": tail_dowel(),
           "BUY-35 servo": servo_case(), "BUY-35 horn": horn(state(theta)[2]),
           "BUY-37 link pins": pin_hardware(theta)}
    for k, v in pivot_hardware().items():
        out[f"BUY-31/32 pivot {k}"] = v
    led, rx = ir_pair()
    out["BUY-36 IR LED"] = led
    out["BUY-36 IR receiver"] = rx
    return out


def moving_printed(theta):
    return {"C-03": lever(theta), "C-04": link(theta)}


def assembly(theta, printed):
    parts = []
    for pid, shape in printed.items():
        if pid in ("C-03", "C-04"):
            continue
        shape.label = pid
        parts.append(shape)
    for pid, shape in moving_printed(theta).items():
        shape.label = pid
        parts.append(shape)
    for name, shape in bought(theta).items():
        shape.label = name
        parts.append(shape)
    return Compound(children=parts)


def preview(theta, printed, fruit):
    """Coloured assembly for the renders, with a reference stem and tomato (60 mm) seated in the throat."""
    parts = []

    def add(shape, look):
        shape.color = COLOR[look]
        parts.append(shape)

    add(printed["C-01"], "petg")
    add(printed["C-02"], "petg")
    for shape in moving_printed(theta).values():
        add(shape, "petg")
    for name, shape in bought(theta).items():
        look = "stainless" if "blade" in name else "steel" if ("dowel" in name or "pivot" in name or "pins" in name) else "bought"
        add(shape, look)
    if fruit:
        top = G.CUT_Z - G.CUT_ABOVE_FRUIT
        add(cyl(G.STEM_D / 2, top, 40.0, *G.STEM_XY), "stem")
        add(Pos(G.STEM_XY[0], G.STEM_XY[1], top - 30.0) * Sphere(30.0), "fruit")
    return Compound(children=parts)


def volume_g(shape, material):
    return shape.volume / 1000.0 * DENSITY[material]


def main():
    if OUT.exists():
        for sub in ("step", "stl"):
            shutil.rmtree(OUT / sub, ignore_errors=True)
    (OUT / "step").mkdir(parents=True, exist_ok=True)
    (OUT / "stl").mkdir(parents=True, exist_ok=True)

    printed = {}
    rows = []
    for pid, (name, mat, fn) in PRINTED.items():
        shape = fn()
        printed[pid] = shape
        export_step(shape, str(OUT / "step" / f"{pid.lower()}.step"))
        export_stl(shape, str(OUT / "stl" / f"{pid.lower()}.stl"), tolerance=0.02, angular_tolerance=0.1)
        bb = shape.bounding_box()
        rows.append(dict(id=pid, name=name, material=mat, qty=1, mass_g=round(volume_g(shape, mat)),
                         size_mm=f"{bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f}"))
        print(f"{pid}: {name}  {rows[-1]['size_mm']} mm  {rows[-1]['mass_g']} g")

    blades_g = round((fixed_blade().volume + moving_blade(0).volume) / 1000 * DENSITY["steel"])
    rows += [
        dict(id="BUY-30", name="Pruner replacement blade set, 70 mm, 16 mm pivot hole (hook + moving blade)",
             material="steel", qty=1, mass_g=blades_g, size_mm="70 long"),
        dict(id="BUY-31", name="Steel bush OD 16 x ID 8 x 11.9 mm (pivot sleeve)", material="steel", qty=1,
             mass_g=round(pivot_hardware()["bush"].volume / 1000 * 7.85), size_mm=""),
        dict(id="BUY-32", name="M8 x 35 button-head bolt (ISO 7380), 2 x M8 washer, M8 spring washer, M8 nyloc",
             material="steel", qty=1,
             mass_g=25, size_mm=""),
        dict(id="BUY-33", name="Dowel pin 5 x 8 mm (lever to moving blade)", material="steel", qty=1, mass_g=1, size_mm=""),
        dict(id="BUY-34", name="Dowel pin 5 x 10 mm (deck to hook blade tail)", material="steel", qty=1, mass_g=2, size_mm=""),
        dict(id="BUY-35", name="Servo MG996R (on hand); DS3225MG drops in for more force", material="", qty=1,
             mass_g=55, size_mm="40.7 x 19.7 x 37"),
        dict(id="BUY-36", name="3 mm IR LED + 3 mm IR phototransistor (throat beam)", material="", qty=1, mass_g=1, size_mm=""),
        dict(id="BUY-37", name="M3 x 12 + M3 x 16 bolts, 2 x M3 nyloc (link pins)", material="steel", qty=1, mass_g=3,
             size_mm=""),
        dict(id="BUY-38", name="4 x M3 heat-set insert (5.7 long) + 4 x M3 x 10 (body to plate)", material="", qty=1,
             mass_g=4, size_mm=""),
        dict(id="BUY-39", name="5 x M4 x 16 socket head + 5 x M4 nut (plate to J3 coupler)", material="steel", qty=1,
             mass_g=10, size_mm=""),
    ]
    with open(OUT / "parts.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["id", "name", "material", "qty", "mass_g", "size_mm"])
        w.writeheader()
        w.writerows(rows)
    total = sum(r["mass_g"] for r in rows)
    print(f"tool mass about {total} g (estimate)")

    (OUT / "preview").mkdir(exist_ok=True)
    for name, theta, fruit in (("open", 0.0, True), ("closed", G.CLOSED_DEG, False)):
        export_gltf(preview(theta, printed, fruit), str(OUT / "preview" / f"cutter_{name}.glb"), binary=True,
                    linear_deflection=0.05, angular_deflection=0.2)
    export_step(assembly(0.0, printed), str(OUT / "cutter_open.step"))
    export_step(assembly(G.CLOSED_DEG, printed), str(OUT / "cutter_closed.step"))
    on_arm = Pos(*J3_STEP) * Rot(0, 0, ARM2_YAW) * assembly(0.0, printed)
    export_step(on_arm, str(OUT / "cutter_on_stock_arm.step"))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
