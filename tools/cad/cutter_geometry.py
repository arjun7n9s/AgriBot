"""Shared numbers for the stem cutter on the stock HowToMechatronics SCARA (cutter v1).

Used by build_cutter.py (CAD), cutter_calcs.py (linkage and forces) and check_cutter.py
(collisions, cut sweep, bed fit). Units: mm, degrees. Every number carries a tag:

    measured  read from the stock STL/STEP, a datasheet or the blade listing
    assumed   a choice, or a value not yet checked on the real part
    estimate  worked out or scaled from the listing photo; replace with a measurement

The blades are a replacement set for a battery pruning shear (the listing gives 70 mm long,
25 mm cutting capacity, a 16 mm pivot hole and a 5 mm drive hole). They are not bought yet:
every other blade number is scaled from the listing photo at 0.071 mm/px (estimate). When the
blades arrive, measure them and update BLADE_* and the outlines below, then rebuild.

Frames:
    photo frame  origin on the blade pivot, x right, y up, as the listing photo shows the set.
                 The photo shows the moving blade's side, which on the tool faces down.
    tool frame   origin on the J3 axis at the tool-flange face (underside of the stock J3 coupler),
                 x toward the cutter mouth, z up. Everything hangs below z = 0.
"""
import math

# ---------------------------------------------------------------- stock J3 coupler (tool flange), measured from the STL
FLANGE_D = 78.0                 # measured: J3 coupler flange diameter (stock STEP; the faceted STL reads 77.6)
FLANGE_T = 7.0                  # measured: flange thickness above the tool face
FLANGE_PCD = 67.5               # measured: 10 x 4.1 mm through holes on this circle, every 36 deg
FLANGE_HOLE_D = 4.1             # measured
FLANGE_HOLE_ANGLES = [-90.0 + 36.0 * k for k in range(10)]     # measured: holes sit at -90 deg + k * 36 deg
CABLE_BORE_D = 12.5             # measured: centre bore of the coupler
PULLEY_SCREW_PCD = 22.0         # measured: 4 x M4 heads clamping the 90T pulley, in 8.5 mm counterbores
PULLEY_SCREW_RECESS_D = 30.0    # assumed: relief in the tool plate so those heads can stand proud by up to 1 mm
J3_CLEAR_R = 48.0               # estimate: Arm 2 end and J3 housing radius around J3 (stock STEP: 45 mm), plus margin

# ---------------------------------------------------------------- printer
BED = (250.0, 250.0, 250.0)     # measured (Kobra S1 datasheet)
BED_MARGIN = 5.0                # assumed

# ---------------------------------------------------------------- blades (photo frame)
PX = 0.071                      # estimate: mm per photo pixel (16 mm and 70 mm callouts agree within 4%)
BLADE_T_FIXED = 3.0             # assumed: SK5 pruner blades are usually 3-4 mm
BLADE_T_MOVING = 3.0            # assumed
PIVOT_HOLE_D = 16.0             # measured (listing)
DRIVE_HOLE_D = 5.0              # measured (listing)
DRIVE_HOLE = (4.5, 13.1)        # estimate: 13.9 mm from the pivot
TAIL_HOLE_D = 5.0               # assumed: not called out in the listing
TAIL_HOLE = (-10.4, -10.0)      # estimate: 14.4 mm from the pivot
STEM_D = 4.0                    # measured: distal pedicel 3-4 mm (Weng 2024)

# Hook (fixed) blade outline, photo frame, traced from the listing photo (estimate)
FIXED_OUTLINE = [
    (-14.5, -14.0), (-7.0, -15.5), (0.0, -15.5), (8.0, -14.5), (16.0, -12.0), (24.6, -9.7), (30.5, -6.8),
    (38.1, -1.1), (43.2, 4.5), (46.6, 10.2), (49.1, 15.9), (50.3, 20.2), (49.9, 23.7),                 # outer edge to tip
    (47.5, 22.0), (43.5, 18.7), (36.4, 13.1), (29.3, 9.5), (22.2, 7.0), (16.5, 5.6),                  # cutting edge
    (13.5, 8.0), (9.0, 11.0), (3.0, 12.5), (-3.0, 12.2), (-8.5, 9.2), (-12.0, 4.0), (-13.5, -2.0),     # round the pivot
    (-15.0, -7.0), (-16.5, -11.0),
]
FIXED_EDGE = FIXED_OUTLINE[12:19]     # tip -> crotch, the edge the moving blade shears past

# Moving blade outline, photo frame, open as photographed (estimate)
MOVING_OUTLINE = [
    (13.5, -0.5), (15.5, 2.5), (16.6, 4.5), (17.5, 7.4), (21.6, 13.1), (24.3, 18.7), (25.9, 24.4), (26.6, 30.1),
    (26.4, 35.8), (25.4, 41.5), (23.4, 47.1), (22.6, 48.6),                                            # cutting edge to tip
    (19.6, 47.1), (16.3, 41.5), (12.9, 35.8), (9.5, 30.0), (6.0, 24.5), (2.5, 20.0), (-1.5, 17.0),     # back edge
    (-6.0, 13.5), (-10.0, 9.0), (-12.2, 4.0), (-12.5, -1.0), (-11.5, -6.0), (-8.5, -9.8), (-4.0, -12.2),
    (1.0, -12.5), (6.0, -11.0), (10.0, -8.0), (12.5, -4.0),
]
MOVING_EDGE = MOVING_OUTLINE[1:12]    # crotch -> tip



def _inside(pt, poly):
    x, y = pt
    c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            c = not c
    return c


def _edge_dist(pt, poly):
    best = 1e9
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        t = ((pt[0] - ax) * (bx - ax) + (pt[1] - ay) * (by - ay)) / max((bx - ax) ** 2 + (by - ay) ** 2, 1e-12)
        t = min(max(t, 0.0), 1.0)
        best = min(best, math.hypot(pt[0] - ax - t * (bx - ax), pt[1] - ay - t * (by - ay)))
    return best


def _seat_stem(clear=0.3):
    """Deepest point in the open jaws (closest to the pivot) where the stem fits clear of both blades."""
    best = None
    for i in range(0, 400):
        for j in range(0, 400):
            p = (10.0 + i * 0.1, 0.0 + j * 0.1)
            if _inside(p, FIXED_OUTLINE) or _inside(p, MOVING_OUTLINE):
                continue
            if min(_edge_dist(p, FIXED_OUTLINE), _edge_dist(p, MOVING_OUTLINE)) < STEM_D / 2 + clear:
                continue
            r = math.hypot(*p)
            if best is None or r < best[0]:
                best = (r, p)
    return best[1]


STEM_AT = _seat_stem()          # estimate: where a 4 mm stem seats in the throat (photo frame), about 21 mm out

# ---------------------------------------------------------------- blade pose on the tool
# The set is flipped so the moving blade faces down (photo y -> -y), then turned so the mouth points along +x.
BLADE_YAW = 45.0                # assumed: hook tip at about +20 deg, open moving tip at about -20 deg
THROAT_X = 80.0                 # assumed: stem 80 mm ahead of J3, so the stem clears Arm 2 (J3_CLEAR_R) with 32 mm
CLOSE_SIGN = +1                 # the moving blade closes counter-clockwise seen from above (tool frame)
OPEN_DEG = 0.0
CLOSED_DEG = 46.0               # estimate: tip passes the hook edge at 40 deg (cutter_calcs.py), plus 6 deg overtravel


def photo_to_tool_xy(p, rot=0.0):
    """Photo-frame point -> tool-frame xy, with the moving blade turned by rot (deg, closing positive)."""
    x, y = p[0], -p[1]
    a = math.radians(BLADE_YAW + rot)
    xr, yr = x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)
    return (PIVOT[0] + xr, PIVOT[1] + yr)


def _stem_offset():
    x, y = STEM_AT[0], -STEM_AT[1]
    a = math.radians(BLADE_YAW)
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


PIVOT = (THROAT_X - _stem_offset()[0], -_stem_offset()[1])     # about (60.6, -7.5); stem then sits at (80, 0)
STEM_XY = (THROAT_X, 0.0)

# ---------------------------------------------------------------- vertical stack (tool frame z, at the pivot)
PLATE_T = 8.0                   # assumed: tool plate, printed
SKIRT_H = 4.0                   # assumed: skirt rising round the coupler flange, centres the tool on J3
SKIRT_ID = FLANGE_D + 0.4       # assumed: 0.2 mm radial clearance
SKIRT_W = 3.0
DECK_Z = -56.0                  # assumed: underside of the deck = top face of the hook blade (set so the servo fits under the plate)
DECK_T = 7.0
FIXED_Z = (DECK_Z - BLADE_T_FIXED, DECK_Z)
MOVING_Z = (FIXED_Z[0] - BLADE_T_MOVING, FIXED_Z[0])
LEVER_T = 6.0                   # assumed: printed PETG at 100%
LEVER_Z = (MOVING_Z[0] - LEVER_T, MOVING_Z[0])
BUSH = dict(od=16.0, id=8.0, len=BLADE_T_FIXED + BLADE_T_MOVING + LEVER_T - 0.1)  # assumed: steel spacer, 0.1 short
WASHER_T = 1.6                  # M8 washer (DIN 125)
BUTTON_HEAD = dict(d=14.0, h=4.4)   # M8 ISO 7380 button head (standard), under the lever
CUT_Z = MOVING_Z[1]             # the shear plane: moving blade sliding on the hook blade
CUT_ABOVE_FRUIT = 14.0          # estimate: the tool clears a 40-85 mm fruit whose top is this far below the cut (checked)
NUT_H = 8.0                     # M8 nyloc
SPRING_WASHER_T = 2.0           # M8 split or wave washer

# Servo: MG996R (on hand); a DS3225MG has the same case and drops in.
SERVO = dict(l=40.7, w=19.7, h=37.0, tab_l=54.0, tab_t=2.5, tab_z=28.0, shaft_off=10.2)   # measured (datasheets)
SERVO_HORN_HUB = 6.0            # assumed: spline face to the underside of the horn arm, stock single-arm horn
HORN_T = 2.5                    # assumed
HORN_R = 14.0                   # assumed: hole used on the stock single-arm horn (drill if needed)
LINK_T = 5.0                    # assumed: printed PETG at 100%
LINK_Z = (LEVER_Z[1] + 0.5, LEVER_Z[1] + 0.5 + LINK_T)          # link runs in the blade layers, behind the blades
HORN_Z = (LINK_Z[1] + 0.5, LINK_Z[1] + 0.5 + HORN_T)            # horn arm just above the link
SERVO_FACE_Z = HORN_Z[0] + SERVO_HORN_HUB                       # servo output face, shaft pointing down

# Servo stall torque (datasheets): MG996R 9.4 kg.cm at 4.8 V, 11 kg.cm at 6 V; DS3225MG 25 kg.cm at 6.8 V.
SERVO_TORQUE = {"MG996R": 11.0 * 0.0981, "DS3225MG": 21.5 * 0.0981}   # measured (datasheet), N.m at 6 V
USABLE = 0.5                    # assumed: plan on half the stall torque
PIN_EFF = 0.9                   # assumed: friction in the two link pins
PIVOT_FRICTION = 0.15           # estimate: N.m, blade-on-blade and lever-on-washer at a light preload
CUT_FORCE = 40.0                # measured 19.6-33.0 N for 3-4 mm pedicels (Weng 2024), plus margin (assumed)

# ---------------------------------------------------------------- four-bar: servo horn -> link -> lever on the blade
# Chosen by the search in cutter_calcs.py (rerun it after changing the blades or the layout).
LEVER_R = 34.0                  # lever pin radius from the blade pivot
LEVER_PHASE = 190.0             # lever pin angle at the open pose, tool frame (deg)
SERVO_SHAFT = (8.0, -14.0)      # tool frame xy; the servo body points away from the pivot
LINK_L = 29.0                   # pin to pin
HORN_BRANCH = 1                 # which of the two assembly branches
TOOL_BOLT_HEAD_CLEAR = 5.5      # M4 socket head radius plus 1 mm: the body must leave these flange holes open

# ---------------------------------------------------------------- stem guide and throat sensor
# The deck reaches forward as two guide arms either side of the jaws (above the hook blade, so the moving
# blade passes under them). An IR beam between the arm tips crosses the throat.
GUIDE_MOUTH = 40.0              # assumed: opening between the arm tips; catches a stem +-20 mm off the aim
GUIDE_CLEAR = 3.0               # assumed: arms stay this far outside the open jaws
BEAM_Z = DECK_Z + DECK_T / 2    # IR beam height, mid-deck
IR_D = 3.0                      # 3 mm IR LED and 3 mm IR phototransistor (bought)
