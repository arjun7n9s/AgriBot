"""Shared numbers for SCARA arm v2 and the tomato end effector.

Used by build_arm_v2.py (CAD) and arm_v2_calcs.py (reach, belts, forces).
Units: mm, degrees, grams. Every number carries a tag:

    measured  read from the stock STEP or a datasheet we hold
    assumed   a choice or a value we have not checked on the real part
    estimate  worked out, to be replaced by a measurement

Frames:
    STEP frame   the stock "SCARA Robot 3D Model.STEP" frame, at the pose the
                 stock assembly was saved in. The v2 assembly is exported in this
                 frame so it overlays the stock base, column and carriage.
    arm1 frame   origin on the J1 axis, x along L1 toward J2, z as STEP z.
    arm2 frame   origin on the J2 axis, x toward J3, z as STEP z.
    tool frame   origin on the J3 axis at the tool-flange face, x toward the
                 cutter mouth, z up. The deck top is z = 0.
"""
import math

# ---------------------------------------------------------------- locked stock geometry
J1_XY = (42.5, 0.0)                       # measured: J1 axis in the STEP frame
J2_XY_STOCK = (260.9, 65.6)               # measured: J2 axis in the STEP frame
L1 = 228.0                                # measured: 227.98 from the STEP; kept
ARM1_YAW = math.degrees(math.atan2(J2_XY_STOCK[1], J2_XY_STOCK[0] - J1_XY[0]))  # 16.7, measured

CARRIAGE_FACE_X = 62.0                    # measured: carriage face that Arm 1 bolts to (arm1 x)
CARRIAGE_M5_Y = 25.5                      # measured: 4 x M5, at y = +-25.5
CARRIAGE_M5_Z = (245.1, 263.1)            # measured: and z = 245.1, 263.1
J2_MOTOR_X = -10.0                        # measured: J2 motor 20T pulley on the carriage (arm1 x)
IDLER_X = 138.1                           # measured: stock 80T/23T compound pulley axis (arm1 x)
BELT1_Z = 260.5                           # measured: 20T -> 80T stage, 400 mm belt
BELT2_Z = 250.6                           # measured: 23T -> 92T stage, 300 mm belt
COMPOUND_PULLEY_Z = (246.2, 265.0)        # measured: stock "GT2 Pulley - 23 - 80 teeth"

# ---------------------------------------------------------------- v2 choices
L2 = 200.0                                # assumed: chosen from the reach study (arm_v2_calcs.py)
J3_MOTOR_TEETH, J3_PULLEY_TEETH = 20, 90  # 20T on hand; 90T printed (stock ratio 4.5:1 kept)
J2_PULLEY_TEETH = 92                      # printed, same ratio as stock
GT2_PITCH = 2.0
J3_BELT = 400.0                           # on hand (two bought)

# Printer: Anycubic Kobra S1, 250 x 250 x 250 mm bed (datasheet). Every made part is printed on it.
BED = (250.0, 250.0, 250.0)               # measured (datasheet)
BED_MARGIN = 5.0                          # assumed: keep 5 mm off the bed edges
WALL = 3.0                                # assumed: printed side walls, 3 mm (about 7 perimeters of 0.42)

# Arm 1: printed tray (floor, walls, root wall, columns, idler boss) and a printed lid (STEP z)
A1_FLOOR = (224.0, 232.0)                 # assumed: 8 mm PETG floor
A1_LID = (268.0, 273.0)                   # assumed: 5 mm PETG lid
# J2 stack (STEP z). Spindle comes in from below: flange, then 35 mm, then 30 mm, so it passes every bearing.
J2_FLANGE = (196.0, 204.0)
J2_THRUST_LO = (204.0, 216.0)             # 51107, 35 x 52 x 12 (on hand)
J2_HUB = (216.0, 224.0)                   # printed hub under the Arm 1 floor
J2_RADIAL = (216.0, 223.0)                # 6806, 30 x 42 x 7 (on hand)
J2_THRUST_HI = (228.0, 240.0)             # 51107, sits in a 4 mm counterbore in the Arm 1 floor
J2_PULLEY = (240.0, 254.4)
# Arm 2: printed body (top plate, walls, columns) and a printed cover that carries the J3 housing (STEP z)
A2_TOP = (190.0, 196.0)                   # assumed: 6 mm PETG top, carries the J2 spindle flange
A2_COVER = (160.0, 165.0)                 # assumed: 5 mm PETG cover; J3 housing boss below it
J3_BELT_Z = 179.5
# J3 stack (STEP z). 6807 below, 6806 above, so the stepped spindle goes in from below.
J3_HUB = (148.0, 160.0)                   # boss under the cover
J3_RADIAL_LO = (148.0, 155.0)             # 6807, 35 x 47 x 7 (on hand)
J3_WEB = (155.0, 158.0)
J3_RADIAL_HI = (158.0, 165.0)             # 6806, 30 x 42 x 7 (on hand)
J3_FLANGE = (138.0, 146.0)                # tool flange; its lower face is the tool frame z = 0
J3_SPINDLE_TOP = 172.0
TOOL_Z0 = J3_FLANGE[0]

# Heat-set inserts (to buy): M4 x 8 long, 5.6 mm hole [assumed: common brass knurled insert]
INSERT_M4 = dict(hole=5.6, depth=8.5)
# M4 hex nut 7.0 mm across flats, 3.2 thick; side-entry trap
NUT_M4 = dict(af=7.0, t=3.2)

# J3 motor: NEMA17 "4.2 kg.cm economy" (on hand). Body length assumed 40 mm (17HS4401 class).
NEMA17 = dict(side=42.3, length=40.0, boss_d=22.0, boss_h=2.0, shaft_d=5.0, shaft_len=24.0, hole_sq=31.0)


def gt2_pitch_radius(teeth):
    return teeth * GT2_PITCH / (2 * math.pi)


def belt_length(c, t1, t2):
    """Closed GT2 loop over two pulleys at centre distance c (pitch-line formula)."""
    d1, d2 = 2 * gt2_pitch_radius(t1), 2 * gt2_pitch_radius(t2)
    return 2 * c + math.pi * (d1 + d2) / 2 + (d2 - d1) ** 2 / (4 * c)


def centre_for_belt(length, t1, t2):
    lo, hi = 10.0, 2000.0
    for _ in range(80):
        mid = (lo + hi) / 2
        if belt_length(mid, t1, t2) < length:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


# Joint limits checked for collisions by tools/cad/check_arm_v2.py
J2_LIMITS = (-150.0, 150.0)   # no Arm 1 / Arm 2 / tool contact over the full range
J3_LIMITS = (-105.0, 85.0)    # beyond these the camera or its bracket meets the underside of Arm 2 (5 deg sweep)

J3_BELT_C = centre_for_belt(J3_BELT, J3_MOTOR_TEETH, J3_PULLEY_TEETH)   # 143.3
J3_MOTOR_X = L2 - J3_BELT_C                                             # arm2 x, 56.7

# ---------------------------------------------------------------- end effector (tool frame)
THROAT_X = 90.0          # assumed: centre of the slot's round end
SLOT_W = 8.0             # assumed: passes a 3-5 mm distal pedicel (Weng 2024), guides it
SLOT_LEN = 12.0          # straight slot ahead of the round end
V_HALF_ANGLE = 30.0      # assumed: 60 deg included V
MOUTH_W = 50.0           # assumed: catches a pedicel up to +-25 mm off the aim (sim errors were 6-24 mm)
DECK_T = 6.0             # printed PETG deck
LEVER_T = 6.5            # printed PETG blade-lever; the moving blade sits in a pocket on its underside
# Blades: 18 mm snap-off cutter blade (Olfa LB / generic), snapped to 3 segments. Bought, replaceable.
BLADE = dict(w=18.0, t=0.5, seg=12.5, n=3, score_deg=60.0)   # assumed: check against your blades (pitch, angle)
BLADE_EXPOSED = 3.0      # assumed: edge strip left outside the lever outline
FIXED_EDGE_Y = SLOT_W / 2   # the fixed blade's edge is the +y side of the slot, flush with the deck top
PIVOT = (68.0, 0.0)      # blade-lever pivot (M4 bolt, spring preload)
EDGE_R = THROAT_X - 2.0 - PIVOT[0]   # pivot to pedicel centre, 20 mm
EDGE_SLICE = -15.0       # assumed: edge raked so the cut pulls the pedicel into the slot end
EDGE_S = (-14.0, 23.5)   # edge span along the edge line (lever frame), one 3-segment blade piece
LEVER_OPEN, LEVER_CLOSED = -32.0, 18.0

# Four-bar from the servo horn to the lever (chosen by the search in arm_v2_calcs.py)
SERVO_SHAFT = (28.3, -42.0)
HORN_R = 14.0            # hole at 14 mm on the stock MG996R single-arm horn (drill if needed)
LINK_L = 40.0
LEVER_IN_R = 30.0
LEVER_IN_PHASE = 200.0   # input arm direction relative to the blade radial line

BASKET_IN_X = (THROAT_X - 47.6, THROAT_X + 47.5)   # 95 mm inside, front open
BASKET_IN_HALF_W = 47.5
BASKET_WALL = 2.4
PAD_TOP_Z = -118.0       # fruit lands here (10 mm foam on the door)
LIP_TOP_Z = -108.0       # 10 mm EVA strip glued on the door's front edge (no TPU on hand)
DOOR_HINGE = (47.5, -131.0)   # (y, z), hinge axis along x

# MG996R and DS3225 share the standard servo case and 25T spline (datasheets)
SERVO = dict(l=40.7, w=19.7, h=37.0, tab_l=54.0, tab_t=2.5, tab_z=28.0, shaft_off=10.2)

CAMERA = dict(w=42.0, h=42.0, d=23.0, tilt=20.0)   # Intel RealSense D405 class, datasheet size (to buy)


def servo_angle_for_lever(theta):
    """Horn angle (deg) that puts the blade-lever at theta (deg), on the working branch."""
    q = (PIVOT[0] + LEVER_IN_R * math.cos(math.radians(theta + LEVER_IN_PHASE)),
         PIVOT[1] + LEVER_IN_R * math.sin(math.radians(theta + LEVER_IN_PHASE)))
    dx, dy = q[0] - SERVO_SHAFT[0], q[1] - SERVO_SHAFT[1]
    d = math.hypot(dx, dy)
    cosv = (HORN_R ** 2 + d ** 2 - LINK_L ** 2) / (2 * HORN_R * d)
    return math.degrees(math.atan2(dy, dx)) - math.degrees(math.acos(cosv)), q
