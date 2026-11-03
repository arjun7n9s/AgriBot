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

# Arm 1 sandwich (STEP z)
A1_BOT = (228.0, 232.0)                   # 6061-T6, 4 mm
A1_TOP = (268.0, 272.0)                   # 6061-T6, 4 mm
# J2 stack (STEP z)
J2_FLANGE = (196.0, 204.0)
J2_THRUST_LO = (204.0, 216.0)             # 51107, 35 x 52 x 12 (on hand)
J2_HUB = (216.0, 228.0)
J2_RADIAL = (216.0, 223.0)                # 6806, 30 x 42 x 7 (on hand)
J2_THRUST_HI = (228.0, 240.0)             # 51107
J2_PULLEY = (240.0, 254.4)
# Arm 2 sandwich (STEP z)
A2_TOP = (192.0, 196.0)                   # 6061-T6, 4 mm
A2_BOT = (164.0, 167.0)                   # 6061-T6, 3 mm
J3_BELT_Z = 179.5
# J3 stack (STEP z)
J3_HUB = (148.0, 164.0)
J3_RADIAL_LO = (148.0, 155.0)             # 6806 (on hand)
J3_RADIAL_HI = (157.0, 164.0)             # 6807, 35 x 47 x 7 (on hand)
J3_FLANGE = (138.0, 144.0)                # tool flange; its lower face is the tool frame z = 0
TOOL_Z0 = J3_FLANGE[0]

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
J3_LIMITS = (-110.0, 95.0)    # beyond these the camera on the tool meets the underside of Arm 2

J3_BELT_C = centre_for_belt(J3_BELT, J3_MOTOR_TEETH, J3_PULLEY_TEETH)   # 143.3
J3_MOTOR_X = L2 - J3_BELT_C                                             # arm2 x, 56.7

# ---------------------------------------------------------------- end effector (tool frame)
THROAT_X = 90.0          # assumed: centre of the slot's round end
SLOT_W = 8.0             # assumed: passes a 3-5 mm distal pedicel (Weng 2024), guides it
SLOT_LEN = 12.0          # straight slot ahead of the round end
V_HALF_ANGLE = 30.0      # assumed: 60 deg included V
MOUTH_W = 50.0           # assumed: catches a pedicel up to +-25 mm off the aim (sim errors were 6-24 mm)
DECK_T = 3.0             # 6061-T6
VPLATE_T = 1.5           # SS304 or SS410 laser cut; its slot edge is the shear edge
LEVER_T = 2.0            # blade-lever, SS420 or EN42 spring steel, hardened, single 25 deg bevel
PIVOT = (68.0, 0.0)      # blade-lever pivot (M4 shoulder bolt)
EDGE_R = THROAT_X - 2.0 - PIVOT[0]   # pivot to pedicel centre, 20 mm
EDGE_SLICE = -15.0       # assumed: edge raked so the cut pulls the pedicel into the slot end
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
LIP_TOP_Z = -108.0       # TPU lip on the door's front edge
DOOR_HINGE = (47.5, -131.0)   # (y, z), hinge axis along x

# MG996R and DS3225 share the standard servo case and 25T spline (datasheets)
SERVO = dict(l=40.7, w=19.7, h=37.0, tab_l=54.0, tab_t=2.5, tab_z=28.0, shaft_off=10.2)

CAMERA = dict(w=42.0, h=42.0, d=23.0, tilt=20.0)   # Intel RealSense D405 class, datasheet size


def servo_angle_for_lever(theta):
    """Horn angle (deg) that puts the blade-lever at theta (deg), on the working branch."""
    q = (PIVOT[0] + LEVER_IN_R * math.cos(math.radians(theta + LEVER_IN_PHASE)),
         PIVOT[1] + LEVER_IN_R * math.sin(math.radians(theta + LEVER_IN_PHASE)))
    dx, dy = q[0] - SERVO_SHAFT[0], q[1] - SERVO_SHAFT[1]
    d = math.hypot(dx, dy)
    cosv = (HORN_R ** 2 + d ** 2 - LINK_L ** 2) / (2 * HORN_R * d)
    return math.degrees(math.atan2(dy, dx)) - math.degrees(math.acos(cosv)), q
