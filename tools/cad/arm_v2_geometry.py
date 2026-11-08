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
J2_LIMITS = (-120.0, 135.0)   # beyond these the folded tool meets the Z column rods (5 deg sweep, any J3)
J3_LIMITS = (-105.0, 85.0)    # beyond these the camera or its bracket meets the underside of Arm 2 (5 deg sweep)

J3_BELT_C = centre_for_belt(J3_BELT, J3_MOTOR_TEETH, J3_PULLEY_TEETH)   # 143.3
J3_MOTOR_X = L2 - J3_BELT_C                                             # arm2 x, 56.7

# ---------------------------------------------------------------- end effector v3 (tool frame)
# Suction draw-in, hood registration, guillotine cut. Tool frame: origin on the J3 axis at the flange face,
# x forward (toward the fruit), z up. Everything hangs below the flange (z < 0).
#
# Bought parts and their sizes (check yours against these before printing):
CUP = dict(d=25.0, h=19.0)          # assumed: 1.5-fold silicone bellows cup, 25 mm (SMC ZP25B class), M5 male stem
CUP_FITTING = dict(d=10.0, l=10.0)  # assumed: M5 hex adapter + PC4-M5 push-in fitting for 4 mm PU tube
ROD = dict(d=8.0, l=160.0)          # 8 mm hardened smooth rod, cut to 160 mm
LM8UU = dict(d=15.0, l=24.0)        # measured (datasheet): LM8UU linear bearing 8 x 15 x 24
UTILITY_BLADE = dict(edge=61.0, back=37.0, h=19.0, t=0.6)   # assumed: standard trapezoid utility blade
SS_STRIP_T = 1.0                    # assumed: strip cut from the stainless ruler on hand; its factory edge shears
SERVO = dict(l=40.7, w=19.7, h=37.0, tab_l=54.0, tab_t=2.5, tab_z=28.0, shaft_off=10.2)  # MG996R / DS3225MG case
CAMERA = dict(w=32.0, h=32.0, d=8.0, lens_d=14.0, lens_l=12.0, tilt=30.0)   # assumed: 32 mm USB board camera

# Hood: the plate the fruit is drawn up under. Its slot takes the pedicel; the cut is just above it.
HOOD_TOP = -30.0
HOOD_T = 4.0
HOOD_BOT = HOOD_TOP - HOOD_T
HOOD_X = (62.0, 160.0)
HOOD_Y = (-46.0, 28.0)
SLOT_W = 8.0                        # assumed: passes a 3-5 mm distal pedicel (Weng 2024)
SLOT_X = (78.0, 130.0)              # straight slot; the calyx lands at x 89-109 for 40-85 mm fruit
MOUTH_W = 50.0                      # V mouth at the hood front: catches the pedicel +-25 mm off centre
SHEAR_EDGE_Y = SLOT_W / 2           # the steel strip's edge, 0.6 mm proud of the printed slot wall
STRIP_X = (76.0, 130.0)

# Suction slide
CUP_Z = HOOD_BOT - 26.0             # cup axis: aimed 26 mm below the fruit top, which then seats on the hood
CUP_FACE_IN = 70.0                  # cup face, slide retracted (fruit under the hood)
SLIDE_SERVO = (1.0, 38.0)           # MG996R shaft (down), crank-slider along x at y 38
SLIDE_CRANK_R, SLIDE_LINK = 36.0, 70.0
TAB_TO_CUP = 35.0                   # slide tab sits this far behind the cup face

# Cutter: utility blade in a carrier that slides across the slot (+y), raked 15 deg to the slot
BLADE_RAKE = 15.0                   # edge at 15 deg to x: 75 deg slicing angle (Weng 2024: 63-78 deg)
BLADE_X_MID = 104.0
BLADE_Y_OPEN, BLADE_Y_CLOSED = -13.0, 13.0   # edge y at BLADE_X_MID: the whole edge clears the slot by 1 mm
BLADE_EXPOSED = 3.0
CARRIER_PINS = [(66.0, -24.0), (142.0, -24.0)]   # M4 shoulder bolts + OD 6 springs (preload)
CUT_SERVO = (102.0, -64.0)          # DS3225MG shaft (up), crank-slider along y
CUT_CRANK_R, CUT_LINK = 13.0, 30.0
CARRIER_PIN_DY = -34.0             # pin y open: -47, closed: -21              # link pin on the carrier, relative to the edge line at BLADE_X_MID


def crank_slider(theta_deg, r, l):
    """Aligned crank-slider: slider position along its axis from the crank centre."""
    t = math.radians(theta_deg)
    return r * math.cos(t) + math.sqrt(l * l - (r * math.sin(t)) ** 2)


def cup_face(s):
    """Cup face x for slide command s (0 = retracted, 1 = extended). Crank goes 180 -> 0 deg through +y."""
    tab = SLIDE_SERVO[0] + crank_slider(180.0 * (1.0 - s), SLIDE_CRANK_R, SLIDE_LINK)
    return tab + TAB_TO_CUP


def blade_y(c):
    """Edge y at BLADE_X_MID for cutter command c (0 = open, 1 = closed)."""
    pin = CUT_SERVO[1] + crank_slider(180.0 * (1.0 - c), CUT_CRANK_R, CUT_LINK)
    return pin - CARRIER_PIN_DY


STROKE = cup_face(1.0) - cup_face(0.0)                     # 72 mm
TOOL_REACH = cup_face(1.0) + 32.0                          # J3 axis to a mid-size fruit centre at first contact
