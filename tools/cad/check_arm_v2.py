"""Collision, function and printability checks for the arm v2 CAD. Exits 1 on a failure.

    .venv-cad/Scripts/python tools/cad/check_arm_v2.py          # several minutes

1. Tool: no part touches another over the slide stroke and the cutter stroke, with 40 and 85 mm fruit held.
2. Cutter: the blade is clear of the slot when open, and when closed its edge has passed the steel edge by at
   least 1 mm along the whole slot; the fruit's calyx lands inside the slot for 40-85 mm fruit.
3. Arm: Arm 1, Arm 2, the tool and the Z column never touch across J2 and at the J3 limits. The Z column comes from
   the stock STEP in media/scara-arm (skipped with a warning if that file is missing).
4. Printing: every printed part fits the Kobra S1 bed (250 x 250 x 250 mm, 5 mm margin) at some yaw.
"""
import itertools
import math
import sys
from pathlib import Path

from build123d import Pos, extrude, import_step

sys.path.insert(0, str(Path(__file__).parent))
import arm_v2_geometry as G  # noqa: E402
import build_arm_v2 as B  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
STOCK = ROOT / "media" / "scara-arm" / "scara-robot-arm" / "SCARA Robot 3D Model.STEP"
# Contacts that are meant to be there: belts on pulleys, horns on splines, the slide arm on its home switch at
# full retraction, the cup on the fruit, the blade in its carrier.
EXPECTED = {frozenset(p) for p in (("J2-02", "BELT-2"), ("STK-1", "BELT-1"), ("STK-1", "BELT-2"), ("BUY-04", "BELT-1"),
                                   ("BUY-04", "BELT-2"), ("J3-02", "BELT-3"), ("BUY-07", "BELT-3"),
                                   ("BUY-24", "BUY-26"), ("BUY-25", "BUY-26"), ("BUY-27", "T-05"), ("BUY-21", "REF-1"),
                                   ("BUY-20", "T-03"))}
COLUMN_KEEP = ("Smooth_Rod", "Lead_Screw", "Z-axis_Top_Plate", "Top_cover", "Z-axis_Mount_Platform", "Linear_Bearing",
               "Stepper", "GT2_Pulley_-_Parametric")


def overlap(a, b):
    try:
        return (a & b).volume
    except Exception:
        return -1.0


def clashes(group_a, group_b=None):
    pairs = itertools.combinations(group_a, 2) if group_b is None else itertools.product(group_a, group_b)
    out = []
    for (ia, sa), (ib, sb) in pairs:
        if frozenset((ia, ib)) in EXPECTED:
            continue
        v = overlap(sa, sb)
        if v > 0.5:
            out.append((ia, ib, round(v, 1)))
    return out


def tool_parts(slide, cut, fruit):
    B.PARTS.clear()
    return [(p["id"], p["solid"]) for p in B.build_tool(slide, cut, fruit)] + [("FLANGE", B.flange_stub())]


def stock_column():
    if not STOCK.exists():
        print("WARNING: stock STEP not found; the Z column is not checked")
        return []
    out = []
    for k in import_step(str(STOCK)).children:
        bb = k.bounding_box()
        if any(n in k.label for n in COLUMN_KEEP) and bb.max.Z > 140 and bb.max.X < 120:
            out.append(("COL " + k.label.split("_")[0], k))
    return out


def main():
    fails = []
    # 1. tool over both strokes, small and large fruit
    for slide, cut, fruit in ((0, 0, 20), (0, 0.5, 20), (0, 1, 20), (0, 0, 42), (0, 1, 42),
                              (0.5, 0, 0), (1, 0, 0), (1, 1, 0)):
        for c in clashes(tool_parts(slide, cut, fruit)):
            fails.append(f"tool slide {slide} cut {cut} fruit {2 * fruit}: {c}")
    # 2. cutter function
    slot = Pos(0, 0, G.HOOD_TOP + G.SS_STRIP_T) * extrude(B.slot_and_v(), G.UTILITY_BLADE["t"])
    slot &= B.box(G.SLOT_X[0], G.SLOT_X[1], -10, 10, -100, 100)
    if overlap(B.blade(0.0) + B.carrier(0.0), slot) > 0.01:
        fails.append("blade or carrier is over the slot when open")
    xs = [G.SLOT_X[0] + i * (G.SLOT_X[1] - G.SLOT_X[0]) / 40 for i in range(41)]
    tan = math.tan(math.radians(G.BLADE_RAKE))
    closed = min(G.blade_y(1.0) + (x - G.BLADE_X_MID) * tan for x in xs) - G.SHEAR_EDGE_Y
    opened = -G.SLOT_W / 2 - max(G.blade_y(0.0) + (x - G.BLADE_X_MID) * tan for x in xs)
    print(f"closed: edge past the steel edge by {closed:.2f} mm along the whole slot; open: clear of the slot by {opened:.2f} mm")
    if closed < 1.0:
        fails.append(f"closed edge passes the steel edge by only {closed:.2f} mm")
    if opened < 0.5:
        fails.append(f"open edge clears the slot by only {opened:.2f} mm")
    edge_x = (G.BLADE_X_MID - G.UTILITY_BLADE["edge"] / 2 * math.cos(math.radians(G.BLADE_RAKE)),
              G.BLADE_X_MID + G.UTILITY_BLADE["edge"] / 2 * math.cos(math.radians(G.BLADE_RAKE)))
    if not (edge_x[0] < G.SLOT_X[0] and edge_x[1] > G.SLOT_X[1]):
        fails.append(f"blade edge x {edge_x} does not span the slot {G.SLOT_X}")
    for r in (20.0, 42.0):
        zc = G.HOOD_BOT - r
        xc = G.CUP_FACE_IN + math.sqrt(r * r - (G.CUP_Z - zc) ** 2)
        inside = G.SLOT_X[0] + G.SLOT_W / 2 <= xc <= G.SLOT_X[1]
        print(f"{2 * r:.0f} mm fruit: calyx at x {xc:.1f}, slot {G.SLOT_X}: {'ok' if inside else 'OUTSIDE'}")
        if not inside:
            fails.append(f"{2 * r:.0f} mm fruit: calyx outside the slot")
    # 3. arm sweep, with the Z column
    B.build_parts(fruit=0)
    groups = {fr: [p for p in B.PARTS if p["frame"] == fr] for fr in ("arm1", "arm2", "tool")}
    column = stock_column()
    for fr in ("arm1", "arm2"):
        for c in clashes([(p["id"], p["solid"]) for p in groups[fr]]):
            fails.append(f"{fr} static: {c}")
    for j2 in (G.J2_LIMITS[0], -90.0, -45.0, 0.0, 45.0, 90.0, 120.0, G.J2_LIMITS[1]):
        for j3 in (G.J3_LIMITS[0], 0.0, G.J3_LIMITS[1]):
            placed = {fr: [(p["id"], B.frame_transform(fr, 0, j2, j3) * p["solid"]) for p in groups[fr]] for fr in groups}
            for fa, fb in (("arm1", "arm2"), ("arm1", "tool"), ("arm2", "tool")):
                for c in clashes(placed[fa], placed[fb]):
                    fails.append(f"j2 {j2} j3 {j3}: {c}")
            for fr in ("arm2", "tool"):
                for c in clashes(placed[fr], column):
                    fails.append(f"j2 {j2} j3 {j3} column: {c}")
    # 4. printability
    for p in B.PARTS:
        if p["export"] and p["material"] in B.PRINTED:
            fit = B.bed_fit(B.manufacturing_frame(p))
            if fit is None:
                fails.append(f"{p['id']} does not fit the bed")
            else:
                print(f"{p['id']:6s} fits at yaw {fit[0]:2d}: {fit[1]:.0f} x {fit[2]:.0f} x {fit[3]:.0f} mm")
    for f in fails:
        print("FAIL", f)
    print(f"{len(fails)} failures")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
