"""Collision and function checks for the arm v2 CAD. Exits 1 on a failure.

    .venv-cad/Scripts/python tools/cad/check_arm_v2.py          # about 10 minutes

1. Cutter: no part touches another over the blade stroke, door shut and open.
2. Blade: clear of the slot and V when open; covers the whole throat when closed.
3. Arm: Arm 1, Arm 2 and the tool never touch across J2 and at the J3 limits.
"""
import itertools
import sys
from pathlib import Path

from build123d import Pos, extrude

sys.path.insert(0, str(Path(__file__).parent))
import arm_v2_geometry as G  # noqa: E402
import build_arm_v2 as B  # noqa: E402

# Contacts that are meant to be there: belts wrap their pulleys and idlers press on them; the horn sits on the spline.
EXPECTED = {frozenset(p) for p in (("BUY-08", "BUY-09"), ("J2-02", "BELT-2"), ("STK-1", "BELT-1"), ("STK-1", "BELT-2"),
                                   ("BUY-04", "BELT-1"), ("BUY-04", "BELT-2"), ("J3-02", "BELT-3"), ("BUY-07", "BELT-3"))}


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


def main():
    fails = []
    # 1. cutter stroke and door
    for th in (G.LEVER_OPEN, -6.0, 6.0, G.LEVER_CLOSED):
        for door in (0.0, 75.0):
            B.build_parts(lever_theta=th, door_open=door)
            tool = [(p["id"], p["solid"]) for p in B.PARTS if p["frame"] == "tool"] + [("FLANGE", B.flange_stub())]
            for c in clashes(tool):
                fails.append(f"tool lever {th} door {door}: {c}")
    # 2. blade against the slot
    v, _ = B.v_outline()
    slot = Pos(0, 0, G.VPLATE_T) * extrude(v, G.LEVER_T)
    throat = slot & B.box(G.THROAT_X - 5, G.THROAT_X + G.SLOT_LEN, -10, 10, 0, 10)
    if overlap(B.lever_at(G.LEVER_OPEN), slot) > 0.01:
        fails.append("blade is in the slot or V when open")
    covered = overlap(B.lever_at(G.LEVER_CLOSED), throat) / throat.volume
    if covered < 0.95:
        fails.append(f"blade covers only {covered:.0%} of the throat when closed")
    # 3. arm sweep
    B.build_parts()
    groups = {fr: [p for p in B.PARTS if p["frame"] == fr] for fr in ("arm1", "arm2", "tool")}
    for fr in ("arm1", "arm2"):
        for c in clashes([(p["id"], p["solid"]) for p in groups[fr]]):
            fails.append(f"{fr} static: {c}")
    for j2 in (G.J2_LIMITS[0], -90.0, 0.0, 90.0, G.J2_LIMITS[1]):
        for j3 in (G.J3_LIMITS[0], 0.0, G.J3_LIMITS[1]):
            placed = {fr: [(p["id"], B.frame_transform(fr, 0, j2, j3) * p["solid"]) for p in groups[fr]] for fr in groups}
            for fa, fb in (("arm1", "arm2"), ("arm1", "tool"), ("arm2", "tool")):
                for c in clashes(placed[fa], placed[fb]):
                    fails.append(f"j2 {j2} j3 {j3}: {c}")
    for f in fails:
        print("FAIL", f)
    print(f"{len(fails)} failures")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
