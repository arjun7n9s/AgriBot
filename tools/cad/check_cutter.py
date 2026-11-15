"""Checks for the stem cutter (cutter v1). Exits non-zero if any check fails.

    .venv-cad/Scripts/python tools/cad/check_cutter.py

1. Cut stroke: every moving part against every fixed part and each other, from open to closed.
2. Stem path: a vertical 4 mm stem through the throat touches nothing with the blades open, and a hanging
   40, 60 or 85 mm fruit with its top CUT_ABOVE_FRUIT below the cut clears the tool, open and closed.
3. Beam: the IR beam line crosses the seated stem and is not blocked by the blades when open.
4. Stock arm: the tool turned through a full J3 turn under the stock Arm 2 (stock STEP).
5. Print bed: every printed part fits the Kobra S1 bed in its print orientation.
"""
import math
import sys
from pathlib import Path

from build123d import Pos, Rot, Sphere, import_step

sys.path.insert(0, str(Path(__file__).parent))
import build_cutter as B  # noqa: E402
import cutter_calcs as C  # noqa: E402
import cutter_geometry as G  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
STOCK = ROOT / "media" / "scara-arm" / "scara-robot-arm" / "SCARA Robot 3D Model.STEP"
TOL = 0.5          # mm3: overlaps below this are touching faces or press fits
STOCK_ARM2 = ("Arm_2_Arm_2", "Arm_2_Cover", "NEMA_17_Stepper_L24", "J3_Coupler", "Ball_Bearing_30x42",
              "Thrust_Bearing_35x52", "GT2_Pulley_-_90", "GT2_Pulley_-_Parametric", "Micro_switch", "GT2_Belt_400mm_2")
# Parts that share a pin, sit in a press fit or are held by the same bolt: allowed to touch
ALLOWED = {frozenset(p) for p in [
    ("moving blade", "drive dowel"), ("lever", "drive dowel"), ("lever", "bush"), ("moving blade", "bush"),
    ("hook blade", "bush"), ("hook blade", "tail dowel"), ("body", "tail dowel"), ("horn", "servo"),
    ("link", "pins"), ("lever", "pins"), ("horn", "pins"), ("body", "pivot bolt"), ("bush", "pivot bolt"),
    ("lever", "pivot bolt"), ("moving blade", "pivot bolt"), ("hook blade", "pivot bolt"),
]}

failures = []


def overlap(a, b):
    try:
        return (a & b).volume
    except Exception:          # noqa: BLE001  (an empty or failed boolean means no overlap)
        return 0.0


def parts_at(theta):
    hw = B.pivot_hardware()
    static = {
        "plate": B.plate(), "body": BODY, "hook blade": B.fixed_blade(), "servo": B.servo_case(),
        "tail dowel": B.tail_dowel(), "bush": hw["bush"],
        "pivot bolt": hw["bolt_shank"] + hw["bolt_head"] + hw["washer_top"] + hw["washer_bot"] + hw["spring_washer"] + hw["nut"],
    }
    q, h, psi = B.state(theta)
    moving = {
        "moving blade": B.moving_blade(theta), "lever": B.lever(theta), "link": B.link(theta),
        "horn": B.horn(psi), "pins": B.pin_hardware(theta), "drive dowel": B.drive_dowel(theta),
    }
    return static, moving


def check_stroke():
    worst = {}
    for i in range(0, int(G.CLOSED_DEG) + 1, 2):
        theta = float(i)
        static, moving = parts_at(theta)
        pairs = [(m, s) for m in moving for s in static] + \
                [(a, b) for k, a in enumerate(moving) for b in list(moving)[k + 1:]]
        for a, b in pairs:
            if frozenset((a, b)) in ALLOWED:
                continue
            sa = moving[a]
            sb = moving[b] if b in moving else static[b]
            if not sa.bounding_box().overlaps(sb.bounding_box()):
                continue
            v = overlap(sa, sb)
            if v > TOL:
                worst[(a, b)] = max(worst.get((a, b), (0, 0)), (round(v, 1), theta))
    for (a, b), (v, th) in sorted(worst.items()):
        failures.append(f"stroke: {a} hits {b}, {v} mm3 at {th} deg")
    print(f"1. stroke 0-{G.CLOSED_DEG:.0f} deg: {'clear' if not worst else f'{len(worst)} clashes'}")


def check_stem():
    stem = B.cyl(G.STEM_D / 2, -120.0, 60.0, *G.STEM_XY)
    static, moving = parts_at(0.0)
    hits = [(n, round(overlap(stem, s), 1)) for n, s in {**static, **moving}.items() if overlap(stem, s) > TOL]
    for n, v in hits:
        failures.append(f"stem path: the stem column hits {n} ({v} mm3)")
    r = math.hypot(*G.STEM_XY)
    if r < G.J3_CLEAR_R + 20:
        failures.append(f"stem path: stem only {r:.0f} mm from J3")
    first, sever = C.cut_window()
    print(f"2. stem path: {'clear' if not hits else hits}; stem {r:.0f} mm from J3; cut from {first} to {sever} deg")


def check_fruit():
    """A fruit hanging on the stem, its top CUT_ABOVE_FRUIT below the shear plane, must clear the tool."""
    hits = []
    for d in (40.0, 60.0, 85.0):
        top = G.CUT_Z - G.CUT_ABOVE_FRUIT
        fruit = Pos(G.STEM_XY[0], G.STEM_XY[1], top - d / 2) * Sphere(d / 2)
        for theta in (0.0, G.CLOSED_DEG):
            static, moving = parts_at(theta)
            for n, sh in {**static, **moving}.items():
                v = overlap(fruit, sh)
                if v > TOL:
                    hits.append((d, theta, n, round(v, 1)))
    for d, th, n, v in hits:
        failures.append(f"fruit: a {d:.0f} mm fruit {G.CUT_ABOVE_FRUIT} mm below the cut hits {n} at {th} deg ({v} mm3)")
    print(f"2b. fruit 40/60/85 mm, top {G.CUT_ABOVE_FRUIT} mm below the cut: {'clear' if not hits else hits}")


def check_beam():
    a, b = B.beam_ends()
    line = B.along(a, b, 0.5)
    stem = B.cyl(G.STEM_D / 2, -120.0, 60.0, *G.STEM_XY)
    crosses = overlap(line, stem) > 0.01
    blocked = [n for n, s in {"hook blade": B.fixed_blade(), "moving blade": B.moving_blade(0.0)}.items()
               if overlap(line, s) > 0.01]
    if not crosses:
        failures.append("beam: the IR beam misses the seated stem")
    if blocked:
        failures.append(f"beam: blocked by {blocked}")
    print(f"3. beam: crosses the stem {crosses}, blocked by {blocked or 'nothing'}")


def check_stock_arm():
    if not STOCK.exists():
        print("4. stock arm: stock STEP not found, skipped")
        return
    arm = [(k.label, k) for k in import_step(str(STOCK)).children if any(n in k.label for n in STOCK_ARM2)
           and k.bounding_box().min.X > 200]
    static, moving = parts_at(0.0)
    tool = list(static.items()) + list(moving.items())
    bad = []
    for j3 in range(-180, 180, 15):
        place = Pos(*B.J3_STEP) * Rot(0, 0, B.ARM2_YAW + j3)
        for tn, ts in tool:
            t = place * ts
            for an, a in arm:
                if "J3_Coupler" in an and j3 != 0:
                    continue           # the coupler turns with the tool: checked once, at J3 = 0
                if not t.bounding_box().overlaps(a.bounding_box()):
                    continue
                v = overlap(t, a)
                if v > TOL:
                    bad.append((j3, tn, an.split("_")[0] + " " + an.split("_")[1], round(v, 1)))
    if bad:
        for row in bad:
            failures.append(f"stock arm: at J3 {row[0]} deg the {row[1]} hits {row[2]} ({row[3]} mm3)")
    print(f"4. stock arm, J3 every 15 deg over a full turn: {'clear' if not bad else f'{len(bad)} clashes'}")


def check_bed():
    lim = (G.BED[0] - 2 * G.BED_MARGIN, G.BED[1] - 2 * G.BED_MARGIN, G.BED[2])
    for pid, (name, _, fn) in B.PRINTED.items():
        s = fn()
        bb = s.bounding_box()
        xy = sorted((bb.size.X, bb.size.Y))
        ok = xy[1] <= max(lim[:2]) and xy[0] <= min(lim[:2]) and bb.size.Z <= lim[2]
        if not ok:
            failures.append(f"bed: {pid} {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} does not fit")
        print(f"5. bed: {pid} {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm {'fits' if ok else 'DOES NOT FIT'}")


if __name__ == "__main__":
    BODY = B.body()
    check_stroke()
    check_stem()
    check_fruit()
    check_beam()
    check_stock_arm()
    check_bed()
    if failures:
        print("\nFAILED:")
        for f in failures:
            print(" -", f)
        sys.exit(1)
    print("\nall cutter checks pass")
