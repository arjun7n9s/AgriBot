"""Linkage and force calculations for the stem cutter (cutter v1 on the stock SCARA).

    .venv-cad/Scripts/python tools/cad/cutter_calcs.py           # report for the linkage in cutter_geometry.py
    .venv-cad/Scripts/python tools/cad/cutter_calcs.py --search  # search for a better linkage, print the best

Writes cad/cutter/calcs.json and docs/images/cutter_v1_force.png. Every result is an estimate:
the blade outlines are scaled from the listing photo and the servo torque is a datasheet value.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import cutter_geometry as G  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "cad" / "cutter"


# ---------------------------------------------------------------- blade geometry
def inside(pt, poly):
    x, y = pt
    c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            c = not c
    return c


def moving_poly(rot):
    return [G.photo_to_tool_xy(p, rot) for p in G.MOVING_OUTLINE]


def fixed_poly():
    return [G.photo_to_tool_xy(p) for p in G.FIXED_OUTLINE]


def stem_points(n=48):
    r = G.STEM_D / 2
    return [(G.STEM_XY[0] + r * math.cos(2 * math.pi * k / n), G.STEM_XY[1] + r * math.sin(2 * math.pi * k / n))
            for k in range(n)] + [G.STEM_XY]


def cut_window():
    """Blade rotation (deg) at first contact with the seated stem, and when the stem is fully severed."""
    pts = stem_points()
    fixed = fixed_poly()
    assert not any(inside(p, fixed) for p in pts), "stem overlaps the hook blade: move STEM_AT"
    assert not any(inside(p, moving_poly(0.0)) for p in pts), "stem overlaps the open moving blade: move STEM_AT"
    first = last = None
    for i in range(0, int(G.CLOSED_DEG * 10) + 1):
        rot = i / 10
        mp = moving_poly(rot)
        n_in = sum(inside(p, mp) for p in pts)
        if first is None and n_in:
            first = rot
        if n_in == len(pts):
            last = rot
            break
    return first, last


def tip_passes_hook():
    """Smallest rotation at which the moving tip is past the hook edge (inside the hook blade outline or beyond it)."""
    fixed = fixed_poly()
    tip_idx = G.MOVING_OUTLINE.index(G.MOVING_EDGE[-1])
    for i in range(0, 900):
        rot = i / 10
        tip = moving_poly(rot)[tip_idx]
        if inside(tip, fixed):
            return rot
    return None


# ---------------------------------------------------------------- four-bar
def lever_pin(theta, lever_r=G.LEVER_R, phase=G.LEVER_PHASE):
    a = math.radians(phase + G.CLOSE_SIGN * theta)
    return G.PIVOT[0] + lever_r * math.cos(a), G.PIVOT[1] + lever_r * math.sin(a)


def horn_pin(q, s=G.SERVO_SHAFT, horn_r=G.HORN_R, link=G.LINK_L, branch=G.HORN_BRANCH):
    """Horn pin position for lever pin q: circle(s, horn_r) meets circle(q, link). None if no assembly."""
    dx, dy = q[0] - s[0], q[1] - s[1]
    d = math.hypot(dx, dy)
    if d > horn_r + link or d < abs(horn_r - link) or d == 0:
        return None
    a = (horn_r ** 2 - link ** 2 + d ** 2) / (2 * d)
    h = math.sqrt(max(horn_r ** 2 - a ** 2, 0.0))
    mx, my = s[0] + a * dx / d, s[1] + a * dy / d
    return mx - branch * h * dy / d, my + branch * h * dx / d


def stroke(n=101, **kw):
    """Horn angle, mechanical advantage and transmission along the closing stroke."""
    lk = {k: kw[k] for k in ("lever_r", "phase") if k in kw}
    hk = {k: kw[k] for k in ("s", "horn_r", "link", "branch") if k in kw}
    s = kw.get("s", G.SERVO_SHAFT)
    rows = []
    for i in range(n):
        th = G.CLOSED_DEG * i / (n - 1)
        q = lever_pin(th, **lk)
        h = horn_pin(q, **hk)
        if h is None:
            return None
        psi = math.degrees(math.atan2(h[1] - s[1], h[0] - s[0]))
        rows.append((th, psi, q, h))
    out = []
    psi0 = rows[0][1]
    unwrapped = [psi0]
    for r in rows[1:]:
        d = (r[1] - unwrapped[-1] + 180) % 360 - 180
        unwrapped.append(unwrapped[-1] + d)
    for i, (th, _, q, h) in enumerate(rows):
        j0, j1 = max(i - 1, 0), min(i + 1, n - 1)
        ma = (unwrapped[j1] - unwrapped[j0]) / (rows[j1][0] - rows[j0][0])
        lx, ly = q[0] - h[0], q[1] - h[1]                       # link direction
        rx, ry = q[0] - G.PIVOT[0], q[1] - G.PIVOT[1]           # lever radius
        hx, hy = h[0] - s[0], h[1] - s[1]                       # horn radius
        ln = math.hypot(lx, ly)
        sin_lever = abs(lx * ry - ly * rx) / (ln * math.hypot(rx, ry))
        sin_horn = abs(lx * hy - ly * hx) / (ln * math.hypot(hx, hy))
        out.append(dict(theta=th, psi=unwrapped[i], ma=ma, sin_lever=sin_lever, sin_horn=sin_horn, q=q, h=h))
    return out


def seg_dist(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    t = ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / max((bx - ax) ** 2 + (by - ay) ** 2, 1e-9)
    t = min(max(t, 0.0), 1.0)
    return math.hypot(p[0] - ax - t * (bx - ax), p[1] - ay - t * (by - ay))


def servo_footprint(s, toward):
    """Servo case corners (tabs included) in xy. The body runs from the shaft toward the angle `toward`."""
    sv = G.SERVO
    off = sv["l"] / 2 - sv["shaft_off"]
    c, si = math.cos(math.radians(toward)), math.sin(math.radians(toward))
    pts = []
    for u in (off - sv["tab_l"] / 2, off + sv["tab_l"] / 2):
        for v in (-sv["w"] / 2, sv["w"] / 2):
            pts.append((s[0] + u * c - v * si, s[1] + u * si + v * c))
    return pts


def servo_yaw(s):
    """The servo body points away from the pivot."""
    return math.degrees(math.atan2(s[1] - G.PIVOT[1], s[0] - G.PIVOT[0]))


BLADE_REAR_R = 21.0      # estimate: furthest reach of either blade behind the pivot (tail lobe), from the outlines
LINK_HALF_W = 4.5        # printed link half-width round its pins


def evaluate(cfg, need_ma):
    st = stroke(**cfg)
    if st is None:
        return None
    psis = [r["psi"] for r in st]
    mas = [r["ma"] for r in st]
    if not (all(m > 0 for m in mas) or all(m < 0 for m in mas)):
        return None                                            # horn reverses: dead point inside the stroke
    span = abs(psis[-1] - psis[0])
    if span > 140:
        return None
    if min(r["sin_lever"] for r in st) < 0.45 or min(r["sin_horn"] for r in st) < 0.15:
        return None
    if min(seg_dist(G.PIVOT, r["h"], r["q"]) for r in st) < BLADE_REAR_R + LINK_HALF_W:
        return None
    s = cfg["s"]
    fp = servo_footprint(s, servo_yaw(s))
    if max(math.hypot(*p) for p in fp) > 41.0 or max(p[0] for p in fp) > 40.0:
        return None
    window = [abs(r["ma"]) for r in st if r["theta"] <= need_ma["until"]]
    rest = [abs(r["ma"]) for r in st if r["theta"] > need_ma["until"]]
    return dict(min_cut_ma=min(window), min_rest_ma=min(rest), span=span)


def search(need_ma):
    best = None
    for sx in np.arange(-14, 36, 2.0):
        for sy in np.arange(-34, 34, 2.0):
            s = (float(sx), float(sy))
            if math.hypot(*s) > 30:
                continue
            for lever_r in (28.0, 30.0, 32.0, 34.0, 36.0, 38.0):
                for phase in np.arange(120, 245, 5.0):
                    for link in np.arange(20, 72, 2.0):
                        for branch in (1, -1):
                            cfg = dict(s=s, lever_r=lever_r, phase=float(phase), link=float(link), branch=branch)
                            ev = evaluate(cfg, need_ma)
                            if ev is None or ev["min_rest_ma"] < 0.6:
                                continue
                            key = (min(ev["min_cut_ma"], need_ma["target"] * 1.3), -ev["span"])
                            if best is None or key > best[0]:
                                best = (key, cfg, ev)
    return best


def blade_torque(ma, servo):
    return G.SERVO_TORQUE[servo] * G.USABLE * G.PIN_EFF * abs(ma) - G.PIVOT_FRICTION


def report():
    first, sever = cut_window()
    tip = tip_passes_hook()
    r_stem = math.hypot(G.STEM_XY[0] - G.PIVOT[0], G.STEM_XY[1] - G.PIVOT[1])
    # the stem is pushed out along the hook as the blade closes; allow for it being cut up to 6 mm further out
    r_cut = r_stem + 6.0
    need_torque = G.CUT_FORCE * r_cut / 1000 + G.PIVOT_FRICTION
    until = (sever or G.CLOSED_DEG) + 8.0
    need = dict(until=until, target=need_torque / (G.SERVO_TORQUE["MG996R"] * G.USABLE * G.PIN_EFF))

    if "--search" in sys.argv:
        best = search(need)
        print("best", best)
        return

    st = stroke()
    assert st is not None, "linkage in cutter_geometry.py does not assemble over the stroke"
    ev = evaluate(dict(s=G.SERVO_SHAFT, lever_r=G.LEVER_R, phase=G.LEVER_PHASE, link=G.LINK_L,
                       branch=G.HORN_BRANCH), need)
    res = {
        "_note": "estimates; blades scaled from the listing photo, servo torque from datasheets",
        "pivot_xy": [round(v, 1) for v in G.PIVOT],
        "stem_xy": list(G.STEM_XY),
        "stem_radius_from_pivot_mm": round(r_stem, 1),
        "cut_starts_deg": first,
        "stem_severed_deg": sever,
        "tip_past_hook_deg": tip,
        "closed_deg": G.CLOSED_DEG,
        "torque_needed_at_blade_Nm": round(need_torque, 2),
        "ma_needed_in_cut_window_mg996r": round(need["target"], 2),
        "linkage_checks_pass": ev is not None,
        "horn_span_deg": round(abs(st[-1]["psi"] - st[0]["psi"]), 1),
        "horn_open_deg": round(st[0]["psi"], 1),
        "horn_closed_deg": round(st[-1]["psi"], 1),
        "min_ma_cut_window": round(min(abs(r["ma"]) for r in st if r["theta"] <= until), 2),
        "min_sin_transmission_lever": round(min(r["sin_lever"] for r in st), 2),
        "min_sin_transmission_horn": round(min(r["sin_horn"] for r in st), 2),
    }
    for servo in ("MG996R", "DS3225MG"):
        f = [blade_torque(r["ma"], servo) / (r_cut / 1000) for r in st if r["theta"] <= until]
        res[f"min_cut_force_N_{servo}"] = round(min(f), 1)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "calcs.json").write_text(json.dumps(res, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(res, indent=2))
    plot(st, until, r_cut)


def plot(st, until, r_cut):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    th = [r["theta"] for r in st]
    fig, ax = plt.subplots(figsize=(7, 4), dpi=130)
    for servo, col in (("MG996R", "#2a7f62"), ("DS3225MG", "#3b6fb6")):
        ax.plot(th, [blade_torque(r["ma"], servo) / (r_cut / 1000) for r in st], color=col, label=f"{servo}, half stall")
    ax.axhline(G.CUT_FORCE, color="#b23a3a", ls="--", lw=1, label="target 40 N (stems need 20-33 N)")
    ax.axvspan(0, until, color="#e8e2c8", alpha=0.6, lw=0, label="stem is being cut")
    ax.set_xlabel("blade closing angle (deg)")
    ax.set_ylabel(f"force at the stem, {r_cut:.0f} mm from the pivot (N)")
    ax.set_ylim(0, None)
    ax.set_xlim(0, G.CLOSED_DEG)
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("Cutter v1: force at the stem along the stroke (estimate)", fontsize=10)
    fig.tight_layout()
    path = ROOT / "docs" / "images" / "cutter_v1_force.png"
    fig.savefig(path)
    print("wrote", path)


if __name__ == "__main__":
    report()
