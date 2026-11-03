"""Numbers behind SCARA arm v2: belts, ratios, reach, cutter force, joint loads.

    .venv-cad/Scripts/python tools/cad/arm_v2_calcs.py

Writes cad/arm-v2/calcs.json and docs/images/arm_v2_reach.png, docs/images/cutter_force.png.
Run build_arm_v2.py first (masses come from cad/arm-v2/parts.csv).
"""
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import arm_v2_geometry as G  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "cad" / "arm-v2"
IMG = ROOT / "docs" / "images"

# ------------------------------------------------------------------ field and rover (assumed)
PLANT_LINE = 305.0        # J1 axis to the plant line, mm: sim layout (J1 at y -0.395 m, line at -0.70 m). Ask Tejesh.
FRUIT_IN, FRUIT_OUT = 150.0, 100.0   # fruit hang from 150 mm outside the plant line to 100 mm beyond it [assumed]
STOP_HALF = 225.0         # one rover stop covers +-225 mm along the row (half the 450 mm plant spacing x 2) [assumed]
J2_LIM = 150.0            # deg, as stock

# ------------------------------------------------------------------ bought-part masses, g [assumed, typical datasheet values]
BOUGHT_G = {"51107": 85, "6806": 19, "6807": 25, "608": 12, "nema17_40": 280, "servo": 60, "camera": 60,
            "idler": 8, "gt2_20t": 9, "belt": 6, "fasteners_arm1": 60, "fasteners_arm2": 40, "fasteners_tool": 25,
            "switch": 3, "wiring_tool": 20}

# ------------------------------------------------------------------ servo torque (usable, at about half of stall) [assumed]
SERVOS = {"MG996R (on hand), 0.5 N m usable": 0.5, "DS3225MG (recommended), 1.2 N m usable": 1.2}
PEDICEL_FORCE = {"3.0 mm": 19.6, "3.5 mm": 26.7, "4.0 mm": 33.0}   # measured, Weng et al. 2024 (distal pedicel)
FORCE_TARGET = 40.0


def belts():
    r = {}
    s1 = G.belt_length(G.IDLER_X - G.J2_MOTOR_X, 20, 80)
    s2 = G.belt_length(G.L1 - G.IDLER_X, 23, 92)
    s3 = G.belt_length(G.J3_BELT_C, 20, 90)
    r["J2 stage 1 (20T-80T, motor to idler)"] = dict(centre=round(G.IDLER_X - G.J2_MOTOR_X, 1), length=round(s1, 1), belt=400)
    r["J2 stage 2 (23T-92T, idler to J2)"] = dict(centre=round(G.L1 - G.IDLER_X, 1), length=round(s2, 1), belt=300)
    r["J3 (20T-90T, motor to J3)"] = dict(centre=round(G.J3_BELT_C, 1), length=round(s3, 1), belt=400,
                                       motor_from_J2=round(G.J3_MOTOR_X, 1))
    return r


def ratios():
    steps = 200 * 16
    out = {}
    for name, ratio, arm in (("J1", 20.0, None), ("J2", 16.0, None), ("J3", 4.5, None)):
        out[name] = dict(ratio=ratio, deg_per_microstep=round(360.0 / steps / ratio, 4))
    out["J1"]["tip_mm_per_microstep_at_full_reach"] = round(math.radians(out["J1"]["deg_per_microstep"]) * (G.L1 + G.L2), 3)
    out["J2"]["tip_mm_per_microstep"] = round(math.radians(out["J2"]["deg_per_microstep"]) * G.L2, 3)
    return out


def ik_ok(xw, yw, l1, l2):
    d2 = xw * xw + yw * yw
    c = (d2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    if abs(c) > 1:
        return False
    return math.degrees(math.acos(c)) <= J2_LIM


def reach(l2, tool, plant_line=None):
    """Fraction of the fruit zone where the tool can reach a cut point, approaching square to the row."""
    plant_line = PLANT_LINE if plant_line is None else plant_line
    ok = tot = 0
    pts_ok, pts_no = [], []
    for i in range(0, 61):
        x = -STOP_HALF + i * (2 * STOP_HALF / 60)
        for j in range(0, 41):
            y = plant_line - FRUIT_IN + j * ((FRUIT_IN + FRUIT_OUT) / 40)
            tot += 1
            good = ik_ok(x, y - tool, G.L1, l2)          # wrist sits `tool` behind the cut point
            ok += good
            (pts_ok if good else pts_no).append((x, y))
    return ok / tot, pts_ok, pts_no


def four_bar_force():
    out = {}
    ths = [G.LEVER_OPEN + i * 0.25 for i in range(int((G.LEVER_CLOSED - G.LEVER_OPEN) / 0.25) + 1)]
    al = [G.servo_angle_for_lever(t)[0] for t in ths]
    ma = [abs((al[i + 1] - al[i]) / (ths[i + 1] - ths[i])) for i in range(len(ths) - 1)]
    r_s = G.EDGE_R / 1000.0
    for name, torque in SERVOS.items():
        out[name] = [(round((ths[i] + ths[i + 1]) / 2, 2), round(torque * ma[i] / r_s, 1)) for i in range(len(ma))]
    span = abs(al[-1] - al[0])
    cut = [m for t, m in zip(ths, ma) if -6 <= t <= 12]
    return out, dict(servo_travel_deg=round(span, 1), min_ratio_in_cut=round(min(cut), 2),
                     edge_radius_mm=G.EDGE_R)


def masses():
    made = {}
    with open(OUT / "parts.csv", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["mass_g_total_est"]:
                made[row["id"]] = float(row["mass_g_total_est"])
    arm1 = sum(v for k, v in made.items() if k.startswith(("A1-", "J2-", "STK"))) + 2 * BOUGHT_G["51107"] \
        + BOUGHT_G["6806"] + 2 * BOUGHT_G["608"] + 2 * BOUGHT_G["idler"] + 2 * BOUGHT_G["belt"] + BOUGHT_G["fasteners_arm1"]
    arm2 = sum(v for k, v in made.items() if k.startswith(("A2-", "J3-"))) + BOUGHT_G["6806"] + BOUGHT_G["6807"] \
        + BOUGHT_G["nema17_40"] + BOUGHT_G["gt2_20t"] + BOUGHT_G["belt"] + BOUGHT_G["fasteners_arm2"]
    tool = sum(v for k, v in made.items() if k.startswith("T-")) + 2 * BOUGHT_G["servo"] + BOUGHT_G["camera"] \
        + BOUGHT_G["switch"] + BOUGHT_G["wiring_tool"] + BOUGHT_G["fasteners_tool"]
    return dict(arm1_g=round(arm1), arm2_g=round(arm2), tool_g=round(tool))


def loads(m):
    g = 9.81
    a2, tl = m["arm2_g"] / 1000, m["tool_g"] / 1000
    fruit = 0.25
    # J2 bending moment from Arm 2 (CoM ~ 0.45 L2, motor at J3_MOTOR_X) + tool + heaviest fruit at L2 + 60 mm
    motor = BOUGHT_G["nema17_40"] / 1000
    m_j2 = g * ((a2 - motor) * 0.45 * G.L2 + motor * G.J3_MOTOR_X + (tl + fruit) * (G.L2 + 60)) / 1000
    # J1 inertia about J1 (point masses), and torque for 3 rad/s^2
    a1 = m["arm1_g"] / 1000
    i_j1 = (a1 * (0.55 * G.L1) ** 2 + a2 * (G.L1 + 0.45 * G.L2) ** 2 + (tl + fruit) * (G.L1 + G.L2 + 60) ** 2) / 1e6
    z_load = g * (a1 + a2 + tl + fruit + 0.5)        # + carriage share
    return dict(J2_moment_Nm=round(m_j2, 2), J1_inertia_kgm2=round(i_j1, 4),
                J1_torque_for_3_rad_s2_Nm=round(i_j1 * 3.0, 3),
                J1_motor_torque_available_Nm=round(0.69 * 20 * 0.85 * 0.5, 2),
                Z_lift_N=round(z_load, 1))


def plot_reach(by_line):
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10.5, 4.6), dpi=150, gridspec_kw=dict(width_ratios=[1.35, 1]))
    line = 355.0
    for (l2, tool), name, col in (((136.5, 64.0), "stock: L2 136.5", "#c0392b"), ((G.L2, G.THROAT_X - 2.0), "v2: L2 200", "#1f77b4")):
        frac, ok, no = reach(l2, tool, line)
        ax.scatter([p[0] for p in ok], [p[1] for p in ok], s=5 if col == "#1f77b4" else 2.5, color=col,
                   alpha=0.45 if col == "#1f77b4" else 0.9, label=f"{name}: {frac*100:.0f}% of the zone", zorder=2 if col == "#c0392b" else 1)
    ax.axhline(line, color="green", lw=1.5, label=f"plant line {line:.0f} mm out")
    ax.add_patch(plt.Rectangle((-STOP_HALF, line - FRUIT_IN), 2 * STOP_HALF, FRUIT_IN + FRUIT_OUT,
                               fill=False, ls="--", color="k", label="fruit zone per stop (assumed)"))
    ax.plot([0], [0], "ko")
    ax.annotate("J1 axis", (0, 0), (8, -18), fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlim(-300, 300)
    ax.set_ylim(-60, 520)
    ax.set_xlabel("along the row, mm")
    ax.set_ylabel("out from the J1 axis, mm")
    ax.set_title("Reachable cut points, tool square to the row", fontsize=10)
    ax.legend(fontsize=7, loc="lower left")
    xs = sorted(by_line)
    bx.plot(xs, [by_line[x]["stock"] * 100 for x in xs], "o-", color="#c0392b", label="stock (L2 136.5)")
    bx.plot(xs, [by_line[x]["v2"] * 100 for x in xs], "o-", color="#1f77b4", label="v2 (L2 200)")
    bx.axvline(PLANT_LINE, color="grey", ls=":", lw=1)
    bx.annotate("sim layout", (PLANT_LINE + 4, 52), fontsize=7, color="grey")
    bx.set_ylim(50, 102)
    bx.set_xlabel("J1 axis to plant line, mm (field number, ask Tejesh)")
    bx.set_ylabel("fruit zone reachable, %")
    bx.set_title("Sensitivity to where the rover drives", fontsize=10)
    bx.legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    fig.savefig(IMG / "arm_v2_reach.png")


def plot_force(curves):
    fig, ax = plt.subplots(figsize=(7.0, 3.8), dpi=150)
    for (name, pts), col in zip(curves.items(), ("#c0392b", "#1f77b4")):
        ax.plot([p[0] for p in pts], [min(p[1], 400) for p in pts], color=col, label=name)
    for name, f in PEDICEL_FORCE.items():
        ax.axhline(f, color="grey", lw=0.8, ls=":")
    ax.annotate("measured pedicel cut force, 3.0 / 3.5 / 4.0 mm (Weng 2024)", (G.LEVER_OPEN + 0.5, 8), fontsize=7, color="grey")
    ax.axhline(FORCE_TARGET, color="k", lw=1, ls="--", label="design target 40 N")
    ax.axvspan(-6, 12, color="orange", alpha=0.12, label="blade crossing the pedicel")
    ax.set_xlabel("blade-lever angle, deg (open -32, closed +18)")
    ax.set_ylabel("force at the pedicel, N")
    ax.set_ylim(0, 260)
    ax.legend(fontsize=7)
    ax.set_title("Cutter force over the stroke", fontsize=10)
    fig.tight_layout()
    fig.savefig(IMG / "cutter_force.png")


def main():
    res = {"belts": belts(), "ratios": ratios()}
    stock = reach(136.5, 64.0)
    v2 = reach(G.L2, G.THROAT_X - 2.0)
    res["reach"] = {"stock_fraction": round(stock[0], 3), "v2_fraction": round(v2[0], 3),
                    "assumptions": dict(plant_line_mm=PLANT_LINE, fruit_in_mm=FRUIT_IN, fruit_out_mm=FRUIT_OUT,
                                        stop_half_mm=STOP_HALF, j2_limit_deg=J2_LIM)}
    for l2 in (160.0, 180.0, 200.0, 228.0):
        res["reach"][f"L2_{int(l2)}"] = round(reach(l2, G.THROAT_X - 2.0)[0], 3)
    res["reach"]["by_plant_line_mm"] = {
        int(d): dict(stock=round(reach(136.5, 64.0, d)[0], 3), v2=round(reach(G.L2, G.THROAT_X - 2.0, d)[0], 3))
        for d in (255.0, 305.0, 355.0, 405.0)}
    curves, fb = four_bar_force()
    res["cutter"] = fb
    for name, pts in curves.items():
        res["cutter"][f"min_force_in_cut_N {name}"] = min(f for t, f in pts if -6 <= t <= 12)
    m = masses()
    res["mass"] = m
    res["loads"] = loads(m)
    (OUT / "calcs.json").write_text(json.dumps(res, indent=2))
    plot_reach(res["reach"]["by_plant_line_mm"])
    plot_force(curves)
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
