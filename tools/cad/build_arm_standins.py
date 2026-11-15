"""Printed stand-ins for stock SCARA bought parts that are missing (test only).

    .venv-cad/Scripts/python tools/cad/build_arm_standins.py

T-95: plain bushing in place of the J3 deep-groove bearing 6806 (30 x 42 x 7 mm, stock STEP).
The outer face sits in the Arm 2 bearing seat, and the J3 coupler hub turns inside it. PETG, greased.
Slow test moves only: swap in a real 6806-2RS before running picks.
"""
from pathlib import Path

from build123d import Cylinder, chamfer, export_stl

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "cad" / "arm" / "stl" / "test"

BEARING_ID = 30.0      # mm, measured: stock STEP Ball_Bearing_30x42x7mm
BEARING_OD = 42.0      # mm, measured: stock STEP
BEARING_W = 7.0        # mm, measured: stock STEP
RUN_CLEAR = 0.4        # mm on the diameter, assumed: running fit on the coupler hub for FDM PETG
EDGE = 0.5             # mm chamfer, assumed: lead-in and removes elephant foot


def j3_bushing():
    ring = Cylinder(BEARING_OD / 2, BEARING_W) - Cylinder((BEARING_ID + RUN_CLEAR) / 2, BEARING_W)
    return chamfer(ring.edges(), EDGE)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    part = j3_bushing()
    path = OUT / "T-95_j3_bushing_6806_standin.stl"
    export_stl(part, str(path))
    bb = part.bounding_box().size
    print(f"T-95 {bb.X:.1f} x {bb.Y:.1f} x {bb.Z:.1f} mm, {part.volume:.0f} mm3 -> {path}")
