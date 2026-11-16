"""Full assembly STEP: the stock HowToMechatronics SCARA plus cutter v1 on J3 (open).

    .venv-cad/Scripts/python tools/cad/export_full_assembly.py

Writes cad/cutter/scara_with_cutter_v1.step (not committed: it contains the third-party stock model).
"""
import sys
from pathlib import Path

from build123d import Compound, Pos, Rot, export_step, import_step

sys.path.insert(0, str(Path(__file__).parent))
import build_cutter as B  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DROP = ("Laser", "Arduino", "Gripper")    # stock gripper, laser module and Arduino case: not on AgriBOT
STOCK = ROOT / "media" / "scara-arm" / "scara-robot-arm" / "SCARA Robot 3D Model.STEP"
OUT = ROOT / "cad" / "cutter" / "scara_with_cutter_v1.step"

if __name__ == "__main__":
    stock = import_step(str(STOCK))
    kept = [k for k in stock.children if not any(d in k.label for d in DROP)]
    stock = Compound(label="Stock SCARA", children=kept)
    print(len(kept), "stock parts kept")
    printed = {pid: fn() for pid, (_, _, fn) in B.PRINTED.items()}
    cutter = Pos(*B.J3_STEP) * Rot(0, 0, B.ARM2_YAW) * B.assembly(0.0, printed)
    cutter.label = "Cutter v1"
    full = Compound(label="SCARA with cutter v1", children=[stock, cutter])
    export_step(full, str(OUT))
    print("wrote", OUT, OUT.stat().st_size // 1024, "KB")
