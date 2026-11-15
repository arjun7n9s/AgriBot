"""Export the whole stock HowToMechatronics SCARA (without its gripper) to a GLB, for viewing cutter v1 on it.

    .venv-cad/Scripts/python tools/cad/export_stock_arm.py

Reads the third-party stock STEP in media/scara-arm (see CREDITS.md) and writes cad/cutter/preview/stock_arm.glb,
in the stock STEP frame, the same frame as cutter_on_arm.glb from build_cutter.py. The preview folder is
gitignored, so the stock geometry is not committed. The gripper, laser module and Arduino cases are left out.
"""
from pathlib import Path

from build123d import Color, Compound, export_gltf, import_step

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "media" / "scara-arm" / "scara-robot-arm" / "SCARA Robot 3D Model.STEP"
OUT = ROOT / "cad" / "cutter" / "preview" / "stock_arm.glb"
DROP = ("Laser", "Arduino", "Gripper")
PRINTED = ("Base", "Arm", "coupler", "Coupler", "Plate", "Clamp", "Platform", "cover", "Cover", "Pulley")


def main():
    kids = []
    for k in import_step(str(SRC)).children:
        if any(d in k.label for d in DROP):
            continue
        k.color = Color(0.86, 0.86, 0.84) if any(p in k.label for p in PRINTED) else Color(0.35, 0.36, 0.38)
        kids.append(k)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    export_gltf(Compound(children=kids, label="stock_arm"), str(OUT), binary=True,
                linear_deflection=0.1, angular_deflection=0.3)
    print(f"{len(kids)} stock parts -> {OUT}")


if __name__ == "__main__":
    main()
