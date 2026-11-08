"""Export the already-built stock SCARA parts (base, Z column, carriage, top cap) to a GLB for the web viewer.

    .venv-cad/Scripts/python tools/cad/export_stock_base.py

Reads the third-party stock STEP in media/scara-arm (see CREDITS.md) and writes
cad/arm-v2/preview/stock_base.glb, in the same frame as arm_v2.glb. The preview folder is gitignored,
so the stock geometry is not committed. The stock arms, gripper, laser module and Arduino cases are left out.
"""
from pathlib import Path

from build123d import Color, Compound, export_gltf, import_step

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "media" / "scara-arm" / "scara-robot-arm" / "SCARA Robot 3D Model.STEP"
OUT = ROOT / "cad" / "arm-v2" / "preview" / "stock_base.glb"
DROP = ("Laser", "Arduino", "Gripper")
PRINTED = ("Base", "coupler", "Plate", "Clamp", "Platform", "cover", "Pulley")


def keep(k):
    if any(d in k.label for d in DROP):
        return False
    bb = k.bounding_box()
    return bb.max.X < 120 or bb.max.Z < 140      # the stock Arm 1, Arm 2 and their parts sit beyond both


def main():
    stock = import_step(str(SRC))
    kids = []
    for k in stock.children:
        if keep(k):
            k.color = Color(0.86, 0.86, 0.84) if any(p in k.label for p in PRINTED) else Color(0.35, 0.36, 0.38)
            kids.append(k)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    export_gltf(Compound(children=kids, label="stock_base"), str(OUT), binary=True,
                linear_deflection=0.1, angular_deflection=0.3)
    print(f"{len(kids)} stock parts -> {OUT}")


if __name__ == "__main__":
    main()
