"""Studio renders of the stem cutter (cutter v1), from the GLBs that build_cutter.py writes.

    blender -b --factory-startup --python tools/cad/render_cutter.py

Output: docs/images/cutter_v1_parts.png (the printed parts, as on the bed), docs/images/cutter_v1_on_arm.png (on the stock SCARA), docs/images/cutter_v1.png (open, with a reference tomato), docs/images/cutter_v1_front.png (the jaws
from the front, low), docs/images/cutter_v1_closed.png
"""
import pathlib
import sys

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[0] / "blender"))
import agri_common as ac  # noqa: E402
import render_arm_v2 as rv  # noqa: E402

PREVIEW = ac.REPO / "cad" / "cutter" / "preview"
IMG = ac.REPO / "docs" / "images"
rv.LOOKS.append(((0.30, 0.48, 0.18), ("stem", "#4E7A2E", 0.6, 0.0)))
rv.LOOKS.append(((0.86, 0.86, 0.84), ("stock_print", "#D9D9D4", 0.55, 0.0)))     # export_stock_arm.py colours
rv.LOOKS.append(((0.35, 0.36, 0.38), ("stock_metal", "#5A5D62", 0.4, 0.6)))


def shot(glb, out, view, lens=70):
    ac.reset_scene()
    objs = rv.load(PREVIEW / glb)
    ac.studio_render(objs, IMG / out, view=view, res=(1600, 1100), lens=lens, samples=96)


def shot_on_arm(out, view, lens):
    """Cutter on the stock arm. Needs export_stock_arm.py (stock geometry, not committed)."""
    if not (PREVIEW / "stock_arm.glb").exists():
        print("stock_arm.glb missing: run tools/cad/export_stock_arm.py; skipping", out)
        return
    ac.reset_scene()
    objs = rv.load(PREVIEW / "stock_arm.glb") + rv.load(PREVIEW / "cutter_on_arm.glb")
    ac.studio_render(objs, IMG / out, view=view, res=(1600, 1100), lens=lens, samples=96)


def main():
    shot("cutter_open.glb", "cutter_v1.png", (1.0, -1.25, 0.7))
    shot("cutter_open.glb", "cutter_v1_front.png", (1.5, 0.25, 0.12))
    shot("cutter_closed.glb", "cutter_v1_closed.png", (0.9, 1.2, 0.75))
    shot_on_arm("cutter_v1_on_arm.png", (1.1, -1.35, 0.55), 50)
    shot("cutter_parts.glb", "cutter_v1_parts.png", (0.25, -1.3, 1.0), lens=60)


if __name__ == "__main__":
    main()
