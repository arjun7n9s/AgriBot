"""Studio renders of the stem cutter (cutter v1), from the GLBs that build_cutter.py writes.

    blender -b --factory-startup --python tools/cad/render_cutter.py

Output: docs/images/cutter_v1.png (open, with a reference tomato), docs/images/cutter_v1_front.png (the jaws
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


def shot(glb, out, view, lens=70):
    ac.reset_scene()
    objs = rv.load(PREVIEW / glb)
    ac.studio_render(objs, IMG / out, view=view, res=(1600, 1100), lens=lens, samples=96)


def main():
    shot("cutter_open.glb", "cutter_v1.png", (1.0, -1.25, 0.7))
    shot("cutter_open.glb", "cutter_v1_front.png", (1.5, 0.25, 0.12))
    shot("cutter_closed.glb", "cutter_v1_closed.png", (0.9, 1.2, 0.75))


if __name__ == "__main__":
    main()
