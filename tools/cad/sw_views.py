"""SolidWorks pictures of cutter v1 and of the stock SCARA with the cutter, from several angles.

Run on Windows with SolidWorks open (system Python with pywin32), after build_cutter.py and
export_full_assembly.py:

    python tools/cad/sw_views.py

Writes PNGs to cad/cutter/solidworks/views/ (not committed).
"""
import os
import sys
from pathlib import Path

SKILL = Path(os.environ.get("SW_SKILL_DIR", Path.home() / ".claude" / "skills" / "solidworks-automation")) / "scripts"
sys.path.insert(0, str(SKILL))

from sw_review import save_preview  # noqa: E402
from sw_session import SolidWorksSession  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "cad" / "cutter"
OUT = SRC / "solidworks" / "views"
VIEWS = ("isometric", "*Trimetric", "*Dimetric", "front", "*Back", "right", "*Left", "top", "*Bottom")
JOBS = (("full", SRC / "scara_with_cutter_v1.step"), ("cutter_open", SRC / "cutter_open.step"),
        ("cutter_closed", SRC / "cutter_closed.step"))


def crop_png(bmp):
    """BMP -> PNG cropped to the model, so the pictures are not mostly empty background."""
    from PIL import Image
    im = Image.open(bmp).convert("RGB")
    box = im.convert("L").point(lambda v: 255 if v < 200 else 0).getbbox()
    if box:
        p = 40
        im = im.crop((max(box[0] - p, 0), max(box[1] - p, 0), min(box[2] + p, im.width), min(box[3] + p, im.height)))
    im.save(bmp.with_suffix(".png"))
    bmp.unlink()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    with SolidWorksSession() as s:
        for name, step in JOBS:
            model = s.open(str(step))
            model.ForceRebuild3(False)
            for v in VIEWS:
                bmp = OUT / f"{name}_{v.strip('*').lower()}.bmp"
                save_preview(model, bmp, v, 1800, 1200)
                crop_png(bmp)
                print(bmp.with_suffix(".png").name)
            s.close(model)


if __name__ == "__main__":
    main()
