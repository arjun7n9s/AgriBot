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


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    with SolidWorksSession() as s:
        for name, step in JOBS:
            model = s.open(str(step))
            model.ForceRebuild3(False)
            for v in VIEWS:
                bmp = OUT / f"{name}_{v.strip('*').lower()}.bmp"
                save_preview(model, bmp, v, 1800, 1200)
                print(bmp.name)
            s.close(model)


if __name__ == "__main__":
    main()
