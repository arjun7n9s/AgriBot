"""Bring the cutter v1 CAD into SolidWorks with the solidworks-automation skill.

Run after build_cutter.py, on the Windows laptop with SolidWorks open (system Python with pywin32):

    python tools/cad/sw_cutter.py

It imports the generated STEP files, saves SolidWorks parts and an assembly in cad/cutter/solidworks/
(not committed: regenerate them with this script), runs SolidWorks interference detection on the open and
closed assemblies, reads the volumes back, and saves preview images to cad/cutter/solidworks/preview/.
The geometry itself comes from build_cutter.py; change it there, never in SolidWorks.
"""
import json
import os
import sys
from pathlib import Path

SKILL = Path(os.environ.get("SW_SKILL_DIR", Path.home() / ".claude" / "skills" / "solidworks-automation")) / "scripts"
sys.path.insert(0, str(SKILL))

from sw_assembly import get_interference_detection  # noqa: E402
from sw_review import save_preview  # noqa: E402
from sw_session import SolidWorksSession  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "cad" / "cutter"
OUT = SRC / "solidworks"
PREVIEW = OUT / "preview"


def volume_mm3(model):
    total = 0.0
    for body in model.GetBodies2(0, True) or ():
        props = body.GetMassProperties(1.0)           # density 1 -> [cx, cy, cz, volume m3, area, mass, ...]
        total += props[3] * 1e9
    return total


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    PREVIEW.mkdir(parents=True, exist_ok=True)
    report = {"parts": {}, "assemblies": {}}
    with SolidWorksSession() as s:
        for step in sorted((SRC / "step").glob("*.step")):
            pid = step.stem.upper()
            model = s.open(str(step))
            model.ForceRebuild3(False)
            target = OUT / f"{pid}.SLDPRT"
            if not s.save(model, str(target)):
                raise RuntimeError(f"could not save {target}")
            report["parts"][pid] = {"volume_mm3": round(volume_mm3(model), 1), "file": target.name}
            save_preview(model, PREVIEW / f"{pid}_iso.bmp", "isometric", 1200, 900)
            s.close(model)
            print(pid, report["parts"][pid])

        for name in ("cutter_open", "cutter_closed"):
            model = s.open(str(SRC / f"{name}.step"))
            model.ForceRebuild3(False)
            target = OUT / f"{name}.SLDASM"
            if not s.save(model, str(target)):
                raise RuntimeError(f"could not save {target}")
            interference = get_interference_detection(model)
            items = [{k: v for k, v in it.items() if k in ("components", "volume_mm3")} for it in interference["items"]]
            report["assemblies"][name] = {"file": target.name, "interference_count": len(items), "interference": items,
                                          "errors": interference.get("errors", [])}
            for view in ("isometric", "front", "top", "right"):
                save_preview(model, PREVIEW / f"{name}_{view}.bmp", view, 1600, 1000)
            s.close(model)
            print(name, report["assemblies"][name]["interference_count"], "interferences")

    (OUT / "report.json").write_text(json.dumps(report, indent=2, default=str) + "\n", encoding="utf-8")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
