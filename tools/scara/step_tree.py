"""Read a STEP assembly and list its leaf parts with world placement.

Shared by extract_links.py. Run directly to print the tree:

    .venv/Scripts/python tools/scara/step_tree.py "path/to/SCARA Robot 3D Model.STEP"
"""
import sys

from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool


def _name(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def read_leaves(path):
    """Return a list of (instance_path, part_name, shape_with_world_location)."""
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"could not read {path}")
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    leaves = []

    def walk(label, loc, trail):
        if tool.IsReference_s(label):
            ref = TDF_Label()
            tool.GetReferredShape_s(label, ref)
            loc = loc * tool.GetLocation_s(label)
            walk(ref, loc, trail + [_name(label)])
            return
        if tool.IsAssembly_s(label):
            kids = TDF_LabelSequence()
            tool.GetComponents_s(label, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), loc, trail)
            return
        shape = tool.GetShape_s(label)
        leaves.append(("/".join(t for t in trail if t), _name(label), shape.Moved(loc)))

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    return leaves


if __name__ == "__main__":
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    for inst, name, shape in read_leaves(sys.argv[1]):
        box = Bnd_Box()
        BRepBndLib.Add_s(shape, box)
        lo, hi = box.CornerMin(), box.CornerMax()
        x0, y0, z0, x1, y1, z1 = lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z()
        print(f"{inst[:60]:60s} | {name[:34]:34s} | "
              f"x {x0:7.1f}..{x1:7.1f} y {y0:7.1f}..{y1:7.1f} z {z0:7.1f}..{z1:7.1f}")
