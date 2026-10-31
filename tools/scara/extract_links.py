"""Split the HowToMechatronics SCARA STEP into URDF links.

Reads the STEP assembly, assigns every part to a kinematic link, moves each link
into its own joint frame (metres, ROS axes: x forward, z up) and writes one
binary STL per link and material class into build/scara/. Blender
(tools/blender/build_scara.py) turns those into the final visual and collision
meshes.

It also writes build/scara/kinematics.json with the joint origins and limits it
measured. tools/scara/write_urdf.py uses that file.

    .venv/Scripts/python tools/scara/extract_links.py

Frames and zero pose
--------------------
- base_link: on the J1 axis, at the bottom face of the base.
- J1 (base_link -> column_link): at q1 = 0, Arm 1 points along +x of base_link.
- Z (column_link -> carriage_link): prismatic. q = 0 is the lowest position,
  where the Z mount platform rests on the base cover.
- J2 (carriage_link -> arm2_link): at q2 = 0, Arm 2 lies straight along Arm 1.
- J3 (arm2_link -> wrist_link): at q3 = 0, the wrist is in its STEP pose
  relative to Arm 2.
- tool0: on the J3 axis, level with the bottom of the gripper fingers.

Fasteners (bolts, nuts) and the laser add-on are left out.
"""
import json
import math
import pathlib
import sys

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from step_tree import read_leaves  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
STEP = ROOT / "media/scara-arm/HowToMechatronics/scara-robot-arm/SCARA Robot 3D Model.STEP"
OUT = ROOT / "build/scara"

# STEP instance id (NAUOn) -> link. Taken from the part names and positions
# listed by step_tree.py; see docs/assets.md for the reasoning.
LINK_OF = {
    "base_link": [1, 2, 3, 4, 5, 8, 9, 10, 56, 64, 65, 66, 67, 71],
    "column_link": [6, 7, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
                    25, 26, 27, 28, 61, 62, 63],
    "carriage_link": [29, 30, 31, 32, 34, 35, 37, 38, 39, 40, 41, 42, 43, 44, 45,
                      48, 58, 60, 68, 69],
    "arm2_link": [33, 36, 46, 47, 49, 51, 52, 53, 55, 57, 70],
    "wrist_link": [50, 54],
    "stock_gripper": [59],      # HowToMechatronics gripper, bolted to the J3 flange
}

# Material class by part-name keyword, checked in order.
MATERIALS = [
    ("belt", ["GT2 Belt"]),
    ("motor", ["Stepper", "NEMA 17", "Servo Motor"]),
    ("metal", ["Smooth Rod D10", "Lead Screw", "Bearing", "Washer", "rail"]),
    ("switch", ["Micro switch"]),
    ("printed", [""]),
]

# Measured from the STEP (mm, STEP frame, z up). See the module docstring.
J1 = np.array([42.5, 0.0])
J2 = np.array([260.85, 65.55])
J3 = np.array([367.6, -19.5])
BASE_BOTTOM_Z = -35.0
COLUMN_Z = 0.0
CARRIAGE_Z_STEP = 228.6        # Z mount platform underside in the STEP pose
CARRIAGE_Z_MIN = 62.5          # base cover top: platform underside at q = 0
CARRIAGE_UP_CLEAR = 82.9       # J2 motor top to the Z top limit switch
ARM2_Z = 209.6                 # top face of Arm 2
WRIST_Z = 170.6                # underside of Arm 2 cover
FLANGE_Z = 152.6               # J3 coupler underside = tool-flange face
TOOL_Z = 37.6                  # bottom of the stock gripper fingers


# Effective densities (kg/m3) used for the inertial estimate. Printed parts
# assume about 45% of solid PLA (perimeters plus infill); motors are averaged
# over their outline (a 48 mm NEMA 17 is about 0.35 kg).
DENSITY = {"printed": 560.0, "metal": 7850.0, "motor": 3800.0, "belt": 1200.0, "switch": 1500.0}


def mass_props(tris, density):
    """Mass, centre of mass and inertia tensor about the COM of a closed
    triangle mesh (metres), by signed tetrahedra from the origin."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    v = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
    vol = v.sum()
    if abs(vol) < 1e-12:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    com = (v[:, None] * (a + b + c) / 4.0).sum(0) / vol
    # Second moments of each tetrahedron (origin, a, b, c).
    s = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            s[i, j] = (v * (2 * (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j])
                            + a[:, i] * b[:, j] + a[:, j] * b[:, i]
                            + a[:, i] * c[:, j] + a[:, j] * c[:, i]
                            + b[:, i] * c[:, j] + b[:, j] * c[:, i]) / 20.0).sum()
    mass = density * abs(vol)
    s = s * density * np.sign(vol)
    inertia_origin = np.trace(s) * np.eye(3) - s
    d = com
    inertia_com = inertia_origin - mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return mass, com, inertia_com


def combine(parts):
    m = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / m
    inertia = np.zeros((3, 3))
    for pm, pc, pi in parts:
        d = pc - com
        inertia += pi + pm * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    r = lambda x: float(f"{x:.6g}")
    return {"mass": r(m), "com": [r(x) for x in com],
            "ixx": r(inertia[0, 0]), "iyy": r(inertia[1, 1]), "izz": r(inertia[2, 2]),
            "ixy": r(inertia[0, 1]), "ixz": r(inertia[0, 2]), "iyz": r(inertia[1, 2])}


def material_of(name):
    for mat, keys in MATERIALS:
        if any(k.lower() in name.lower() for k in keys):
            return mat
    return "printed"


def triangles(shape, deflection=0.05):
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.3, True)
    tris = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        loc = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(face, loc)
        if poly is not None:
            trsf = loc.Transformation()
            nodes = np.array([[p.X(), p.Y(), p.Z()] for p in
                              (poly.Node(i).Transformed(trsf) for i in range(1, poly.NbNodes() + 1))])
            idx = np.array([poly.Triangle(i).Get() for i in range(1, poly.NbTriangles() + 1)]) - 1
            if face.Orientation() == TopAbs_REVERSED:
                idx = idx[:, [0, 2, 1]]
            tris.append(nodes[idx])
        exp.Next()
    return np.concatenate(tris) if tris else np.zeros((0, 3, 3))


def frame(origin_xy, z, yaw):
    """4x4 world-from-link transform (mm)."""
    c, s = math.cos(yaw), math.sin(yaw)
    m = np.eye(4)
    m[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    m[:3, 3] = [origin_xy[0], origin_xy[1], z]
    return m


def write_stl(path, tris):
    path.parent.mkdir(parents=True, exist_ok=True)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(tris), dtype=[("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"AgriBOT SCARA link mesh".ljust(80, b" "))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def main():
    yaw1 = math.atan2(*(J2 - J1)[::-1])          # Arm 1 direction in the STEP pose
    yaw2 = math.atan2(*(J3 - J2)[::-1])          # Arm 2 direction in the STEP pose
    l1 = float(np.linalg.norm(J2 - J1))
    l2 = float(np.linalg.norm(J3 - J2))

    world_from = {
        "base_link": frame(J1, BASE_BOTTOM_Z, 0.0),
        "column_link": frame(J1, COLUMN_Z, yaw1),
        "carriage_link": frame(J1, CARRIAGE_Z_STEP, yaw1),
        "arm2_link": frame(J2, ARM2_Z, yaw2),
        "wrist_link": frame(J3, WRIST_Z, yaw2),
        "stock_gripper": frame(J3, FLANGE_Z, yaw2),
    }
    link_of_id = {i: link for link, ids in LINK_OF.items() for i in ids}

    groups = {}
    for inst, name, shape in read_leaves(STEP):
        top = int(inst.split("/")[0].replace("NAUO", ""))
        link = link_of_id.get(top)
        if link is None:
            continue
        tris = triangles(shape)
        if not len(tris):
            continue
        inv = np.linalg.inv(world_from[link])
        pts = np.c_[tris.reshape(-1, 3), np.ones(len(tris) * 3)] @ inv.T
        tris = (pts[:, :3] / 1000.0).reshape(-1, 3, 3)          # mm -> m
        groups.setdefault((link, material_of(name)), []).append(tris)

    OUT.mkdir(parents=True, exist_ok=True)
    inertial = {}
    for (link, mat), parts in sorted(groups.items()):
        tris = np.concatenate(parts)
        write_stl(OUT / f"{link}__{mat}.stl", tris)
        print(f"{link:14s} {mat:8s} {len(tris):7d} triangles")
        m, c, i = mass_props(tris, DENSITY[mat])
        acc = inertial.setdefault(link, [])
        acc.append((m, c, i))
    inertial = {link: combine(parts) for link, parts in inertial.items()}

    z_step = CARRIAGE_Z_STEP - CARRIAGE_Z_MIN
    kin = {
        "source": str(STEP.relative_to(ROOT)).replace("\\", "/"),
        "units": "metres, radians",
        "link_lengths": {"L1": round(l1 / 1000, 5), "L2": round(l2 / 1000, 5)},
        "joints": {
            "j1": {"type": "revolute", "parent": "base_link", "child": "column_link",
                   "xyz": [0, 0, (COLUMN_Z - BASE_BOTTOM_Z) / 1000], "rpy": [0, 0, 0],
                   "axis": [0, 0, 1], "limit": [-math.radians(170), math.radians(170)],
                   "limit_source": "assumed; set from the J1 home switch and cabling"},
            "z": {"type": "prismatic", "parent": "column_link", "child": "carriage_link",
                  "xyz": [0, 0, CARRIAGE_Z_MIN / 1000], "rpy": [0, 0, 0], "axis": [0, 0, 1],
                  "limit": [0.0, round((z_step + CARRIAGE_UP_CLEAR) / 1000, 4)],
                  "limit_source": "part clearances in the STEP: platform on base cover, "
                                  "J2 motor at the Z top limit switch"},
            "j2": {"type": "revolute", "parent": "carriage_link", "child": "arm2_link",
                   "xyz": [l1 / 1000, 0, (ARM2_Z - CARRIAGE_Z_STEP) / 1000], "rpy": [0, 0, 0],
                   "axis": [0, 0, 1], "limit": [-math.radians(150), math.radians(150)],
                   "limit_source": "assumed; check Arm 2 against the column"},
            "j3": {"type": "revolute", "parent": "arm2_link", "child": "wrist_link",
                   "xyz": [l2 / 1000, 0, (WRIST_Z - ARM2_Z) / 1000], "rpy": [0, 0, 0],
                   "axis": [0, 0, 1], "limit": [-math.radians(150), math.radians(150)],
                   "limit_source": "assumed; keeps the tool cables from winding up"},
            "tool_flange": {"type": "fixed", "parent": "wrist_link", "child": "tool_flange",
                            "xyz": [0, 0, (FLANGE_Z - WRIST_Z) / 1000], "rpy": [0, 0, 0]},
            "stock_gripper_tool0": {"type": "fixed", "parent": "stock_gripper", "child": "tool0",
                                    "xyz": [0, 0, (TOOL_Z - FLANGE_Z) / 1000], "rpy": [0, 0, 0]},
        },
        "inertial": inertial,
        "inertial_source": "mesh volumes x effective density: " + ", ".join(
            f"{k} {v:g} kg/m3" for k, v in DENSITY.items()) + ". Estimates; weigh the real parts.",
        "step_pose": {"j1": round(yaw1, 5), "z": round(z_step / 1000, 5),
                      "j2": round(yaw2 - yaw1, 5), "j3": 0.0},
        "check_points_step_mm": {"J2": J2.tolist(), "J3": J3.tolist(),
                                 "tool0": [J3[0], J3[1], TOOL_Z]},
        "step_frame_from_base_link_mm": [J1[0], J1[1], BASE_BOTTOM_Z],
    }
    (OUT / "kinematics.json").write_text(json.dumps(kin, indent=2))
    print(json.dumps(kin["link_lengths"]), "Z stroke", kin["joints"]["z"]["limit"][1], "m")


if __name__ == "__main__":
    main()
