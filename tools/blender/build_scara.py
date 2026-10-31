"""Build the SCARA link meshes for ROS/Gazebo and Unreal.

Input:  build/scara/<link>__<material>.stl and kinematics.json, written by
        tools/scara/extract_links.py.
Output: sim/agribot_description/meshes/scara/visual/<link>.glb and .obj
        sim/agribot_description/meshes/scara/collision/<link>.stl
        sim/agribot_description/config/scara_kinematics.json
        docs/images/scara_preview.png  (arm posed by forward kinematics)

    blender -b --factory-startup --python tools/blender/build_scara.py
"""
import json
import math
import pathlib
import shutil
import sys

import bpy
from mathutils import Matrix

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import agri_common as ac  # noqa: E402

SRC = ac.REPO / "build/scara"
PKG = ac.REPO / "sim/agribot_description"
OUT = PKG / "meshes/scara"

LINKS = ["base_link", "column_link", "carriage_link", "arm2_link", "wrist_link", "stock_gripper"]
PRINTED_BY_LINK = {
    "base_link": "pla_charcoal",
    "column_link": "pla_charcoal",
    "carriage_link": "pla_green",
    "arm2_link": "pla_green",
    "wrist_link": "pla_green",
    "stock_gripper": "pla_bone",
}
MAT_BY_CLASS = {"metal": "steel", "motor": "motor_black", "belt": "rubber", "switch": "switch"}
# Fraction of triangles kept after planar cleanup, per material class.
COLLAPSE = {"printed": 0.6, "metal": 0.12, "motor": 0.35, "belt": 0.6, "switch": 0.4}


def build_link(link):
    parts = []
    for stl in sorted(SRC.glob(f"{link}__*.stl")):
        cls = stl.stem.split("__")[1]
        obj = ac.import_stl(stl, f"{link}_{cls}")
        ac.cleanup(obj, planar_deg=0.8, collapse=COLLAPSE.get(cls), smooth_deg=32)
        mat = PRINTED_BY_LINK[link] if cls == "printed" else MAT_BY_CLASS[cls]
        ac.assign(obj, ac.pal(mat))
        parts.append(obj)
    return parts


def fk(kin, q):
    """World-from-link matrices for joint values q (dict)."""
    J = kin["joints"]

    def joint_mat(j, val):
        xyz, rpy = J[j]["xyz"], J[j]["rpy"]
        m = Matrix.Translation(xyz) @ Matrix.Rotation(rpy[2], 4, "Z")
        if J[j]["type"] == "revolute":
            m = m @ Matrix.Rotation(val, 4, "Z")
        elif J[j]["type"] == "prismatic":
            m = m @ Matrix.Translation((0, 0, val))
        return m

    w = {"base_link": Matrix.Identity(4)}
    w["column_link"] = w["base_link"] @ joint_mat("j1", q["j1"])
    w["carriage_link"] = w["column_link"] @ joint_mat("z", q["z"])
    w["arm2_link"] = w["carriage_link"] @ joint_mat("j2", q["j2"])
    w["wrist_link"] = w["arm2_link"] @ joint_mat("j3", q["j3"])
    w["tool_flange"] = w["wrist_link"] @ joint_mat("tool_flange", 0)
    w["stock_gripper"] = w["tool_flange"]
    w["tool0"] = w["stock_gripper"] @ joint_mat("stock_gripper_tool0", 0)
    return w


def check_fk(kin):
    """FK at the STEP pose must land J3 and tool0 where the STEP has them."""
    w = fk(kin, kin["step_pose"])
    off = kin["step_frame_from_base_link_mm"]
    errs = []
    for name, link, z_pt in (("J3", "wrist_link", None), ("tool0", "tool0", None)):
        p = w[link].translation * 1000
        ref = kin["check_points_step_mm"][name]
        got = (p.x + off[0], p.y + off[1])
        errs.append((name, math.dist(got, ref[:2])))
    tool_z = w["tool0"].translation.z * 1000 + off[2]
    errs.append(("tool0 z", abs(tool_z - kin["check_points_step_mm"]["tool0"][2])))
    for name, e in errs:
        ac.log(f"FK check {name}: {e:.3f} mm")
    assert all(e < 0.5 for _, e in errs), "forward kinematics does not match the STEP"


def main():
    kin = json.loads((SRC / "kinematics.json").read_text())
    check_fk(kin)

    if OUT.exists():
        shutil.rmtree(OUT)
    (PKG / "config").mkdir(parents=True, exist_ok=True)
    shutil.copy(SRC / "kinematics.json", PKG / "config/scara_kinematics.json")

    built = {}
    for link in LINKS:
        ac.reset_scene()
        parts = build_link(link)
        ac.export_all(parts, OUT / "visual" / link)
        hull = ac.convex_hull([p for p in parts if "belt" not in p.name], f"{link}_collision")
        ac.export_stl([hull], OUT / "collision" / f"{link}.stl")
        ac.log(f"{link}: visual {ac.tri_count(parts)} tris, collision {ac.tri_count([hull])} tris")
        built[link] = True

    # Preview: all links posed in a working pose, rendered in the studio.
    ac.reset_scene()
    pose = {"j1": math.radians(-25), "z": 0.12, "j2": math.radians(55), "j3": math.radians(-30)}
    w = fk(kin, pose)
    objs = []
    for link in LINKS:
        before = set(bpy.data.objects)
        bpy.ops.import_scene.gltf(filepath=str(OUT / "visual" / f"{link}.glb"))
        new = [o for o in bpy.data.objects if o not in before and o.type == "MESH"]
        for o in new:
            o.matrix_world = w[link] @ o.matrix_world
        objs += new
    ac.studio_render(objs, ac.REPO / "docs/images/scara_preview.png",
                     view=(1.1, -1.4, 0.75), lens=70, samples=96)
    ac.log("done")


if __name__ == "__main__":
    main()
