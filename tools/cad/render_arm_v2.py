"""Studio renders of SCARA arm v2 and the cutter, from the GLBs that build_arm_v2.py writes.

    blender -b --factory-startup --python tools/cad/render_arm_v2.py

Output: docs/images/arm_v2.png, docs/images/end_effector.png, docs/images/end_effector_below.png
"""
import pathlib
import sys

import bpy

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE.parents[0] / "blender"))
import agri_common as ac  # noqa: E402

PREVIEW = ac.REPO / "cad" / "arm-v2" / "preview"
IMG = ac.REPO / "docs" / "images"

# base colour written by build_arm_v2.py -> (name, hex, roughness, metallic)
LOOKS = [((0.72, 0.74, 0.77), ("aluminium", "#C9CDD2", 0.32, 0.9)),
         ((0.82, 0.83, 0.85), ("stainless", "#D8DADC", 0.22, 1.0)),
         ((0.55, 0.57, 0.60), ("tool_steel", "#8A8F96", 0.3, 1.0)),
         ((0.15, 0.45, 0.25), ("petg", "#2E7D4F", 0.45, 0.0)),
         ((0.10, 0.10, 0.10), ("tpu", "#1E1E1E", 0.8, 0.0)),
         ((0.90, 0.85, 0.60), ("foam", "#E8DDB0", 0.95, 0.0)),
         ((0.20, 0.20, 0.22), ("bought", "#34363A", 0.5, 0.2)),
         ((0.05, 0.05, 0.05), ("belt", "#111111", 0.7, 0.0))]


def restyle(objs):
    cache = {}
    for o in objs:
        if o.type != "MESH":
            continue
        for slot in o.material_slots:
            m = slot.material
            if m is None or not m.use_nodes:
                continue
            bsdf = m.node_tree.nodes.get("Principled BSDF")
            if bsdf is None:
                continue
            c = bsdf.inputs["Base Color"].default_value
            # glTF stores linear colour; compare in sRGB-ish space
            rgb = [x ** (1 / 2.2) for x in c[:3]]
            best = min(LOOKS, key=lambda k: sum((a - b) ** 2 for a, b in zip(k[0], rgb)))
            name, hexc, rough, metal = best[1]
            if name not in cache:
                cache[name] = ac.material(f"v2_{name}", hexc, roughness=rough, metallic=metal)
            slot.material = cache[name]


def load(path):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=str(path))
    objs = [o for o in bpy.data.objects if o not in before and o.type == "MESH"]
    restyle(objs)
    for o in objs:
        ac.shade(o, smooth_deg=30)
    return objs


def main():
    ac.reset_scene()
    objs = load(PREVIEW / "arm_v2.glb")
    ac.studio_render(objs, IMG / "arm_v2.png", view=(0.9, -1.3, 0.9), res=(1600, 1000), lens=55, samples=96)

    ac.reset_scene()
    objs = load(PREVIEW / "end_effector.glb")
    ac.studio_render(objs, IMG / "end_effector.png", view=(1.25, -0.9, 0.85), res=(1600, 1100), lens=70, samples=96)
    ac.reset_scene()
    objs = load(PREVIEW / "end_effector.glb")
    ac.studio_render(objs, IMG / "end_effector_front.png", view=(1.4, 0.35, 0.25), res=(1600, 1100), lens=70,
                     samples=96)


if __name__ == "__main__":
    main()
