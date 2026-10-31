"""Hero render: the AgriBOT rover in the tomato field, cutting a ripe fruit.

Poses the robot from sim/agribot_description/urdf/agribot.urdf with the URDF
forward kinematics in tools/description/urdf_fk.py. The arm pose comes from IK
onto a real fruit site: the throat of the cutter meets the fruit's distal
pedicel. Needs the SCARA meshes, which are built locally from the
HowToMechatronics STEP (see docs/assets.md).

    blender -b --factory-startup --python tools/blender/render_hero.py

Output: docs/images/hero_field.png, docs/images/hero_closeup.png
"""
import math
import pathlib
import sys

import bpy
import numpy as np
from mathutils import Matrix

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "description"))
import agri_common as ac  # noqa: E402
import build_field as bf  # noqa: E402
from urdf_fk import Robot, transform, rpy_matrix  # noqa: E402

PKG = ac.REPO / "sim/agribot_description"
URDF = PKG / "urdf/agribot.urdf"
ARM = ["scara_j1", "scara_z", "scara_j2", "scara_j3"]


def mesh_path(uri):
    rel = uri.replace("package://agribot_description/", "")
    return PKG / pathlib.Path(rel).with_suffix(".glb")


_cache = {}


def place_robot(robot, q, base):
    """Import each link's visual meshes and place them at the FK pose."""
    poses = robot.fk(q, base)
    objs = []
    for name in robot.links:
        for uri, org in robot.visuals(name):
            path = mesh_path(uri)
            if path not in _cache:
                _cache[path] = bf.import_glb(path)
                for o in _cache[path]:
                    o.hide_render = True
            world = poses[name] @ org
            for o in _cache[path]:
                c = o.copy()
                bpy.context.scene.collection.objects.link(c)
                c.matrix_world = Matrix(world.tolist()) @ o.matrix_world
                c.hide_render = False
                objs.append(c)
    return objs, poses


def pick_fruit(robot, placed, furrow_y):
    """Find a ripe fruit on the near plant line that the arm can reach with the
    cutter pointing across the row, for some rover x and mast pin."""
    best = None
    for kind, m in placed:
        if kind != "tomato_ripe":
            continue
        knuckle = np.array(m.translation)
        if not (0.3 < knuckle[1] < furrow_y - 0.45 and 1.8 < knuckle[0] < 4.2):
            continue
        target = np.array((m @ Matrix.Translation((0, 0, -bf.PEDICEL / 2))).translation)
        for rover_dx in (-0.06, -0.12, 0.02):
            x_r = target[0] + rover_dx
            base = transform((x_r, furrow_y, 0.0))
            q, e = robot.solve(ARM, "scara_cut_point", target, target_yaw=-math.pi / 2,
                               q0={"scara_j1": -1.0, "scara_z": 0.12, "scara_j2": 1.0}, base=base,
                               iters=80)
            if e > 2e-3:
                continue
            # Prefer an open elbow and a fruit at mid height.
            score = abs(abs(q["scara_j2"]) - 1.1) + abs(target[2] - 0.7)
            if best is None or score < best[0]:
                best = (score, x_r, q, kind, target)
            break
    return best[1:] if best else None


def main():
    ac.reset_scene()
    placed = bf.field_scene(n_beds=3, n_segments=3, seed=7)
    furrow_y = bf.LAYOUT["pitch"] / 2

    # Try each mast pin; keep the pose whose fruit sits nearest the middle of the frame.
    chosen = None
    for pin in (2, 3, 1):
        text = URDF.read_text()
        z = [0.10 + 0.18 * i for i in range(5)][pin]
        text = text.replace('xyz="0 -0.095 0.46"', f'xyz="0 -0.095 {z:.2f}"')
        tmp = ac.REPO / "build/agribot_pin.urdf"
        tmp.parent.mkdir(exist_ok=True)
        tmp.write_text(text)
        robot = Robot(tmp)
        found = pick_fruit(robot, placed, furrow_y)
        if found:
            chosen = (robot, pin, found)
            break
    if not chosen:
        raise SystemExit("no reachable ripe fruit found")
    robot, pin, (x_r, q, kind, target) = chosen
    q.update({"scara_cutter_blade_joint": -0.6, "scout_pan_joint": -0.7, "steer_joint": 0.08})
    ac.log(f"mast pin {pin}, rover x {x_r:.2f}, target {np.round(target, 3)}, "
           f"q = {{{', '.join(f'{k}: {v:.3f}' for k, v in q.items() if k in ARM)}}}")
    base = transform((x_r, furrow_y, 0.0))
    objs, poses = place_robot(robot, q, base)

    cut = poses["scara_cut_point"][:3, 3]
    ac.outdoor_render(ac.REPO / "docs/images/hero_field.png",
                      cam_loc=(x_r - 1.75, furrow_y + 0.08, 1.75), target=(x_r - 0.05, furrow_y - 0.40, 0.55),
                      lens=28, res=(1920, 1080), samples=128, sun_elev=17, sun_azim=-40)
    ac.outdoor_render(ac.REPO / "docs/images/hero_closeup.png",
                      cam_loc=(cut[0] + 0.32, cut[1] + 0.30, cut[2] + 0.22), target=tuple(cut - np.array((0, 0, 0.03))),
                      lens=45, res=(1920, 1080), samples=160, sun_elev=22, sun_azim=-40, dof=5.6)
    ac.log("hero done")


if __name__ == "__main__":
    main()
