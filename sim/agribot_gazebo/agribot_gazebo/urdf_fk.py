"""Minimal URDF forward kinematics and position IK (numpy only).

Used by the Blender render scripts to pose the robot straight from the
generated URDF, which also checks that the URDF is what we think it is. Not a
replacement for KDL or MoveIt.

    from urdf_fk import Robot
    robot = Robot("sim/agribot_description/urdf/agribot.urdf")
    T = robot.fk({"scara_j1": 0.3, ...})      # {link: 4x4 world-from-link}
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(xyz=(0, 0, 0), rot=np.eye(3)):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = xyz
    return t


def axis_angle(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


class Robot:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {l.get("name"): l for l in root.findall("link")}
        self.joints = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(x) for x in o.get("xyz", "0 0 0").split()] if o is not None else [0, 0, 0]
            rpy = [float(x) for x in o.get("rpy", "0 0 0").split()] if o is not None else [0, 0, 0]
            ax = j.find("axis")
            lim = j.find("limit")
            self.joints[j.get("name")] = dict(
                type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                origin=transform(xyz, rpy_matrix(*rpy)),
                axis=[float(x) for x in ax.get("xyz").split()] if ax is not None else [1, 0, 0],
                lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else -math.inf,
                upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else math.inf)
        children = {j["child"] for j in self.joints.values()}
        self.root = next(l for l in self.links if l not in children)

    def fk(self, q=None, base=np.eye(4)):
        q = q or {}
        out = {self.root: base}
        pending = list(self.joints.items())
        while pending:
            rest = []
            for name, j in pending:
                if j["parent"] not in out:
                    rest.append((name, j))
                    continue
                t = out[j["parent"]] @ j["origin"]
                val = q.get(name, 0.0)
                if j["type"] in ("revolute", "continuous"):
                    t = t @ transform(rot=axis_angle(j["axis"], val))
                elif j["type"] == "prismatic":
                    t = t @ transform(np.asarray(j["axis"]) * val)
                out[j["child"]] = t
            if len(rest) == len(pending):
                raise ValueError("disconnected joints: " + ", ".join(n for n, _ in rest))
            pending = rest
        return out

    def visuals(self, name):
        """[(mesh filename, 4x4 origin)] for a link's visual meshes."""
        out = []
        for vis in self.links[name].findall("visual"):
            m = vis.find("geometry/mesh")
            if m is None:
                continue
            o = vis.find("origin")
            xyz = [float(x) for x in o.get("xyz").split()] if o is not None else [0, 0, 0]
            rpy = [float(x) for x in o.get("rpy").split()] if o is not None else [0, 0, 0]
            out.append((m.get("filename"), transform(xyz, rpy_matrix(*rpy))))
        return out

    def solve(self, joints, frame, target_xyz, target_yaw=None, q0=None, base=np.eye(4),
              iters=200, tol=1e-4):
        """Damped least-squares IK on `joints` so that `frame` reaches target_xyz
        (and, optionally, its x axis points at target_yaw in the world xy plane)."""
        q = dict(q0 or {})
        for n in joints:
            q.setdefault(n, 0.0)

        def err(qq):
            t = self.fk(qq, base)[frame]
            e = list(np.asarray(target_xyz) - t[:3, 3])
            if target_yaw is not None:
                yaw = math.atan2(t[1, 0], t[0, 0])
                e.append(0.2 * math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw)))
            return np.array(e)

        for _ in range(iters):
            e = err(q)
            if np.linalg.norm(e) < tol:
                break
            jac = np.zeros((len(e), len(joints)))
            for k, n in enumerate(joints):
                dq = dict(q)
                dq[n] += 1e-5
                jac[:, k] = (err(q) - err(dq)) / 1e-5
            step = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(len(e)), e)
            for k, n in enumerate(joints):
                j = self.joints[n]
                q[n] = float(np.clip(q[n] + step[k], j["lower"], j["upper"]))
        return q, float(np.linalg.norm(err(q)))
