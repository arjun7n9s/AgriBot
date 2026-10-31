"""Generate the AgriBOT robot description (URDF/xacro) from measured data.

Inputs (all written by the asset tools):
    sim/agribot_description/config/scara_kinematics.json    tools/scara/extract_links.py
    sim/agribot_description/meshes/cutter/cutter_frames.json tools/blender/build_cutter.py
    sim/agribot_description/meshes/rover/rover_dims.json     tools/blender/build_rover.py

Outputs (sim/agribot_description/urdf/):
    scara.urdf.xacro, stock_gripper.urdf.xacro, cutter.urdf.xacro, rover.urdf.xacro
                         xacro macros, one per subassembly
    agribot.urdf.xacro   the whole robot; args tool:=cutter|gripper, mast_pin:=0..4
    agribot.urdf         expanded with the defaults (cutter, mast pin 2), for tools
                         that do not run xacro
    scara_standalone.urdf  arm alone on a fixed base with the stock gripper

    python tools/description/write_urdf.py

Masses marked "estimate" come from mesh volumes or typical part masses; replace
them with weighed values when the hardware exists.
"""
import json
import math
import pathlib
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[2]
PKG = ROOT / "sim/agribot_description"
MESH = "package://agribot_description/meshes"
KIN = json.loads((PKG / "config/scara_kinematics.json").read_text())
CUT = json.loads((PKG / "meshes/cutter/cutter_frames.json").read_text())
ROV = json.loads((PKG / "meshes/rover/rover_dims.json").read_text())

SCARA_LINKS = ["base_link", "column_link", "carriage_link", "arm2_link", "wrist_link"]
# effort (N*m or N), velocity (rad/s or m/s), damping, friction. Estimates from the
# procured motors (docs/already-procured.md) and the belt and lead-screw ratios.
# Z: 8 mm-lead screw at ~750 rpm gives 0.1 m/s.
SCARA_DYN = {"j1": (2.0, 1.0, 0.5, 0.05), "z": (100.0, 0.1, 2.0, 0.2),
             "j2": (3.0, 1.2, 0.3, 0.05), "j3": (0.8, 2.0, 0.05, 0.01)}
MAST_PINS = [0.10 + 0.18 * i for i in range(5)]     # must match build_rover.py


# ------------------------------------------------------------------ xml helpers

def f(x):
    return f"{x:.6g}"


def v(seq):
    return " ".join(f(x) for x in seq)


def origin(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return f'<origin xyz="{v(xyz)}" rpy="{v(rpy)}"/>'


def inertial(mass, com=(0, 0, 0), ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0, note=""):
    c = f"\n      <!-- {note} -->" if note else ""
    return f"""
    <inertial>{c}
      {origin(com)}
      <mass value="{f(mass)}"/>
      <inertia ixx="{f(ixx)}" ixy="{f(ixy)}" ixz="{f(ixz)}" iyy="{f(iyy)}" iyz="{f(iyz)}" izz="{f(izz)}"/>
    </inertial>"""


def box_inertial(mass, size, com=(0, 0, 0), note=""):
    x, y, z = size
    return inertial(mass, com, mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12,
                    mass * (x * x + y * y) / 12, note=note)


def cyl_inertial_y(mass, r, h, note=""):
    """Cylinder with its axis along y (wheels)."""
    i_side = mass * (3 * r * r + h * h) / 12
    return inertial(mass, (0, 0, 0), i_side, mass * r * r / 2, i_side, note=note)


def mesh_visual(path, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return f"""
    <visual>
      {origin(xyz, rpy)}
      <geometry><mesh filename="{MESH}/{path}"/></geometry>
    </visual>"""


def box_collision(size, xyz=(0, 0, 0), rpy=(0, 0, 0), name=""):
    n = f' name="{name}"' if name else ""
    return f"""
    <collision{n}>
      {origin(xyz, rpy)}
      <geometry><box size="{v(size)}"/></geometry>
    </collision>"""


def link(name, body=""):
    return f'\n  <link name="{name}">{body}\n  </link>' if body else f'\n  <link name="{name}"/>'


def joint(name, kind, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0), axis=None, limit=None,
          dyn=None, note=""):
    out = f'\n  <joint name="{name}" type="{kind}">'
    if note:
        out += f"\n    <!-- {note} -->"
    out += f'\n    <parent link="{parent}"/>\n    <child link="{child}"/>\n    {origin(xyz, rpy)}'
    if axis:
        out += f'\n    <axis xyz="{v(axis)}"/>'
    if limit:
        lo, hi, effort, vel = limit
        lim = f'effort="{f(effort)}" velocity="{f(vel)}"'
        if lo is not None:
            lim = f'lower="{f(lo)}" upper="{f(hi)}" ' + lim
        out += f"\n    <limit {lim}/>"
    if dyn:
        out += f'\n    <dynamics damping="{f(dyn[0])}" friction="{f(dyn[1])}"/>'
    return out + "\n  </joint>"


def optical(p, name):
    """Camera optical frame (z forward, x right, y down) under <name>."""
    return (link(f"{p}{name}_optical_frame")
            + joint(f"{p}{name}_optical_joint", "fixed", f"{p}{name}", f"{p}{name}_optical_frame",
                    rpy=(-math.pi / 2, 0, -math.pi / 2)))


# ----------------------------------------------------------------- subassemblies

def scara(p):
    out = []
    for name in SCARA_LINKS:
        i = KIN["inertial"][name]
        body = (inertial(i["mass"], i["com"], i["ixx"], i["iyy"], i["izz"], i["ixy"], i["ixz"],
                         i["iyz"], note="estimate: " + KIN["inertial_source"])
                + mesh_visual(f"scara/visual/{name}.obj")
                + f'\n    <collision>\n      <geometry><mesh filename="{MESH}/scara/collision/{name}.stl"/></geometry>\n    </collision>')
        out.append(link(f"{p}{name}", body))
    out.append(link(f"{p}tool_flange"))
    for jn in ("j1", "z", "j2", "j3"):
        j = KIN["joints"][jn]
        effort, vel, damp, fric = SCARA_DYN[jn]
        out.append(joint(f"{p}{jn}", j["type"], f"{p}{j['parent']}", f"{p}{j['child']}",
                         j["xyz"], j["rpy"], j["axis"], (j["limit"][0], j["limit"][1], effort, vel),
                         (damp, fric), note=f"limits: {j['limit_source']}; effort/velocity: estimate"))
    j = KIN["joints"]["tool_flange"]
    out.append(joint(f"{p}tool_flange_joint", "fixed", f"{p}wrist_link", f"{p}tool_flange",
                     j["xyz"], j["rpy"], note="J3 coupler underside; tools bolt on here"))
    return "".join(out)


def stock_gripper(p):
    i = KIN["inertial"]["stock_gripper"]
    body = (inertial(i["mass"], i["com"], i["ixx"], i["iyy"], i["izz"], i["ixy"], i["ixz"], i["iyz"],
                     note="estimate from mesh volumes")
            + mesh_visual("scara/visual/stock_gripper.obj")
            + f'\n    <collision>\n      <geometry><mesh filename="{MESH}/scara/collision/stock_gripper.stl"/></geometry>\n    </collision>')
    t = KIN["joints"]["stock_gripper_tool0"]
    return (link(f"{p}stock_gripper", body) + link(f"{p}tool0")
            + joint(f"{p}stock_gripper_mount", "fixed", f"{p}tool_flange", f"{p}stock_gripper")
            + joint(f"{p}stock_gripper_tool0", "fixed", f"{p}stock_gripper", f"{p}tool0", t["xyz"], t["rpy"],
                    note="bottom of the gripper fingers"))


def cutter(p):
    cb = CUT["collision_boxes"]
    body_cols = "".join(box_collision(cb[k]["size"], cb[k]["xyz"], name=k)
                        for k in ("top_plate", "spine", "wall_right", "wall_left", "servo1", "camera"))
    body = (box_inertial(0.28, (0.13, 0.09, 0.15), com=(0.03, 0, -0.07),
                         note="estimate: two MG996R, PETG body, camera (docs/scara-redesign.md)")
            + mesh_visual("cutter/visual/cutter_body.obj") + body_cols)
    blade = (box_inertial(0.008, (0.03, 0.01, 0.004), com=(0.01, 0, 0.001), note="estimate")
             + mesh_visual("cutter/visual/cutter_blade.obj"))
    door = (box_inertial(0.03, (0.104, 0.083, 0.011), com=(-0.052, 0, 0.004), note="estimate")
            + mesh_visual("cutter/visual/cutter_door.obj")
            + box_collision(cb["door"]["size"], cb["door"]["xyz"], name="floor")
            + box_collision(cb["lip"]["size"], cb["lip"]["xyz"], name="lip"))
    bj, dj, cam = CUT["blade_joint"], CUT["door_joint"], CUT["camera_link"]
    return (link(f"{p}cutter_body", body) + link(f"{p}cutter_blade", blade)
            + link(f"{p}cutter_door", door) + link(f"{p}cut_point") + link(f"{p}tool0")
            + link(f"{p}arm_camera_link") + optical(p, "arm_camera_link")
            + joint(f"{p}cutter_mount", "fixed", f"{p}tool_flange", f"{p}cutter_body")
            + joint(f"{p}cutter_blade_joint", "revolute", f"{p}cutter_body", f"{p}cutter_blade",
                    bj["xyz"], bj["rpy"], bj["axis"], (bj["limit"][0], bj["limit"][1], 0.9, 3.0),
                    (0.01, 0.002), note="0 = open, lower limit = fully closed across the throat")
            + joint(f"{p}cutter_door_joint", "revolute", f"{p}cutter_body", f"{p}cutter_door",
                    dj["xyz"], dj["rpy"], dj["axis"], (dj["limit"][0], dj["limit"][1], 0.9, 3.0),
                    (0.01, 0.002), note="0 = closed; positive swings the floor down to drop the fruit")
            + joint(f"{p}cut_point_joint", "fixed", f"{p}cutter_body", f"{p}cut_point", CUT["cut_point"],
                    note="centre of the 6 mm throat: the pedicel is cut here")
            + joint(f"{p}tool0_joint", "fixed", f"{p}cutter_body", f"{p}tool0", CUT["cut_point"])
            + joint(f"{p}arm_camera_joint", "fixed", f"{p}cutter_body", f"{p}arm_camera_link",
                    cam["xyz"], cam["rpy"], note="RGB-D camera front face, pitched down"))


def rover(p):
    R, m = ROV["wheel_radius"], ROV["mounts"]
    wb, tr = ROV["assumed"]["wheelbase"], ROV["assumed"]["track"]
    steer = math.radians(ROV["assumed"]["steer_limit_deg"])
    col = ROV["collision"]["body"]
    out = [link(f"{p}base_footprint"),
           link(f"{p}base_link", box_inertial(30.0, col["size"], com=col["xyz"],
                                              note="estimate: ride-on chassis with batteries")
                + mesh_visual("rover/visual/rover_body.obj") + box_collision(col["size"], col["xyz"])),
           joint(f"{p}base_joint", "fixed", f"{p}base_footprint", f"{p}base_link", (0, 0, R)),
           link(f"{p}steer_link", box_inertial(0.5, (0.05, tr - 0.2, 0.03), note="estimate")),
           joint(f"{p}steer_joint", "revolute", f"{p}base_link", f"{p}steer_link", (wb / 2, 0, 0),
                 axis=(0, 0, 1), limit=(-steer, steer, 20.0, 1.0), dyn=(1.0, 0.1),
                 note="one steering motor, parallel steer until the real linkage is measured; "
                      "wheelbase, track and limit are assumed (docs/work-plan.md)")]
    wheel = (cyl_inertial_y(1.5, R, ROV["measured"]["tyre_w"], note="estimate")
             + mesh_visual("rover/visual/wheel.obj")
             + f'\n    <collision>\n      {origin(rpy=(math.pi / 2, 0, 0))}\n      <geometry><cylinder radius="{f(R)}" length="{f(ROV["measured"]["tyre_w"])}"/></geometry>\n    </collision>')
    for name, parent, xyz in (("front_left", "steer_link", (0, tr / 2, 0)),
                              ("front_right", "steer_link", (0, -tr / 2, 0)),
                              ("rear_left", "base_link", (-wb / 2, tr / 2, 0)),
                              ("rear_right", "base_link", (-wb / 2, -tr / 2, 0))):
        out += [link(f"{p}{name}_wheel", wheel),
                joint(f"{p}{name}_wheel_joint", "continuous", f"{p}{parent}", f"{p}{name}_wheel", xyz,
                      axis=(0, 1, 0), limit=(None, None, 40.0, 6.0), dyn=(0.2, 0.05))]
    # Sensors and fixtures.
    for name, xyz, yaw in (("front_camera", m["front_camera"], 0.0),
                           ("rear_camera", m["rear_camera"], math.pi)):
        out += [link(f"{p}{name}_link"), optical(p, f"{name}_link"),
                joint(f"{p}{name}_joint", "fixed", f"{p}base_link", f"{p}{name}_link",
                      (xyz[0] + 0.022 * math.cos(yaw), xyz[1], xyz[2]), (0, 0, yaw))]
    out += [link(f"{p}imu_link"), joint(f"{p}imu_joint", "fixed", f"{p}base_link", f"{p}imu_link", m["imu"]),
            link(f"{p}harvest_crate", box_inertial(0.6, (0.4, 0.3, 0.2), com=(0, 0, 0.1), note="estimate, empty")
                 + mesh_visual("rover/visual/harvest_crate.obj")
                 + box_collision((0.40, 0.30, 0.008), (0, 0, 0.004), name="floor")
                 + box_collision((0.40, 0.008, 0.20), (0, 0.146, 0.10), name="wall_left")
                 + box_collision((0.40, 0.008, 0.20), (0, -0.146, 0.10), name="wall_right")
                 + box_collision((0.008, 0.30, 0.20), (0.196, 0, 0.10), name="wall_front")
                 + box_collision((0.008, 0.30, 0.20), (-0.196, 0, 0.10), name="wall_rear")),
            joint(f"{p}harvest_crate_joint", "fixed", f"{p}base_link", f"{p}harvest_crate", m["harvest_crate"]),
            link(f"{p}scout_mast", box_inertial(0.4, (0.04, 0.04, 0.5), com=(0, 0, 0.25), note="estimate")
                 + mesh_visual("rover/visual/scout_mast.obj")),
            joint(f"{p}scout_mast_joint", "fixed", f"{p}base_link", f"{p}scout_mast", m["scout_mast"]),
            link(f"{p}scout_pan_link", box_inertial(0.15, (0.07, 0.09, 0.08), com=(0, 0, 0.05), note="estimate")
                 + mesh_visual("rover/visual/scout_head.obj")),
            joint(f"{p}scout_pan_joint", "revolute", f"{p}scout_mast", f"{p}scout_pan_link", (0, 0, 0.50),
                  axis=(0, 0, 1), limit=(-math.pi, math.pi, 1.0, 1.5), dyn=(0.05, 0.01),
                  note="pan mechanism for the rotating scouting camera"),
            link(f"{p}scout_camera_link"), optical(p, "scout_camera_link"),
            joint(f"{p}scout_camera_joint", "fixed", f"{p}scout_pan_link", f"{p}scout_camera_link", (0.04, 0, 0.06)),
            link(f"{p}scara_mast", box_inertial(2.5, (0.08, 0.04, 0.95), com=(0, 0, 0.475), note="estimate")
                 + mesh_visual("rover/visual/scara_mast.obj")
                 + box_collision((0.08, 0.04, 0.95), (0, 0, 0.475))),
            joint(f"{p}scara_mast_joint", "fixed", f"{p}base_link", f"{p}scara_mast", m["scara_mast"],
                  note="side mast; the SCARA base pins on at one of five heights")]
    return "".join(out)


def scara_on_mast(p, pin_z):
    """SCARA base pinned to the mast: J1 axis 95 mm outboard of the rail centre.
    The base's long side (with the electronics box) runs toward the rover's
    rear, along the body, so nothing but the arm reaches into the plant row."""
    return joint(f"{p}scara_mount", "fixed", f"{p}scara_mast", f"{p}scara_base_link",
                 (0, -0.095, pin_z), (0, 0, 0))


# ------------------------------------------------------------------- assembly

HEADER = """<?xml version="1.0"?>
<!--
  AgriBOT robot description. Generated by tools/description/write_urdf.py from
  measured and estimated data; edit the generator, not this file.
  {note}
-->
"""


def macro_file(name, params, body, note):
    return (HEADER.format(note=note)
            + f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
            + f'<xacro:macro name="{name}" params="{params}">{body}\n</xacro:macro>\n</robot>\n')


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print("wrote", path.relative_to(ROOT))


def check(path):
    """Structural check: one root, every joint parent/child exists, meshes exist."""
    r = ET.parse(path).getroot()
    links = {l.get("name") for l in r.findall("link")}
    children = set()
    for j in r.findall("joint"):
        par, ch = j.find("parent").get("link"), j.find("child").get("link")
        assert par in links and ch in links, (j.get("name"), par, ch)
        assert ch not in children, f"{ch} has two parents"
        children.add(ch)
    roots = links - children
    assert len(roots) == 1, f"expected one root link, got {roots}"
    for m in r.iter("mesh"):
        rel = m.get("filename").replace("package://agribot_description/", "")
        assert (PKG / rel).exists(), f"missing mesh {rel}"
    print(f"  ok: {len(links)} links, {len(r.findall('joint'))} joints, root {roots.pop()}")


def main():
    out = PKG / "urdf"
    sc = KIN["link_lengths"]
    write(out / "scara.urdf.xacro", macro_file(
        "agribot_scara", "prefix", scara("${prefix}"),
        f"SCARA arm (HowToMechatronics design, measured from its STEP): L1 {sc['L1']} m, "
        f"L2 {sc['L2']} m, Z stroke {KIN['joints']['z']['limit'][1]} m."))
    write(out / "stock_gripper.urdf.xacro", macro_file(
        "agribot_stock_gripper", "prefix", stock_gripper("${prefix}"), "Stock HowToMechatronics gripper."))
    write(out / "cutter.urdf.xacro", macro_file(
        "agribot_cutter", "prefix", cutter("${prefix}"),
        "V-capture bypass cutter, concept v0 (docs/scara-redesign.md, section 3)."))
    write(out / "rover.urdf.xacro", macro_file(
        "agribot_rover", "prefix", rover("${prefix}"),
        "Ride-on rover chassis at measured size; wheelbase, track and steer limit assumed."))

    pins = " ".join(f"{z:.2f}" for z in MAST_PINS)
    top = (HEADER.format(note="Whole robot. Args: tool:=cutter|gripper, mast_pin:=0..4 "
                              f"(SCARA base {pins} m above the mast rail bottom).")
           + """<robot name="agribot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="tool" default="cutter"/>
  <xacro:arg name="mast_pin" default="2"/>
  <xacro:property name="mast_pins" value="${[""" + ", ".join(f"{z:.2f}" for z in MAST_PINS) + """]}"/>

  <xacro:include filename="$(find agribot_description)/urdf/rover.urdf.xacro"/>
  <xacro:include filename="$(find agribot_description)/urdf/scara.urdf.xacro"/>
  <xacro:include filename="$(find agribot_description)/urdf/cutter.urdf.xacro"/>
  <xacro:include filename="$(find agribot_description)/urdf/stock_gripper.urdf.xacro"/>

  <xacro:agribot_rover prefix=""/>
  <xacro:agribot_scara prefix="scara_"/>
  <joint name="scara_mount" type="fixed">
    <parent link="scara_mast"/>
    <child link="scara_base_link"/>
    <origin xyz="0 -0.095 ${mast_pins[$(arg mast_pin)]}" rpy="0 0 0"/>
  </joint>
  <xacro:if value="${'$(arg tool)' == 'cutter'}">
    <xacro:agribot_cutter prefix="scara_"/>
  </xacro:if>
  <xacro:if value="${'$(arg tool)' == 'gripper'}">
    <xacro:agribot_stock_gripper prefix="scara_"/>
  </xacro:if>
</robot>
""")
    write(out / "agribot.urdf.xacro", top)

    expanded = (HEADER.format(note="Expanded defaults: tool = cutter, mast_pin = 2.")
                + '<robot name="agribot">' + rover("") + scara("scara_")
                + scara_on_mast("", MAST_PINS[2]) + cutter("scara_") + "\n</robot>\n")
    write(out / "agribot.urdf", expanded)
    check(out / "agribot.urdf")

    standalone = (HEADER.format(note="SCARA alone, fixed to the world, with the stock gripper.")
                  + '<robot name="agribot_scara">' + link("world")
                  + joint("world_mount", "fixed", "world", "base_link")
                  + scara("") + stock_gripper("") + "\n</robot>\n")
    write(out / "scara_standalone.urdf", standalone)
    check(out / "scara_standalone.urdf")
    for x in ("scara", "stock_gripper", "cutter", "rover", "agribot"):
        ET.parse(out / f"{x}.urdf.xacro")
    print("  ok: xacro files are well-formed XML")


if __name__ == "__main__":
    main()
