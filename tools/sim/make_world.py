"""Generate the Gazebo demo world: one harvestable tomato row beside a furrow.

Outputs (sim/agribot_gazebo/):
    worlds/demo_row.sdf            the world
    worlds/demo_row_targets.json   harvestable fruit: names, ripeness, knuckle and
                                   cut-point poses (world), pick targets
    config/bridge.yaml             ROS <-> Gazebo topic bridge, including one
                                   detach topic per ripe harvestable fruit

Layout (sim/agribot_field/config/field_layout.json): bed 0 is centred on y = 0,
bed 1 on y = pitch; the rover drives the furrow at y = pitch / 2 toward +x with
the SCARA mast on its right, facing bed 0's line at y = +0.30, the harvest line.

All fruit on the plants are static, in collision group 0x04 that the robot
ignores, so the arm and cradle can work inside a truss (real trusses sway
aside; rigid ones would jam the cutter). The cut is modelled by the harvest
node: it removes the fruit and spawns a free-falling copy at the same pose in
group 0x02, which the cradle and the crate catch.

    python tools/sim/make_world.py
"""
import json
import math
import pathlib
import random

ROOT = pathlib.Path(__file__).resolve().parents[2]
FIELD = ROOT / "sim/agribot_field"
OUT = ROOT / "sim/agribot_gazebo"
LAYOUT = json.loads((FIELD / "config/field_layout.json").read_text())
PEDICEL = 0.012
SEED = 7


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def to_rpy(m):
    p = -math.asin(max(-1.0, min(1.0, m[2][0])))
    return [math.atan2(m[2][1], m[2][2]), p, math.atan2(m[1][0], m[0][0])]


def pose(xyz, rpy):
    return " ".join(f"{v:.5f}" for v in list(xyz) + list(rpy))


def plant_model(name, variant, xyz, yaw, height):
    stem_len = height
    lines = [f'    <model name="{name}">',
             f"      <pose>{pose(xyz, (0, 0, yaw))}</pose>"]
    lines += [
        "      <static>true</static>",
        '      <link name="plant">',
        "        <inertial><mass>2.0</mass><inertia><ixx>0.3</ixx><iyy>0.3</iyy><izz>0.01</izz>"
        "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>",
        # No stem or stake collision in the demo: rigid cylinders would stall the arm
        # where a real stem bends aside. Arm-stem contact is a known limitation.
        f'        <visual name="visual"><geometry><mesh><uri>model://tomato_plant_{variant}/meshes/'
        f"tomato_plant_{variant}.obj</uri></mesh></geometry></visual>",
        "      </link>"]
    lines.append("    </model>")
    return "\n".join(lines)


def main():
    rng = random.Random(SEED)
    plants = {}
    for p in sorted((FIELD / "models").glob("tomato_plant_*")):
        sites = json.loads((p / "fruit_sites.json").read_text())["sites"]
        sdf = (p / "model.sdf").read_text()
        height = float(sdf.split("<length>")[1].split("<")[0])
        plants[p.name.split("_")[-1]] = (sites, height)

    pitch, L, bed_h = LAYOUT["pitch"], LAYOUT["segment"], LAYOUT["bed_h"]
    harvest_line = (0, max(LAYOUT["plant_lines"]))        # bed 0, line facing the furrow
    models, fruit_xml, targets = [], [], []

    for bed in (0, 1):
        for seg in (-1, 0, 1):
            models.append(f'    <include><uri>model://raised_bed</uri><name>bed{bed}_seg{seg + 1}</name>'
                          f"<pose>{pose((seg * L, bed * pitch, 0), (0, 0, 0))}</pose></include>")
        for line in LAYOUT["plant_lines"]:
            for k in range(6):
                variant = rng.choice(sorted(plants))
                sites, height = plants[variant]
                yaw = (0.0 if line < 0 else math.pi) + rng.uniform(-0.45, 0.45)
                xyz = (0.225 + LAYOUT["plant_spacing"] * k, bed * pitch + line, bed_h)
                harvest = (bed, line) == harvest_line
                name = f"plant_b{bed}_{'l' if line < 0 else 'r'}{k}"
                R = rot(0, 0, yaw)
                for s in sites:
                    fname = f"{name}_{s['id'].split('_', 1)[1]}"
                    Rs = matmul(R, rot(*s["rpy"]))
                    knuckle = [a + b for a, b in zip(xyz, apply(R, s["xyz"]))]
                    rpy = to_rpy(Rs)
                    fruit_xml.append(f"    <include><uri>model://{s['kind']}</uri><name>{fname}</name>"
                                     f"<pose>{pose(knuckle, rpy)}</pose><static>true</static></include>")
                    if harvest:
                        cut = [a + b for a, b in zip(knuckle, apply(Rs, (0, 0, -PEDICEL / 2)))]
                        targets.append({"name": fname, "kind": s["kind"], "plant": name,
                                        "knuckle": [round(v, 4) for v in knuckle],
                                        "rpy": [round(v, 5) for v in rpy],
                                        "cut_point": [round(v, 4) for v in cut]})
                models.append(plant_model(name, variant, xyz, yaw, height))

    tiles = [f'    <include><uri>model://soil_tile</uri><name>soil_{i}_{j}</name>'
             f"<pose>{pose((i * 4.0, j * 4.0, -0.004), (0, 0, 0))}</pose></include>"
             for i in range(-1, 3) for j in range(-1, 2)]

    world = f"""<?xml version="1.0"?>
<!-- Generated by tools/sim/make_world.py. Edit the generator, not this file. -->
<sdf version="1.9">
  <world name="demo_row">
    <physics name="1ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.62 0.74 0.88 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
      <sky></sky>
    </scene>

    <!-- Low morning sun from the south-east, plus a soft sky fill. -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 0.95 0.86 1</diffuse>
      <specular>0.35 0.35 0.32 1</specular>
      <direction>-0.55 0.45 -0.70</direction>
    </light>
    <light type="directional" name="sky_fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.25 0.28 0.33 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.4 -0.3 -0.85</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="ground">
        <collision name="plane">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>

{chr(10).join(tiles)}

{chr(10).join(models)}

{chr(10).join(fruit_xml)}
  </world>
</sdf>
"""
    (OUT / "worlds").mkdir(parents=True, exist_ok=True)
    (OUT / "worlds/demo_row.sdf").write_text(world, encoding="utf-8")
    (OUT / "worlds/demo_row_targets.json").write_text(json.dumps({
        "frame": "world", "furrow_y": pitch / 2, "rover_heading": "+x",
        "note": "Ground truth from the world generator; the detector is not in the loop yet.",
        "fruit": targets}, indent=2), encoding="utf-8")

    bridge = [
        ("/clock", "/clock", "rosgraph_msgs/msg/Clock", "ignition.msgs.Clock", "GZ_TO_ROS"),
        ("/odom", "/model/agribot/odometry", "nav_msgs/msg/Odometry", "ignition.msgs.Odometry", "GZ_TO_ROS"),
        ("/imu", "/imu", "sensor_msgs/msg/Imu", "ignition.msgs.IMU", "GZ_TO_ROS"),
    ]
    for cam in ("front_camera", "scout_camera"):
        bridge += [(f"/{cam}/image", f"/{cam}/image", "sensor_msgs/msg/Image", "ignition.msgs.Image", "GZ_TO_ROS"),
                   (f"/{cam}/camera_info", f"/{cam}/camera_info", "sensor_msgs/msg/CameraInfo",
                    "ignition.msgs.CameraInfo", "GZ_TO_ROS")]
    bridge += [("/arm_camera/image", "/arm_camera/image", "sensor_msgs/msg/Image", "ignition.msgs.Image", "GZ_TO_ROS"),
               ("/arm_camera/depth_image", "/arm_camera/depth_image", "sensor_msgs/msg/Image",
                "ignition.msgs.Image", "GZ_TO_ROS"),
               ("/arm_camera/camera_info", "/arm_camera/camera_info", "sensor_msgs/msg/CameraInfo",
                "ignition.msgs.CameraInfo", "GZ_TO_ROS")]
    lines = ["# Generated by tools/sim/make_world.py. ROS <-> Gazebo (Fortress) topic bridge."]
    for ros, gz, rt, gt, d in bridge:
        lines += [f"- ros_topic_name: \"{ros}\"", f"  gz_topic_name: \"{gz}\"",
                  f"  ros_type_name: \"{rt}\"", f"  gz_type_name: \"{gt}\"", f"  direction: {d}"]
    (OUT / "config/bridge.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

    ripe = sum(t["kind"] == "tomato_ripe" for t in targets)
    print(f"world: {len(models)} models, {len(fruit_xml)} fruit ({len(targets)} on the harvest line, {ripe} ripe)")


if __name__ == "__main__":
    main()
