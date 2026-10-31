"""Turn the robot URDF into the SDF that is spawned in Gazebo, with collision groups.

Fruit models use collide_bitmask 0x02. Here every robot collision gets 0x01,
except the parts that must hold fruit (the cutter's top plate, spine, walls,
trap-door floor and lip, and the harvest crate), which get 0x03. Two collisions
touch only if their bitmasks share a bit, so the arm links pass through fruit on
the trusses (which would sway aside in a real field) while the cradle and the
crate still catch them. Ground and beds keep the default 0xff.
"""
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

CATCH = ("top_plate", "spine", "wall_", "floor", "lip")
ROBOT_MASK, CATCH_MASK = "0x01", "0x03"


def urdf_to_sdf(urdf_text):
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(urdf_text)
    out = subprocess.run(["ign", "sdf", "-p", f.name], capture_output=True, text=True, check=True)
    return out.stdout


def add_collision_groups(sdf_text):
    root = ET.fromstring(sdf_text)
    counts = {CATCH_MASK: 0, ROBOT_MASK: 0}
    for col in root.iter("collision"):
        mask = CATCH_MASK if any(k in col.get("name", "") for k in CATCH) else ROBOT_MASK
        surface = col.find("surface")
        if surface is None:
            surface = ET.SubElement(col, "surface")
        contact = surface.find("contact")
        if contact is None:
            contact = ET.SubElement(surface, "contact")
        bitmask = contact.find("collide_bitmask")
        if bitmask is None:
            bitmask = ET.SubElement(contact, "collide_bitmask")
        bitmask.text = mask
        counts[mask] += 1
    return ET.tostring(root, encoding="unicode"), counts


def build(urdf_text, out_path):
    sdf, counts = add_collision_groups(urdf_to_sdf(urdf_text))
    with open(out_path, "w") as f:
        f.write(re.sub(r"^\s*<\?xml[^>]*>\s*", "", sdf))
    return counts
