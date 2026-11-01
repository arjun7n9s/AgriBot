"""Visual-only scenery for the demo world: drip lines, a crate stack, a fence and a tree line.

Everything here is a static model made of SDF primitives (boxes, cylinders, spheres,
ellipsoids) with a plain material: no meshes, no textures, no collision. It costs a few
hundred draw calls and nothing in physics, and none of it is within reach of the rover.
Positions use the field layout of make_world.py: beds along x, the furrow at y = pitch / 2.

Every shape is generated here, not copied from a public model.
"""
import math
import random


def pose(x, y, z, yaw=0.0, pitch=0.0):
    return f"{x:.4f} {y:.4f} {z:.4f} 0 {pitch:.4f} {yaw:.4f}"


def material(rgb, shine=0.1):
    r, g, b = rgb
    return (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
            f"<specular>{shine:.2f} {shine:.2f} {shine:.2f} 1</specular></material>")


def visual(name, geometry, rgb, xyz=(0, 0, 0), yaw=0.0, shine=0.1):
    return (f'<visual name="{name}"><pose>{pose(*xyz, yaw)}</pose><geometry>{geometry}</geometry>'
            f"{material(rgb, shine)}</visual>")


def box(sx, sy, sz):
    return f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>"


def cylinder(r, length):
    return f"<cylinder><radius>{r:.4f}</radius><length>{length:.4f}</length></cylinder>"


def sphere(r):
    return f"<sphere><radius>{r:.4f}</radius></sphere>"


def ellipsoid(rx, ry, rz):
    return f"<ellipsoid><radii>{rx:.3f} {ry:.3f} {rz:.3f}</radii></ellipsoid>"


def model(name, xyz, visuals, yaw=0.0, pitch=0.0):
    body = "\n        ".join(visuals)
    return (f'    <model name="{name}"><static>true</static><pose>{pose(*xyz, yaw, pitch)}</pose>\n'
            f'      <link name="link">\n        {body}\n      </link>\n    </model>')


def drip_lines(pitch, bed_h, half_length):
    """Two black drip-irrigation tapes along each bed, just above the bed top (cylinders pitched onto the x axis)."""
    return [model(f"drip_b{bed}_{k}", (0, bed * pitch + dy, bed_h + 0.012),
                  [visual("tape", cylinder(0.009, 2 * half_length), (0.05, 0.06, 0.08), shine=0.3)], pitch=math.pi / 2)
            for bed in (0, 1) for k, dy in enumerate((-0.12, 0.12))]


def crate_stack(x, y, yaw=0.0):
    """A wooden pallet with two layers of harvest crates, beside the furrow's far end."""
    wood, blue, rim = (0.55, 0.40, 0.24), (0.14, 0.34, 0.56), (0.20, 0.45, 0.70)
    parts = [visual("deck", box(1.2, 0.8, 0.03), wood, (0, 0, 0.115)),
             visual("stringer_a", box(1.2, 0.09, 0.09), wood, (0, -0.32, 0.055)),
             visual("stringer_b", box(1.2, 0.09, 0.09), wood, (0, 0.0, 0.055)),
             visual("stringer_c", box(1.2, 0.09, 0.09), wood, (0, 0.32, 0.055))]
    for layer in range(2):
        for i, cx in enumerate((-0.30, 0.30)):
            for j, cy in enumerate((-0.20, 0.20)):
                z = 0.13 + 0.10 + layer * 0.20
                parts.append(visual(f"crate_{layer}_{i}_{j}", box(0.40, 0.30, 0.20), blue, (cx, cy, z), shine=0.2))
                parts.append(visual(f"rim_{layer}_{i}_{j}", box(0.41, 0.31, 0.02), rim, (cx, cy, z + 0.10), shine=0.2))
    return model("crate_stack", (x, y, 0), parts, yaw)


def barrels(x, y):
    """Two water barrels."""
    out = []
    for k, (dx, dy, rgb) in enumerate(((0, 0, (0.10, 0.28, 0.55)), (0.62, 0.15, (0.12, 0.42, 0.30)))):
        out.append(model(f"barrel_{k}", (x + dx, y + dy, 0),
                         [visual("body", cylinder(0.27, 0.85), rgb, (0, 0, 0.425), shine=0.35),
                          visual("lid", cylinder(0.275, 0.03), tuple(c * 0.7 for c in rgb), (0, 0, 0.86), shine=0.35)]))
    return out


def fence(y, x0, x1, spacing=3.0):
    """Wooden posts and three taut wires."""
    out = []
    n = max(1, int(round((x1 - x0) / spacing)))
    step = (x1 - x0) / n
    wood, wire = (0.42, 0.31, 0.20), (0.62, 0.64, 0.66)
    for i in range(n + 1):
        out.append(model(f"fence_post_{i}", (x0 + i * step, y, 0), [visual("post", box(0.09, 0.09, 1.15), wood, (0, 0, 0.575))]))
    for k, z in enumerate((0.35, 0.70, 1.05)):
        out.append(model(f"fence_wire_{k}", (0.5 * (x0 + x1), y, z),
                         [visual("wire", cylinder(0.004, x1 - x0), wire, shine=0.5)], pitch=math.pi / 2))
    return out


def trees(rng, ring):
    """Low-poly trees in a loose ring: a trunk and two or three overlapping crowns each."""
    out = []
    greens = [(0.18, 0.36, 0.14), (0.22, 0.42, 0.16), (0.15, 0.30, 0.12), (0.26, 0.40, 0.18)]
    for i, (x, y) in enumerate(ring):
        h = rng.uniform(2.0, 3.2)
        r = rng.uniform(1.1, 1.8)
        base = rng.choice(greens)
        vis = [visual("trunk", cylinder(0.14, h), (0.30, 0.21, 0.14), (0, 0, h / 2))]
        for k in range(rng.choice((2, 3))):
            shade = tuple(min(1.0, c * rng.uniform(0.85, 1.15)) for c in base)
            vis.append(visual(f"crown_{k}", sphere(r * rng.uniform(0.7, 1.0)), shade,
                              (rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), h + rng.uniform(0.0, 0.9))))
        out.append(model(f"tree_{i}", (x, y, 0), vis, yaw=rng.uniform(0, math.tau)))
    return out


def hills():
    """Soft hills at the horizon, in haze colours."""
    out = []
    spec = ((-10, 48, 30, 9, 6, (0.45, 0.55, 0.45)), (14, 55, 38, 10, 8, (0.50, 0.60, 0.52)),
            (-34, 40, 26, 8, 5, (0.42, 0.52, 0.44)), (38, 44, 34, 9, 7, (0.48, 0.58, 0.50)),
            (2, -50, 36, 10, 7, (0.48, 0.58, 0.50)), (-28, -46, 30, 8, 6, (0.44, 0.54, 0.46)),
            (30, -52, 32, 9, 6, (0.50, 0.60, 0.52)), (-52, 4, 28, 8, 6, (0.46, 0.56, 0.48)),
            (56, 10, 30, 9, 7, (0.47, 0.57, 0.49)))
    for i, (x, y, rx, ry, rz, rgb) in enumerate(spec):
        out.append(model(f"hill_{i}", (x, y, -rz * 0.55), [visual("hill", ellipsoid(rx, ry, rz), rgb, shine=0.0)]))
    return out


def headland(x0, x1, y0, y1):
    """A strip of packed gravel across the end of the rows."""
    return [model("headland", ((x0 + x1) / 2, (y0 + y1) / 2, 0.004),
                  [visual("gravel", box(x1 - x0, y1 - y0, 0.008), (0.62, 0.55, 0.43), shine=0.02)])]


def scenery(pitch, bed_h, half_length, seed=11):
    rng = random.Random(seed)
    furrow = pitch / 2
    ring = []
    for i in range(16):                                        # trees on a ring, away from the field and the hills
        a = i / 16 * math.tau + rng.uniform(-0.12, 0.12)
        d = rng.uniform(17, 24)
        x, y = 2 + d * math.cos(a) * 1.35, furrow + d * math.sin(a)
        if abs(y - furrow) < 8 and -8 < x < 12:
            continue
        ring.append((x, y))
    parts = []
    parts += headland(half_length + 0.4, half_length + 3.2, furrow - 4.5, furrow + 4.5)
    parts += drip_lines(pitch, bed_h, half_length)
    parts += [crate_stack(half_length + 2.2, furrow + 1.9, yaw=0.25)]
    parts += barrels(half_length + 1.6, furrow - 2.4)
    parts += fence(furrow - 5.2, -half_length - 4, half_length + 6)
    parts += trees(rng, ring)
    parts += hills()
    return parts
