"""Build the tomato-field assets: fruit, plants, raised beds and soil.

Everything is procedural and seeded, so a rebuild gives the same assets.
Output is a set of Gazebo models (model.config + model.sdf + meshes) that also
carry .glb files for Unreal:

    sim/agribot_field/models/
        tomato_ripe/  tomato_half/  tomato_green/      harvestable fruit
        tomato_plant_a/ _b/ _c/                         staked plants (no fruit)
                                                        + fruit_sites.json
        raised_bed/                                     2.7 m tileable bed segment
        soil_tile/                                      4 m x 4 m furrow ground
    docs/images/tomatoes.png, tomato_plant.png, field_overview.png

Field layout used by these assets (also written to sim/agribot_field/config/field_layout.json):
    bed top 0.90 m, base 1.10 m, height 0.15 m; furrow 0.90 m; bed pitch 2.00 m
    paired plant lines at +/-0.30 m from the bed centre, 0.45 m apart in the row

    blender -b --factory-startup --python tools/blender/build_field.py
"""
import json
import math
import pathlib
import random
import shutil
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import agri_common as ac  # noqa: E402

PKG = ac.REPO / "sim/agribot_field"
MODELS = PKG / "models"

LAYOUT = dict(bed_top=0.90, bed_base=1.10, bed_h=0.15, furrow=0.90, pitch=2.00,
              plant_lines=[-0.30, 0.30], plant_spacing=0.45, segment=2.70)

FRUIT = {
    # name: equatorial diameter (m), height ratio, mass (kg), semantic label
    "tomato_green": dict(d=0.056, hr=0.86, mass=0.080, label=1),
    "tomato_half": dict(d=0.060, hr=0.85, mass=0.090, label=2),
    "tomato_ripe": dict(d=0.062, hr=0.84, mass=0.095, label=3),
}
PEDICEL = 0.012        # distal pedicel, knuckle to calyx (m)


# ----------------------------------------------------------------- noise / images

def fbm(h, w, octaves=5, seed=0, periodic=True):
    """Fractal value noise in [0, 1], tileable."""
    rng = np.random.default_rng(seed)
    out = np.zeros((h, w))
    amp, total = 1.0, 0.0
    for o in range(octaves):
        gh, gw = 4 * 2 ** o, 8 * 2 ** o
        grid = rng.random((gh, gw))
        ys = np.linspace(0, gh, h, endpoint=False)
        xs = np.linspace(0, gw, w, endpoint=False)
        y0, x0 = np.floor(ys).astype(int), np.floor(xs).astype(int)
        fy, fx = (ys - y0)[:, None], (xs - x0)[None, :]
        fy, fx = fy * fy * (3 - 2 * fy), fx * fx * (3 - 2 * fx)
        y1, x1 = (y0 + 1) % gh, (x0 + 1) % gw
        a = grid[y0][:, x0]
        b = grid[y0][:, x1]
        c = grid[y1][:, x0]
        d = grid[y1][:, x1]
        out += amp * ((a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy)
        total += amp
        amp *= 0.5
    return out / total


def rgb(hexs):
    h = hexs.lstrip("#")
    return np.array([int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)])


def save_image(arr, path):
    """arr: (h, w, 3) sRGB floats in [0, 1], row 0 = top of the image."""
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    h, w, _ = arr.shape
    img = bpy.data.images.new(path.stem, w, h, alpha=False)
    rgba = np.concatenate([np.clip(arr[::-1], 0, 1), np.ones((h, w, 1))], axis=2)
    img.pixels.foreach_set(rgba.astype(np.float32).ravel())
    img.filepath_raw = str(path)
    img.file_format = "PNG"
    img.save()
    img.filepath = str(path)
    return img


def textured_material(name, img, roughness=0.5, coat=0.0, subsurface=0.0, sheen=0.0):
    mat = ac.material(name, "#FFFFFF", roughness=roughness, coat=coat, subsurface=subsurface,
                      sheen=sheen)
    nt = mat.node_tree
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = img
    nt.links.new(tex.outputs["Color"], nt.nodes["Principled BSDF"].inputs["Base Color"])
    return mat


# ------------------------------------------------------------------- tube helper

def tube(name, pts, radii, mat, res=6, smooth=True, caps=True):
    """Smooth tube through points with per-point radius (metres)."""
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = 1.0
    cu.bevel_resolution = max(1, res // 2 - 1)
    cu.use_fill_caps = caps
    cu.resolution_u = 4 if smooth else 1
    sp = cu.splines.new("NURBS" if smooth and len(pts) > 2 else "POLY")
    sp.points.add(len(pts) - 1)
    for p, co, r in zip(sp.points, pts, radii):
        p.co = (*co, 1.0)
        p.radius = r
    if sp.type == "NURBS":
        sp.order_u = min(4, len(pts))
        sp.use_endpoint_u = True
    obj = bpy.data.objects.new(name, cu)
    bpy.context.scene.collection.objects.link(obj)
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.convert(target="MESH")
    obj = bpy.context.active_object
    ac.assign(obj, mat)
    ac.shade(obj, None)
    return obj


# ------------------------------------------------------------------------ fruit

def fruit_shape(spec):
    """Return (R, c, d, zc): equatorial radius, vertical semi-axis, cavity depth,
    centre height (fruit frame: knuckle at origin, fruit below)."""
    R = spec["d"] / 2
    c = R * spec["hr"]
    d = 0.11 * c
    zt = -PEDICEL - 0.0012                  # fruit top (cavity centre)
    zc = zt - (c - d)
    return R, c, d, zc


def fruit_texture(kind, seed):
    h, w = 256, 512
    v = np.linspace(1, 0, h)[:, None] * np.ones((1, w))      # 1 = top (stem end)
    u = np.ones((h, 1)) * np.linspace(0, 1, w)[None, :]
    n1, n2 = fbm(h, w, 5, seed), fbm(h, w, 3, seed + 7)
    lobe = 0.5 + 0.5 * np.cos(2 * np.pi * 5 * u)
    def mix(a, b, t):
        t = np.clip(t, 0, 1)[..., None]
        return a * (1 - t) + b * t
    if kind == "tomato_ripe":
        base = mix(rgb("#B5170F"), rgb("#D32A1C"), n1 * 1.2 - 0.1)
        base = mix(base, rgb("#8F120B"), (v - 0.86) * 6 * (0.6 + 0.4 * n2))
        base = mix(base, rgb("#E2462E"), (n2 - 0.72) * 3)
    elif kind == "tomato_half":
        t = np.clip((v - 0.25) / 0.6, 0, 1) + (n1 - 0.5) * 0.35 + (lobe - 0.5) * 0.12
        base = mix(rgb("#D9472A"), rgb("#EE8E3C"), t * 1.6)
        base = mix(base, rgb("#B8B04A"), (t - 0.62) * 2.6)
        base = mix(base, rgb("#7E9A35"), (v - 0.88) * 7)
    else:
        base = mix(rgb("#9CBF55"), rgb("#86AE45"), n1)
        base = mix(base, rgb("#B9CF77"), (lobe - 0.55) * 0.9 * (1 - v))
        base = mix(base, rgb("#4F7D2A"), (v - 0.78) * 4.5 * (0.7 + 0.3 * n2))
    speck = (fbm(h, w, 2, seed + 3) > 0.83)[..., None] * 0.05
    return np.clip(base + speck, 0, 1)


def build_fruit(kind, out_dir, seed=1):
    spec = FRUIT[kind]
    R, c, d, zc = fruit_shape(spec)
    tex = save_image(fruit_texture(kind, seed), out_dir / "meshes" / f"{kind}.png")
    skin = textured_material(f"{kind}_skin", tex, roughness=0.24, coat=0.5, subsurface=0.04)
    green = ac.material("calyx_green", "#2A6420", roughness=0.55)
    stem_mat = ac.material("pedicel_green", "#44752A", roughness=0.6)

    bpy.ops.mesh.primitive_uv_sphere_add(segments=64, ring_count=32, radius=1.0)
    body = bpy.context.active_object
    body.name = kind
    rng = random.Random(seed)
    phase = rng.uniform(0, math.tau)
    for vtx in body.data.vertices:
        x, y, z = vtx.co
        rho = math.hypot(x, y)
        th = math.atan2(y, x)
        lobe = 1 + 0.032 * math.cos(5 * th + phase) * rho ** 1.5 * (1 - 0.4 * z)
        Z = c * z
        if z > 0:
            Z -= d * math.exp(-(rho / 0.33) ** 2) * (0.6 + 0.4 * z)
        else:
            Z *= 0.97
        vtx.co = (R * lobe * x, R * lobe * y, zc + Z)
    ac.assign(body, skin)
    ac.shade(body, None)

    def surface(r):
        return zc + c * math.sqrt(max(0.0, 1 - (r / R) ** 2)) - d * math.exp(-(r / (0.33 * R)) ** 2)

    # Calyx: six sepals following the shoulder, tips curling up.
    sepals = []
    for k in range(6):
        a = k * math.tau / 6 + rng.uniform(-0.15, 0.15)
        length = rng.uniform(0.013, 0.017)
        bm = bmesh.new()
        rows = []
        for i in range(9):
            t = i / 8
            r = 0.001 + t * length
            z = surface(min(r, R * 0.9)) + 0.0007 + (0.004 * t ** 3 if t > 0.6 else 0)
            w = 0.0038 * (1 - t) ** 0.8 + 0.0002
            cx, cy = r * math.cos(a), r * math.sin(a)
            tx, ty = -math.sin(a), math.cos(a)
            rows.append((bm.verts.new((cx + tx * w / 2, cy + ty * w / 2, z)),
                         bm.verts.new((cx - tx * w / 2, cy - ty * w / 2, z))))
        for (a0, b0), (a1, b1) in zip(rows, rows[1:]):
            bm.faces.new((a0, b0, b1, a1))
        mesh = bpy.data.meshes.new("sepal")
        bm.to_mesh(mesh)
        bm.free()
        o = bpy.data.objects.new("sepal", mesh)
        bpy.context.scene.collection.objects.link(o)
        sol = o.modifiers.new("solid", "SOLIDIFY")
        sol.thickness = 0.0005
        ac.apply_modifiers(o)
        ac.assign(o, green)
        ac.shade(o, None)
        sepals.append(o)
    calyx = ac.join(sepals, "calyx")
    hub = ac.cylinder("calyx_hub", 0.0028, 0.002, loc=(0, 0, surface(0) + 0.001), mat=green,
                      verts=16)

    # Distal pedicel from the calyx up to the knuckle, then the knuckle itself.
    ped = tube("pedicel", [(0, 0, surface(0) + 0.001), (0.0012, 0, -PEDICEL * 0.55),
                           (0.0008, 0, -0.002), (0, 0, 0.0)],
               [0.00175, 0.0017, 0.0018, 0.0019], stem_mat, res=8)
    knuckle = ac.sphere("knuckle", 0.0032, loc=(0, 0, 0.0004), mat=stem_mat, segments=16, rings=8)
    knuckle.scale = (1, 1, 0.85)
    stub = tube("proximal_stub", [(0, 0, 0.0), (0, 0, 0.004)], [0.0019, 0.0019], stem_mat,
                res=8, smooth=False)
    parts = [body, calyx, hub, ped, knuckle, stub]
    return parts, dict(R=R, c=c, zc=zc)


def fruit_sdf(kind, geo):
    spec = FRUIT[kind]
    m, R = spec["mass"], geo["R"]
    i = 0.4 * m * R * R
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{kind}">
    <!-- Model origin = the knuckle (abscission zone). The world attaches the
         fruit to its plant here with a detachable joint; cutting the distal
         pedicel releases it. Semantic label {spec['label']} (config/semantic_labels.yaml). -->
    <link name="fruit">
      <inertial>
        <pose>0 0 {geo['zc']:.5f} 0 0 0</pose>
        <mass>{m}</mass>
        <inertia><ixx>{i:.3e}</ixx><iyy>{i:.3e}</iyy><izz>{i:.3e}</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="body">
        <pose>0 0 {geo['zc']:.5f} 0 0 0</pose>
        <geometry><sphere><radius>{R * 0.95:.4f}</radius></sphere></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
                 <bounce><restitution_coefficient>0.05</restitution_coefficient></bounce></surface>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://{kind}/meshes/{kind}.obj</uri></mesh></geometry>
      </visual>
    </link>
    <frame name="cut_point"><pose relative_to="fruit">0 0 {-PEDICEL / 2:.4f} 0 0 0</pose></frame>
  </model>
</sdf>
"""


# ------------------------------------------------------------------------ plant

def leaflet(name, length, width, mat, rng):
    """Serrated ovate leaflet along +x, folded on the midrib and drooping."""
    n = 12
    bm = bmesh.new()
    left, right = [], []
    for i in range(n + 1):
        t = i / n
        w = width * math.sin(math.pi * t ** 0.75) * (1 - 0.25 * t)
        w *= 1 + 0.10 * math.sin(2 * math.pi * 7 * t) * (t > 0.08) * (t < 0.95)
        x = t * length
        droop = -0.25 * (x / length) ** 2 * length
        fold = 0.18 * w
        left.append(bm.verts.new((x, w / 2, droop + fold)))
        right.append(bm.verts.new((x, -w / 2, droop + fold)))
    mid = [bm.verts.new((i / n * length, 0, -0.25 * (i / n) ** 2 * length)) for i in range(n + 1)]
    for i in range(n):
        bm.faces.new((mid[i], mid[i + 1], left[i + 1], left[i]))
        bm.faces.new((mid[i], right[i], right[i + 1], mid[i + 1]))
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    o = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(o)
    sol = o.modifiers.new("solid", "SOLIDIFY")
    sol.thickness = 0.0006
    ac.apply_modifiers(o)
    ac.assign(o, mat)
    ac.shade(o, None)
    return o


def place(obj, origin, direction, roll=0.0):
    """Orient an object built along +x to point along `direction` at `origin`."""
    d = Vector(direction).normalized()
    q = d.to_track_quat("X", "Z")
    obj.matrix_world = Matrix.Translation(origin) @ q.to_matrix().to_4x4() @ Matrix.Rotation(roll, 4, "X")


def build_plant(variant, seed):
    rng = random.Random(seed)
    leaf_ms = [ac.material("tomato_leaf", "#2B6420", roughness=0.5),
               ac.material("tomato_leaf_dark", "#22561A", roughness=0.52),
               ac.material("tomato_leaf_light", "#357226", roughness=0.48)]
    stem_m = ac.material("tomato_stem", "#3F6E26", roughness=0.65)
    bamboo = ac.material("bamboo", "#C7A66A", roughness=0.55)
    node_m = ac.material("bamboo_node", "#A98A52", roughness=0.6)
    twine = ac.material("jute_twine", "#D6C59C", roughness=0.9)

    height = rng.uniform(1.25, 1.50)
    parts, sites = [], []

    # Main stem: gently zig-zagging, tapering, leaning on the stake.
    nodes_z = np.arange(0.15, height - 0.08, rng.uniform(0.10, 0.12))
    pts, radii = [(0, 0, 0)], [0.0072]
    for i, z in enumerate(nodes_z):
        jit = 0.008 if i % 2 else -0.008
        pts.append((rng.uniform(-0.01, 0.01) + jit, rng.uniform(-0.006, 0.006), z))
        radii.append(0.0072 - 0.0036 * z / height)
    pts.append((pts[-1][0] * 0.5, pts[-1][1], height))
    radii.append(0.003)
    parts.append(tube("stem", pts, radii, stem_m, res=8))

    def stem_at(z):
        zs = [p[2] for p in pts]
        k = max(0, min(len(zs) - 2, int(np.searchsorted(zs, z) - 1)))
        t = (z - zs[k]) / max(1e-6, zs[k + 1] - zs[k])
        return Vector(pts[k]).lerp(Vector(pts[k + 1]), t)

    # Bamboo stake behind the stem (+y), with nodes and three jute ties.
    stake_h = height + 0.30
    parts.append(tube("stake", [(0.0, 0.045, -0.10), (0.0, 0.045, stake_h)], [0.012, 0.0095],
                      bamboo, res=10, smooth=False))
    for z in np.arange(0.12, stake_h, 0.26):
        parts.append(ac.cylinder("stake_node", 0.0128 - 0.003 * z / stake_h, 0.008,
                                 loc=(0, 0.045, z), mat=node_m, verts=20))
    for z in (0.35, 0.75, 1.10):
        if z < height:
            s = stem_at(z)
            bpy.ops.mesh.primitive_torus_add(major_radius=0.03, minor_radius=0.0016,
                                             major_segments=24, minor_segments=6,
                                             location=((s.x) / 2, (s.y + 0.045) / 2, z))
            t = bpy.context.active_object
            t.scale = (0.55, 1.0, 1.0)
            ac.apply_transform(t)
            ac.assign(t, twine)
            parts.append(t)

    # Compound leaves at each node, spiral phyllotaxis.
    az = rng.uniform(0, math.tau)
    truss_nodes = set(range(2, len(nodes_z), 3))
    for i, z in enumerate(nodes_z):
        az += math.radians(137.5)
        base = stem_at(z)
        top_frac = z / height
        # Compound leaf: an arching rachis carrying 3-4 pairs of large leaflets,
        # small interstitial leaflets between them, and a terminal leaflet.
        plen = rng.uniform(0.30, 0.42) * (1.0 - 0.40 * top_frac)
        dir_xy = Vector((math.cos(az), math.sin(az), 0))
        lift = rng.uniform(0.10, 0.22)
        p0 = base
        p1 = base + dir_xy * plen * 0.30 + Vector((0, 0, plen * lift))
        p2 = base + dir_xy * plen * 0.65 + Vector((0, 0, plen * lift * 0.6))
        p3 = base + dir_xy * plen + Vector((0, 0, -plen * rng.uniform(0.05, 0.25)))
        pet = [p0, p1, p2, p3]
        parts.append(tube("petiole", [tuple(p) for p in pet], [0.0034, 0.0028, 0.0021, 0.0014],
                          stem_m, res=6))

        def on_rachis(t):
            # piecewise-linear point along the rachis polyline (t in 0..1)
            seg = min(2, int(t * 3))
            return Vector(pet[seg]).lerp(Vector(pet[seg + 1]), t * 3 - seg)

        mat = rng.choice(leaf_ms)
        scale = rng.uniform(0.85, 1.15) * (1.0 - 0.35 * top_frac)
        n_pairs = rng.choice([4, 4, 5])
        for k in range(n_pairs):
            t = 0.30 + 0.62 * k / (n_pairs - 1 + 0.4)
            size = 0.70 + 0.30 * k / max(1, n_pairs - 1)
            q = on_rachis(t)
            for side in (1, -1):
                side_dir = Vector((-dir_xy.y, dir_xy.x, 0)) * side
                d = (side_dir * 0.85 + dir_xy * 0.55 + Vector((0, 0, rng.uniform(-0.25, 0.05)))).normalized()
                lf = leaflet("leaflet", 0.115 * size * scale, 0.064 * size * scale, mat, rng)
                place(lf, q, d, roll=side * rng.uniform(0.05, 0.35))
                parts.append(lf)
                if k < n_pairs - 1:
                    q2 = on_rachis(t + 0.1)
                    d2 = (side_dir + dir_xy * 0.3).normalized()
                    small = leaflet("leaflet_small", 0.04 * scale, 0.022 * scale, mat, rng)
                    place(small, q2, d2, roll=side * 0.2)
                    parts.append(small)
        tip = Vector(pet[3])
        lf = leaflet("leaflet", 0.13 * scale, 0.07 * scale, mat, rng)
        place(lf, tip, (dir_xy + Vector((0, 0, -0.35))).normalized(), roll=rng.uniform(-0.2, 0.2))
        parts.append(lf)

        # Trusses every third node, biased to the -y (harvest) face.
        if i in truss_nodes:
            k = len(sites) and len({s["truss"] for s in sites})
            taz = math.radians(-90 + rng.uniform(-55, 55))
            tdir = Vector((math.cos(taz), math.sin(taz), 0))
            zt = z + 0.05
            s0 = stem_at(zt)
            n_fruit = rng.choice([3, 4, 4, 5])
            length = 0.05 + 0.045 * n_fruit
            ped_pts = [s0, s0 + tdir * 0.03 + Vector((0, 0, 0.015)),
                       s0 + tdir * length * 0.6 + Vector((0, 0, 0.0)),
                       s0 + tdir * length + Vector((0, 0, -0.03))]
            parts.append(tube("peduncle", [tuple(p) for p in ped_pts],
                              [0.0035, 0.0032, 0.0028, 0.0022], stem_m, res=6))
            ripe_bias = 1.0 - k * 0.38
            for f in range(n_fruit):
                t = 0.35 + 0.65 * f / max(1, n_fruit - 1)
                q = Vector(ped_pts[1]).lerp(Vector(ped_pts[3]), t)
                lateral = Vector((-tdir.y, tdir.x, 0)) * (0.026 if f % 2 else -0.026)
                knuck = q + lateral + Vector((0, 0, -0.018))
                parts.append(tube("proximal_pedicel", [tuple(q), tuple(q + lateral * 0.6 + Vector((0, 0, -0.004))),
                                                       tuple(knuck + Vector((0, 0, 0.004)))],
                                  [0.0022, 0.002, 0.0019], stem_m, res=6))
                r = ripe_bias + rng.uniform(-0.35, 0.35) - 0.08 * f
                kind = "tomato_ripe" if r > 0.55 else "tomato_half" if r > 0.15 else "tomato_green"
                sites.append({"id": f"{variant}_t{k}_f{f}", "truss": k, "kind": kind,
                              "xyz": [round(knuck.x, 4), round(knuck.y, 4), round(knuck.z, 4)],
                              "rpy": [round(rng.uniform(-0.12, 0.12), 3),
                                      round(rng.uniform(-0.12, 0.12), 3),
                                      round(rng.uniform(-math.pi, math.pi), 3)]})
    return parts, sites, height


def plant_sdf(name, height):
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <!-- Stem, leaves, trusses and the bamboo stake. Fruit are separate models,
         spawned at the sites in fruit_sites.json (plant frame) and attached with
         detachable joints. Only the stem and stake collide; leaves are visual
         so the arm can push through them. -->
    <link name="plant">
      <collision name="stem">
        <pose>0 0 {height / 2:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>0.008</radius><length>{height:.3f}</length></cylinder></geometry>
      </collision>
      <collision name="stake">
        <pose>0 0.045 {(height + 0.2) / 2:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>{height + 0.2:.3f}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://{name}/meshes/{name}.obj</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


# ------------------------------------------------------------------ bed and soil

def soil_texture(seed=11, size=1024):
    rng = np.random.default_rng(seed)
    n1, n2 = fbm(size, size, 7, seed), fbm(size, size, 4, seed + 1)
    dark, mid, light = rgb("#4B3727"), rgb("#634832"), rgb("#7A5D42")
    t = np.clip(0.25 + 0.6 * n1, 0, 1)[..., None]
    col = dark * (1 - t) + mid * t
    dry = np.clip((n2 - 0.55) * 2.5, 0, 1)[..., None]          # drier, paler patches
    col = col * (1 - 0.35 * dry) + light * 0.35 * dry
    grain = rng.normal(0, 0.025, (size, size, 1))               # fine soil grain
    pebble = (rng.random((size, size)) > 0.9985)
    for _ in range(2):                                           # grow pebbles to 3 px
        pebble = pebble | np.roll(pebble, 1, 0) | np.roll(pebble, 1, 1)
    col = col + grain + pebble[..., None] * (rgb("#8D7B66") - col) * 0.55
    return np.clip(col, 0, 1)


def uv_planar(obj, scale):
    """Box-style UVs from world position, picking the plane by face normal."""
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    uv = bm.loops.layers.uv.verify()
    for f in bm.faces:
        n = f.normal
        for loop in f.loops:
            co = obj.matrix_world @ loop.vert.co
            if abs(n.z) >= max(abs(n.x), abs(n.y)):
                loop[uv].uv = (co.x * scale, co.y * scale)
            elif abs(n.x) > abs(n.y):
                loop[uv].uv = (co.y * scale, co.z * scale)
            else:
                loop[uv].uv = (co.x * scale, co.z * scale)
    bm.to_mesh(obj.data)
    bm.free()


def build_bed(out_dir):
    L = LAYOUT["segment"]
    top, base, h = LAYOUT["bed_top"] / 2, LAYOUT["bed_base"] / 2, LAYOUT["bed_h"]
    mulch = ac.material("mulch_film", "#101112", roughness=0.46, coat=0.12)
    soil_img = save_image(soil_texture(), out_dir / "meshes" / "soil.png")
    soil = textured_material("soil", soil_img, roughness=0.95)
    hole_m = ac.material("soil_hole", "#3C2B1F", roughness=1.0)
    drip_m = ac.material("drip_line", "#0E0F10", roughness=0.45)

    holes = [(0.225 + LAYOUT["plant_spacing"] * i, y) for i in range(6) for y in LAYOUT["plant_lines"]]
    section = [(-base, 0.0)] + [(-top + k * (2 * top) / 18, h) for k in range(19)] + [(base, 0.0)]
    nx = 90
    bm = bmesh.new()
    rows = []
    for i in range(nx + 1):
        x = L * i / nx
        row = []
        for y, z in section:
            on_top = z > 0
            wr = (0.0022 * math.sin(2 * math.pi * 3 * x / L + 1.3 * y * 9)
                  + 0.0016 * math.sin(2 * math.pi * 7 * x / L + 2.1) * math.cos(17 * y)
                  + 0.0010 * math.sin(2 * math.pi * 13 * x / L + 5 * y))
            near = min(math.hypot(x - hx, y - hy) for hx, hy in holes) if on_top else 1
            wr *= min(1.0, near / 0.09)
            row.append(bm.verts.new((x, y, z + (wr if on_top else wr * 0.5))))
        rows.append(row)
    for r0, r1 in zip(rows, rows[1:]):
        for j in range(len(section) - 1):
            bm.faces.new((r0[j], r1[j], r1[j + 1], r0[j + 1]))
    for row in (rows[0], rows[-1]):
        bm.faces.new(row if row is rows[-1] else list(reversed(row)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    mesh = bpy.data.meshes.new("raised_bed")
    bm.to_mesh(mesh)
    bm.free()
    bed = bpy.data.objects.new("raised_bed", mesh)
    bpy.context.scene.collection.objects.link(bed)
    ac.assign(bed, mulch)
    ac.shade(bed, 40)

    parts = [bed]
    # Soil berms where the film edge is buried.
    for side in (1, -1):
        berm = tube("berm", [(0, side * (base + 0.01), 0.0), (L, side * (base + 0.01), 0.0)],
                    [0.035, 0.035], soil, res=8, smooth=False)
        berm.scale.z = 0.55
        ac.apply_transform(berm)
        uv_planar(berm, 1.0)
        parts.append(berm)
    for hx, hy in holes:
        parts.append(ac.cylinder("planting_hole", 0.05, 0.004, loc=(hx, hy, h + 0.0015),
                                 mat=hole_m, verts=32))
    for y in LAYOUT["plant_lines"]:
        parts.append(tube("drip", [(0, y - 0.07, h + 0.008), (L, y - 0.07, h + 0.008)],
                          [0.008, 0.008], drip_m, res=8, smooth=False, caps=False))
    return parts


def bed_sdf():
    L, h = LAYOUT["segment"], LAYOUT["bed_h"]
    top, base = LAYOUT["bed_top"], LAYOUT["bed_base"]
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="raised_bed">
    <static>true</static>
    <!-- {L} m bed segment, tile along +x. Origin at the start of the segment, on
         the bed centreline, at ground level. Plant holes at x = 0.225 + 0.45 i,
         y = +/-0.30. Collision is the bed's trapezoid as two boxes. -->
    <link name="bed">
      <collision name="core">
        <pose>{L / 2} 0 {h / 2} 0 0 0</pose>
        <geometry><box><size>{L} {top} {h}</size></box></geometry>
      </collision>
      <collision name="shoulders">
        <pose>{L / 2} 0 {h / 4} 0 0 0</pose>
        <geometry><box><size>{L} {(top + base) / 2} {h / 2}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://raised_bed/meshes/raised_bed.obj</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def build_soil_tile(out_dir):
    img = save_image(soil_texture(seed=23), out_dir / "meshes" / "soil.png")
    soil = textured_material("soil_ground", img, roughness=0.95)
    bpy.ops.mesh.primitive_grid_add(x_subdivisions=80, y_subdivisions=80, size=4.0)
    g = bpy.context.active_object
    g.name = "soil_tile"
    for v in g.data.vertices:
        x, y = v.co.x, v.co.y
        v.co.z = (0.006 * math.sin(2 * math.pi * x / 4 * 5) * math.sin(2 * math.pi * y / 4 * 3)
                  + 0.004 * math.sin(2 * math.pi * x / 4 * 11 + 1) * math.cos(2 * math.pi * y / 4 * 9))
    uv_planar(g, 1.0)
    ac.assign(g, soil)
    ac.shade(g, None)
    return [g]


def tile_sdf():
    return """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="soil_tile">
    <static>true</static>
    <!-- 4 m x 4 m furrow soil, visual only; the world supplies the ground plane. -->
    <link name="tile">
      <visual name="visual">
        <geometry><mesh><uri>model://soil_tile/meshes/soil_tile.obj</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def model_config(name, desc):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>0.1</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>AgriBOT</name></author>
  <description>{desc}</description>
</model>
"""


def write_model(name, parts, sdf, desc, extra=None):
    d = MODELS / name
    ac.export_all(parts, d / "meshes" / name)
    (d / "model.sdf").write_text(sdf, encoding="utf-8")
    (d / "model.config").write_text(model_config(name, desc), encoding="utf-8")
    if extra:
        for fname, data in extra.items():
            (d / fname).write_text(json.dumps(data, indent=2), encoding="utf-8")
    ac.log(f"{name}: {ac.tri_count(parts)} tris")


def import_glb(path):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=str(path))
    return [o for o in bpy.data.objects if o not in before and o.type == "MESH"]


def instance(objs, matrix):
    out = []
    for o in objs:
        c = o.copy()
        c.data = o.data
        bpy.context.scene.collection.objects.link(c)
        c.matrix_world = matrix @ o.matrix_world
        out.append(c)
    return out


def main():
    if MODELS.exists():
        shutil.rmtree(MODELS)
    (PKG / "config").mkdir(parents=True, exist_ok=True)
    (PKG / "config/field_layout.json").write_text(json.dumps(LAYOUT, indent=2))
    (PKG / "config/semantic_labels.yaml").write_text(
        "# Semantic label ids for segmentation cameras and the detector fallback.\n"
        "background: 0\ntomato_green: 1\ntomato_half: 2\ntomato_ripe: 3\n"
        "tomato_plant: 10\nraised_bed: 20\nsoil: 21\nrover: 30\n", encoding="utf-8")

    geos = {}
    for seed, kind in enumerate(FRUIT, start=3):
        ac.reset_scene()
        parts, geo = build_fruit(kind, MODELS / kind, seed)
        geos[kind] = geo
        desc = {"tomato_green": "Mature-green tomato", "tomato_half": "Half-ripe (turning) tomato",
                "tomato_ripe": "Ripe tomato"}[kind]
        write_model(kind, parts, fruit_sdf(kind, geo),
                    f"{desc}, {FRUIT[kind]['d'] * 1000:.0f} mm, {FRUIT[kind]['mass'] * 1000:.0f} g. "
                    "Origin at the knuckle.")

    plants = {}
    for variant, seed in (("a", 101), ("b", 202), ("c", 303)):
        ac.reset_scene()
        parts, sites, height = build_plant(variant, seed)
        name = f"tomato_plant_{variant}"
        plants[name] = (sites, height)
        write_model(name, parts, plant_sdf(name, height),
                    f"Staked indeterminate tomato plant, {height:.2f} m, {len(sites)} fruit sites.",
                    extra={"fruit_sites.json": {"frame": "plant model origin (stem base, ground)",
                                                "sites": sites}})

    ac.reset_scene()
    write_model("raised_bed", build_bed(MODELS / "raised_bed"), bed_sdf(),
                "Raised bed segment with plastic mulch, drip lines and planting holes; tiles along x.")
    ac.reset_scene()
    write_model("soil_tile", build_soil_tile(MODELS / "soil_tile"), tile_sdf(),
                "4 m x 4 m furrow soil tile.")

    render_previews(plants)
    ac.log("field done")


def render_previews(plants):
    img_dir = ac.REPO / "docs/images"
    fruit = {k: import_glb(MODELS / k / "meshes" / f"{k}.glb") for k in FRUIT}

    # Three fruits side by side.
    ac.reset_scene()
    objs = []
    for i, k in enumerate(FRUIT):
        src = import_glb(MODELS / k / "meshes" / f"{k}.glb")
        for o in src:
            o.matrix_world = Matrix.Translation((i * 0.085 - 0.085, 0, 0.05)) @ o.matrix_world
        objs += src
    ac.studio_render(objs, img_dir / "tomatoes.png", view=(0.05, -1.0, 0.35), lens=90,
                     res=(1600, 800), samples=128)

    # One plant with its fruit.
    ac.reset_scene()
    fruit = {k: import_glb(MODELS / k / "meshes" / f"{k}.glb") for k in FRUIT}
    for objs_ in fruit.values():
        for o in objs_:
            o.hide_render = True
    sites, _ = plants["tomato_plant_a"]
    objs = import_glb(MODELS / "tomato_plant_a" / "meshes" / "tomato_plant_a.glb")
    for s in sites:
        m = Matrix.Translation(s["xyz"]) @ Matrix.Rotation(s["rpy"][2], 4, "Z") \
            @ Matrix.Rotation(s["rpy"][1], 4, "Y") @ Matrix.Rotation(s["rpy"][0], 4, "X")
        new = instance(fruit[s["kind"]], m)
        for o in new:
            o.hide_render = False
        objs += new
    ac.studio_render(objs, img_dir / "tomato_plant.png", view=(0.35, -1.0, 0.25), lens=50,
                     res=(1000, 1400), samples=96)

    # Down the furrow between two beds, golden-hour light.
    ac.reset_scene()
    field_scene(n_beds=3, n_segments=3)
    furrow_y = LAYOUT["pitch"] / 2
    ac.outdoor_render(img_dir / "field_overview.png", cam_loc=(-1.6, furrow_y - 0.15, 1.15),
                      target=(3.5, furrow_y + 0.05, 0.55), lens=26, res=(1920, 1080),
                      samples=96, sun_elev=16, sun_azim=-35, haze=0.8)


def field_scene(n_beds=3, n_segments=3, seed=7):
    """Instance beds, plants and fruit from the built models into the current
    scene. Beds run along +x; bed b is centred at y = b * pitch, so the furrow
    between beds 0 and 1 is centred at y = pitch / 2. Returns fruit placements."""
    plants = {p.name: json.loads((p / "fruit_sites.json").read_text())["sites"]
              for p in sorted(MODELS.glob("tomato_plant_*"))}
    fruit = {k: import_glb(MODELS / k / "meshes" / f"{k}.glb") for k in FRUIT}
    plant_objs = {n: import_glb(MODELS / n / "meshes" / f"{n}.glb") for n in plants}
    bed = import_glb(MODELS / "raised_bed" / "meshes" / "raised_bed.glb")
    tile = import_glb(MODELS / "soil_tile" / "meshes" / "soil_tile.glb")
    sources = list(fruit.values()) + list(plant_objs.values()) + [bed, tile]
    rng = random.Random(seed)
    pitch, L = LAYOUT["pitch"], LAYOUT["segment"]
    placed = []
    for bx in range(n_segments):
        for by in range(n_beds):
            instance(bed, Matrix.Translation((bx * L, by * pitch, 0)))
            for hx_i in range(6):
                for line in LAYOUT["plant_lines"]:
                    name = rng.choice(list(plants))
                    # Face each plant's truss side (-y in the plant frame) toward its furrow.
                    yaw = 0.0 if line < 0 else math.pi
                    base = (Matrix.Translation((bx * L + 0.225 + 0.45 * hx_i, by * pitch + line,
                                                LAYOUT["bed_h"]))
                            @ Matrix.Rotation(yaw + rng.uniform(-0.45, 0.45), 4, "Z"))
                    instance(plant_objs[name], base)
                    for s in plants[name]:
                        m = (base @ Matrix.Translation(s["xyz"]) @ Matrix.Rotation(s["rpy"][2], 4, "Z")
                             @ Matrix.Rotation(s["rpy"][1], 4, "Y") @ Matrix.Rotation(s["rpy"][0], 4, "X"))
                        instance(fruit[s["kind"]], m)
                        placed.append((s["kind"], m))
    x_span = n_segments * L
    for tx in range(-2, int(x_span / 4) + 4):
        for ty in range(-3, int(n_beds * pitch / 4) + 4):
            instance(tile, Matrix.Translation((tx * 4 + 2, ty * 4, -0.005)))
    for group in sources:                      # hide the source copies
        for o in group:
            bpy.data.objects.remove(o, do_unlink=True)
    return placed


if __name__ == "__main__":
    main()
