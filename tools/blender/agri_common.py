"""Shared helpers for the AgriBOT Blender asset scripts.

Every asset script imports this module. It covers:

- scene reset
- the shared material library
- mesh cleanup
- exporters (GLB for Unreal/glTF, OBJ+MTL and STL for ROS/Gazebo)
- a studio render rig for preview images

Coordinate convention: assets are modelled in metres, z up, x forward (the ROS
convention). OBJ and STL are written with those raw coordinates, so URDF and SDF
files can use them directly. GLB follows the glTF standard (y up); the exporter
does that conversion and engines convert back on import.
"""
import math
import os
import pathlib
import re

import bpy
import bmesh
from mathutils import Matrix, Vector

REPO = pathlib.Path(__file__).resolve().parents[2]


# --------------------------------------------------------------------------- scene

def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    return scene


def new_collection(name):
    col = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(col)
    return col


def link_to(obj, col):
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    col.objects.link(obj)


# ----------------------------------------------------------------------- materials

def _hex(c):
    c = c.lstrip("#")
    srgb = [int(c[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    lin = [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in srgb]
    return (*lin, 1.0)


def material(name, color, roughness=0.5, metallic=0.0, subsurface=0.0,
             coat=0.0, transmission=0.0, sheen=0.0, emission=None):
    """Principled BSDF material, reused if it already exists."""
    mat = bpy.data.materials.get(name)
    if mat:
        return mat
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    rgba = _hex(color) if isinstance(color, str) else color
    bsdf.inputs["Base Color"].default_value = rgba
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    for key, val in (("Subsurface Weight", subsurface), ("Coat Weight", coat),
                     ("Transmission Weight", transmission), ("Sheen Weight", sheen)):
        if key in bsdf.inputs and val:
            bsdf.inputs[key].default_value = val
    if subsurface and "Subsurface Radius" in bsdf.inputs:
        bsdf.inputs["Subsurface Radius"].default_value = (0.9, 0.25, 0.12)
    if emission and "Emission Color" in bsdf.inputs:
        bsdf.inputs["Emission Color"].default_value = _hex(emission)
        bsdf.inputs["Emission Strength"].default_value = 1.0
    mat.diffuse_color = rgba          # viewport and solid-colour fallback
    return mat


# The AgriBOT palette. Asset scripts use these names so every asset matches.
PALETTE = {
    "pla_charcoal": dict(color="#2B2E33", roughness=0.55),
    "pla_green": dict(color="#3E8E41", roughness=0.45),
    "pla_bone": dict(color="#E9E5DC", roughness=0.5),
    "steel": dict(color="#B9BEC4", roughness=0.28, metallic=1.0),
    "steel_dark": dict(color="#6F757C", roughness=0.35, metallic=1.0),
    "motor_black": dict(color="#1D1F23", roughness=0.4, metallic=0.4),
    "rubber": dict(color="#121314", roughness=0.85),
    "switch": dict(color="#25292F", roughness=0.5),
    "blade": dict(color="#D9DDE2", roughness=0.15, metallic=1.0),
    "foam": dict(color="#7FC8B2", roughness=0.95),
    "tpu_orange": dict(color="#E8833A", roughness=0.6),
    "rover_body": dict(color="#F2F0EA", roughness=0.35, coat=0.6),
    "rover_accent": dict(color="#2F7D4A", roughness=0.35, coat=0.6),
    "rover_dark": dict(color="#22252A", roughness=0.5),
    "tyre": dict(color="#1A1A1A", roughness=0.9),
    "rim": dict(color="#C9CDD2", roughness=0.3, metallic=0.9),
    "lens": dict(color="#0B0D10", roughness=0.05, coat=1.0),
}


def pal(name):
    return material(f"agri_{name}", **PALETTE[name])


def assign(obj, mat):
    obj.data.materials.clear()
    obj.data.materials.append(mat)


# ---------------------------------------------------------------------- mesh tools

def import_stl(path, name=None):
    bpy.ops.wm.stl_import(filepath=str(path))
    obj = bpy.context.selected_objects[0]
    if name:
        obj.name = obj.data.name = name
    return obj


def cleanup(obj, merge=1e-6, planar_deg=0.5, collapse=None, smooth_deg=35):
    """Merge duplicate vertices, dissolve flat CAD facets, then optionally
    collapse to a triangle budget. Sharp edges are kept above smooth_deg."""
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=merge)
    if planar_deg:
        bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(planar_deg),
                                 verts=bm.verts, edges=bm.edges)
    bm.to_mesh(obj.data)
    bm.free()
    if collapse and collapse < 1.0:
        mod = obj.modifiers.new("decimate", "DECIMATE")
        mod.ratio = collapse
        apply_modifiers(obj)
    shade(obj, smooth_deg)
    return obj


def shade(obj, smooth_deg=35):
    mesh = obj.data
    mesh.shade_smooth()
    if smooth_deg is not None:
        mesh.set_sharp_from_angle(angle=math.radians(smooth_deg))


def apply_modifiers(obj):
    bpy.context.view_layer.objects.active = obj
    for m in list(obj.modifiers):
        bpy.ops.object.modifier_apply(modifier=m.name)


def tri_count(objs):
    n = 0
    for o in objs:
        o.data.calc_loop_triangles()
        n += len(o.data.loop_triangles)
    return n


def join(objs, name):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1:
        bpy.ops.object.join()
    out = bpy.context.view_layer.objects.active
    out.name = out.data.name = name
    return out


def convex_hull(objs, name):
    """A new object holding the convex hull of all given objects (world space)."""
    bm = bmesh.new()
    for o in objs:
        tmp = bmesh.new()
        tmp.from_mesh(o.data)
        tmp.transform(o.matrix_world)
        for v in tmp.verts:
            bm.verts.new(v.co)
        tmp.free()
    bmesh.ops.convex_hull(bm, input=bm.verts)
    loose = [v for v in bm.verts if not v.link_faces]
    bmesh.ops.delete(bm, geom=loose, context="VERTS")
    bmesh.ops.triangulate(bm, faces=bm.faces)
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def primitive_collision(kind, name, size, location=(0, 0, 0), rotation=(0, 0, 0)):
    """Box or cylinder collision proxy. size = (x, y, z) or (radius, depth)."""
    if kind == "box":
        bpy.ops.mesh.primitive_cube_add(size=1, location=location, rotation=rotation)
        obj = bpy.context.active_object
        obj.scale = size
    else:
        bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=size[0], depth=size[1],
                                            location=location, rotation=rotation)
        obj = bpy.context.active_object
    apply_transform(obj)
    obj.name = obj.data.name = name
    return obj


def apply_transform(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)


# ----------------------------------------------------------------------- modelling

def _finish(obj, name, mat, bevel, segments, smooth_deg):
    obj.name = obj.data.name = name
    if bevel:
        m = obj.modifiers.new("bevel", "BEVEL")
        m.width = bevel
        m.segments = segments
        m.limit_method = "ANGLE"
        m.harden_normals = False
        apply_modifiers(obj)
    if mat:
        assign(obj, mat)
    shade(obj, smooth_deg)
    return obj


def box(name, size, loc=(0, 0, 0), rot=(0, 0, 0), mat=None, bevel=0.0, segments=3,
        smooth_deg=30):
    """Box of size (x, y, z) centred at loc, with an optional rounded bevel."""
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc, rotation=rot)
    obj = bpy.context.active_object
    obj.scale = size
    apply_transform(obj)
    return _finish(obj, name, mat, bevel, segments, smooth_deg)


def cylinder(name, radius, depth, loc=(0, 0, 0), rot=(0, 0, 0), mat=None, verts=48,
             bevel=0.0, segments=3, smooth_deg=40):
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=radius, depth=depth,
                                        location=loc, rotation=rot)
    obj = bpy.context.active_object
    apply_transform(obj)
    return _finish(obj, name, mat, bevel, segments, smooth_deg)


def sphere(name, radius, loc=(0, 0, 0), mat=None, segments=48, rings=24):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=segments, ring_count=rings, radius=radius,
                                         location=loc)
    obj = bpy.context.active_object
    apply_transform(obj)
    return _finish(obj, name, mat, 0, 0, None)


def plate(name, outline, thickness, z=0.0, mat=None, bevel=0.0, segments=2, plane="XY",
          smooth_deg=30):
    """Extrude a 2D outline (list of (u, v), counter-clockwise) by thickness.
    plane 'XY' extrudes along +z from z; 'XZ' lays the outline in x/z and
    extrudes along +y from y = z; 'YZ' lays it in y/z and extrudes along +x."""
    bm = bmesh.new()
    def p3(u, v, w):
        return {"XY": (u, v, w), "XZ": (u, w, v), "YZ": (w, u, v)}[plane]
    bottom = [bm.verts.new(p3(u, v, z)) for u, v in outline]
    face = bm.faces.new(bottom)
    ext = bmesh.ops.extrude_face_region(bm, geom=[face])
    top = [e for e in ext["geom"] if isinstance(e, bmesh.types.BMVert)]
    off = {"XY": (0, 0, thickness), "XZ": (0, thickness, 0), "YZ": (thickness, 0, 0)}[plane]
    bmesh.ops.translate(bm, verts=top, vec=off)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    return _finish(obj, name, mat, bevel, segments, smooth_deg)


def arc(cx, cy, r, a0, a1, n=16):
    """Points on a circular arc (degrees), for building outlines."""
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def boolean(target, cutter, op="DIFFERENCE", keep=False):
    m = target.modifiers.new("bool", "BOOLEAN")
    m.object = cutter
    m.operation = op
    m.solver = "EXACT"
    apply_modifiers(target)
    if not keep:
        bpy.data.objects.remove(cutter, do_unlink=True)
    return target


def subdivide(obj, levels=2):
    m = obj.modifiers.new("subsurf", "SUBSURF")
    m.levels = m.render_levels = levels
    apply_modifiers(obj)
    return obj


def set_origin(obj, point):
    """Move the object origin to a world point without moving the geometry."""
    cursor = bpy.context.scene.cursor
    old = cursor.location.copy()
    cursor.location = point
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.origin_set(type="ORIGIN_CURSOR")
    cursor.location = old


def bake_to_origin(obj, point):
    """Shift geometry so `point` becomes the object's local origin, and place the
    object at the world origin. Used for parts exported in their own joint frame."""
    obj.data.transform(Matrix.Translation(-Vector(point)) @ obj.matrix_world)
    obj.matrix_world = Matrix.Identity(4)


# ----------------------------------------------------------------------- exporters

def _select(objs):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]


def export_glb(objs, path):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    _select(objs)
    bpy.ops.export_scene.gltf(filepath=str(path), export_format="GLB", use_selection=True,
                              export_apply=True, export_yup=True)


def export_obj(objs, path):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    _select(objs)
    bpy.ops.wm.obj_export(filepath=str(path), export_selected_objects=True,
                          forward_axis="Y", up_axis="Z", apply_modifiers=True,
                          export_materials=True, export_normals=True, export_uv=True,
                          export_triangulated_mesh=True, path_mode="RELATIVE")
    fix_mtl(path.with_suffix(".mtl"))


def fix_mtl(mtl):
    """Blender omits Kd when the base colour comes from a texture; Gazebo then
    multiplies the texture by a black diffuse. Add Kd 1 1 1 to such materials."""
    if not mtl.exists():
        return
    blocks = re.split(r"(?m)^(?=newmtl )", mtl.read_text())
    out = []
    for b in blocks:
        if b.startswith("newmtl") and "map_Kd" in b and not re.search(r"(?m)^Kd ", b):
            first, rest = b.split(NL, 1)
            b = first + NL + "Kd 1.000000 1.000000 1.000000" + NL + rest
        out.append(b)
    mtl.write_text("".join(out), newline=NL)


NL = chr(10)



def export_stl(objs, path):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    _select(objs)
    bpy.ops.wm.stl_export(filepath=str(path), export_selected_objects=True,
                          forward_axis="Y", up_axis="Z", apply_modifiers=True, ascii_format=False)


def export_all(objs, stem, formats=("glb", "obj")):
    """Write stem.glb / stem.obj for the given objects, after removing
    zero-length edges and zero-area faces."""
    stem = pathlib.Path(stem)
    for o in objs:
        if o.type != "MESH" or o.data.users > 1:
            continue
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bmesh.ops.dissolve_degenerate(bm, dist=1e-6, edges=bm.edges)
        bm.to_mesh(o.data)
        bm.free()
    if "glb" in formats:
        export_glb(objs, stem.with_suffix(".glb"))
    if "obj" in formats:
        export_obj(objs, stem.with_suffix(".obj"))
    if "stl" in formats:
        export_stl(objs, stem.with_suffix(".stl"))


# ----------------------------------------------------------------------- rendering

def bounds(objs):
    pts = [o.matrix_world @ Vector(c) for o in objs for c in o.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return lo, hi


def studio_render(objs, path, view=(1.0, -1.25, 0.65), res=(1600, 1000), lens=60,
                  backdrop="#E2E0DB", margin=0.78, samples=64, engine="CYCLES",
                  sun=False, world_strength=0.25):
    """Soft three-light studio render of objs on a seamless backdrop."""
    scene = bpy.context.scene
    lo, hi = bounds(objs)
    centre = (lo + hi) / 2
    size = max((hi - lo).length, 1e-3)

    # Backdrop: a floor plane under the objects.
    bpy.ops.mesh.primitive_plane_add(size=size * 40, location=(centre.x, centre.y, lo.z))
    floor = bpy.context.active_object
    floor.name = "studio_floor"
    assign(floor, material("studio_backdrop", backdrop, roughness=0.9))

    # Camera aimed at the centre from the view direction.
    d = Vector(view).normalized()
    cam_data = bpy.data.cameras.new("studio_cam")
    cam_data.lens = lens
    cam = bpy.data.objects.new("studio_cam", cam_data)
    scene.collection.objects.link(cam)
    half_h = math.atan(cam_data.sensor_width / (2 * lens))       # sensor fits the width
    half_v = math.atan(math.tan(half_h) * res[1] / res[0])
    dist = (size / 2) * margin / math.tan(min(half_h, half_v))
    cam.location = centre + d * dist
    cam.rotation_euler = (centre - cam.location).to_track_quat("-Z", "Y").to_euler()
    cam_data.clip_end = dist * 20
    scene.camera = cam

    # Key, fill and rim area lights scaled to the subject.
    def area(name, offset, energy, size_k):
        ld = bpy.data.lights.new(name, "AREA")
        ld.energy = energy * size * size
        ld.size = size * size_k
        lo_ = bpy.data.objects.new(name, ld)
        scene.collection.objects.link(lo_)
        lo_.location = centre + Vector(offset) * size
        lo_.rotation_euler = (centre - lo_.location).to_track_quat("-Z", "Y").to_euler()

    area("key", (1.2, -1.0, 1.6), 140, 1.2)
    area("fill", (-1.6, -0.6, 0.8), 45, 2.0)
    area("rim", (-0.4, 1.6, 1.4), 90, 1.0)
    if sun:
        sd = bpy.data.lights.new("sun", "SUN")
        sd.energy = 3.5
        sd.angle = math.radians(4)
        so = bpy.data.objects.new("sun", sd)
        scene.collection.objects.link(so)
        so.rotation_euler = (math.radians(50), 0, math.radians(35))

    world = bpy.data.worlds.new("studio_world")
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value = _hex("#F4F3EF")
    bg.inputs["Strength"].default_value = world_strength
    scene.world = world

    return _render(scene, path, engine, samples, res)


def outdoor_render(path, cam_loc, target, lens=35, res=(1920, 1080), samples=128,
                   sun_elev=18, sun_azim=-60, sun_energy=4.5, haze=0.0, dof=None):
    """Render the current scene outdoors: physical sky, low sun, optional mist
    and depth of field. cam_loc/target in metres; sun angles in degrees
    (azimuth measured from +x toward +y)."""
    scene = bpy.context.scene
    cam_data = bpy.data.cameras.new("field_cam")
    cam_data.lens = lens
    cam_data.clip_end = 400
    cam = bpy.data.objects.new("field_cam", cam_data)
    scene.collection.objects.link(cam)
    cam.location = cam_loc
    cam.rotation_euler = (Vector(target) - Vector(cam_loc)).to_track_quat("-Z", "Y").to_euler()
    scene.camera = cam
    if dof:
        cam_data.dof.use_dof = True
        cam_data.dof.focus_distance = (Vector(target) - Vector(cam_loc)).length
        cam_data.dof.aperture_fstop = dof

    el, az = math.radians(sun_elev), math.radians(sun_azim)
    sd = bpy.data.lights.new("sun", "SUN")
    sd.energy = sun_energy
    sd.angle = math.radians(1.5)
    sd.color = (1.0, 0.93, 0.82)
    so = bpy.data.objects.new("sun", sd)
    scene.collection.objects.link(so)
    sun_dir = Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    so.rotation_euler = (-sun_dir).to_track_quat("-Z", "Y").to_euler()

    world = bpy.data.worlds.new("field_world")
    world.use_nodes = True
    nt = world.node_tree
    bg = nt.nodes.get("Background")
    sky = nt.nodes.new("ShaderNodeTexSky")
    for kind in ("MULTIPLE_SCATTERING", "NISHITA", "HOSEK_WILKIE"):
        try:
            sky.sky_type = kind
            break
        except TypeError:
            continue
    for attr, val in (("sun_elevation", el), ("sun_rotation", az + math.pi / 2),
                      ("altitude", 50.0), ("air_density", 1.0), ("aerosol_density", 1.5 + haze),
                      ("sun_disc", False)):
        if hasattr(sky, attr):
            setattr(sky, attr, val)
    nt.links.new(sky.outputs["Color"], bg.inputs["Color"])
    bg.inputs["Strength"].default_value = 0.35
    scene.world = world
    return _render(scene, path, "CYCLES", samples, res)


def _render(scene, path, engine, samples, res, look="AgX - Punchy"):
    scene.render.engine = engine
    if engine == "CYCLES":
        scene.cycles.samples = samples
        scene.cycles.use_denoising = True
        _use_gpu(scene)
    scene.render.resolution_x, scene.render.resolution_y = res
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    try:
        scene.view_settings.view_transform = "AgX"
        scene.view_settings.look = look
    except TypeError:
        pass
    scene.render.image_settings.file_format = "PNG"
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    return path


def _use_gpu(scene):
    prefs = bpy.context.preferences.addons.get("cycles")
    if not prefs:
        return
    cp = prefs.preferences
    for backend in ("OPTIX", "CUDA", "HIP", "ONEAPI", "METAL"):
        try:
            cp.compute_device_type = backend
            cp.get_devices()
            if any(d.type == backend for d in cp.devices):
                for d in cp.devices:
                    d.use = d.type == backend
                scene.cycles.device = "GPU"
                return
        except TypeError:
            continue


def log(msg):
    print(f"[agri] {msg}", flush=True)


def argv_after_dashes():
    import sys
    return sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


os.environ.setdefault("AGRI_REPO", str(REPO))
