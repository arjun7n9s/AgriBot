"""Quality checks for every simulation asset.

    .venv/Scripts/python tools/qa/check_assets.py      (python3 on Ubuntu)

Checks, per mesh (OBJ, STL, GLB) under sim/:
- parses, has faces, has no NaN or infinite coordinates
- degenerate (zero-area) triangles below 0.5%
- OBJ normals present; MTL file and every texture it names exist, with
  relative paths only
- GLB: valid header and JSON, every image embedded, at least one material
- size is plausible in metres (no millimetre or centimetre export slips)

Per description:
- every package:// URI in the URDFs and every model:// URI in the SDFs
  resolves to a file
- every Gazebo model has model.config and model.sdf

Prints a table and exits non-zero on any failure. Warnings do not fail.
"""
import json
import pathlib
import re
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
SIM = ROOT / "sim"
# Largest plausible extent per asset family, in metres.
MAX_EXTENT = {"meshes/rover": 1.3, "meshes/cutter": 0.25, "meshes/scara": 0.6, "tomato_plant": 2.2,
              "tomato_": 0.12, "raised_bed": 3.0, "soil_tile": 4.2}

fails, warns, rows = [], [], []


def fail(msg):
    fails.append(msg)


def warn(msg):
    warns.append(msg)


def tri_stats(tris, name):
    if not len(tris):
        fail(f"{name}: no faces")
        return 0, np.zeros(3)
    if not np.isfinite(tris).all():
        fail(f"{name}: NaN or infinite coordinates")
    area = np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1) / 2
    degenerate = float((area < 1e-12).mean())
    if degenerate > 0.005:
        warn(f"{name}: {degenerate:.1%} degenerate triangles")
    pts = tris.reshape(-1, 3)
    extent = pts.max(0) - pts.min(0)
    for key, lim in MAX_EXTENT.items():
        if key in name:
            if extent.max() > lim:
                fail(f"{name}: extent {extent.max():.3f} m exceeds {lim} m (unit slip?)")
            if extent.max() < lim / 200:
                fail(f"{name}: extent {extent.max():.5f} m is implausibly small (unit slip?)")
            break
    return len(tris), extent


def check_obj(path):
    verts, faces, has_vn, mtllibs, usemtl = [], [], False, [], set()
    for line in path.read_text(errors="replace").splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("vn "):
            has_vn = True
        elif line.startswith("f "):
            idx = [int(t.split("/")[0]) for t in line.split()[1:]]
            for k in range(1, len(idx) - 1):
                faces.append((idx[0], idx[k], idx[k + 1]))
        elif line.startswith("mtllib "):
            mtllibs.append(line[7:].strip())
        elif line.startswith("usemtl "):
            usemtl.add(line[7:].strip())
    v = np.array(verts)
    f = np.array(faces)
    f = np.where(f < 0, len(v) + f + 1, f) - 1
    n, extent = tri_stats(v[f] if len(f) else np.zeros((0, 3, 3)), path.relative_to(ROOT).as_posix())
    if not has_vn:
        warn(f"{path.name}: no vertex normals")
    for lib in mtllibs:
        mtl = path.parent / lib
        if not mtl.exists():
            fail(f"{path.name}: missing {lib}")
            continue
        text = mtl.read_text(errors="replace")
        defined = set(re.findall(r"^newmtl (.+)$", text, re.M))
        if usemtl - defined:
            fail(f"{path.name}: materials not in MTL: {usemtl - defined}")
        for tex in re.findall(r"^\s*map_\w+\s+(.+)$", text, re.M):
            tex = tex.strip().split()[-1]
            if re.match(r"^[A-Za-z]:|^/", tex) or "\\" in tex:
                fail(f"{mtl.name}: absolute or Windows texture path {tex}")
            elif not (mtl.parent / tex).exists():
                fail(f"{mtl.name}: missing texture {tex}")
    return n, extent


def check_stl(path):
    b = path.read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    if len(b) != 84 + 50 * n:
        fail(f"{path.name}: binary STL size mismatch")
        return 0, np.zeros(3)
    rec = np.frombuffer(b[84:], dtype=[("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    return tri_stats(rec["v"].reshape(-1, 3, 3).astype(float), path.relative_to(ROOT).as_posix())


def check_glb(path):
    b = path.read_bytes()
    magic, version, length = struct.unpack("<4sII", b[:12])
    if magic != b"glTF" or version != 2 or length != len(b):
        fail(f"{path.name}: bad GLB header")
        return
    clen, ctype = struct.unpack("<I4s", b[12:20])
    doc = json.loads(b[20:20 + clen])
    if not doc.get("meshes"):
        fail(f"{path.name}: no meshes")
    if not doc.get("materials"):
        warn(f"{path.name}: no materials")
    for img in doc.get("images", []):
        if "bufferView" not in img:
            fail(f"{path.name}: image not embedded ({img.get('uri')})")


def check_descriptions():
    for urdf in sorted((SIM / "agribot_description/urdf").glob("*.urdf")):
        for m in ET.parse(urdf).getroot().iter("mesh"):
            rel = m.get("filename").replace("package://agribot_description/", "")
            if not (SIM / "agribot_description" / rel).exists():
                fail(f"{urdf.name}: unresolved mesh {m.get('filename')}")
    models = SIM / "agribot_field/models"
    for d in sorted(p for p in models.iterdir() if p.is_dir()):
        for need in ("model.config", "model.sdf"):
            if not (d / need).exists():
                fail(f"{d.name}: missing {need}")
        sdf = (d / "model.sdf").read_text()
        for uri in re.findall(r"model://([^<\s]+)", sdf):
            if not (models / uri).exists():
                fail(f"{d.name}/model.sdf: unresolved model://{uri}")
        ET.fromstring(sdf.encode())


def main():
    for path in sorted(SIM.rglob("*")):
        suf = path.suffix.lower()
        rel = str(path.relative_to(SIM)).replace("\\", "/")
        try:
            if suf == ".obj":
                n, ext = check_obj(path)
                rows.append((rel, n, ext))
            elif suf == ".stl":
                n, ext = check_stl(path)
                rows.append((rel, n, ext))
            elif suf == ".glb":
                check_glb(path)
        except Exception as e:      # report and keep going
            fail(f"{rel}: {type(e).__name__}: {e}")
    check_descriptions()

    print(f"{'mesh':72s} {'tris':>8s}  extent x y z (m)")
    for rel, n, ext in rows:
        print(f"{rel:72s} {n:8d}  {ext[0]:.3f} {ext[1]:.3f} {ext[2]:.3f}")
    for w in warns:
        print("WARN ", w)
    for f in fails:
        print("FAIL ", f)
    print(f"\n{len(rows)} meshes, {len(warns)} warnings, {len(fails)} failures")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
