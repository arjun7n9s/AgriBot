#!/usr/bin/env python3
"""Read `ign topic -e` text for dynamic_pose/info on stdin and log one model's
position whenever it moves more than 1 mm (wall time, x, y, z)."""
import sys, time, re
target = sys.argv[1]
last, name, pos, grab = None, None, {}, False
for line in sys.stdin:
    s = line.strip()
    m = re.match(r'name: "(.*)"', s)
    if m:
        name = m.group(1); grab = False; pos = {}
        continue
    if name == target:
        if s.startswith("position"):
            grab = True; continue
        if grab:
            k = re.match(r"([xyz]): (\S+)", s)
            if k:
                pos[k.group(1)] = float(k.group(2))
                if len(pos) == 3:
                    p = (pos["x"], pos["y"], pos["z"]); grab = False
                    if last is None or max(abs(a - b) for a, b in zip(p, last)) > 0.001:
                        print(f"{time.time():.3f} {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}", flush=True)
                        last = p
