#!/usr/bin/env python3
"""Contact sheet of N evenly spaced frames from a video: contact.py <video> <out.png> [n]"""
import sys
import cv2
import numpy as np
src, out = sys.argv[1], sys.argv[2]
n = int(sys.argv[3]) if len(sys.argv) > 3 else 12
cap = cv2.VideoCapture(src)
frames = []
while True:
    ok, f = cap.read()
    if not ok:
        break
    frames.append(f)
print("frames", len(frames), "duration s", round(len(frames) / 30, 1))
pick = [frames[int(i * (len(frames) - 1) / (n - 1))] for i in range(n)]
thumbs = [cv2.resize(f, (480, 270)) for f in pick]
for i, t in enumerate(thumbs):
    cv2.putText(t, f"{int(i * (len(frames) - 1) / (n - 1)) / 30:.0f}s", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
rows = [np.hstack(thumbs[r * 4:(r + 1) * 4]) for r in range(n // 4)]
cv2.imwrite(out, np.vstack(rows))
