"""Turn a filmed demo into publishable media.

Reads <capture_dir>/desktop.webm (GNOME screen recording of the Gazebo GUI),
<capture_dir>/launch.log (harvest phases with wall-clock stamps) and
<capture_dir>/cast_start.txt (wall-clock start of the recording). Writes:

    <capture_dir>/agribot_demo.mp4        1280x720, 30 fps, cropped to the 3D view,
                                           black start trimmed
    <capture_dir>/stills/<n>_<phase>.png  one clean still per harvest phase
    <capture_dir>/agribot_pick.gif        the first cut, 432 px wide, 8 fps

GNOME records at a variable frame rate (it skips unchanged frames), so frames
are placed by their own timestamps and resampled onto an even 30 fps timeline.
Everything streams; no full-resolution frames are kept in memory.

    python3 tools/sim/make_video.py ~/agribot_capture
"""
import pathlib
import re
import sys

import cv2
from PIL import Image

VIEW = (72, 112, 1920, 1080)      # Gazebo 3D view inside the 1920x1080 desktop
OUT_W, OUT_H, FPS = 1280, 720, 30.0
GIF_W, GIF_H, GIF_FPS = 432, 243, 8.0
STILL_DELAY = 1.8                 # seconds after a phase starts


def crop(frame):
    x0, y0, x1, y1 = VIEW
    view = frame[y0:y1, x0:x1]
    h, w = view.shape[:2]
    target_w = int(h * OUT_W / OUT_H)
    if target_w < w:
        off = (w - target_w) // 2
        view = view[:, off:off + target_w]
    return cv2.resize(view, (OUT_W, OUT_H), interpolation=cv2.INTER_AREA)


def phases(capture):
    """[(seconds into the recording, label)] from the harvest log."""
    t0 = float((capture / "cast_start.txt").read_text().strip())
    keep = ("start", "drive", "approach", "cut", "retreat", "drop", "done")
    out = []
    for line in (capture / "launch.log").read_text(errors="replace").splitlines():
        m = re.search(r"\[(\d{10}\.\d+)\] \[harvest_demo\]: (\w+)", re.sub(r"\x1b\[[0-9;]*m", "", line))
        if m and m.group(2) in keep:
            out.append((float(m.group(1)) - t0, m.group(2)))
    return out


def main():
    capture = pathlib.Path(sys.argv[1]).expanduser()
    marks = phases(capture)
    stills_due = [(t + STILL_DELAY, f"{n:02d}_{label}") for n, (t, label) in enumerate(marks)]
    approach = next((t for t, l in marks if l == "approach"), None)
    cut = next((t for t, l in marks if l == "cut"), None)
    gif_window = (approach + 2.0, cut + 3.0) if approach is not None and cut is not None else None
    (capture / "stills").mkdir(exist_ok=True)

    cap = cv2.VideoCapture(str(capture / "desktop.webm"))
    writer = cv2.VideoWriter(str(capture / "agribot_demo.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), FPS,
                             (OUT_W, OUT_H))
    prev, out_t, gif_t, gif, written, started = None, None, None, [], 0, None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        ts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        view = crop(frame)
        if started is None:
            if view.std() < 12:                   # still the blank (white or black) GUI load
                continue
            started, out_t, gif_t = ts, ts, gif_window[0] if gif_window else None
        if prev is not None:
            while out_t < ts:                     # repeat the last frame up to this timestamp
                writer.write(prev)
                written += 1
                while stills_due and stills_due[0][0] <= out_t:
                    cv2.imwrite(str(capture / "stills" / f"{stills_due.pop(0)[1]}.png"), prev)
                if gif_window and gif_t is not None and gif_window[0] <= out_t <= gif_window[1] and out_t >= gif_t:
                    gif.append(Image.fromarray(cv2.cvtColor(cv2.resize(prev, (GIF_W, GIF_H)), cv2.COLOR_BGR2RGB)))
                    gif_t += 1.0 / GIF_FPS
                out_t += 1.0 / FPS
        prev = view
    writer.release()
    for _, name in stills_due:                    # phases after the last frame
        cv2.imwrite(str(capture / "stills" / f"{name}.png"), prev)
    if gif:
        frames = [g.quantize(colors=96, method=0) for g in gif]  # 0 = median cut (PIL 9 has no enum)
        frames[0].save(capture / "agribot_pick.gif", save_all=True, append_images=frames[1:],
                       duration=int(1000 / GIF_FPS), loop=0, optimize=True)
    print(f"video {written / FPS:.1f} s from {started:.1f} s; {len(marks)} stills; gif {len(gif)} frames")


if __name__ == "__main__":
    main()
