#!/usr/bin/env python3
"""Record the GNOME desktop until interrupted (SIGINT/SIGTERM) or for N seconds.
GNOME stops a screencast when the caller leaves the session bus, so this process
holds the connection open for the whole recording.

    python3 screencast.py <out.webm> [max_seconds] [framerate]
"""
import signal
import sys
import time

from gi.repository import Gio, GLib

out = sys.argv[1]
limit = float(sys.argv[2]) if len(sys.argv) > 2 else 600
fps = int(sys.argv[3]) if len(sys.argv) > 3 else 30
stop = False


def handler(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, handler)
signal.signal(signal.SIGTERM, handler)
bus = Gio.bus_get_sync(Gio.BusType.SESSION, None)
proxy = Gio.DBusProxy.new_sync(bus, 0, None, "org.gnome.Shell.Screencast",
                               "/org/gnome/Shell/Screencast", "org.gnome.Shell.Screencast", None)
ok, path = proxy.call_sync("Screencast", GLib.Variant("(sa{sv})", (out, {
    "framerate": GLib.Variant("i", fps), "draw-cursor": GLib.Variant("b", False)})), 0, -1, None).unpack()
print("recording" if ok else "failed", path, flush=True)
t0 = time.time()
while not stop and time.time() - t0 < limit:
    time.sleep(0.2)
print("stopped", proxy.call_sync("StopScreencast", None, 0, -1, None).unpack(), flush=True)
