#!/usr/bin/env python3
"""
Live MJPEG view of the camera, in a browser, with blown highlights marked.

Written to answer one question: is the frame clipping everywhere, or only in
the sky? A percentage cannot tell you that and a still image is awkward to aim
with, so this paints every saturated pixel RED and streams it.

It reads the RAW stream, not the processed one. The ISP's RGB path on this
camera returns an image of exactly zero (see docs/development-log.md, Camera bringup), so
anything built on rpicam-vid or the `main` stream would show a black rectangle
and prove nothing.

    python3 cam_stream.py                      # sensor's shortest exposure
    python3 cam_stream.py --shutter 200
    python3 cam_stream.py --rot180             # camera mounted inverted
    python3 cam_stream.py --port 8080

Then open http://<pi>:8080/ from any machine on the network.

Red pixels are at 255 and carry NO gradient -- a corner detector finds nothing
there. Blue pixels are at 0, the same problem at the other end. Anything grey
is usable.
"""
import argparse
import io
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import numpy as np
except ImportError:
    sys.exit("needs numpy:  sudo apt install -y python3-numpy")

try:
    import simplejpeg
    def encode(rgb):
        return simplejpeg.encode_jpeg(np.ascontiguousarray(rgb),
                                      quality=80, colorspace="RGB")
except ImportError:
    from PIL import Image
    def encode(rgb):
        b = io.BytesIO()
        Image.fromarray(rgb).save(b, format="JPEG", quality=80)
        return b.getvalue()


PAGE = b"""<!doctype html><meta charset=utf-8>
<title>viopi camera</title>
<style>
 body{background:#111;color:#ddd;font:14px system-ui;margin:0;padding:12px}
 img{width:100%;max-width:960px;image-rendering:pixelated;border:1px solid #333}
 #s{margin:8px 0;font-variant-numeric:tabular-nums;line-height:1.6}
 #c{margin:8px 0}
 input{width:90px;background:#222;color:#eee;border:1px solid #444;padding:4px}
 button{background:#333;color:#eee;border:1px solid #555;padding:4px 10px;cursor:pointer}
 button:hover{background:#444}
 b{color:#fff} .r{color:#f55} .b{color:#59f}
</style>
<img src="/stream">
<div id=c>
  exposure <input id=e type=number min=1 max=1000000 step=1 value=20> us
  <button onclick="bump(0.5)">/2</button>
  <button onclick="bump(2)">x2</button>
  &nbsp; gain <input id=g type=number min=1 max=16 step=0.1 value=1>
  <button onclick="apply()">apply</button>
</div>
<div id=s>connecting...</div>
<div style="color:#888;max-width:960px">
<span class=r>red</span> = pixel at 255, saturated, no gradient, no corners.
<span class=b>blue</span> = pixel at 0. Grey is usable.
</div>
<script>
async function apply(){
  const e = document.getElementById('e').value, g = document.getElementById('g').value;
  await fetch(`/set?shutter=${e}&gain=${g}`);
}
function bump(f){
  const el = document.getElementById('e');
  el.value = Math.max(1, Math.round(el.value * f));
  apply();
}
document.addEventListener('keydown', ev => {
  if (ev.target.tagName === 'INPUT' && ev.key === 'Enter') apply();
});
setInterval(async()=>{
  try{
    const d = await (await fetch('/stats')).json();
    document.getElementById('s').innerHTML =
      `clip hi <b class=r>${d.hi.toFixed(1)}%</b> &nbsp; clip lo <b class=b>${d.lo.toFixed(1)}%</b>`
      + ` &nbsp;|&nbsp; mean <b>${d.mean.toFixed(1)}</b> &nbsp; std <b>${d.std.toFixed(1)}</b>`
      + ` &nbsp;|&nbsp; focus <b>${d.focus.toFixed(2)}</b>`
      + ` &nbsp;|&nbsp; exposure <b>${d.exp}</b> us &nbsp; gain <b>${d.gain.toFixed(2)}</b>`;
  }catch(e){}
}, 500);
</script>
"""


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.stats = {"hi": 0.0, "lo": 0.0, "mean": 0.0, "std": 0.0,
                      "focus": 0.0, "exp": 0, "gain": 1.0}
        self.pending = None   # controls to apply on the next capture
        self.stop = False


def smooth(a):
    b = (a[:, :-2] + 2.0 * a[:, 1:-1] + a[:, 2:]) / 4.0
    return (b[:-2, :] + 2.0 * b[1:-1, :] + b[2:, :]) / 4.0


def focus_score(g):
    """Same noise-suppressed measure focus_check.py reports, so the number on
    screen is comparable with the one from the sweep."""
    a = smooth(g.astype(np.float64))
    lap = (-4.0 * a[1:-1, 1:-1] + a[:-2, 1:-1] + a[2:, 1:-1]
           + a[1:-1, :-2] + a[1:-1, 2:])
    return float(lap.var())


def capture_loop(shared, args):
    from picamera2 import Picamera2
    p = Picamera2()
    controls = {"AeEnable": False, "AnalogueGain": args.gain}
    if args.shutter:
        controls["ExposureTime"] = args.shutter
    dur = int(round(1e6 / args.fps))
    controls["FrameDurationLimits"] = (dur, dur)
    cfg = p.create_video_configuration(
        main={"size": (64, 64)},
        raw={"size": (args.width, args.height), "format": "R8"},
        buffer_count=6, controls=controls)
    p.configure(cfg)
    rawcfg = p.camera_configuration()["raw"]
    stride = rawcfg.get("stride", args.width * 2)
    p.start()
    print(f"raw stream {rawcfg['format']} {rawcfg['size']} stride={stride}")

    while not shared.stop:
        with shared.lock:
            pend, shared.pending = shared.pending, None
        if pend:
            # Applied from the capture thread rather than the HTTP thread:
            # libcamera does not expect concurrent control writes.
            try:
                p.set_controls(pend)
            except Exception as e:
                print("set_controls failed:", e, flush=True)
        req = p.capture_request()
        buf = req.make_buffer("raw")
        md = req.get_metadata()
        req.release()

        # Reinterpret the flat buffer honouring stride, then take the high
        # byte -- the R8 mode delivers 8-bit data inside a 16-bit container.
        rows = np.frombuffer(buf, dtype=np.uint8).reshape(args.height, stride)
        g = rows[:, :args.width * 2].view("<u2")[:, :args.width] >> 8
        g = g.astype(np.uint8)
        if args.rot180:
            # VIEWING CONVENIENCE ONLY. Never do this in the recording or
            # estimator path: Kalibr solves the camera-IMU transform for the
            # camera AS MOUNTED, so silently rotating frames afterwards makes
            # the calibration describe an orientation the images no longer
            # have. Rotate the picture you look at, never the one you measure.
            g = np.ascontiguousarray(g[::-1, ::-1])

        rgb = np.repeat(g[:, :, None], 3, axis=2)
        hot = g >= 255
        cold = g == 0
        rgb[hot] = (255, 40, 40)
        rgb[cold] = (40, 90, 255)

        jpg = encode(rgb)
        with shared.lock:
            shared.jpeg = jpg
            shared.stats = {
                "hi": 100.0 * hot.mean(), "lo": 100.0 * cold.mean(),
                "mean": float(g.mean()), "std": float(g.std()),
                "focus": focus_score(g),
                "exp": int(md.get("ExposureTime", 0)),
                "gain": float(md.get("AnalogueGain", 0.0)),
            }
    p.stop()


def make_handler(shared):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == "/stats":
                with shared.lock:
                    body = json.dumps(shared.stats).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path.startswith("/set"):
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                ctrl = {}
                try:
                    if "shutter" in q:
                        us = int(float(q["shutter"][0]))
                        us = max(1, min(us, 1000000))
                        ctrl["ExposureTime"] = us
                        # Exposure cannot exceed the frame duration, so open
                        # the duration limit up if the request needs it --
                        # otherwise long exposures are silently clamped.
                        dur = max(int(1e6 / 15), us + 2000)
                        ctrl["FrameDurationLimits"] = (dur, dur)
                    if "gain" in q:
                        ctrl["AnalogueGain"] = max(1.0, min(16.0,
                                                           float(q["gain"][0])))
                except ValueError:
                    self.send_error(400, "bad number")
                    return
                if ctrl:
                    with shared.lock:
                        shared.pending = ctrl
                body = json.dumps({"applied": {k: str(v) for k, v in ctrl.items()}}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=FRAME")
                self.end_headers()
                try:
                    while True:
                        with shared.lock:
                            j = shared.jpeg
                        if j is None:
                            time.sleep(0.05)
                            continue
                        self.wfile.write(b"--FRAME\r\n")
                        self.wfile.write(
                            b"Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                            % len(j))
                        self.wfile.write(j)
                        self.wfile.write(b"\r\n")
                        time.sleep(1.0 / 15)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # browser navigated away; not an error
            else:
                self.send_error(404)
    return H


def main():
    ap = argparse.ArgumentParser(description="Live camera view with clipping overlay.")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--shutter", type=int, metavar="US",
                    help="fixed exposure in us; omit to let the sensor pick "
                         "its shortest")
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--rot180", action="store_true",
                    help="rotate the VIEW 180 deg, for when the camera is "
                         "mounted inverted relative to how you are holding it. "
                         "Display only -- see the note in capture_loop()")
    args = ap.parse_args()

    shared = Shared()
    t = threading.Thread(target=capture_loop, args=(shared, args), daemon=True)
    t.start()
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(shared))
    print(f"open  http://{__import__('socket').gethostname()}:{args.port}/"
          f"   (or http://<ip>:{args.port}/)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        shared.stop = True


if __name__ == "__main__":
    main()
