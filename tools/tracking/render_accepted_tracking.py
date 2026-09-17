#!/usr/bin/env python3
"""Render feature observations accepted by an offline OpenVINS replay."""
import argparse
import json
import re
import subprocess
from pathlib import Path

from render_tracking import inspect


def load_accepts(log_path):
    accepts = {}
    frame = None
    for line in Path(log_path).read_text(errors="replace").splitlines():
        m = re.search(r"DIAG_FRAME\s+(\d+)\s+[0-9.]+", line)
        if m:
            frame = int(m.group(1))
            accepts.setdefault(frame, set())
            continue
        if frame is not None:
            m = re.search(r"DIAG_(?:MSCKF|SLAM) accept_id\s+(-?\d+)", line)
            if m:
                accepts[frame].add(int(m.group(1)))
    return accepts


def render(prefix, replay_log, fps=20, rotate180=True):
    import cv2
    import numpy as np
    import imageio_ffmpeg

    paths, stamps, tracks, w, h, warnings, ending = inspect(prefix)
    accepted = load_accepts(replay_log)
    output = Path(str(prefix) + ".accepted-tracking.mp4")
    summary_path = Path(str(prefix) + ".accepted-tracking-summary.json")
    raw = np.memmap(paths["raw"], mode="r", dtype=np.uint8)
    period = int(np.median(np.diff(stamps))) if len(stamps) > 1 else round(1e9 / fps)
    duration = (stamps[-1] - stamps[0] + period) / 1e9
    proc = subprocess.Popen([
        imageio_ffmpeg.get_ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
        "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-", "-an",
        "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-crf", "20",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)], stdin=subprocess.PIPE)
    try:
        for k in range(int(np.ceil(duration * fps))):
            target = stamps[0] + round(k * 1e9 / fps)
            i = min(len(stamps) - 1, max(0, int(np.searchsorted(stamps, target, side="right") - 1)))
            start = i * w * h * 2
            frame = cv2.cvtColor(raw[start:start + w * h * 2].reshape(h, w, 2)[:, :, 1].copy(), cv2.COLOR_GRAY2BGR)
            row = tracks.get(i)
            ids = accepted.get(i, set())
            if row:
                for fid, x, y in row["features"]:
                    if fid in ids:
                        pt = (round(x), round(y))
                        cv2.circle(frame, pt, 4, (0, 220, 0), -1)
                        cv2.circle(frame, pt, 6, (255, 255, 255), 1)
            if rotate180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            cv2.putText(frame, f"ESTIMATOR ACCEPTED: {len(ids)}", (18, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f"tracked: {len(row['features']) if row else 0}  t={((stamps[i]-stamps[0])/1e9):.2f}s",
                        (18, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            proc.stdin.write(frame.tobytes())
    finally:
        proc.stdin.close()
        rc = proc.wait()
        if rc:
            raise RuntimeError(f"ffmpeg exited {rc}")
    result = {"output": str(output), "accepted_frames": sum(bool(v) for v in accepted.values()),
              "rotation_degrees": 180 if rotate180 else 0, "replay_log": str(replay_log)}
    summary_path.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix", type=Path)
    ap.add_argument("replay_log", type=Path)
    ap.add_argument("--fps", type=float, default=20)
    ap.add_argument("--no-rotate180", action="store_true")
    a = ap.parse_args()
    render(a.prefix, a.replay_log, a.fps, rotate180=not a.no_rotate180)
