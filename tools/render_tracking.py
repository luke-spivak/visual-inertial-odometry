#!/usr/bin/env python3
"""Render recorded live tracks without rerunning OpenVINS. Requires numpy, opencv-python, ffmpeg."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

VERSION = 1


def inspect(prefix, width=1280, height=800):
    """Validate exact frame associations, including interrupted recordings."""
    paths = {k: Path(str(prefix) + suffix) for k, suffix in
             [('raw', '.y16'), ('meta', '.meta.json'), ('features', '.features.jsonl')]}
    warnings = []
    stamps = [int(v) for v in re.findall(r'"SensorTimestamp":\s*(\d+)', paths['meta'].read_text())]
    if not stamps or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('Camera timestamps must be present and strictly increasing')
    tracks, header, ending = {}, None, None
    if paths['features'].exists():
        with paths['features'].open() as f:
            for line in f:
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    if not line.endswith('\n'):
                        warnings.append('Truncated final feature record'); break
                    raise ValueError('Invalid feature JSON')
                if row['type'] == 'header':
                    header = row
                    if row.get('version') != 1 or row.get('coordinates') != 'raw_pixels':
                        raise ValueError('Unsupported feature schema')
                elif row['type'] == 'end':
                    ending = row
                elif row['type'] == 'frame':
                    if header is None:
                        raise ValueError('Missing feature header')
                    i, ts = row['frame_index'], row['timestamp_ns']
                    if i < 0 or i >= len(stamps) or stamps[i] != ts or i in tracks:
                        raise ValueError('Feature/frame timestamp or index mismatch')
                    if row['camera_id'] != 0:
                        raise ValueError('Only camera 0 is supported')
                    width, height = row['width'], row['height']
                    if tracks and (width, height) != (next(iter(tracks.values()))['width'], next(iter(tracks.values()))['height']):
                        raise ValueError('Image dimensions changed during run')
                    ids = set()
                    for fid, x, y in row['features']:
                        if fid in ids or not all(math.isfinite(v) for v in (x, y)) or not (0 <= x < width and 0 <= y < height):
                            raise ValueError('Invalid feature coordinates or duplicate ID')
                        ids.add(fid)
                    tracks[i] = row
        if ending is None:
            warnings.append('Feature recording has no completion marker')
    else:
        warnings.append('No live feature log: rendering plain camera video')
    if width <= 0 or height <= 0:
        raise ValueError('Invalid dimensions')
    per = width * height * 2
    count, partial = divmod(paths['raw'].stat().st_size, per)
    if abs(count - len(stamps)) > 8:
        raise ValueError('Raw/metadata counts differ by more than eight frames')
    n = min(count, len(stamps))
    if n == 0:
        raise ValueError('No complete frames')
    if partial or count != len(stamps):
        warnings.append('Incomplete recording: rendering common complete prefix')
    tracks = {i: v for i, v in tracks.items() if i < n}
    return paths, stamps[:n], tracks, width, height, warnings, ending


def signature(paths, options):
    return hashlib.sha256(json.dumps({'version': VERSION, 'options': options,
        'files': {k: [str(p.resolve()), p.stat().st_size, p.stat().st_mtime_ns]
                  for k, p in paths.items() if p.exists()}}, sort_keys=True).encode()).hexdigest()


def render(prefix, width=1280, height=800, fps=20, force=False, rotate180=False, gamma=1.0, age_dots=False):
    import cv2
    import numpy as np
    import imageio_ffmpeg
    if not math.isfinite(fps) or not 1 <= fps <= 120:
        raise ValueError('FPS must be between 1 and 120')
    if not math.isfinite(gamma) or gamma <= 0:
        raise ValueError('Gamma must be positive and finite')
    lut = np.round(255 * (np.arange(256) / 255.0) ** (1 / gamma)).astype(np.uint8)
    paths, stamps, tracks, w, h, warnings, ending = inspect(prefix, width, height)
    suffix = '.tracking' if gamma == 1 else '.tracking-bright'
    if age_dots: suffix = '.tracking-age-dots'
    output = Path(str(prefix) + suffix + '.mp4')
    summary_path = Path(str(prefix) + suffix + '-summary.json')
    sig = signature(paths, [width, height, fps, rotate180, gamma, age_dots])
    if not force and output.exists() and summary_path.exists():
        previous = json.loads(summary_path.read_text())
        if previous.get('signature') == sig and previous.get('output_bytes') == output.stat().st_size:
            print(f'Skipping unchanged {output}'); return previous
    raw = np.memmap(paths['raw'], mode='r', dtype=np.uint8)
    period = int(np.median(np.diff(stamps))) if len(stamps) > 1 else round(1e9 / fps)
    duration = (stamps[-1] - stamps[0] + period) / 1e9
    tmp = Path(str(prefix) + suffix + '.partial.mp4')
    proc = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{w}x{h}', '-r', str(fps), '-i', '-',
        '-an', '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-c:v', 'libx264', '-crf', '20',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(tmp)], stdin=subprocess.PIPE)
    history, cached, last_i = {}, None, -1
    # Track age uses capture timestamps, independent of output resampling.
    born = {}
    if age_dots:
        previous = set()
        for j in range(len(stamps)):
            features = tracks.get(j, {}).get('features', [])
            if j and stamps[j] - stamps[j-1] > period * 1.5:
                previous = set()
            births = {}
            for fid, _, _ in features:
                births[fid] = born[j-1][fid] if fid in previous else stamps[j]
            born[j] = births
            previous = set(births)
    i, frames_out = 0, math.ceil(duration * fps)
    try:
        for k in range(frames_out):
            target = stamps[0] + round(k * 1e9 / fps)
            while i + 1 < len(stamps) and stamps[i + 1] <= target:
                i += 1
            stale = target - stamps[i] > period * 1.5
            if i != last_i:
                if i != last_i + 1: history.clear()
                start = i * w * h * 2
                image = raw[start:start + w * h * 2].reshape(h, w, 2)
                if image[:, :, 0].any():
                    raise ValueError('Not the expected high-byte R8 image layout')
                cached = cv2.cvtColor(cv2.LUT(image[:, :, 1].copy(), lut), cv2.COLOR_GRAY2BGR)
                row = tracks.get(i)
                if row is None:
                    history.clear()
                else:
                    current = {}
                    for fid, x, y in row['features']:
                        if age_dots:
                            age = (stamps[i] - born[i][fid]) / 1e9
                            color = (0, 230, round(255 * (1 - min(age / 2.0, 1))))
                            cv2.circle(cached, (round(x), round(y)), 2, color, -1, cv2.LINE_AA)
                            continue
                        pts = (history.get(fid, []) + [(round(x), round(y))])[-12:]
                        current[fid] = pts
                        color = (80 + fid * 37 % 176, 80 + fid * 67 % 176, 80 + fid * 97 % 176)
                        if len(pts) > 1: cv2.polylines(cached, [np.array(pts, np.int32)], False, color, 1)
                        cv2.circle(cached, pts[-1], 3, color, -1)
                    history = current
                last_i = i
            frame = cached.copy()
            row = tracks.get(i)
            if stale:
                # Explicit blank instead of carrying stale observations across a capture gap.
                frame[:] = 0; history.clear()
                label = 'CAPTURE GAP'
            elif row is None:
                label = 'NO LIVE FEATURE OBSERVATIONS'
            else:
                label = f"LIVE TRACKS: {len(row['features'])} | " + ('initialized' if row['initialized'] else 'initializing')
            if rotate180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            cv2.rectangle(frame, (0, 0), (w, min(h, 55)), (0, 0, 0), -1)
            cv2.putText(frame, f'{k/fps:.2f}s | {label}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            detail = 'Track age: yellow = new | green = 2+ seconds' if age_dots else f'capture_ns={stamps[i]} frame={i}'
            cv2.putText(frame, detail, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, .45, (200, 200, 200), 1)
            proc.stdin.write(frame.tobytes())
        proc.stdin.close()
        if proc.wait() != 0: raise RuntimeError('FFmpeg encoding failed')
        tmp.replace(output)
    except BaseException:
        proc.kill(); proc.wait(); tmp.unlink(missing_ok=True); raise
    result = {'signature': sig, 'output_bytes': output.stat().st_size,
              'display_gamma': gamma,
              'overlay_style': 'age_dots' if age_dots else 'trails',
              'rotation_degrees': 180 if rotate180 else 0,
              'camera_frames': len(stamps), 'frames_with_observations': len(tracks),
              'frames_without_observations': len(stamps) - len(tracks),
              'duration_seconds': duration, 'output_fps': fps, 'output_frames': frames_out,
              'timing': 'capture timestamps resampled at constant FPS; error less than one output frame',
              'warnings': warnings, 'feature_log_end': ending}
    summary_path.write_text(json.dumps(result, indent=2) + '\n')
    print(f'Rendered {output}: {len(tracks)}/{len(stamps)} frames with live observations')
    for warning in warnings: print('WARNING:', warning)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('prefix'); p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=800); p.add_argument('--fps', type=float, default=20)
    p.add_argument('--force', action='store_true')
    p.add_argument('--rotate180', action='store_true', help='Rotate imagery and tracks upright, keeping labels readable')
    p.add_argument('--gamma', type=float, default=1.0, help='Display correction; values above 1 brighten raw imagery before overlays')
    p.add_argument('--age-dots', action='store_true', help='Small dots, yellow to green over two seconds of continuous tracking')
    a = p.parse_args()
    render(a.prefix, a.width, a.height, a.fps, a.force, a.rotate180, a.gamma, a.age_dots)
