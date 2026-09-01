#!/usr/bin/env python3
"""
Generate a textured ground overlay for the VIO sim world.

The runway model is not broken and neither is the renderer -- both were
measured. The problem is SPATIAL SCALE: its 2K texture is stretched across a
1500 x 100 m plane, so at flight altitude the whole visible patch of ground
falls inside a couple of texels and minifies to one flat grey. Measured on a
probe frame, 57% of 16x16 blocks had a local standard deviation of 0.0. A KLT
tracker needs a local gradient; zero gradient is zero features, and it looks
exactly like a rendering failure without being one.

So size the texture to the camera instead of to the world:

    ground sample distance = 2 * alt * tan(hfov/2) / width
                           = 2 * 10 * tan(0.698) / 640  =  2.6 cm/px at 10 m

Tiles are 10 m wide carrying a 512 px texture => 2.0 cm/texel, just under the
GSD, so the detail the camera can resolve is actually there. Tiles are
VISUAL-ONLY (no <collision>): the vehicle still lands on the runway plane and
physics is untouched.

Four distinct textures are generated and assigned so that no tile shares one
with its neighbour -- a single repeated texture would give the tracker
identical patches 10 m apart and invite false correspondences.

    python3 make_ground.py --out-dir ../models/vio_ground
"""
import argparse
import math
import os
import struct
import zlib

import numpy as np


def write_png_gray(path, arr):
    """8-bit greyscale PNG. Pure stdlib so the generator has one dependency
    (numpy) rather than an image stack."""
    h, w = arr.shape
    raw = b"".join(b"\x00" + arr[r].tobytes() for r in range(h))

    def chunk(tag, body):
        c = tag + body
        return struct.pack(">I", len(body)) + c + struct.pack(">I", zlib.crc32(c))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def tileable_noise(size, cells, rng):
    """One octave of value noise that wraps at the edges, so adjacent tiles do
    not show a seam. Bilinear on a `cells` x `cells` lattice, sampled with
    modular indexing."""
    g = rng.random((cells, cells))
    ax = np.arange(size) * cells / size
    i0 = np.floor(ax).astype(int) % cells
    i1 = (i0 + 1) % cells
    t = (ax - np.floor(ax))[:, None]
    # Smoothstep: bilinear alone leaves visible lattice creases.
    t = t * t * (3 - 2 * t)
    top = g[np.ix_(i0, i0)] * (1 - t.T) + g[np.ix_(i0, i1)] * t.T
    bot = g[np.ix_(i1, i0)] * (1 - t.T) + g[np.ix_(i1, i1)] * t.T
    return top * (1 - t) + bot * t


def make_texture(size, seed):
    rng = np.random.default_rng(seed)
    # Octaves from coarse patches down to grit a few texels across. The high
    # octaves are what the tracker actually latches onto; the low ones stop the
    # image looking like uniform sandpaper.
    img = np.zeros((size, size))
    amp_total = 0.0
    for cells, amp in ((4, 1.0), (8, 0.6), (16, 0.4), (32, 0.3),
                       (64, 0.22), (128, 0.16), (256, 0.12)):
        if cells > size:
            break
        img += amp * tileable_noise(size, cells, rng)
        amp_total += amp
    img /= amp_total
    # Uncorrelated per-texel grit, then spread to [0.12, 0.88] albedo. Full
    # black and full white are avoided so shading still modulates the surface.
    img = 0.88 * img + 0.12 * rng.random((size, size))
    # Standardise to a FIXED mean and spread, not to each image's own min/max.
    # Per-image normalisation gave every variant a different local brightness,
    # so the 10 m tile boundaries showed up as straight, perfectly regular
    # edges -- strong features that exist nowhere in reality and repeat on a
    # grid, which is an invitation for false correspondences.
    img = (img - img.mean()) / img.std()
    return np.clip(0.50 + 0.155 * img, 0.06, 0.96)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default="../models/vio_ground")
    # 15 m tiles at 768 px give 1.95 cm/texel, still under the 2.62 cm GSD at
    # 10 m, for 36 tiles instead of the 81 that 10 m tiles needed. Tile COUNT
    # turned out to cost real time: 81 visual-only tiles with no collision
    # halved Gazebo's RTF (0.75 -> 0.40) before a single pixel was rendered,
    # purely as scene-graph load. See PROJECT.md "Real-time factor is a budget".
    ap.add_argument("--tile", type=float, default=15.0, help="tile size, metres")
    ap.add_argument("--extent", type=float, default=45.0, help="half-width, metres")
    ap.add_argument("--texture-px", type=int, default=768)
    ap.add_argument("--variants", type=int, default=4)
    # A high-resolution pad under the takeoff point. The 15 m tiles are sized
    # for the 2.6 cm GSD at 10 m altitude, which leaves the view from 0.215 m
    # on the ground a smooth blur: each 1.95 cm texel spans ~35 px, measured at
    # 18 FAST corners in 4 of 25 grid cells against 184 in 23 of 25 airborne.
    # OpenVINS' static initialiser needs a stationary window WITH trackable
    # features, so with nothing to track on the ground it can never initialise.
    # 2 m at 2048 px is 0.098 cm/texel, ~1.8 px per texel at that range.
    ap.add_argument("--pad-size", type=float, default=2.0,
                    help="side of the high-detail takeoff pad, metres (0 = none)")
    ap.add_argument("--pad-px", type=int, default=2048)
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()

    tex_dir = os.path.join(a.out_dir, "materials", "textures")
    os.makedirs(tex_dir, exist_ok=True)
    for v in range(a.variants):
        arr = (make_texture(a.texture_px, a.seed + v) * 255).astype(np.uint8)
        write_png_gray(os.path.join(tex_dir, f"ground{v}.png"), arr)

    if a.pad_size > 0:
        arr = (make_texture(a.pad_px, a.seed + 100) * 255).astype(np.uint8)
        write_png_gray(os.path.join(tex_dir, "pad.png"), arr)

    n = int(math.ceil(2 * a.extent / a.tile))
    half = n * a.tile / 2.0
    parts = []
    for i in range(n):
        for j in range(n):
            x = -half + (i + 0.5) * a.tile
            y = -half + (j + 0.5) * a.tile
            # Coprime strides so no tile touches another of the same variant.
            v = (i * 1 + j * 2) % a.variants
            parts.append(f"""      <visual name="g{i}_{j}">
        <pose>{x:.2f} {y:.2f} 0.01 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>{a.tile:.2f} {a.tile:.2f} 0.02</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr><metal>
            <albedo_map>model://vio_ground/materials/textures/ground{v}.png</albedo_map>
            <roughness>0.9</roughness>
            <metalness>0</metalness>
          </metal></pbr>
        </material>
      </visual>""")

    if a.pad_size > 0:
        # Sits above the tiles (their top is z=0.02) so it wins the depth test
        # rather than z-fighting with them.
        parts.append(f"""      <visual name="takeoff_pad">
        <pose>0 0 0.025 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>{a.pad_size:.2f} {a.pad_size:.2f} 0.01</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr><metal>
            <albedo_map>model://vio_ground/materials/textures/pad.png</albedo_map>
            <roughness>0.9</roughness>
            <metalness>0</metalness>
          </metal></pbr>
        </material>
      </visual>""")

    sdf = f"""<?xml version="1.0" ?>
<!-- GENERATED by gazebo/worlds/make_ground.py; do not hand-edit.
     tile={a.tile} m  extent={a.extent} m  texture={a.texture_px} px
     => {a.tile / a.texture_px * 100:.2f} cm/texel, against a 2.6 cm/px GSD at 10 m.
     Visual only: no collision, so the vehicle still lands on the runway plane. -->
<sdf version="1.9">
  <model name="vio_ground">
    <static>true</static>
    <link name="link">
{chr(10).join(parts)}
    </link>
  </model>
</sdf>
"""
    with open(os.path.join(a.out_dir, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(a.out_dir, "model.config"), "w") as f:
        f.write("""<?xml version="1.0"?>
<model>
  <name>vio_ground</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Visual-only textured ground tiles, sized so texture detail lands above the
    camera's ground sample distance. Generated by gazebo/worlds/make_ground.py.
  </description>
</model>
""")
    pad = (f" + {a.pad_size} m pad at {a.pad_px} px "
           f"({a.pad_size / a.pad_px * 100:.3f} cm/texel)") if a.pad_size > 0 else ""
    print(f"wrote {n*n} tiles ({n}x{n}, {a.tile} m) + {a.variants} textures{pad} -> {a.out_dir}")


if __name__ == "__main__":
    main()
