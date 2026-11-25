#!/usr/bin/env python3
"""Replace the fuselage, wing and pylon normal maps with FBW's shipped ones (the MSFS texture folder).

The exported maps (glTF source) carry FBW's detail amplified 2-9x on top of broad tilts that are constant
over whole UV islands (fuselage tilt ~16/127 vs FBW ~2). X-Plane lights each island as if the skin were
bent: the nose shows a bulge and stair-stepped bands. FBW's own DDS has the same detail (correlation
0.82-0.91, same green sign) on a flat base. Each map is decoded, resized to the size installed now (the
normal-map halving budget stays), written as RGB with blue 128 (X-Plane rebuilds Z), and its
normal-map-manifest.json entry is updated so halve_normal_maps.py does not halve it again.
--min-size raises a map to that size (never above FBW's own), e.g. 2048 for sharper fuselage and wing
panel lines (+~0.2 GB texture memory over the halved 1024 px maps).
Engine materials (prepare_engine_materials.py) and the 256 px stubs (gear, bays, windows) are left alone.

    python3 tools/restore_fbw_exterior_normals.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" \
        --fbw-textures ~/Downloads/texture --backup <dir>
Needs Pillow >= 10 (BC5 DDS) and numpy.
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image

from convert_textures_to_dds import sha256

MAPS = {   # installed normal map (relative to the aircraft) -> FBW texture
    "textures/exterior/fuselage/image-009.png": "A380X_FUSE1_NORMAL.PNG.DDS",
    "textures/exterior/fuselage/image-006.png": "A380X_FUSE2_NORMAL.PNG.DDS",
    "textures/exterior/fuselage/image-019.png": "A380X_FUSE3_NORMAL.PNG.DDS",
    "textures/exterior/fuselage/image-022.png": "A380X_FUSE4_NORMAL.PNG.DDS",
    "textures/exterior/fuselage/image-028.png": "A380X_FUSE5_NORMAL.PNG.DDS",
    "textures/exterior/wings/image-035.png": "A380_EXTERIOR_WING1_NORM.PNG.DDS",
    "textures/exterior/gear-wing-lh/image-006.png": "A380_EXTERIOR_WING1_NORM.PNG.DDS",
    "textures/exterior/wings/image-010.png": "A380_EXTERIOR_WING2_NORM.PNG.DDS",
    "textures/exterior/gear-wing-rh/image-006.png": "A380_EXTERIOR_WING2_NORM.PNG.DDS",
    "textures/exterior/wings/image-041.png": "A380_EXTERIOR_WING1_2_NORM.PNG.DDS",
    "textures/exterior/wings/image-000.png": "A380_EXTERIOR_WING2_2_NORM.PNG.DDS",
    "textures/exterior/wings/image-021.png": "A380_EXTERTIOR_PYLON_NORM.PNG.DDS",
}
MANIFEST = "normal-map-manifest.json"


def x_plane_normal(fbw: Image.Image, size: tuple[int, int]) -> Image.Image:
    """FBW X / Y slopes in red / green at `size`, blue 128 (the installed maps' layout)."""
    rg = fbw.convert("RGB").resize(size, Image.Resampling.LANCZOS)
    pixels = np.asarray(rg).copy()
    pixels[..., 2] = 128
    return Image.fromarray(pixels, "RGB")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--fbw-textures", type=Path, required=True, help="FBW MSFS texture folder (DDS)")
    parser.add_argument("--backup", type=Path, required=True, help="folder for the replaced PNGs")
    parser.add_argument("--min-size", type=int, default=0, help="raise each map to at least this size")
    args = parser.parse_args()
    aircraft = args.aircraft.resolve()
    manifest_path = aircraft / MANIFEST
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else None   # None: not halved (output/)
    for rel, fbw_name in MAPS.items():
        path, source = aircraft / rel, args.fbw_textures / fbw_name
        if not path.is_file() or not source.is_file():
            raise SystemExit(f"missing {path if not path.is_file() else source}")
        fbw = Image.open(source)
        size = Image.open(path).size
        if args.min_size > size[0]:
            side = min(args.min_size, fbw.size[0])
            size = (side, side)
        target = args.backup / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            shutil.copy2(path, target)          # keep the first original across re-runs
        tmp = path.with_name(path.name + ".tmp-fbw.png")
        x_plane_normal(fbw, size).save(tmp, optimize=True)
        tmp.replace(path)
        if manifest is not None:   # final size: halve_normal_maps.py must never halve it again
            manifest.setdefault(rel, {}).update(halved_sha256=sha256(path), fbw_source=fbw_name)
        print(f"{rel} <- {fbw_name} at {size[0]} px")
    if manifest is not None:
        manifest_path.write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
