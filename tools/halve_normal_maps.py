#!/usr/bin/env python3
"""Halve the aircraft's normal maps to cut texture memory (they stay PNG: block compression damages them,
see convert_textures_to_dds.py). Normal maps were 1.5 GB of the ~2.7 GB the A380 keeps in video memory;
at half resolution they take a quarter of that, and 8K cockpit panels still get 4K normals.

A normal map is any PNG an attached OBJ uses as TEXTURE_MAP normal / TEXTURE_NORMAL (and only as that),
plus the same file in liveries/*/. Each is halved by plain area filtering (never below MIN_SIZE; the maps
hold only X / Y slopes, so nothing is re-normalised), and a completely flat map becomes 16 x 16.
Originals are copied to --backup first; <aircraft>/normal-map-manifest.json records what was halved, so a
re-run never halves twice (only changed sources are halved again: re-run it after syncing output/).

    python3 tools/halve_normal_maps.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" --backup <dir> [--dry-run]
Needs Pillow and numpy.
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image

from convert_textures_to_dds import png_size, referenced_textures, sha256  # noqa: E402

MIN_SIZE = 1024
MANIFEST = "normal-map-manifest.json"
FLAT_SIZE = 16


def halve(image: Image.Image) -> Image.Image:
    """Half-size normal map, plain area filtering. The FBW maps store only X / Y slopes in red / green
    (blue 128, alpha 255; X-Plane rebuilds Z), so no channel is re-normalised. A map with every pixel equal
    carries no detail at all and becomes FLAT_SIZE x FLAT_SIZE."""
    pixels = np.asarray(image)
    if (pixels == pixels.reshape(-1, *pixels.shape[2:])[0]).all():
        return image.resize((FLAT_SIZE, FLAT_SIZE), Image.Resampling.NEAREST)
    return image.resize((image.width // 2, image.height // 2), Image.Resampling.LANCZOS)


def normal_maps(aircraft: Path) -> list[Path]:
    roles, _cutoffs = referenced_textures(aircraft)
    out = []
    for png, kinds in roles.items():
        if kinds != {"normal"}:
            continue                     # also used as colour / gloss somewhere: leave it alone
        try:
            rel = png.relative_to(aircraft)
        except ValueError:
            continue
        out += [p for p in [aircraft / rel, *(aircraft / "liveries").glob(f"*/{rel}")] if p.is_file()]
    return sorted(set(out))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--backup", type=Path, required=True, help="folder for the original PNGs")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    aircraft = args.aircraft.resolve()
    manifest_path = aircraft / MANIFEST
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    done = skipped = 0
    before = after = 0
    for path in normal_maps(aircraft):
        rel = str(path.relative_to(aircraft))
        size = png_size(path)
        if size is None or max(size) <= MIN_SIZE:
            skipped += 1
            continue
        digest = sha256(path)
        if manifest.get(rel, {}).get("halved_sha256") == digest:
            skipped += 1                 # already halved by this tool
            continue
        w, h = size
        small = halve(Image.open(path))
        before += w * h * 4 * 4 // 3
        after += small.width * small.height * 4 * 4 // 3
        done += 1
        if args.dry_run:
            continue
        target = args.backup / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        tmp = path.with_name(path.name + ".tmp-halve.png")
        small.save(tmp, optimize=True)
        tmp.replace(path)
        manifest[rel] = {"original_sha256": digest, "original_size": [w, h], "halved_sha256": sha256(path)}
        manifest_path.write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n")
    print(f"{done} normal maps {'would be ' if args.dry_run else ''}halved, {skipped} skipped (small or done); "
          f"texture memory {before / 1e9:.2f} GB -> {after / 1e9:.2f} GB")


if __name__ == "__main__":
    main()
