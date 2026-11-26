#!/usr/bin/env python3
"""Clean the fuselage paint: no window streaks, dirt blotches, gloss starbursts or white door outlines.

FBW's fuselage maps carry baked weathering: tapered streaks under every window and dithered dirt patches in
the colour map, and in the gloss map the same streaks and dirt plus radial "starburst" wedges around every
window and a matte band along every door and hatch outline. On X-Plane's glossy paint the matte bands
read as bright white outlines and the wedges as star lines around the windows.

For each fuselage object (a380x_fuse1..5) and each livery's copy of its colour map:
- colour: a clean paint estimate (15x15 median of a ~128 px copy, scaled back up) replaces every texel within
  --threshold (80) of it, i.e. weathering (8-45 levels darker in FBW's maps) and the light grey door and hatch
  outlines (55-70); lettering (120+ levels) differs by more and stays. Near livery edges and among the small parts' islands the estimate is
  a blend of paints, so nothing there is touched. Alpha is kept.
- gloss / metal: texels within 90 of the gloss map's own clean estimate take it (weathering, streaks, the
  window starbursts, the matte door-outline bands), so the paint has one even gloss across livery colours;
  different materials (120+ apart) keep theirs.
cleaned-paint.json records each result, so a re-run changes nothing. Afterwards re-run
tools/convert_textures_to_dds.py so the DDS copies follow.

    python3 tools/clean_fuselage_paint.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" --backup <dir>
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter

from convert_textures_to_dds import sha256

Image.MAX_IMAGE_PIXELS = None
OBJECTS = [f"objects/exterior/fuselage/a380x_fuse{i}.obj" for i in range(1, 6)]
STATE = "cleaned-paint.json"


def textures_of(root: Path, obj: str) -> dict[str, str]:
    """{'albedo': rel path, 'gloss': rel path} from the object's header."""
    out = {}
    with open(root / obj, errors="replace") as f:
        for line in f:
            if line.startswith(("POINT_COUNTS", "VT")):
                break
            field = line.split()
            kind = "albedo" if field[:1] == ["TEXTURE"] else "gloss" if field[:2] == ["TEXTURE_MAP", "material_gloss"] else None
            if kind:
                out[kind] = os.path.normpath(os.path.join(os.path.dirname(obj), field[-1])).replace(os.sep, "/")
    return out


def close_thin_dark(lum: np.ndarray, size: int) -> np.ndarray:
    """Grey-scale closing (max then min, separable, `size` texels): dark lines thinner than ~size vanish."""
    def run(a, op, axis):
        pad = [(0, 0), (0, 0)]; pad[axis] = (size // 2, size // 2)
        return op(np.lib.stride_tricks.sliding_window_view(np.pad(a, pad, mode="edge"), size, axis=axis), axis=-1)
    out = run(run(lum, np.max, 0), np.max, 1)
    return run(run(out, np.min, 0), np.min, 1)


def clean_estimate(rgb: np.ndarray, median: int = 15, flat: float = 24.0, busy_max: float = 50.0) -> tuple[np.ndarray, np.ndarray]:
    """Smooth paint colour per texel and where it is trustworthy.

    A 15x15 median of a ~128 px copy (window ~480 texels at 4096, so dirt patches up to ~340 texels across,
    FBW's largest, are outvoted), scaled back up. Where that estimate itself changes by more than `flat`
    levels within 3 cells (livery edges, lettering, the small parts' islands) it is a blend of two paints, so
    those texels are not touched."""
    h, w = rgb.shape[:2]
    factor = max(4, w // 128)
    small = Image.fromarray(rgb.astype(np.uint8)).resize((max(1, w // factor), max(1, h // factor)), Image.Resampling.BOX)
    small = small.filter(ImageFilter.MedianFilter(median))
    spread = np.asarray(small.filter(ImageFilter.MaxFilter(3)), np.float32) - np.asarray(small.filter(ImageFilter.MinFilter(3)), np.float32)
    spread = Image.fromarray(np.clip(spread.max(-1), 0, 255).astype(np.uint8)).filter(ImageFilter.MaxFilter(3))
    trusted = np.asarray(spread.resize((w, h), Image.Resampling.BILINEAR), np.float32) <= flat
    # and only painted skin: blocks whose own brightness spread (25th..75th percentile, thin dark lines closed
    # away first, so door outlines count as paint) is small; the small
    # parts' patchwork of grey, white and blue is busy even where the estimate averages out flat
    lum = close_thin_dark(rgb.mean(-1), max(3, w // 160) | 1)   # door/hatch outlines and streaks are paint
    bh, bw = h // factor, w // factor
    blocks = lum[:bh * factor, :bw * factor].reshape(bh, factor, bw, factor).transpose(0, 2, 1, 3).reshape(bh, bw, -1)
    busy = np.percentile(blocks, 75, axis=-1) - np.percentile(blocks, 25, axis=-1)   # thin seams (<1/4 of a block) don't count
    busy = Image.fromarray(np.clip(busy, 0, 255).astype(np.uint8)).filter(ImageFilter.MaxFilter(3))
    trusted &= np.asarray(busy.resize((w, h), Image.Resampling.BILINEAR), np.float32) <= busy_max
    return np.asarray(small.resize((w, h), Image.Resampling.BILINEAR), np.float32), trusted


def clean_albedo(image: Image.Image, threshold: float) -> Image.Image:
    rgba = np.asarray(image.convert("RGBA"), np.float32)
    rgb = rgba[..., :3]
    est, trusted = clean_estimate(rgb)
    weathering = trusted & (np.abs(rgb - est).max(-1) < threshold)
    rgb[weathering] = est[weathering]
    rgba[..., :3] = rgb
    return Image.fromarray(np.clip(np.rint(rgba), 0, 255).astype(np.uint8), "RGBA").convert(image.mode)


def clean_gloss(image: Image.Image, threshold: float = 90.0) -> Image.Image:
    """Even paint gloss: texels within `threshold` of the clean estimate (weathering, streaks, starbursts) take
    it, judged with thin matte lines (the door and hatch outline bands) closed away so those go whole;
    different materials wider than a seam and 90+ apart keep theirs."""
    rgba = np.asarray(image.convert("RGBA"), np.float32)
    est, flat = clean_estimate(rgba[..., :3], busy_max=255.0)
    w = rgba.shape[1]
    closed = np.stack([close_thin_dark(rgba[..., c], max(3, w // 160) | 1) for c in range(3)], -1)
    paint = flat & (np.abs(closed - est).max(-1) < threshold)
    rgba[..., :3][paint] = est[paint]
    return Image.fromarray(np.clip(np.rint(rgba), 0, 255).astype(np.uint8), "RGBA").convert(image.mode)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    ap.add_argument("--backup", type=Path)
    ap.add_argument("--threshold", type=float, default=80.0)
    args = ap.parse_args()
    root = args.aircraft
    state_path = root / STATE
    state = json.loads(state_path.read_text()) if state_path.exists() else {}
    liveries = sorted(p.relative_to(root) for p in (root / "liveries").glob("*") if p.is_dir()) if (root / "liveries").is_dir() else []
    for obj in OBJECTS:
        maps = textures_of(root, obj)
        for prefix in [""] + [f"{lv}/" for lv in liveries]:
            for rel, kind in ((prefix + maps["albedo"], "albedo"), (prefix + maps.get("gloss", ""), "gloss")):
                path = root / rel
                if kind == "gloss" and not maps.get("gloss") or not path.is_file() or state.get(rel) == sha256(path):
                    continue
                if args.backup:
                    (args.backup / rel).parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(path, args.backup / rel)
                with Image.open(path) as im:
                    out = clean_albedo(im, args.threshold) if kind == "albedo" else clean_gloss(im)
                out.save(path)
                state[rel] = sha256(path)
                print(f"{rel}: {kind} cleaned")
    state_path.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")
    print("now re-run tools/convert_textures_to_dds.py on the aircraft so the DDS copies follow")


if __name__ == "__main__":
    main()
