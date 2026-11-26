#!/usr/bin/env python3
"""Smooth the exterior parts that looked dented or speckled: nacelles, engine cores, wing fence, outer wings.

Normal maps: the exported / converted maps of these parts carry broad tilts across whole panels (baked
for a tangent space X-Plane does not use, and for the cores and wing fence amplified 3-25x over FBW's own
maps), so smooth painted skin looked dented and wavy. Each is rebuilt from FBW's shipped DDS at --size (at most FBW's own size):
the installed map's per-channel sign (fitted on it) at no more than FBW's own strength, then high-passed
(Gaussian sigma --sigma texels at 2048) so panel lines and fasteners stay and the broad tilt goes.
Gloss maps: an edge-preserving 5x5 median removes the speckle that broke the reflections up, panel and
material boundaries stay.

The cockpit, cabin, gear, tyres and fans are left alone: their broad shading is intended (fabric, padding,
knobs, cables, grime). smoothed-maps.json in the aircraft folder records each processed file's result, so a
re-run skips maps already smoothed (only a fresh export is processed again). Normal maps stay PNG
(normal-map-manifest.json is updated so halve_normal_maps.py leaves them alone); afterwards re-run
tools/convert_textures_to_dds.py so the gloss maps' DDS copies follow.

    python3 tools/smooth_exterior_maps.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" \\
        --fbw-textures ~/Downloads/texture --backup <dir>
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter

from convert_textures_to_dds import sha256

Image.MAX_IMAGE_PIXELS = None
WINGS = "textures/exterior/wings"
NORMALS = {   # installed normal map -> FBW's shipped normal map
    "a380x_eng_lh_normal.png": "A380_EXTERIOR_ENG_LH_NORM.PNG.DDS",
    "a380x_eng_rh_normal.png": "A380_EXTERIOR_ENG_RH_NORM.PNG.DDS",
    "a380x_eng_cores_lh_normal.png": "A380_EXTERIOR_ENG_CORES_LH_NORM.PNG.DDS",
    "a380x_eng_cores_rh_normal.png": "A380_EXTERIOR_ENG_CORES_RH_NORM.PNG.DDS",
    "image-013.png": "A380X_EXTERIOR_WINGFENCE_NORM.PNG.DDS",          # a380x_wingfence.obj
}
GLOSS = ["a380x_eng_lh_material_gloss.png", "a380x_eng_rh_material_gloss.png",
         "a380x_wing1_2_material_gloss.png", "a380x_wing2_2_material_gloss.png"]
STATE = "smoothed-maps.json"


def gaussian_blur(a: np.ndarray, sigma: float) -> np.ndarray:
    """Per-channel Gaussian blur of an HxWxC float array (FFT, wrapping like a tiled texture)."""
    h, w = a.shape[:2]
    fy, fx = np.fft.fftfreq(h)[:, None], np.fft.rfftfreq(w)[None, :]
    kernel = np.exp(-2 * (np.pi * sigma) ** 2 * (fx ** 2 + fy ** 2))
    return np.stack([np.fft.irfft2(np.fft.rfft2(a[..., c]) * kernel, s=(h, w)) for c in range(a.shape[2])], -1)


def channel_gain(fbw_rg: np.ndarray, installed_rg: np.ndarray) -> np.ndarray:
    """Per-channel sign and scale that map FBW's slopes onto the installed map's (least squares)."""
    return np.array([(f.ravel() @ i.ravel()) / max(f.ravel() @ f.ravel(), 1e-9)
                     for f, i in ((fbw_rg[..., c], installed_rg[..., c]) for c in range(2))])


def high_pass(rg: np.ndarray, sigma: float) -> np.ndarray:
    return rg - gaussian_blur(rg, sigma)


def smooth_normal(fbw: Image.Image, installed: Image.Image, size: int, sigma_2048: float) -> Image.Image:
    small = np.asarray(fbw.convert("RGB").resize(installed.size, Image.Resampling.LANCZOS), float)[..., :2] - 127.5
    gain = channel_gain(small, np.asarray(installed.convert("RGB"), float)[..., :2] - 127.5)
    gain = np.sign(gain) * np.minimum(np.abs(gain), 1.0)          # never stronger than FBW's own map
    rg = np.asarray(fbw.convert("RGB").resize((size, size), Image.Resampling.LANCZOS), float)[..., :2] - 127.5
    rg = high_pass(rg * gain, sigma_2048 * size / 2048)
    out = np.full((size, size, 3), 128, np.uint8)
    out[..., :2] = np.clip(np.rint(rg + 127.5), 0, 255).astype(np.uint8)
    return Image.fromarray(out, "RGB")


def smooth_gloss(image: Image.Image, size: int = 5) -> Image.Image:
    return image.filter(ImageFilter.MedianFilter(size))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    ap.add_argument("--fbw-textures", type=Path, required=True)
    ap.add_argument("--backup", type=Path)
    ap.add_argument("--size", type=int, default=2048)
    ap.add_argument("--sigma", type=float, default=24.0, help="broad-tilt blur radius in texels at 2048")
    args = ap.parse_args()
    wings = args.aircraft / WINGS
    state_path = args.aircraft / STATE
    state = json.loads(state_path.read_text()) if state_path.exists() else {}
    manifest_path = args.aircraft / "normal-map-manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else None

    def todo(name):
        path = wings / name
        if not path.is_file():
            raise SystemExit(f"missing {path}")
        if state.get(f"{WINGS}/{name}") == sha256(path):
            print(f"{name}: already smoothed")
            return None
        if args.backup:
            (args.backup / WINGS).mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, args.backup / WINGS / name)
        return path

    for name, dds in NORMALS.items():
        path = todo(name)
        if path is None:
            continue
        with Image.open(args.fbw_textures / dds) as fbw, Image.open(path) as installed:
            size = min(args.size, fbw.size[0])                         # no larger than FBW's own map
            smooth_normal(fbw, installed, size, args.sigma).save(path)
        state[f"{WINGS}/{name}"] = sha256(path)
        if manifest is not None:
            manifest.setdefault(f"{WINGS}/{name}", {}).update(halved_sha256=sha256(path), smoothed_from=dds, smoothed_size=size)
        print(f"{name}: rebuilt from {dds} at {size} px, high-passed (sigma {args.sigma} @2048)")
    for name in GLOSS:
        path = todo(name)
        if path is None:
            continue
        with Image.open(path) as g:
            smooth_gloss(g.convert("RGBA")).save(path)
        state[f"{WINGS}/{name}"] = sha256(path)
        print(f"{name}: median 5")
    state_path.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")
    if manifest is not None:
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print("now re-run tools/convert_textures_to_dds.py on the aircraft so the gloss DDS copies follow")


if __name__ == "__main__":
    main()
