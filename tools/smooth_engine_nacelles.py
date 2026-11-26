#!/usr/bin/env python3
"""Smooth the two engine nacelles: no dents from baked broad tilts, no speckled reflections.

FBW's nacelle normal maps (and the glTF ones prepare_engine_materials.py converted) carry broad tilts
across whole cowling panels: baked from a high-poly model, they only cancel out in a renderer whose
tangent space matches the baker's. X-Plane builds its own, so the painted cowling looks dented and wavy.
Each nacelle normal map is rebuilt from FBW's shipped DDS at --size (FBW's is 8192): same convention and
strength as the installed map (fitted on it: sign and scale per channel), then high-passed so only the
fine detail (panel lines, fasteners) stays, the broad tilt (Gaussian sigma --sigma texels at 2048) goes.
The nacelle gloss/metal map gets an edge-preserving median filter: the speckle that broke the
reflections up goes, the panel and material boundaries stay.

Normal maps stay PNG (normal-map-manifest.json is updated so halve_normal_maps.py leaves them alone);
afterwards re-run tools/convert_textures_to_dds.py so the gloss maps' DDS copies follow.

    python3 tools/smooth_engine_nacelles.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" \\
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
NACELLES = {"a380x_eng_lh": "A380_EXTERIOR_ENG_LH_NORM.PNG.DDS", "a380x_eng_rh": "A380_EXTERIOR_ENG_RH_NORM.PNG.DDS"}
WINGS = "textures/exterior/wings"


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
    manifest_path = args.aircraft / "normal-map-manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else None
    for name, dds in NACELLES.items():
        normal, gloss = wings / f"{name}_normal.png", wings / f"{name}_material_gloss.png"
        for path in (normal, gloss):
            if not path.is_file():
                raise SystemExit(f"missing {path}")
            if args.backup:
                (args.backup / WINGS).mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, args.backup / WINGS / path.name)
        with Image.open(args.fbw_textures / dds) as fbw, Image.open(normal) as installed:
            new_normal = smooth_normal(fbw, installed, args.size, args.sigma)
        with Image.open(gloss) as g:
            new_gloss = smooth_gloss(g.convert("RGBA"))
        new_normal.save(normal)
        new_gloss.save(gloss)
        if manifest is not None:
            rel = f"{WINGS}/{normal.name}"
            manifest.setdefault(rel, {}).update(halved_sha256=sha256(normal), smoothed_from=dds, smoothed_size=args.size)
        print(f"{name}: normal {new_normal.size[0]} px high-passed (sigma {args.sigma} @2048), gloss median 5")
    if manifest is not None:
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print("now re-run tools/convert_textures_to_dds.py on the aircraft so the gloss DDS copies follow")


if __name__ == "__main__":
    main()
