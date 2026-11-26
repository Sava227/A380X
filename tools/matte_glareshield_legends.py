#!/usr/bin/env python3
"""Matte glareshield pushbutton legends (MASTER WARN / CAUT, AUTO LAND, SIDESTICK PRIORITY, ATC MSG).

The legend decal (decal_a380_pushbutton_decal.obj) gives these lenses FBW's glossy glass (gloss ~82/255).
The glareshield legends face up towards the windshield, and X-Plane reflects the bright sky in them:
on a lens this dark (albedo ~45/255) the reflection outshines the paint, so the buttons read as pale,
see-through squares with a faint legend (user sim check 2026-11-24). The overhead legends face down at
the cockpit and are unaffected; the glareshield panel around the buttons stays dark because its
restored material is matte (gloss 20/255).

In the legend atlas, each PUSH_GLARESHIELD_* face's texel rectangle (its own; no other face shares
it) gets the panel's gloss: material_gloss green = GLOSS, and normal-map alpha (the legacy, pre-
NORMAL_METALNESS gloss) = 0. Every other legend keeps its material. Idempotent. Re-run
convert_textures_to_dds.py afterwards: X-Plane loads the material map's DDS.

    python3 tools/matte_glareshield_legends.py --aircraft "<X-Plane>/Aircraft/A380X-XP12"
"""
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import numpy as np
from PIL import Image

DECAL = "objects/interior/cockpit-panels/decal_a380_pushbutton_decal.obj"
GLOSS = 20       # a380_cockpit_glareshield_4k_material_gloss.png: the panel around the buttons


def texture(text: str, kind: str) -> str:
    """Path (relative to the OBJ) of TEXTURE_MAP `kind`."""
    return next(l.split()[-1] for l in text.splitlines() if l.split()[:2] == ["TEXTURE_MAP", kind])


def face_rects(text: str, width: int, height: int) -> dict[str, tuple[int, int, int, int]]:
    """{face: (x0, x1, y0, y1)} texel rectangles of the glareshield legend faces (image rows top-down)."""
    lines = text.splitlines()
    uv = np.array([[float(x) for x in l.split()[7:9]] for l in lines if l.startswith("VT")])
    idx = [int(x) for l in lines if l.startswith("IDX") for x in l.split()[1:]]
    rects = {}
    for i, l in enumerate(lines):
        m = re.search(r"LIT DECAL (GLOW|HIDE) (PUSH_GLARESHIELD_\S+)", l)
        if not m:
            continue
        t = next(lines[j] for j in range(i, i + 6) if lines[j].strip().startswith("TRIS")).split()
        st = uv[idx[int(t[1]):int(t[1]) + int(t[2])]]
        rects[m.group(2)] = (math.floor(st[:, 0].min() * width), math.ceil(st[:, 0].max() * width),
                             math.floor((1 - st[:, 1].max()) * height), math.ceil((1 - st[:, 1].min()) * height))
    if not rects:
        raise ValueError("no glareshield legend faces found")
    return rects


def paint(path: Path, rects, channel: int, value: int) -> int:
    """Set `channel` to `value` inside every rect; returns texels changed."""
    image = Image.open(path)
    mode = image.mode
    pixels = np.array(image.convert("RGBA"))
    before = pixels.copy()
    for x0, x1, y0, y1 in rects:
        pixels[y0:y1, x0:x1, channel] = value
    changed = int((pixels != before).any(-1).sum())
    if changed:
        out = Image.fromarray(pixels, "RGBA")
        tmp = path.with_name(path.name + ".tmp-matte.png")
        (out if mode == "RGBA" else out.convert(mode)).save(tmp, optimize=True)
        tmp.replace(path)
    return changed


OPAQUE = "ATTR_no_blend\t0.5"


def draw_opaque(text: str) -> tuple[str, int]:
    """Draw the glareshield legend faces solid, not blended (user sim check 2026-11-26: still a mirror-white
    sheen after the matte maps). The faces are opaque; X-Plane draws blended geometry in a later pass that
    did not take their matte material. ATTR_no_blend before each face's TRIS, ATTR_blend after. Idempotent."""
    lines = text.split("\n")
    out, n, face = [], 0, False
    for i, l in enumerate(lines):
        if re.search(r"LIT DECAL (GLOW|HIDE) PUSH_GLARESHIELD_\S+", l):
            face = True
        elif face and l.strip().startswith("TRIS"):
            indent = l[:len(l) - len(l.lstrip())]
            if not (out and out[-1].strip() == OPAQUE.strip()):
                out.append(indent + OPAQUE)
                n += 1
            out.append(l)
            if not (i + 1 < len(lines) and lines[i + 1].strip() == "ATTR_blend"):
                out.append(indent + "ATTR_blend")
            face = False
            continue
        out.append(l)
    return "\n".join(out), n


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    args = parser.parse_args()
    obj = args.aircraft / DECAL
    text, n = draw_opaque(obj.read_text())
    if n:
        tmp = obj.with_name(obj.name + ".tmp-matte")
        tmp.write_text(text)
        tmp.replace(obj)
    print(f"{obj.name}: {n} glareshield legend faces drawn solid")
    for kind, channel, value in (("material_gloss", 1, GLOSS), ("normal", 3, 0)):
        path = (obj.parent / texture(text, kind)).resolve()
        w, h = Image.open(path).size
        n = paint(path, face_rects(text, w, h).values(), channel, value)
        print(f"{path.name}: {n} texels set ({kind})")


if __name__ == "__main__":
    main()
