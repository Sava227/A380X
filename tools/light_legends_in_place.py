#!/usr/bin/env python3
"""Light the overhead pushbutton legends in place, the way Laminar's A330 does it.

Laminar (A330 cockpit_annunciators.obj): each legend is a face of the button with a day texture (dim
printed legend, buttonsUV) and a TEXTURE_LIT (the glowing legend, buttonsUV_LIT); ATTR_light_level on the
lamp dataref fades the glow in. No overlay quad, no blending, nothing hidden.

User sim check 2026-11-23: our overlay quads (fcu_labels atlas, generic font) read as hazy text floating
over the button. FBW ships the lit art with the same UVs as the decal albedo (image-129):
A380X_PUSH_TEXT_EMIS = fbw-cockpit-source .../texture/push_text_emis_2048.png. So:

1. a380_pushbutton_decal_LIT.png from that art. MSFS scales emissive at runtime (FBW stores e.g. green
   at 80/255), so each legend is scaled to full brightness by its local peak; antialiasing is kept.
2. decal_a380_pushbutton_decal.obj: TEXTURE_LIT; every '# A380X LIT DECAL HIDE <face>' block
   (hide_lit_legend_decals.py) with one lamp becomes '# A380X LIT DECAL GLOW <face>': ATTR_light_level on
   that lamp instead of ANIM_hide. Everything else in the object stays dark (a never-written dataref,
   as Laminar's laminar/no_ref). Two-lamp faces (ENG 2 BLEED: one face covers both legends) keep the
   hide + overlay quads.
3. a380x_cockpit_controls.obj: the overlay quads of the converted lamps get an unconditional ANIM_hide.

Run after hide_lit_legend_decals.py and seat_lamp_quads.py. Re-running is a no-op. Needs numpy + Pillow.

    python3 tools/light_legends_in_place.py --aircraft "<repo>/output/A380X-XP12" [--dry-run]
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
from PIL import Image

from hide_lit_legend_decals import CONTROLS_OBJ, GLOW, MARKER as HIDE

DECAL = "objects/interior/cockpit-panels/decal_a380_pushbutton_decal.obj"
LIT_NAME = "a380_pushbutton_decal_LIT.png"
LIT_REF = f"TEXTURE_LIT ../../../textures/interior/cockpit-panels/{LIT_NAME}"
DARK = "ATTR_light_level\t0\t1\ta380x/xp12/no_ref"
FBW_LIT = Path.home() / "Developer/fbw-cockpit-source/a380-cockpit/texture/push_text_emis_2048.png"
QUAD_PREFIXES = ("# OVHD LAMP lamp/", "# ELEC LAMP lamp/", "# ADIRS ANNUNCIATOR ", "# FCU ANNUNCIATOR ",
                 "# A380X FEEDBACK V1 LAMP ")
OFF = "ANIM_hide\t-1000000\t1000000\t"


def lit_texture(rgba: np.ndarray, window: int = 101, floor: int = 16) -> np.ndarray:
    """Each legend scaled so its brightest pixel is 255 (local peak over `window` px)."""
    rgb = rgba[..., :3].astype(np.float32)
    peak = rgb.max(-1)
    pad = window // 2
    for axis in (0, 1):     # separable running max
        p = np.pad(peak, [(pad, pad) if a == axis else (0, 0) for a in (0, 1)])
        peak = np.max(np.stack([np.take(p, range(k, k + peak.shape[axis]), axis=axis) for k in range(window)]), 0)
    scale = np.where(peak >= floor, 255.0 / np.maximum(peak, 1), 0.0)
    return np.clip(rgb * scale[..., None] + 0.5, 0, 255).astype(np.uint8)


def convert_decal(text: str):
    """(text, {face: lamp dataref}) with single-lamp hide blocks turned into glow blocks."""
    lines = text.splitlines(keepends=True)
    out, faces, i = [], {}, 0
    while i < len(lines):
        m = re.match(rf"(\s*){HIDE} (\S+)\n", lines[i])
        hides = []
        if m:
            j = i + 2
            while lines[j].split()[:1] == ["ANIM_hide"]:
                hides.append(lines[j].split()[3]); j += 1
        if m and lines[i + 1].split() == ["ANIM_begin"] and len(hides) == 1 and lines[j].split()[:1] == ["TRIS"]:
            ind, face = m.groups()
            faces[face] = hides[0]
            out += [f"{ind}{GLOW} {face}\n", f"{ind}ATTR_light_level\t0\t1\t{hides[0]}\n", lines[j],
                    f"{ind}{DARK}\n"]
            assert lines[j + 1].split() == ["ANIM_end"], face
            i = j + 2
            continue
        out.append(lines[i]); i += 1
    text = "".join(out)
    if LIT_REF not in text:
        text = re.sub(r"(TEXTURE [^\n]*\n)", rf"\1{LIT_REF}\n", text, count=1)
    if f"\n{DARK}\n" not in text:
        # first command after the vertex / index tables: everything starts dark
        text = re.sub(r"((?:IDX(?:10)?\t[^\n]*\n)+)(?!IDX)", rf"\1{DARK}\n", text, count=1)
        assert f"\n{DARK}\n" in text
    return text, {**faces, **dict(re.findall(rf"{GLOW} (\S+)\n\s*ATTR_light_level\t0\t1\t(\S+)", text))}


def hide_quads(text: str, refs: set[str]):
    """Overlay quads of these lamp datarefs get an unconditional ANIM_hide before their TRIS."""
    lines = text.splitlines(keepends=True)
    insert, hidden = {}, []
    for i, line in enumerate(lines):
        if not line.startswith(QUAD_PREFIXES):
            continue
        ref = None
        for j in range(i + 1, min(len(lines), i + 14)):
            w = lines[j].split()
            if ref is None and w[:1] == ["ANIM_hide"] and len(w) == 4 and "avionics_power" not in w[3]:
                ref = w[3]
            if w[:1] == ["TRIS"]:
                if ref in refs:
                    hidden.append(ref)
                    if not lines[j - 1].lstrip().startswith(OFF):
                        insert[j] = lines[j][:len(lines[j]) - len(lines[j].lstrip())] + OFF + ref + "\n"
                break
    out = [insert.get(k, "") + line for k, line in enumerate(lines)]
    return "".join(out), hidden


def run(aircraft: Path, dry_run: bool):
    decal = aircraft / DECAL
    text, faces = convert_decal(decal.read_bytes().decode("latin-1"))
    controls = aircraft / CONTROLS_OBJ
    ctl, hidden = hide_quads(controls.read_bytes().decode("latin-1"), set(faces.values()))
    lit = aircraft / "textures/interior/cockpit-panels" / LIT_NAME
    if not dry_run:
        for path, new in ((decal, text), (controls, ctl)):
            path.write_bytes(new.encode("latin-1"))
        if not lit.is_file():
            Image.fromarray(lit_texture(np.asarray(Image.open(FBW_LIT).convert("RGBA")))).save(lit)
    return faces, hidden


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    faces, hidden = run(args.aircraft, args.dry_run)
    print(f"{len(faces)} legend faces lit in place; {len(hidden)} overlay quads hidden")
    missing = set(faces.values()) - set(hidden)
    if missing:
        print(f"  lamps without an overlay quad ({len(missing)}): {' '.join(sorted(missing)[:10])}")


if __name__ == "__main__":
    main()
