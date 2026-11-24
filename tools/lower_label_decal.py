#!/usr/bin/env python3
"""Bring the panel-label decal back onto the panel (user sim check 2026-11-23).

persist_cockpit_material_patch.py exports decal_a380x_cockpit_decal_emis.obj (every panel caption and
the thin outlines FBW draws around the overhead pushbuttons) 2 mm out along each vertex normal
(decal_offset_m) with ATTR_poly_os 0. Seen at an angle, the outlines floated as a hazy white copy of
the button frame. FBW's source sits 0.06 mm above the panel. Every VT moves back along its own normal
(so local-frame keys move correctly too) to LIFT above FBW's position; nothing else changes.
Run after the material export; re-running is a no-op.

    python3 tools/lower_label_decal.py --aircraft "<repo>/output/A380X-XP12"
"""
from __future__ import annotations

import argparse
from pathlib import Path

from obj8_numbers import fmt

DECAL = "objects/interior/cockpit-panels/decal_a380x_cockpit_decal_emis.obj"
EXPORT_LIFT = 0.002
LIFT = 0.00025
MARKER = f"# A380X LABEL DECAL LIFT {LIFT}\n"


def lower(text: str) -> str:
    if MARKER in text:
        return text
    out, shift = [], EXPORT_LIFT - LIFT
    for line in text.splitlines(keepends=True):
        if line.startswith("VT"):
            w = line.split()
            p, n = [float(v) for v in w[1:4]], [float(v) for v in w[4:7]]
            line = "\t".join(["VT", *(fmt(p[k] - shift * n[k]) for k in range(3)), *w[4:]]) + "\n"
        out.append(line)
    head = out.index("OBJ\n") + 1
    return "".join(out[:head] + [MARKER] + out[head:])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    path = parser.parse_args().aircraft / DECAL
    text = path.read_bytes().decode("latin-1")
    new = lower(text)
    path.write_bytes(new.encode("latin-1"))
    print("already lowered" if new == text else f"lowered {new.count(chr(10) + 'VT')} vertices to {LIFT * 1000:g} mm")


if __name__ == "__main__":
    main()
