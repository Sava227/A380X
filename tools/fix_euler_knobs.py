#!/usr/bin/env python3
"""Detent knobs exported as three Euler tables -> one rotation about the knob's own axis.

The generic glTF export animated the EFIS RANGE (ZOOM .. 640) knobs with three keyed Euler rotations
(z, y, x) of the node's whole orientation, spreading FBW's 10 clip keys over range_index 0..7. Between keys
OBJ8 interpolates each Euler angle linearly, so at every real detent (an integer index) the knob was tilted
off its axis, "rotating on the wrong axis". FBW's clip is a single-axis rotation of 36 degrees per detent,
detent n = key n.

Each such block becomes:  ANIM_trans <pivot> (unchanged), one ANIM_rotate_begin about the FBW axis keyed
0..9 -> the FBW angles, then the three tables frozen at their rest value (the export's local orientation).
Only those lines change; vertex / index / texture payloads are untouched. Re-running is a no-op.

    python3 tools/fix_euler_knobs.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" [--dry-run]
"""
from __future__ import annotations

import argparse
from pathlib import Path

from cockpit_feedback import DEFAULT_SOURCE, PANEL_DIRS, Source, compile_clip
from cockpit_feedback_contract import Control
from obj8_numbers import fmt
from patch_efis_animations import vertex_payload

MARKER = "# A380X EULER KNOB FIX V1"
KNOBS = (("a380x/xp12/efis/captain/range_index", "KNOB_EFIS_CS_RANGE"),
         ("a380x/xp12/efis/first_officer/range_index", "KNOB_EFIS_FO_RANGE"))


def knob_motion(source: Source, ref: str, node: str) -> dict:
    control = Control("euler", "selector", node, node, ref, positions=10)
    stage = compile_clip(source, control, node, ref, "selector")["stages"][0]
    if stage["kind"] != "rotation":
        raise ValueError(f"{node}: FBW clip is not a rotation")
    return stage


def fix_text(text: str, motions: dict[str, dict]) -> tuple[str, int]:
    lines = text.splitlines(keepends=True)
    out, i, fixed = [], 0, 0
    while i < len(lines):
        words = lines[i].split()
        ref = words[-1] if words and words[0] == "ANIM_rotate_begin" else None
        if ref in motions:
            tables, j = [], i
            while j < len(lines) and lines[j].split()[:1] == ["ANIM_rotate_begin"] and lines[j].split()[-1] == ref:
                k = j + 1
                keys = []
                while lines[k].split()[0] == "ANIM_rotate_key":
                    keys.append(tuple(map(float, lines[k].split()[1:3])))
                    k += 1
                if lines[k].split()[0] != "ANIM_rotate_end":
                    raise ValueError(f"unterminated rotation table at line {j + 1}")
                tables.append((lines[j].split()[1:4], keys))
                j = k + 1
            if len(tables) == 3:
                indent = lines[i][:len(lines[i]) - len(lines[i].lstrip())]
                stage = motions[ref]
                block = [f"{MARKER} {ref}",
                         f"ANIM_rotate_begin\t{chr(9).join(fmt(v) for v in stage['axis'])}\t{ref}",
                         *(f"ANIM_rotate_key\t{fmt(v)}\t{fmt(a)}" for v, a in zip(stage["values"], stage["poses"])),
                         "ANIM_rotate_end"]
                for axis, keys in tables:
                    rest = min(keys, key=lambda kv: abs(kv[0]))[1]       # the table at range_index 0
                    block.append(f"ANIM_rotate\t{chr(9).join(axis)}\t{fmt(rest)}\t{fmt(rest)}")
                out.extend(indent + row + "\n" for row in block)
                fixed += 1
                i = j
                continue
        out.append(lines[i])
        i += 1
    return "".join(out), fixed


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    source = Source(args.source)
    motions = {ref: knob_motion(source, ref, node) for ref, node in KNOBS}
    files = blocks = 0
    for directory in PANEL_DIRS:
        for path in sorted((args.aircraft / directory).glob("*.obj")):
            if path.name.startswith("._"):
                continue
            text = path.read_bytes().decode("latin-1")
            fixed_text, fixed = fix_text(text, motions)
            if not fixed:
                continue
            if vertex_payload(text) != vertex_payload(fixed_text):
                raise ValueError(f"{path.name}: payload changed")
            files += 1
            blocks += fixed
            if not args.dry_run:
                tmp = path.with_name(path.name + ".tmp-euler")
                tmp.write_bytes(fixed_text.encode("latin-1"))
                tmp.replace(path)
    print(f"{blocks} Euler knob blocks in {files} OBJ files {'would be ' if args.dry_run else ''}replaced")


if __name__ == "__main__":
    main()
