#!/usr/bin/env python3
"""APU START travel and the buttons that wrongly shared it.

The generic export keyed the APU START cap on sim/cockpit2/electrical/APU_starter_switch at 0 and 2 only
(0 OFF, 1 ON, 2 START). OBJ8 interpolates between keys, so with the APU MASTER SW on (1) the START cap,
its legends and lamps sat half pressed in. FBW's PUSH_OVHD_APU_START clip also targets other nodes
(PUSH_CKPT_DOOR, PRINTER_BLANK2 ...), and the export gave every pushbutton instance sharing that mesh the
same motion, so pedestal buttons moved with the APU switch too.

  * a block at the APU START button: a key at 1 equal to the key at 0 is added (START travels only at 2);
  * a block elsewhere keyed on the APU switch (other than APU MASTER SW, keyed 0..1): the motion is
    frozen at its key-0 pose (a static ANIM_trans), which is where it rests today.

Only those lines change; vertex / index / texture payloads are untouched; re-running is a no-op.

    python3 tools/fix_apu_switch_travel.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" [--dry-run]
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

from obj8_numbers import fmt
from patch_efis_animations import vertex_payload

REF = "sim/cockpit2/electrical/APU_starter_switch"
APU_START = (0.001, 3.3397, 32.5286)      # FBW PUSH_OVHD_APU_START origin (OBJ frame)
APU_MASTER = (0.0008, 3.3571, 32.4887)
NEAR = 0.015
DIRS = ("objects/interior/cockpit-panels", "objects/interior/cockpit", "objects/interior/cockpit-controls")


def _block_location(lines, start, end, vertices, indices):
    """The block's static ANIM_trans pivot (local-frame export) or the centroid of its triangles."""
    for line in lines[start:end]:
        w = line.split()
        if w and w[0] == "ANIM_trans" and len(w) == 7:
            return tuple(map(float, w[1:4]))
    points = []
    for line in lines[start:end]:
        w = line.split()
        if w and w[0] == "TRIS":
            a, n = int(w[1]), int(w[2])
            points += [vertices[i] for i in indices[a:a + n]]
    if not points:
        return None
    return tuple(sum(p[k] for p in points) / len(points) for k in range(3))


def fix_text(text: str):
    lines = text.splitlines(keepends=True)
    vertices = [tuple(map(float, l.split()[1:4])) for l in lines if l.startswith("VT")]
    indices = [int(v) for l in lines if l.startswith(("IDX\t", "IDX10\t", "IDX ", "IDX10 ")) for v in l.split()[1:]]
    # ANIM_begin index of every line
    opens, owner = [], {}
    for i, line in enumerate(lines):
        w = line.split()
        if w[:1] == ["ANIM_begin"]:
            opens.append(i)
        elif w[:1] == ["ANIM_end"] and opens:
            begin = opens.pop()
            owner[begin] = i
    edits, report = {}, {"start": 0, "frozen": 0}
    for i, line in enumerate(lines):
        w = line.split()
        if w[:1] != ["ANIM_trans_begin"] or w[-1] != REF:
            continue
        k, keys = i + 1, []
        while lines[k].split()[:1] == ["ANIM_trans_key"]:
            keys.append((k, [float(v) for v in lines[k].split()[1:5]]))
            k += 1
        values = [kv[1][0] for kv in keys]
        if values != [0.0, 2.0]:
            continue
        begin = max((b for b in owner if b < i and owner[b] > i), default=None)
        if begin is None:
            continue
        where = _block_location(lines, begin, owner[begin], vertices, indices)
        indent = line[:len(line) - len(line.lstrip())]
        rest = keys[0][1][1:]
        if where is not None and math.dist(where, APU_START) <= NEAR:
            edits[keys[0][0]] = lines[keys[0][0]] + f"{indent}ANIM_trans_key\t1\t{fmt(rest[0])}\t{fmt(rest[1])}\t{fmt(rest[2])}\n"
            report["start"] += 1
        elif where is None or math.dist(where, APU_MASTER) > NEAR:
            edits[i] = f"{indent}ANIM_trans\t{chr(9).join(fmt(v) for v in (*rest, *rest))}\n"
            for kk, _v in keys:
                edits[kk] = ""
            edits[k] = ""                                 # ANIM_trans_end
            report["frozen"] += 1
    out = "".join(edits.get(i, line) for i, line in enumerate(lines))
    return out, report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    total = {"start": 0, "frozen": 0}
    files = 0
    for d in DIRS:
        for path in sorted((args.aircraft / d).glob("*.obj")):
            if path.name.startswith("._"):
                continue
            text = path.read_bytes().decode("latin-1")
            if REF not in text:
                continue
            fixed, report = fix_text(text)
            if fixed == text:
                continue
            if vertex_payload(text) != vertex_payload(fixed):
                raise ValueError(f"{path.name}: payload changed")
            files += 1
            for key in total:
                total[key] += report[key]
            if not args.dry_run:
                tmp = path.with_name(path.name + ".tmp-apu")
                tmp.write_bytes(fixed.encode("latin-1"))
                tmp.replace(path)
    print(f"{files} OBJ files {'would change' if args.dry_run else 'changed'}: {total['start']} APU START blocks "
          f"keyed to stay out at 1, {total['frozen']} other blocks frozen at rest")


if __name__ == "__main__":
    main()
