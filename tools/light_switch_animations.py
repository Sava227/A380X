#!/usr/bin/env python3
"""LAND and NOSE switch animations in the installed cockpit panel objects: the right way round, NOSE in three positions.

The export maps both switches backwards: LAND showed ON with sim/cockpit2/switches/landing_lights_on at 0, NOSE
showed T.O. with taxi_light_on at 0 (user sim checks 2026-11-27: lights on with the switch at OFF). LAND's two keys
are swapped. NOSE now follows the plugin's three-position switch a380x/xp12/elec/lights/nose_switch (0 OFF, 1 TAXI,
2 T.O.), on the export's own three poses (its keys 1, 0.5 and 0). Every panel object that draws the switches is patched;
idempotent (MARKER).

    python3 tools/light_switch_animations.py --aircraft <aircraft>
"""
from __future__ import annotations

import argparse
from pathlib import Path

MARKER = "# light switch animations: tools/light_switch_animations.py"
LAND = "sim/cockpit2/switches/landing_lights_on"
NOSE = "sim/cockpit2/switches/taxi_light_on"
NOSE_SWITCH = "a380x/xp12/elec/lights/nose_switch"


def keys_after(lines: list[str], i: int) -> list[int]:
    out = []
    for j in range(i + 1, len(lines)):
        if lines[j].strip().startswith("ANIM_rotate_key"):
            out.append(j)
        else:
            return out
    return out


def patch(text: str) -> tuple[str, int]:
    """(new text, switches patched)."""
    if MARKER in text:
        return text, 0
    lines = text.split("\n")
    n = 0
    for i, line in enumerate(lines):
        f = line.split()
        if f[:1] != ["ANIM_rotate_begin"] or f[-1] not in (LAND, NOSE):
            continue
        ks = keys_after(lines, i)
        vals = [lines[k].split() for k in ks]
        tab = line[:len(line) - len(line.lstrip())]
        if f[-1] == LAND:
            assert [v[1] for v in vals] == ["0", "1"], f"LAND keys {vals}"
            lines[ks[0]] = f"{tab}ANIM_rotate_key\t0\t{vals[1][2]}"
            lines[ks[1]] = f"{tab}ANIM_rotate_key\t1\t{vals[0][2]}"
        else:
            assert [v[1] for v in vals] == ["0", "0.5", "1"], f"NOSE keys {vals}"
            lines[i] = line.replace(NOSE, NOSE_SWITCH)
            for k, (value, pose) in zip(ks, (("0", vals[2][2]), ("1", vals[1][2]), ("2", vals[0][2]))):
                lines[k] = f"{tab}ANIM_rotate_key\t{value}\t{pose}"
        n += 1
    if not n:
        return text, 0
    return "\n".join(lines).replace("POINT_COUNTS", MARKER + "\nPOINT_COUNTS", 1), n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    args = ap.parse_args()
    files = total = 0
    for path in sorted((args.aircraft / "objects/interior").rglob("*.obj")):
        text = path.read_bytes().decode("latin-1")
        if LAND not in text and NOSE not in text:
            continue
        new, n = patch(text)
        if n:
            tmp = path.with_name(path.name + ".tmp-switches")
            tmp.write_bytes(new.encode("latin-1"))
            tmp.replace(path)
            files += 1
            total += n
    print(f"{files} objects, {total} switch animations patched")


if __name__ == "__main__":
    main()
