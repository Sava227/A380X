#!/usr/bin/env python3
"""Nose gear rides on X-Plane's strut compression, so the tyres sit on the ground at any weight.

The nose gear OBJs had no compression animation: a fixed height either floats in the air (before
2a7a44f, ~13 cm) or sinks (after it, ~0.32 m: X-Plane compressed the nose strut ~0.48 m; user sim
checks 2026-11-26). X-Plane's contact point is the strut end minus the tyre radius; the model is placed so
its tyre bottom is there at zero compression (ACF offsets, see the commit), and every nose gear object
(strut, tyres, lights) is lifted by sim/flightmodel2/gear/tire_vertical_deflection_mtr[0], the way
X-Plane-native gear is animated; retracted, it is lifted back by DROP so it stows where it did before. Idempotent (MARKER).

    python3 tools/nose_gear_compression.py --aircraft <aircraft>
"""
from __future__ import annotations

import argparse
from pathlib import Path

MARKER = "# nose gear compression: tools/nose_gear_compression.py"
DATAREF = "sim/flightmodel2/gear/tire_vertical_deflection_mtr[0]"
DEPLOY = "sim/flightmodel2/gear/deploy_ratio[0]"
DROP = 0.15822   # m the ACF offsets lower the gear (to the contact point); undone as it retracts into the bay
OBJECTS = [f"objects/exterior/gear-nose/{n}.obj" for n in
           ("gear_nose_a380_exterior_nlg", "gear_nose_a380_exterior_tyres", "gear_nose_a380x_accessories")]


def ride_on_compression(text: str) -> str:
    """Wrap everything drawn (after the last IDX line) in one ANIM block lifted by the deflection."""
    if MARKER in text:
        return text
    lines = text.split("\n")
    last_idx = max(i for i, l in enumerate(lines) if l.startswith("IDX"))
    assert not any(l.split()[:1] == ["ATTR_LOD"] for l in lines), "ATTR_LOD cannot sit inside an ANIM block"
    head = [MARKER, "ANIM_begin", f"\tANIM_trans_begin\t{DATAREF}",
            "\t\tANIM_trans_key\t0\t0\t0\t0", "\t\tANIM_trans_key\t2\t0\t2\t0", "\tANIM_trans_end",
            f"\tANIM_trans_begin\t{DEPLOY}", f"\t\tANIM_trans_key\t0\t0\t{DROP}\t0", "\t\tANIM_trans_key\t1\t0\t0\t0",
            "\tANIM_trans_end"]
    body = lines[last_idx + 1:]
    while body and not body[-1].strip():
        body.pop()
    return "\n".join(lines[:last_idx + 1] + head + body + ["ANIM_end", ""])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    args = ap.parse_args()
    for rel in OBJECTS:
        path = args.aircraft / rel
        text = path.read_text(encoding="latin-1")
        new = ride_on_compression(text)
        if new != text:
            tmp = path.with_name(path.name + ".tmp-compression")
            tmp.write_text(new, encoding="latin-1")
            tmp.replace(path)
        print(f"{path.name}: {'lifted by strut compression' if new != text else 'already done'}")


if __name__ == "__main__":
    main()
