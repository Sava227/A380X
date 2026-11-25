#!/usr/bin/env python3
"""Straighten the body-gear rear axles and steer them with the body-wheel steering angle.

FBW's body gear was exported at full rear-axle steer: the rear wheel pair and its axle sit yawed 15 deg
(measured -15.2 / -15.5 deg against the square front and middle axles) at every gear position, with no
animation. This wraps those meshes (two rear tyres in *_tyres.obj, the rear axle/brake mesh in *_blg.obj)
in one rotation about the bogie's vertical (local +z) through the rear-axle pivot on the beam centreline:
+15 deg straightens them, minus X-Plane's steer angle of that gear (sim/flightmodel2/gear/
tire_steer_actual_deg, + right). The plugin (a380x_fctl.h) steers the body gears opposite to the nose
wheel, up to 15 deg beyond 20 deg of nose-wheel angle, so the rear axles turn only with the tiller.
Idempotent (marker comment). Run on the installed aircraft and on output/.

    python3 tools/steer_body_gear_rear_axles.py --aircraft "<X-Plane>/Aircraft/A380X-XP12"
"""
from __future__ import annotations

import argparse
from pathlib import Path

MARKER = "# A380X REAR AXLE STEER"
BAKED_YAW = 15.0     # deg about bogie +z that undoes the exported full-steer pose
MAX_STEER = 15.0     # deg: body-wheel steering limit
# OBJ -> (pivot x, y in the bogie frame, gear index, consecutive TRIS lines of the rear axle)
PATCHES = {
    "objects/exterior/gear-body-lh/gear_body_lh_a380_exterior_tyres.obj": (-0.00296, 1.65862, 3, ["TRIS\t96213 98769", "TRIS\t0 96213"]),
    "objects/exterior/gear-body-lh/gear_body_lh_a380x_blg.obj": (-0.00296, 1.65862, 3, ["TRIS\t17181 10002"]),
    "objects/exterior/gear-body-rh/gear_body_rh_a380_exterior_tyres.obj": (-0.00456, 1.64770, 4, ["TRIS\t148179 148179", "TRIS\t0 148179"]),
    "objects/exterior/gear-body-rh/gear_body_rh_a380x_blg.obj": (-0.00456, 1.64770, 4, ["TRIS\t17181 10002"]),
}


def wrap(text: str, x: float, y: float, gear: int, tris: list[str]) -> str:
    """Wrap the consecutive TRIS lines in the steering rotation (fixed-point numbers: OBJ8 has no exponents)."""
    if MARKER in text:
        return text
    lines = text.split("\n")
    starts = [i for i in range(len(lines) - len(tris) + 1)
              if [lines[i + k].strip() for k in range(len(tris))] == tris]
    if len(starts) != 1:
        raise ValueError(f"expected one run of {tris}, found {len(starts)}")
    i = starts[0]
    tab = lines[i][:len(lines[i]) - len(lines[i].lstrip("\t"))]
    head = [f"{tab}{MARKER}", f"{tab}ANIM_begin",
            f"{tab}ANIM_trans\t{x:.5f}\t{y:.5f}\t0\t{x:.5f}\t{y:.5f}\t0",
            f"{tab}ANIM_rotate_begin\t0\t0\t1\tsim/flightmodel2/gear/tire_steer_actual_deg[{gear}]",
            f"{tab}ANIM_rotate_key\t{-MAX_STEER:.1f}\t{BAKED_YAW + MAX_STEER:.1f}",
            f"{tab}ANIM_rotate_key\t{MAX_STEER:.1f}\t{BAKED_YAW - MAX_STEER:.1f}",
            f"{tab}ANIM_rotate_end",
            f"{tab}ANIM_trans\t{-x:.5f}\t{-y:.5f}\t0\t{-x:.5f}\t{-y:.5f}\t0"]
    return "\n".join(lines[:i] + head + lines[i:i + len(tris)] + [f"{tab}ANIM_end"] + lines[i + len(tris):])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    args = parser.parse_args()
    for rel, (x, y, gear, tris) in PATCHES.items():
        path = args.aircraft / rel
        text = path.read_text()
        new = wrap(text, x, y, gear, tris)
        if new == text:
            print(f"{rel}: already patched")
            continue
        tmp = path.with_name(path.name + ".tmp-steer")
        tmp.write_text(new)
        tmp.replace(path)
        print(f"{rel}: rear axle steers with gear {gear}")


if __name__ == "__main__":
    main()
