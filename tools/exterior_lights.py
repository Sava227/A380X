#!/usr/bin/env python3
"""Exterior lights (nav, logo, beacons, strobes, landing, nose taxi / take-off) at FBW's light positions.

The aircraft had none (user, 2026-11-26). FBW places each light on a named node of a380_exterior.gltf
(systems.cfg [LIGHTS]); those node positions, taken at gear down, become X-Plane 12 parameterised lights
(Laminar's own A330 uses the same kinds, Resources/bitmaps/world/lites/lights.txt):
    LIGHT_PARAM airplane_<kind>_bb|pm  x y z  R G B  INDEX  INTENSITY(cd)  DX DY DZ  WIDTH (cos half-angle)
_bb is the billboard you see, _pm the light it throws. X-Plane's own light systems drive them, and the cockpit
EXT LT switches already set those (config/animation_datarefs.json): NAV & LOGO -> navigation_lights_on (the logo
lights ride the nav system), BEACON, STROBE, LAND -> landing_lights_on, NOSE -> taxi_light_on (taxi and T.O. lights).
The object uses the frame of the other exterior objects (FBW glTF: x left, y up, z forward; ACF psi 180, z 115.108 ft).
Wingtip lights ride the wing flex chain of WING_FLEX_15, copied from the installed wing objects; the nose gear
lights show only with the nose gear down and locked.

shortcut: no RWY TURN OFF or WING / ENG scan lights (their cockpit switches drive nothing yet); wire those switches
first, then add airplane_spot_* / airplane_generic_* lights here.

    python3 tools/exterior_lights.py --aircraft <aircraft> --fbw <a380_exterior.gltf>
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
from obj8_numbers import fmt  # noqa: E402
from telescope_reverser_actuators import Gltf  # noqa: E402
from gear_compression import world_multi  # noqa: E402

OBJ = "objects/exterior/lights/a380x_ext_lights.obj"
ACF_FILE = "exterior/lights/a380x_ext_lights.obj"
FLEX = "a380x/xp12/wing/flex_{}_"
NOSE_DOWN = "sim/flightmodel2/gear/deploy_ratio[0]"
# FBW's wingtip light nodes ride the bare WING_FLEX_15 bone, 0.51 m below the installed wingtip skin (baked from the
# skinned mesh): measured 2026-11-27 at the nav and strobe lights (skin mid-thickness 2.78-2.79 m vs nodes 2.27 m).
TIP_LIFT = np.array([0.0, 0.51, 0.0])


def cone(deg):
    return float(np.cos(np.radians(deg)))


def unit(*v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


RED, GREEN, WHITE = (1, 0, 0), (0.02, 0.74, 0.36), (0.95, 0.82, 0.72)   # Laminar A330's nav colours
LAMP, STROBE = (0.76, 0.66, 0.58), (1, 1, 1)                             # its landing lamp and strobe colours
SIDE = unit(np.sin(np.radians(55)), 0, np.cos(np.radians(55)))           # nav: forward to 110 deg off the nose
# (FBW node, where, kinds, colour, index, candela, direction (+x left, +z forward), cone half-angle deg)
LIGHTS = [
    ("LIGHT_ASOBO_NavigationRed", "flex_L", ("nav",), RED, 0, 3000, SIDE, 55),
    ("LIGHT_ASOBO_NavigationGreen", "flex_R", ("nav",), GREEN, 0, 3000, SIDE * [-1, 1, 1], 55),
    ("LIGHT_ASOBO_OBSTRUCTION_LH", "flex_L", ("nav",), WHITE, 0, 3000, unit(0, 0, -1), 70),
    ("LIGHT_ASOBO_OBSTRUCTION_RH", "flex_R", ("nav",), WHITE, 0, 3000, unit(0, 0, -1), 70),
    ("LIGHT_ASOBO_NavigationTail", "body", ("nav",), WHITE, 0, 3000, unit(0, 0, -1), 70),
    ("LIGHT_ASOBO_StrobeWingLeftOuter", "flex_L", ("strobe",), STROBE, 0, 100000, unit(1, 0, 0), 87.5),
    ("LIGHT_ASOBO_StrobeWingRightOuter", "flex_R", ("strobe",), STROBE, 0, 100000, unit(-1, 0, 0), 87.5),
    ("LIGHT_ASOBO_StrobeTail_LH", "body", ("strobe",), STROBE, 0, 100000, unit(0, 0, -1), 87.5),
    ("LIGHT_ASOBO_BeaconTop", "body", ("beacon",), RED, 0, 40000, unit(0, 1, 0), 87.5),
    ("LIGHT_ASOBO_BeaconBelly", "body", ("beacon",), RED, 0, 40000, unit(0, -1, 0), 87.5),
    ("LIGHT_ASOBO_LAND_1_LH", "body", ("landing",), LAMP, 0, 765000, unit(0, -0.05, 1), 12.5),
    ("LIGHT_ASOBO_LAND_2_LH", "body", ("landing",), LAMP, 0, 765000, unit(0, -0.05, 1), 12.5),
    ("LIGHT_ASOBO_LAND_1_RH", "body", ("landing",), LAMP, 1, 765000, unit(0, -0.05, 1), 12.5),
    ("LIGHT_ASOBO_LAND_2_RH", "body", ("landing",), LAMP, 1, 765000, unit(0, -0.05, 1), 12.5),
    ("LIGHT_ASOBO_TAXI_WING_LH", "body", ("taxi",), LAMP, 0, 200000, unit(0, -0.1, 1), 25),
    ("LIGHT_ASOBO_TAXI_WING_RH", "body", ("taxi",), LAMP, 0, 200000, unit(0, -0.1, 1), 25),
    ("LIGHT_ASOBO_TAKEOFF_1", "nose", ("taxi",), LAMP, 0, 300000, unit(0, -0.1, 1), 25),
    ("LIGHT_ASOBO_TAKEOFF_2", "nose", ("taxi",), LAMP, 0, 500000, unit(0, -0.05, 1), 12.5),
    ("LIGHT_ASOBO_TAKEOFF_3", "nose", ("taxi",), LAMP, 0, 500000, unit(0, -0.05, 1), 12.5),
]
# logo: spill only, onto the fin from the tailplane, on the nav system (one NAV & LOGO switch)
LOGOS = ["LIGHT_ASOBO_LOGO_FWD_LH", "LIGHT_ASOBO_LOGO_FWD_RH"]
FIN = np.array([0.0, 13.0, -40.0])


def light_lines(kind, pos, rgb, index, cd, d, half, tab, spill=True):
    tail = [*map(fmt, rgb), str(index), f"{cd}cd", *map(fmt, d), fmt(cone(half))]
    head = [fmt(c) for c in pos]
    return [f"{tab}LIGHT_PARAM\tairplane_{kind}_{s} " + " ".join(head + tail) for s in (("bb", "pm") if spill else ("pm",))]


def flex_chain(wing_obj: str, side: str) -> list[str]:
    """The deepest wing flex chain in a wing object (root bone to WING_FLEX_15): its lines up to the closing trans."""
    lines = wing_obj.split("\n")
    best, start, n = (0, 0), 0, 0
    for i, l in enumerate(lines):
        w = l.strip()
        if l.startswith("ANIM_begin"):
            start, n = i, 0
        elif w.startswith("ANIM_rotate_begin") and FLEX.format(side) in w:
            n += 1
            if n > best[0]:
                best = (n, start)
    assert best[0] == 13, f"{side}: expected the 13-bone chain, found {best[0]}"
    out, seen = [], 0
    for l in lines[best[1] + 1:]:
        w = l.strip()
        out.append("\t" + w)
        if w.startswith("ANIM_rotate_begin"):
            seen += 1
        elif w.startswith("ANIM_trans") and seen == 13:
            return out
    raise ValueError(f"{side}: flex chain not closed")


def build(positions: dict, chains: dict) -> str:
    groups = {"body": [], "flex_L": [], "flex_R": [], "nose": []}
    for node, where, kinds, rgb, index, cd, d, half in LIGHTS:
        for kind in kinds:
            pos = positions[node] + (TIP_LIFT if where.startswith("flex") else 0)
            groups[where] += light_lines(kind, pos, rgb, index, cd, d, half, "\t" if where != "body" else "")
    for node in LOGOS:
        p = positions[node]
        groups["body"] += light_lines("nav", p, WHITE, 0, 20000, unit(*(FIN - p)), 30, "", spill=False)
    out = ["I", "800", "OBJ", "", "POINT_COUNTS\t0\t0\t0\t0", "", "# exterior lights: tools/exterior_lights.py"]
    out += groups["body"]
    for side in ("L", "R"):
        out += ["ANIM_begin", *chains[side], *groups[f"flex_{side}"], "ANIM_end"]
    out += ["ANIM_begin", f"\tANIM_hide\t0\t0.99\t{NOSE_DOWN}", *groups["nose"], "ANIM_end", ""]
    return "\n".join(out)


def fbw_positions(gltf: Gltf) -> dict:
    """Light node positions with the gear down and the nose wheel centred. The wings stay at rest: the exterior
    glTF's flex clips run -4.17..4.17 s with zero flex at t 0, the rest pose the installed wings are frozen at."""
    names = {n.get("name", ""): i for i, n in enumerate(gltf.nodes)}
    clips = []
    for name, frac in (("c_gear", 1.0), ("c_wheel", 0.5)):
        c = gltf.clip(name)
        clips.append((c, frac * max(v[0][-1] for v in c.values())))
    return {node: world_multi(gltf, names[node], clips)[:3, 3]
            for node in [l[0] for l in LIGHTS] + LOGOS}


def add_to_acf(acf: str) -> str:
    if ACF_FILE in acf:
        return acf
    lines = acf.split("\n")
    at = next(i for i, l in enumerate(lines) if l.startswith("P _obja/count "))
    n = int(lines[at].split()[2])
    keys = [("_obj_flags", "24"), ("_v10_att_body", "-1"), ("_v10_att_file_stl", ACF_FILE), ("_v10_att_gear", "-1"),
            ("_v10_att_phi_ref", "0.000000000"), ("_v10_att_psi_ref", "180.000000000"),
            ("_v10_att_the_ref", "0.000000000"), ("_v10_att_wing", "-1"), ("_v10_att_x_acf_prt_ref", "0.000000000"),
            ("_v10_att_y_acf_prt_ref", "0.000000000"), ("_v10_att_z_acf_prt_ref", "115.108001709"),
            ("_v10_is_internal", "0"), ("_v10_steers_with_gear", "0")]
    lines[at:at + 1] = [f"P _obja/{n}/{k} {v}" for k, v in keys] + [f"P _obja/count {n + 1}"]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    ap.add_argument("--fbw", type=Path, required=True, help="FBW a380_exterior.gltf")
    args = ap.parse_args()
    wings = args.aircraft / "objects/exterior/wings"
    chains = {"L": flex_chain((wings / "a380x_wing1.obj").read_text(encoding="latin-1"), "left"),
              "R": flex_chain((wings / "a380x_wing2.obj").read_text(encoding="latin-1"), "right")}
    out = args.aircraft / OBJ
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build(fbw_positions(Gltf(args.fbw)), chains), encoding="latin-1")
    acf = next(args.aircraft.glob("*.acf"))
    text = acf.read_text(encoding="latin-1")
    new = add_to_acf(text)
    if new != text:
        tmp = acf.with_name(acf.name + ".tmp-lights")
        tmp.write_text(new, encoding="latin-1")
        tmp.replace(acf)
    print(f"{out.name}: {sum(l.lstrip().startswith('LIGHT_PARAM') for l in out.read_text().splitlines())} lights; "
          f"{acf.name}: {'object added' if new != text else 'already attached'}")


if __name__ == "__main__":
    main()
