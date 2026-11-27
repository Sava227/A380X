#!/usr/bin/env python3
"""Dark sleeves behind the passenger window openings, so no sky shows between the fuselage skin and the cabin.

The fuselage skin (a380x_fuse1..3) is single-sided with plain holes at the windows, and the cabin sidewall's
window reveal does not meet every hole: at a steep angle a sliver of sky (the culled inside of the skin)
shows at the rim of ~65 of the 219 windows (user sim check 2026-11-26). Each window hole (a closed boundary
loop of window size) gets a short tube extruded DEPTH inward from its rim, facing the window axis and
textured with the near-black window surround of the premium-economy sidewall, whose OBJ carries the
geometry (cabin geometry, same frame, no livery dependence). Idempotent (MARKER).

    python3 tools/window_backing.py --aircraft <aircraft>
"""
from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
from obj8_preview import parse_obj8  # noqa: E402
from obj8_numbers import fmt  # noqa: E402

MARKER = "# window backing sleeves: tools/window_backing.py"
SKINS = [f"objects/exterior/fuselage/a380x_fuse{i}.obj" for i in (1, 2, 3)]
HOST = "objects/interior/cabin/cabin_a380x_main_deck_pec_sidewalls.obj"
DARK_UV = (0.9604, 0.8492)      # image-025: the black window surround (5x5 neighbourhood max 18/255)
DEPTH = 0.06                    # metres; deeper starts hiding the inner pane at oblique angles


def world_triangles(text: str) -> np.ndarray:
    obj = parse_obj8(text.splitlines())
    pos, idx = np.asarray(obj.positions).reshape(-1, 3), np.asarray(obj.indices)
    out = []
    for d in obj.draws:
        m = np.asarray(d.matrix).reshape(4, 4)
        out.append(pos[idx[d.start:d.start + d.count]] @ m[:3, :3].T + m[:3, 3])
    return np.concatenate(out).reshape(-1, 3, 3)


def window_loops(tris: np.ndarray) -> list[np.ndarray]:
    """Ordered rims of the window-sized holes: closed loops of edges used by one triangle only."""
    points, inverse = np.unique(np.round(tris.reshape(-1, 3), 3), axis=0, return_inverse=True)
    count = defaultdict(int)
    for a, b, c in inverse.reshape(-1, 3):
        for e in ((a, b), (b, c), (c, a)):
            count[min(e), max(e)] += 1
    nbr = defaultdict(list)
    for (a, b), n in count.items():
        if n == 1:
            nbr[a].append(b)
            nbr[b].append(a)
    loops, seen = [], set()
    for start in nbr:
        if start in seen or len(nbr[start]) != 2:
            continue
        order, prev = [start], None
        while len(order) < 64:
            here = order[-1]
            if len(nbr[here]) != 2:
                break
            nxt = nbr[here][0] if nbr[here][0] != prev else nbr[here][1]
            if nxt == start:
                break
            prev = here
            order.append(nxt)
        seen.update(order)
        rim = points[order]
        size = np.ptp(rim, 0)
        if 20 <= len(order) < 64 and nxt == start and 0.2 < size[1] < 0.45 and 0.15 < size[2] < 0.4:
            loops.append(rim)
    return loops


def sleeve(rim: np.ndarray, depth: float) -> tuple[np.ndarray, np.ndarray]:
    """(triangles (k,3,3), vertex normals (k,3,3)): a tube from the rim `depth` inward, seen from inside it.
    OBJ8 draws clockwise faces, so the counter-clockwise normal points away from the tube axis."""
    centre = rim.mean(0)
    axis = np.linalg.svd(rim - centre)[2][2]
    axis = axis if axis[0] * centre[0] > 0 else -axis           # outward: windows sit at |x| > 2.8 m
    tris, normals = [], []
    for a, b in zip(rim, np.roll(rim, -1, 0)):
        for t in ((a, b, b - axis * depth), (a, b - axis * depth, a - axis * depth)):
            t = np.array(t)
            mid = t.mean(0) - centre
            radial = mid - axis * (mid @ axis)
            if np.cross(t[1] - t[0], t[2] - t[0]) @ radial < 0:
                t = t[[0, 2, 1]]
            tris.append(t)
            inward = -radial / (np.linalg.norm(radial) or 1)
            normals.append([inward] * 3)
    return np.array(tris), np.array(normals)


def add_sleeves(host: str, tris: np.ndarray, normals: np.ndarray) -> str:
    lines = host.split("\n")
    n_vt = sum(1 for l in lines if l.startswith("VT"))
    n_idx = next(int(l.split()[4]) for l in lines if l.startswith("POINT_COUNTS"))
    last_vt = max(i for i, l in enumerate(lines) if l.startswith("VT"))
    last_idx = max(i for i, l in enumerate(lines) if l.startswith("IDX"))
    vts = ["VT\t" + "\t".join(fmt(x) for x in (*p, *n, *DARK_UV)) for p, n in zip(tris.reshape(-1, 3), normals.reshape(-1, 3))]
    ids = list(range(n_vt, n_vt + len(vts)))
    idx = ["IDX10\t" + "\t".join(map(str, ids[k:k + 10])) for k in range(0, len(ids) - len(ids) % 10, 10)]
    idx += [f"IDX\t{k}" for k in ids[len(ids) - len(ids) % 10:]]
    out = []
    for i, l in enumerate(lines):
        if l.startswith("POINT_COUNTS"):
            f = l.split()
            l = "\t".join(["POINT_COUNTS", str(n_vt + len(vts)), f[2], f[3], str(n_idx + len(ids))])
        out.append(l)
        if i == last_vt:
            out += vts
        elif i == last_idx:
            out += idx
    while out and not out[-1].strip():
        out.pop()
    return "\n".join(out + [MARKER, "ATTR_cull", "ATTR_draw_enable", f"TRIS\t{n_idx}\t{len(ids)}", ""])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    args = ap.parse_args()
    host = args.aircraft / HOST
    text = host.read_text(encoding="latin-1")
    if MARKER in text:
        print(f"{host.name}: sleeves already present")
        return
    parts = []
    for skin in SKINS:
        rims = window_loops(world_triangles((args.aircraft / skin).read_text(encoding="latin-1")))
        print(f"{Path(skin).name}: {len(rims)} window openings")
        parts += [sleeve(r, DEPTH) for r in rims]
    tris, normals = np.concatenate([p[0] for p in parts]), np.concatenate([p[1] for p in parts])
    tmp = host.with_name(host.name + ".tmp-backing")
    tmp.write_text(add_sleeves(text, tris, normals), encoding="latin-1")
    tmp.replace(host)
    print(f"{host.name}: {len(parts)} sleeves, {len(tris)} triangles")


if __name__ == "__main__":
    main()
