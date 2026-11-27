#!/usr/bin/env python3
"""Nose gear oleo compression from FBW's own animation, driven by X-Plane's strut deflection.

FBW compresses the nose leg with clip c_gear_comp: the lower strut and axle slide up 0.48 m and the two
torque links fold about their hinges; the upper strut, braces, actuators and lights stay put. The export
kept none of it, so on the ground the tyres floated or sank with the aircraft's weight (user sim checks
2026-11-26: X-Plane compressed the nose strut 0.48 m).

Each nose part is matched to its FBW mesh (gear down, steering centred); the parts FBW moves get its motion
keyed on sim/flightmodel2/gear/tire_vertical_deflection_mtr[0]: the struts and links innermost, in the
steered frame; the tyres outermost (they spin inside), as a vertical slide. X-Plane's leg reaches GAP
(~0.16 m) below FBW's extended tyre, so compression starts at deflection GAP and from there the tyre bottom
follows the ground exactly. Idempotent (MARKER).

    python3 tools/nose_gear_compression.py --aircraft <aircraft> --fbw <a380_exterior.gltf>
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
from obj8_preview import parse_obj8  # noqa: E402
from obj8_numbers import fmt  # noqa: E402
from ground_gear_diagnostics import acf_values, extended_gear_contact  # noqa: E402
from telescope_reverser_actuators import Gltf, polar_rotation  # noqa: E402

MARKER = "# nose gear compression: tools/nose_gear_compression.py"
DEFLECTION = "sim/flightmodel2/gear/tire_vertical_deflection_mtr[0]"
DIR = "objects/exterior/gear-nose"
STRUT, TYRES = "gear_nose_a380_exterior_nlg.obj", "gear_nose_a380_exterior_tyres.obj"
ACF_INDEX = {STRUT: 30, TYRES: 31}          # _obja index of each object in the ACF
SAMPLES = 16
STROKE = 0.322     # m of X-Plane deflection past GAP at which the squash and bulge are full (the user's 0.48 m case)
SQUASH = 0.045     # m a fully loaded tyre flattens at the contact patch (the axle comes down with it)
BULGES = ((0.05, 0.4, 0.012), (0.4, 0.75, 0.022), (0.75, 99.0, 0.030))   # load fraction range -> sidewall bulge, m
SPIN = "sim/flightmodel2/gear/tire_rotation_angle_deg[0]"
FT = 0.3048


def world_multi(g: Gltf, node: int, clips: list[tuple[dict, float]]) -> np.ndarray:
    """Node world matrix with each clip evaluated at its own time."""
    def local(j):
        for clip, t in clips:
            if any(k[0] == j for k in clip):
                return g.local(j, clip, t)
        return g.local(j)
    m = local(node)
    while node in g.parent:
        node = g.parent[node]
        m = local(node) @ m
    return m


def fbw_motion(g: Gltf):
    """{mesh node: (rest vertices, [rest -> pose at sample k])}, the poses spanning c_gear_comp."""
    gear, steer, comp = g.clip("c_gear"), g.clip("c_wheel"), g.clip("c_gear_comp")
    end = lambda c: max(v[0][-1] for v in c.values())
    bones = {n for c in (gear, steer, comp) for n, _ in c}
    under = lambda i: any(p in bones for p in _ancestors(g, i))
    out = {}
    for m in (i for i, n in enumerate(g.nodes) if "mesh" in n and under(i)):
        poses = [world_multi(g, m, [(gear, end(gear)), (steer, end(steer) / 2), (comp, t)])
                 for t in np.linspace(0, end(comp), SAMPLES)]
        rest = g.positions(m) @ poses[0][:3, :3].T + poses[0][:3, 3]
        out[m] = (rest, [p @ np.linalg.inv(poses[0]) for p in poses])
    return out


def _ancestors(g, i):
    while i in g.parent:
        i = g.parent[i]
        yield i


def match(points: np.ndarray, fbw: dict) -> int | None:
    """FBW mesh holding most of these points (within 2 cm), or None below half."""
    rng = np.random.default_rng(0)
    q = points[rng.choice(len(points), min(300, len(points)), replace=False)]
    best, node = 0.0, None
    for m, (rest, _) in fbw.items():
        if np.linalg.norm(rest.mean(0) - points.mean(0)) > 1.0:
            continue
        near = np.sqrt(((q[:, None] - rest[None, ::max(1, len(rest) // 3000)]) ** 2).sum(-1)).min(1) < 0.02
        if near.mean() > best:
            best, node = near.mean(), m
    return node if best >= 0.5 else None


def hinge(d: np.ndarray):
    """Rigid 4x4 -> (axis, angle deg, pivot on the axis). Pivot is any point the motion only slides."""
    r = polar_rotation(d[:3, :3])
    angle = np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1, 1)))
    if angle < 1e-4:
        return None, 0.0, np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    axis /= np.linalg.norm(axis)
    pivot = np.linalg.lstsq(np.eye(3) - r, d[:3, 3] - axis * (axis @ d[:3, 3]), rcond=None)[0]
    return axis, angle, pivot - axis * (axis @ pivot)


def keyed(deform: list[np.ndarray], keys: list[float], tab: str) -> list[str]:
    """OBJ lines applying deform[k] at dataref value keys[k]: slide, then swing about one fixed hinge."""
    axis, _, pivot = hinge(deform[-1])
    out = [f"{tab}ANIM_trans_begin\t{DEFLECTION}"]
    for d, v in zip(deform, keys):
        p = d[:3, :3] @ pivot + d[:3, 3] if axis is not None else d[:3, 3]
        out.append(f"{tab}\tANIM_trans_key\t{fmt(v)}\t" + "\t".join(fmt(c) for c in p))
    out.append(f"{tab}ANIM_trans_end")
    if axis is not None:
        out.append(f"{tab}ANIM_rotate_begin\t" + "\t".join(fmt(c) for c in axis) + f"\t{DEFLECTION}")
        for d, v in zip(deform, keys):
            a, angle, _ = hinge(d)
            out.append(f"{tab}\tANIM_rotate_key\t{fmt(v)}\t{fmt(angle if a is None or a @ axis >= 0 else -angle)}")
        out.append(f"{tab}ANIM_rotate_end")
        out.append(f"{tab}ANIM_trans\t" + "\t".join(fmt(c) for c in -pivot) + "\t" + "\t".join(fmt(c) for c in -pivot))
    return out


def draws(text: str):
    obj = parse_obj8(text.splitlines(), {"sim/flightmodel2/gear/deploy_ratio[0]": 1.0})
    pos, idx = np.asarray(obj.positions).reshape(-1, 3), np.asarray(obj.indices)
    for d in obj.draws:
        m = np.asarray(d.matrix).reshape(4, 4)
        yield d.source_line - 1, m, pos[idx[d.start:d.start + d.count]] @ m[:3, :3].T + m[:3, 3]


def squashed(slide: float) -> float:
    """Deflection past contact at which the axle has slid `slide`: the tyre flattens SQUASH * load first."""
    full = STROKE - SQUASH
    return slide * STROKE / full if slide <= full else slide + SQUASH


def bulge_band(tyre: np.ndarray, uv: np.ndarray, normals: np.ndarray, amount: float, steps=(25, 8)):
    """Triangles (k,3,3), uv (k,3,2), normals (k,3,3) of both sidewalls' bulge over the bottom +-60 deg.
    tyre: the tyre's vertices (object frame, wheel axis along x). The band follows the sidewall profile,
    pushed out by amount * cos^2(angle) * sin(radial position), plus 3 mm so it never z-fights."""
    centre = (tyre.min(0) + tyre.max(0)) / 2
    rel = tyre - centre
    rho = np.hypot(rel[:, 1], rel[:, 2])
    radius = rho.max()
    lo, hi = 0.62 * radius, 0.97 * radius
    tris, uvs, nrms = [], [], []
    for side in (1, -1):
        wall = (rel[:, 0] * side > 0) & (rel[:, 1] < -0.3 * radius)
        prof_r, prof_x = rho[wall], np.abs(rel[wall, 0])
        def surface(phi, r, push):
            near = np.abs(prof_r - r) < 0.02 * radius
            x = prof_x[near].max() if near.any() else prof_x.max()
            return centre + [side * (x + push), -r * np.cos(phi), r * np.sin(phi)]
        grid = []
        for a in np.radians(np.linspace(-60, 60, steps[0])):
            row = []
            for r in np.linspace(lo, hi, steps[1]):
                push = 0.003 + amount * np.cos(np.pi * a / np.radians(120)) ** 2 * np.sin(np.pi * (r - lo) / (hi - lo))
                row.append(surface(a, r, push))
            grid.append(row)
        grid = np.array(grid)
        nearest = lambda p: int(np.argmin(np.linalg.norm(tyre - p, axis=1)))
        for i in range(steps[0] - 1):
            for j in range(steps[1] - 1):
                quad = (grid[i, j], grid[i + 1, j], grid[i + 1, j + 1], grid[i, j + 1])
                for t in ((quad[0], quad[1], quad[2]), (quad[0], quad[2], quad[3])):
                    t = np.array(t)
                    if np.cross(t[1] - t[0], t[2] - t[0])[0] * side > 0:     # OBJ8: clockwise faces outward
                        t = t[[0, 2, 1]]
                    ids = [nearest(v) for v in t]
                    tris.append(t); uvs.append(uv[ids]); nrms.append(normals[ids])
    return np.array(tris), np.array(uvs), np.array(nrms)


def top_begin(lines: list[str], tris: int) -> int:
    """Line of the top-level ANIM_begin enclosing line `tris`."""
    depth, top = 0, None
    for i in range(tris):
        w = lines[i].strip()
        if w == "ANIM_begin":
            if depth == 0:
                top = i
            depth += 1
        elif w == "ANIM_end":
            depth -= 1
    assert depth > 0 and top is not None, "TRIS outside any ANIM block"
    return top


def plain_rubber(uv: np.ndarray, albedo: np.ndarray) -> np.ndarray:
    """The tyre UV whose 9x9 texel neighbourhood is the flattest dark rubber (no lettering): the bulge band does
    not spin, so it must not show a frozen copy of the sidewall lettering."""
    gray = albedo[..., :3].mean(-1) if albedo.ndim == 3 else albedo
    h, w = gray.shape
    best, pick = None, uv[0]
    for u in uv[:: max(1, len(uv) // 4000)]:
        x, y = int(np.mod(u[0], 1) * w), int((1 - np.mod(u[1], 1)) * h)
        patch = gray[max(0, y - 4):y + 5, max(0, x - 4):x + 5]
        if patch.size and patch.mean() < 80 and (best is None or patch.std() < best):
            best, pick = patch.std(), u
    return pick


def add_bulges(lines: list[str], inserts: dict, gap: float, albedo: np.ndarray | None = None):
    """Before each tyre's spin: one hidden-unless-loaded bulge band per BULGES level (does not spin)."""
    obj = parse_obj8(lines)
    pos, uv, idx = np.asarray(obj.positions).reshape(-1, 3), np.asarray(obj.uv).reshape(-1, 2), np.asarray(obj.indices)
    vt_lines = [i for i, l in enumerate(lines) if l.startswith("VT")]
    normals = np.array([[float(x) for x in lines[i].split()[4:7]] for i in vt_lines])
    by_line = {d.source_line - 1: d for d in obj.draws}
    new_vt, new_idx = [], []
    base_vt, base_idx = len(pos), len(idx)
    for spin in [i for i, l in enumerate(lines) if l.split()[:1] == ["ANIM_rotate_begin"] and SPIN in l]:
        tris_line = next(i for i in range(spin, len(lines)) if lines[i].strip().startswith("TRIS"))
        draw = by_line[tris_line]
        m = np.asarray(draw.matrix).reshape(4, 4)
        ids = idx[draw.start:draw.start + draw.count]
        tyre = pos[ids] @ m[:3, :3].T + m[:3, 3]
        probe = parse_obj8(lines[:spin] + ["TRIS\t0\t3"] + lines[spin:])
        prefix = np.asarray(next(d for d in probe.draws if d.source_line == spin + 1).matrix).reshape(4, 4)
        inv = np.linalg.inv(prefix)
        tab = lines[spin][:len(lines[spin]) - len(lines[spin].lstrip())]
        block = []
        for lo, hi, amount in BULGES:
            t, u, n = bulge_band(tyre, uv[ids], normals[ids] @ m[:3, :3].T, amount)
            if albedo is not None:
                u[:] = plain_rubber(uv[ids], albedo)
            first = base_vt + len(new_vt)
            local = t.reshape(-1, 3) @ inv[:3, :3].T + inv[:3, 3]
            new_vt += [(v, nn @ inv[:3, :3].T, uu) for v, nn, uu in zip(local, n.reshape(-1, 3), u.reshape(-1, 2))]
            start = base_idx + len(new_idx)
            new_idx += list(range(first, first + len(local)))
            block += [f"{tab}ANIM_begin",
                      f"{tab}\tANIM_hide\t-99\t{fmt(gap + STROKE * lo)}\t{DEFLECTION}",
                      f"{tab}\tANIM_hide\t{fmt(gap + STROKE * hi)}\t99\t{DEFLECTION}",
                      f"{tab}\tTRIS\t{start}\t{len(local)}", f"{tab}ANIM_end"]
        inserts.setdefault(spin, [])[:0] = block
    last_vt, last_idx = vt_lines[-1], max(i for i, l in enumerate(lines) if l.startswith("IDX"))
    inserts.setdefault(last_vt + 1, [])[:0] = ["VT\t" + "\t".join(fmt(c) for c in (*v, *(n / (np.linalg.norm(n) or 1)), *u))
                                              for v, n, u in new_vt]
    inserts.setdefault(last_idx + 1, [])[:0] = (
        ["IDX10\t" + "\t".join(map(str, new_idx[k:k + 10])) for k in range(0, len(new_idx) - len(new_idx) % 10, 10)]
        + [f"IDX\t{k}" for k in new_idx[len(new_idx) - len(new_idx) % 10:]])
    pc = next(i for i, l in enumerate(lines) if l.startswith("POINT_COUNTS"))
    f = lines[pc].split()
    lines[pc] = "\t".join(["POINT_COUNTS", str(int(f[1]) + len(new_vt)), f[2], f[3], str(int(f[4]) + len(new_idx))])


def patch(texts: dict[str, str], gltf: Gltf, acf: dict, offsets_ft: dict[str, float],
          albedo: np.ndarray | None = None) -> dict[str, str]:
    if any(MARKER in t for t in texts.values()):
        return texts
    fbw = fbw_motion(gltf)
    lift = {name: (offsets_ft[name] - offsets_ft[STRUT]) * FT for name in texts}   # tyres relative to the strut frame
    found = {name: [(line, m, n) for line, m, pts in draws(texts[name])
                    if (n := match(np.unique(np.round(pts + [0, lift[name], 0], 4), axis=0), fbw)) is not None]
             for name in texts}
    moving = lambda n: max(np.abs(d - np.eye(4)).max() for d in fbw[n][1]) > 1e-3
    axle = next(n for _, _, n in found[TYRES])
    travel = [d[1, 3] for d in fbw[axle][1]]                     # tyre slides straight up
    tyre_bottom = min(pts[:, 1].min() for _, _, pts in draws(texts[TYRES])) + offsets_ft[TYRES] * FT
    gap = tyre_bottom - extended_gear_contact(acf, 0)[1] * FT
    keys = [0.0] + [gap + squashed(s) for s in travel]
    out = {}
    for name, text in texts.items():
        lines = text.split("\n")
        inserts = {}
        for line, m, node in found[name]:
            if not moving(node):
                continue
            deform = [np.eye(4)] + fbw[node][1]
            if name == TYRES:                                    # outermost: the spin stays inside
                at = top_begin(lines, line) + 1                  # both tyres can share one block: slide it once
                if at not in inserts:
                    inserts[at] = keyed(deform, keys, "\t")
            else:                                                # innermost, in the steered frame
                inv = np.linalg.inv(m)
                local = [inv @ d @ m for d in deform]
                tab = lines[line][:len(lines[line]) - len(lines[line].lstrip())]
                inserts.setdefault(line, []).extend([f"{tab}ANIM_begin", *keyed(local, keys, tab + "\t")])
                inserts.setdefault(line + 1, []).append(f"{tab}ANIM_end")
        if name == TYRES:
            add_bulges(lines, inserts, gap, albedo)
        result = []
        for i, l in enumerate(lines):
            result.extend(inserts.get(i, []))
            result.append(l)
        result.extend(inserts.get(len(lines), []))
        out[name] = "\n".join(result).replace("POINT_COUNTS", MARKER + "\nPOINT_COUNTS", 1)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    ap.add_argument("--fbw", type=Path, required=True, help="FBW a380_exterior.gltf")
    args = ap.parse_args()
    acf_path = next(args.aircraft.glob("*.acf"))
    acf = acf_values(acf_path)
    offsets = {}
    for line in acf_path.read_text(errors="replace").splitlines():
        f = line.split()
        for name, i in ACF_INDEX.items():
            if f[:2] == ["P", f"_obja/{i}/_v10_att_y_acf_prt_ref"]:
                offsets[name] = float(f[2])
    texts = {name: (args.aircraft / DIR / name).read_text(encoding="latin-1") for name in (STRUT, TYRES)}
    tex = next(l.split()[-1] for l in texts[TYRES].splitlines() if l.startswith("TEXTURE "))
    from PIL import Image
    albedo = np.asarray(Image.open((args.aircraft / DIR / tex).resolve()).convert("RGB"), np.float32)
    new = patch(texts, Gltf(args.fbw), acf, offsets, albedo)
    for name, text in new.items():
        if text != texts[name]:
            path = args.aircraft / DIR / name
            tmp = path.with_name(path.name + ".tmp-compression")
            tmp.write_text(text, encoding="latin-1")
            tmp.replace(path)
        print(f"{name}: {'compression animated' if text != texts[name] else 'already done'}")


if __name__ == "__main__":
    main()
