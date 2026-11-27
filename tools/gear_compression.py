#!/usr/bin/env python3
"""Landing gear oleo compression from FBW's own animations, driven by X-Plane's strut deflection, plus a
non-spinning sidewall bulge on every loaded tyre.

FBW compresses each leg with its animation (nose: clip c_gear_comp; main gears: the second half of
l/r_w_gear and l/r_b_gear, the MSFS convention): pistons, axles, bogies and tyres slide up by up to
0.475 / 0.50 / 0.525 m and torque links fold about their hinges. The export kept none of it, so on the
ground the tyres floated or sank with the aircraft's weight (user sim checks 2026-11-26).

Every gear object part is matched to its FBW mesh (gear down, steering centred); the parts FBW's compression
moves get that motion keyed on sim/flightmodel2/gear/tire_vertical_deflection_mtr[n], 1:1 (FBW's stroke is
its max compression). Each motion is inserted just before the part's wheel spin, or around the part's draw,
in the part's own frame, so the bogie tilt and steering keep working. With the ACF contact points at FBW's
(flight_model.cfg [CONTACT_POINTS]), each tyre sits on the ground at any weight, flattened by FBW's 6-10 cm
contact offset. Idempotent (MARKER).

    python3 tools/gear_compression.py --aircraft <aircraft> --fbw <a380_exterior.gltf>
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
from obj8_preview import parse_obj8  # noqa: E402
from obj8_numbers import fmt  # noqa: E402
from telescope_reverser_actuators import Gltf, polar_rotation  # noqa: E402

MARKER = "# gear compression: tools/gear_compression.py"
DEFLECTION = "sim/flightmodel2/gear/tire_vertical_deflection_mtr[{}]"
DEPLOY = "sim/flightmodel2/gear/deploy_ratio[{}]"
SPIN = "sim/flightmodel2/gear/tire_rotation_angle_deg"
# (X-Plane gear index, object folder, FBW compression clip, its compression span as clip fractions,
#  other clips posed at a fraction: gear down, steering centred)
GEARS = (
    (0, "gear-nose", "c_gear_comp", (0.0, 1.0), (("c_gear", 1.0), ("c_wheel", 0.5))),
    (1, "gear-wing-lh", "l_w_gear", (0.5, 1.0), ()),
    (2, "gear-wing-rh", "r_w_gear", (0.5, 1.0), ()),
    (3, "gear-body-lh", "l_b_gear", (0.5, 1.0), (("left_bws", 0.5),)),
    (4, "gear-body-rh", "r_b_gear", (0.5, 1.0), (("right_bws", 0.5),)),
)
SKIP = ("_accessories", "_lg_bay")     # nose lights and bay: nothing in them compresses
SAMPLES = 16
BULGES = ((0.03, 0.15, 0.012), (0.15, 0.30, 0.022), (0.30, 9.0, 0.030))   # strut deflection range, m -> bulge, m
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


def fbw_motion(g: Gltf, comp: str, span, base) -> dict:
    """{mesh node: (vertices fully extended, [extended -> pose k, k over the compression span])}."""
    clips = {name: g.clip(name) for name in (comp, *(b for b, _ in base))}
    end = {name: max(v[0][-1] for v in c.values()) for name, c in clips.items()}
    bones = {n for c in clips.values() for n, _ in c}
    fixed = [(clips[b], f * end[b]) for b, f in base]
    out = {}
    for m, node in enumerate(g.nodes):
        if "mesh" not in node or not any(a in bones for a in ancestors(g, m)):
            continue
        poses = [world_multi(g, m, [(clips[comp], f * end[comp]), *fixed])
                 for f in np.linspace(span[0], span[1], SAMPLES)]
        rest = g.positions(m) @ poses[0][:3, :3].T + poses[0][:3, 3]
        out[m] = (rest, [p @ np.linalg.inv(poses[0]) for p in poses])
    return out


def ancestors(g, i):
    while i in g.parent:
        i = g.parent[i]
        yield i


def match(points: np.ndarray, fbw: dict, tolerance=0.03) -> int | None:
    """FBW mesh holding most of these points (within `tolerance`), or None below 30 %."""
    rng = np.random.default_rng(0)
    q = points[rng.choice(len(points), min(300, len(points)), replace=False)]
    best, node = 0.0, None
    for m, (rest, _) in fbw.items():
        if np.linalg.norm(rest.mean(0) - points.mean(0)) > 2.0:
            continue
        near = np.sqrt(((q[:, None] - rest[None, ::max(1, len(rest) // 3000)]) ** 2).sum(-1)).min(1) < tolerance
        if near.mean() > best:
            best, node = near.mean(), m
    return node if best >= 0.3 else None


def hinge(d: np.ndarray):
    """Rigid 4x4 -> (axis, angle deg, pivot on the axis); axis None for a pure slide."""
    r = polar_rotation(d[:3, :3])
    angle = np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1, 1)))
    if angle < 1e-4:
        return None, 0.0, np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    axis /= np.linalg.norm(axis)
    pivot = np.linalg.lstsq(np.eye(3) - r, d[:3, 3] - axis * (axis @ d[:3, 3]), rcond=None)[0]
    return axis, angle, pivot - axis * (axis @ pivot)


def keyed(deform: list[np.ndarray], keys: list[float], dataref: str, tab: str) -> list[str]:
    """OBJ lines applying deform[k] at dataref value keys[k]: slide, then swing about one fixed hinge."""
    axis, _, pivot = hinge(deform[-1])
    out = [f"{tab}ANIM_trans_begin\t{dataref}"]
    for d, v in zip(deform, keys):
        p = d[:3, :3] @ pivot + d[:3, 3] if axis is not None else d[:3, 3]
        out.append(f"{tab}\tANIM_trans_key\t{fmt(v)}\t" + "\t".join(fmt(c) for c in p))
    out.append(f"{tab}ANIM_trans_end")
    if axis is not None:
        out.append(f"{tab}ANIM_rotate_begin\t" + "\t".join(fmt(c) for c in axis) + f"\t{dataref}")
        for d, v in zip(deform, keys):
            a, angle, _ = hinge(d)
            out.append(f"{tab}\tANIM_rotate_key\t{fmt(v)}\t{fmt(angle if a is None or a @ axis >= 0 else -angle)}")
        out.append(f"{tab}ANIM_rotate_end")
        out.append(f"{tab}ANIM_trans\t" + "\t".join(fmt(c) for c in -pivot) + "\t" + "\t".join(fmt(c) for c in -pivot))
    return out


def chain_lines(lines: list[str], tris: int) -> list[int]:
    """Lines (ascending) whose transforms apply to the draw at `tris`: its block and its ancestors' heads."""
    out, depth = [], 0
    for i in range(tris - 1, -1, -1):
        w = lines[i].strip()
        if w == "ANIM_end":
            depth += 1
        elif w == "ANIM_begin":
            if depth:
                depth -= 1
        elif depth == 0:
            out.append(i)
    return out[::-1]


def anchor(lines: list[str], tris: int) -> tuple[int, bool]:
    """(line, is_spin): the draw's wheel-spin rotation if it has one, else its TRIS line."""
    spin = [i for i in chain_lines(lines, tris) if lines[i].split()[:1] == ["ANIM_rotate_begin"] and SPIN in lines[i]]
    return (spin[0], True) if spin else (tris, False)


def prefix_matrices(lines: list[str], at: list[int], values: dict) -> dict[int, np.ndarray]:
    """Transform in force at each line (a probe draw inserted there), at rest."""
    probe = list(lines)
    for i in sorted(at, reverse=True):
        probe.insert(i, "TRIS\t0\t3")
    obj = parse_obj8(probe, values)
    shifted = {i + sum(1 for j in at if j < i): i for i in at}      # probe line index -> original line
    return {shifted[d.source_line - 1]: np.asarray(d.matrix).reshape(4, 4)
            for d in obj.draws if d.source_line - 1 in shifted and d.count == 3 and d.start == 0}


def bulge_band(tyre: np.ndarray, uv: np.ndarray, normals: np.ndarray, amount: float, steps=(25, 8)):
    """Triangles (k,3,3), uv (k,3,2), normals (k,3,3) of both sidewalls' bulge over the bottom +-60 deg.
    tyre: the tyre's vertices (wheel axis along x). The band follows the sidewall profile pushed out by
    amount * cos^2(angle) * sin(radial position), plus 3 mm so it never z-fights."""
    centre = (tyre.min(0) + tyre.max(0)) / 2
    rel = tyre - centre
    rho = np.hypot(rel[:, 1], rel[:, 2])
    radius = rho.max()
    lo, hi = 0.62 * radius, 0.97 * radius
    tris, uvs, nrms = [], [], []
    nearest = lambda p: int(np.argmin(np.linalg.norm(tyre - p, axis=1)))
    for side in (1, -1):
        wall = (rel[:, 0] * side > 0) & (rel[:, 1] < -0.3 * radius)
        prof_r, prof_x = rho[wall], np.abs(rel[wall, 0])
        grid = []
        for a in np.radians(np.linspace(-60, 60, steps[0])):
            row = []
            for r in np.linspace(lo, hi, steps[1]):
                near = np.abs(prof_r - r) < 0.02 * radius
                x = prof_x[near].max() if near.any() else prof_x.max()
                push = 0.003 + amount * np.cos(np.pi * a / np.radians(120)) ** 2 * np.sin(np.pi * (r - lo) / (hi - lo))
                row.append(centre + [side * (x + push), -r * np.cos(a), r * np.sin(a)])
            grid.append(row)
        grid = np.array(grid)
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


def plain_rubber(uv: np.ndarray, albedo: np.ndarray) -> np.ndarray:
    """The tyre UV whose 9x9 texel neighbourhood is the flattest dark rubber (no lettering): the bulge does not
    spin, so it must not show a frozen copy of the sidewall lettering."""
    gray = albedo[..., :3].mean(-1) if albedo.ndim == 3 else albedo
    h, w = gray.shape
    best, pick = None, uv[0]
    for u in uv[:: max(1, len(uv) // 4000)]:
        x, y = int(np.mod(u[0], 1) * w), int((1 - np.mod(u[1], 1)) * h)
        patch = gray[max(0, y - 4):y + 5, max(0, x - 4):x + 5]
        if patch.size and patch.mean() < 80 and (best is None or patch.std() < best):
            best, pick = patch.std(), u
    return pick


def patch_object(text: str, index: int, offset_m: float, fbw: dict, keys: list[float], albedo=None) -> tuple[str, int]:
    """(new text, parts animated) for one gear object."""
    if MARKER in text:
        return text, 0
    lines = text.split("\n")
    values = {DEPLOY.format(index): 1.0}
    obj = parse_obj8(lines, values)
    pos, uv, idx = np.asarray(obj.positions).reshape(-1, 3), np.asarray(obj.uv).reshape(-1, 2), np.asarray(obj.indices)
    vt_lines = [i for i, l in enumerate(lines) if l.startswith("VT")]
    normals = np.array([[float(x) for x in lines[i].split()[4:7]] for i in vt_lines])
    lift = np.eye(4); lift[1, 3] = offset_m                    # object frame -> FBW (aircraft) frame
    parts = []
    for d in obj.draws:
        m = np.asarray(d.matrix).reshape(4, 4)
        ids = idx[d.start:d.start + d.count]
        world = pos[ids] @ m[:3, :3].T + m[:3, 3]
        node = match(np.unique(np.round(world + [0, offset_m, 0], 4), axis=0), fbw)
        if node is not None and max(np.abs(x - np.eye(4)).max() for x in fbw[node][1]) > 1e-3:
            parts.append((d.source_line - 1, node, ids, m))
    if not parts:
        return text, 0
    anchors = {line: anchor(lines, line) for line, *_ in parts}
    prefix = prefix_matrices(lines, sorted({a for a, _ in anchors.values()}), values)
    dataref = DEFLECTION.format(index)
    inserts, done, tyres = {}, set(), {}
    for line, node, ids, m in parts:
        at, spin = anchors[line]
        if at in done:
            continue
        done.add(at)
        p = prefix[at]
        local = [np.linalg.inv(p) @ np.linalg.inv(lift) @ x @ lift @ p for x in fbw[node][1]]
        tab = lines[at][:len(lines[at]) - len(lines[at].lstrip())]
        if spin:
            inserts.setdefault(at, []).extend(keyed(local, keys, dataref, tab))
            world = pos[ids] @ m[:3, :3].T + m[:3, 3]
            if np.hypot(*(world[:, 1:] - (world[:, 1:].min(0) + world[:, 1:].max(0)) / 2).T).max() > 0.5:
                tyres[at] = (ids, m, p)                         # a tyre (not a hub or brake): it gets bulges
        else:
            inserts.setdefault(at, []).extend([f"{tab}ANIM_begin", *keyed(local, keys, dataref, tab + "\t")])
            inserts.setdefault(at + 1, []).append(f"{tab}ANIM_end")
    new_vt, new_idx = [], []
    for at, (ids, m, p) in tyres.items():                       # bulges, after the slide, before the spin
        tyre = pos[ids] @ m[:3, :3].T + m[:3, 3]
        inv = np.linalg.inv(p)
        tab = lines[at][:len(lines[at]) - len(lines[at].lstrip())]
        for lo, hi, amount in BULGES:
            t, u, n = bulge_band(tyre, uv[ids], normals[ids] @ m[:3, :3].T, amount)
            if albedo is not None:
                u[:] = plain_rubber(uv[ids], albedo)
            first = len(pos) + len(new_vt)
            local = t.reshape(-1, 3) @ inv[:3, :3].T + inv[:3, 3]
            new_vt += [(v, nn @ inv[:3, :3].T, uu) for v, nn, uu in zip(local, n.reshape(-1, 3), u.reshape(-1, 2))]
            start = len(idx) + len(new_idx)
            new_idx += list(range(first, first + len(local)))
            inserts[at] += [f"{tab}ANIM_begin", f"{tab}\tANIM_hide\t-99\t{fmt(lo)}\t{dataref}",
                            f"{tab}\tANIM_hide\t{fmt(hi)}\t99\t{dataref}", f"{tab}\tTRIS\t{start}\t{len(local)}",
                            f"{tab}ANIM_end"]
    if new_vt:
        last_vt, last_idx = vt_lines[-1], max(i for i, l in enumerate(lines) if l.startswith("IDX"))
        inserts.setdefault(last_vt + 1, [])[:0] = ["VT\t" + "\t".join(fmt(c) for c in (*v, *(n / (np.linalg.norm(n) or 1)), *u))
                                                  for v, n, u in new_vt]
        inserts.setdefault(last_idx + 1, [])[:0] = (
            ["IDX10\t" + "\t".join(map(str, new_idx[k:k + 10])) for k in range(0, len(new_idx) - len(new_idx) % 10, 10)]
            + [f"IDX\t{k}" for k in new_idx[len(new_idx) - len(new_idx) % 10:]])
        pc = next(i for i, l in enumerate(lines) if l.startswith("POINT_COUNTS"))
        f = lines[pc].split()
        lines[pc] = "\t".join(["POINT_COUNTS", str(int(f[1]) + len(new_vt)), f[2], f[3], str(int(f[4]) + len(new_idx))])
    out = []
    for i, l in enumerate(lines):
        out.extend(inserts.get(i, []))
        out.append(l)
    out.extend(inserts.get(len(lines), []))
    return "\n".join(out).replace("POINT_COUNTS", MARKER + "\nPOINT_COUNTS", 1), len(parts)


def object_offsets(acf: Path) -> dict[str, float]:
    """{object path (as in the ACF): vertical placement offset, m}."""
    files, ys = {}, {}
    for line in acf.read_text(errors="replace").splitlines():
        f = line.split()
        if len(f) >= 3 and f[1].startswith("_obja/") and f[1].count("/") == 2:
            n, key = f[1].split("/")[1], f[1].split("/")[2]
            if key == "_v10_att_file_stl":
                files[n] = f[2]
            elif key == "_v10_att_y_acf_prt_ref":
                ys[n] = float(f[2]) * FT
    return {files[n]: ys.get(n, 0.0) for n in files}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    ap.add_argument("--fbw", type=Path, required=True, help="FBW a380_exterior.gltf")
    args = ap.parse_args()
    offsets = object_offsets(next(args.aircraft.glob("*.acf")))
    gltf = Gltf(args.fbw)
    from PIL import Image
    for index, folder, comp, span, base in GEARS:
        fbw = fbw_motion(gltf, comp, span, base)
        tyre_node = min(fbw, key=lambda m: fbw[m][0][:, 1].min())   # the lowest mesh: a tyre
        keys = [d[1, 3] for d in fbw[tyre_node][1]]                 # its upward travel per sample: 1:1 deflection
        assert all(b >= a - 1e-6 for a, b in zip(keys, keys[1:])), f"{comp}: travel not monotonic"
        for path in sorted((args.aircraft / "objects/exterior" / folder).glob("*.obj")):
            if any(s in path.name for s in SKIP):
                continue
            text = path.read_text(encoding="latin-1")
            tex = next((l.split()[-1] for l in text.splitlines() if l.startswith("TEXTURE ")), None)
            albedo = np.asarray(Image.open((path.parent / tex).resolve()).convert("RGB"), np.float32) \
                if tex and "tyre" in path.name else None
            new, n = patch_object(text, index, offsets.get(f"exterior/{folder}/{path.name}", 0.0), fbw, keys, albedo)
            if new != text:
                tmp = path.with_name(path.name + ".tmp-compression")
                tmp.write_text(new, encoding="latin-1")
                tmp.replace(path)
            print(f"{folder}/{path.name}: {n} parts compress" if new != text else f"{folder}/{path.name}: unchanged")


if __name__ == "__main__":
    main()
