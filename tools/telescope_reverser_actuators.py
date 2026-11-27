#!/usr/bin/env python3
"""Telescoping thrust-reverser link actuators on engines 2 and 3 (a380x_eng_cores_lh/rh.obj).

FBW moves each of the 12 actuators per reverser (Cylinder.* under Bone.012..035, thrust_rev_1/2) with its
bone: it slides ~0.7 m aft with the translating sleeve, swings ~69 deg about one fixed axis and stretches
(scale 0.71, 1.99, 0.71). OBJ8 has no scale, and the export kept only a constant pose, so in X-Plane the
actuators stayed put while the sleeve slid away (user sim check 2026-11-26).

Each actuator becomes two rigid pieces under the same wing-flex chain, both swinging with FBW's bone
rotation: the barrel (the original mesh) pinned to FBW's near end, and the rod (a copy thinned to FBW's
end radius) pinned to the far end. At rest they coincide; fully deployed they span FBW's stretched
length. Keys are FBW's own (81 samples of the clip). Idempotent (MARKER).

    python3 tools/telescope_reverser_actuators.py --aircraft <aircraft> --fbw <a380_exterior.gltf>
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
from obj8_preview import parse_obj8  # noqa: E402
from obj8_numbers import fmt  # noqa: E402

MARKER = "# telescoping reverser actuators: tools/telescope_reverser_actuators.py"
REVERSE = "sim/flightmodel2/engines/thrust_reverser_deploy_ratio"
SIDES = (("a380x_eng_cores_lh.obj", "thrust_rev_1", 1), ("a380x_eng_cores_rh.obj", "thrust_rev_2", 2))
OBJ_DIR = "objects/exterior/wings"


class Gltf:
    """Just enough glTF: node world matrices at a clip time, mesh positions."""

    def __init__(self, path: Path):
        self.doc = json.loads(path.read_text())
        self.base, self.bins = path.parent, {}
        self.nodes = self.doc["nodes"]
        self.parent = {c: i for i, n in enumerate(self.nodes) for c in n.get("children", [])}

    def accessor(self, i):
        a = self.doc["accessors"][i]
        view = self.doc["bufferViews"][a["bufferView"]]
        uri = self.doc["buffers"][view["buffer"]]["uri"]
        if uri not in self.bins:
            self.bins[uri] = (self.base / uri).read_bytes()
        width = {"SCALAR": 1, "VEC3": 3, "VEC4": 4}[a["type"]]
        offset = view.get("byteOffset", 0) + a.get("byteOffset", 0)
        assert a["componentType"] == 5126, "float accessors only"
        stride = view.get("byteStride", 4 * width)    # vertex attributes are interleaved
        return np.ndarray((a["count"], width), np.float32, self.bins[uri], offset, (stride, 4)).astype(float)

    def clip(self, name):
        anim = next(a for a in self.doc["animations"] if a["name"] == name)
        out = {}
        for ch in anim["channels"]:
            s = anim["samplers"][ch["sampler"]]
            out[(ch["target"]["node"], ch["target"]["path"])] = (self.accessor(s["input"]).ravel(), self.accessor(s["output"]))
        return out

    def local(self, i, clip=None, t=0.0):
        n = self.nodes[i]
        trs = {"translation": n.get("translation", [0, 0, 0]), "rotation": n.get("rotation", [0, 0, 0, 1]),
               "scale": n.get("scale", [1, 1, 1])}
        for path in trs:
            if clip and (i, path) in clip:
                times, values = clip[(i, path)]
                k = int(np.clip(np.searchsorted(times, t) - 1, 0, len(times) - 2))
                f = float(np.clip((t - times[k]) / (times[k + 1] - times[k]), 0, 1))
                a, b = values[k], values[k + 1]
                if path == "rotation" and a @ b < 0:
                    b = -b
                v = a * (1 - f) + b * f
                trs[path] = v / np.linalg.norm(v) if path == "rotation" else v
        x, y, z, w = trs["rotation"]
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        m = np.eye(4)
        m[:3, :3] = r * np.asarray(trs["scale"], float)
        m[:3, 3] = trs["translation"]
        return m

    def world(self, i, clip=None, t=0.0):
        m = self.local(i, clip, t)
        while i in self.parent:
            i = self.parent[i]
            m = self.local(i, clip, t) @ m
        return m

    def positions(self, i):
        mesh = self.doc["meshes"][self.nodes[i]["mesh"]]
        return np.concatenate([self.accessor(p["attributes"]["POSITION"]) for p in mesh["primitives"]])


def polar_rotation(a):
    u, _, vt = np.linalg.svd(a)
    if np.linalg.det(u @ vt) < 0:
        u[:, -1] *= -1
    return u @ vt


def telescope(rest, bone_axis, bone_origin, deform, radial):
    """Two rigid pieces that follow an actuator stretched along its bone's Y axis.

    rest: (n,3) actuator vertices at rest. bone_axis/bone_origin: the bone's Y axis and origin at rest.
    deform: list of 4x4 affine maps (rest -> pose) per key, scale included. radial: the rod's thickness
    factor. Returns the rotation axis and, per piece, (anchor, vertices relative to anchor, [(key position,
    angle deg)])."""
    axis_dir = bone_axis / np.linalg.norm(bone_axis)
    along = (rest - bone_origin) @ axis_dir
    rotations = [polar_rotation(d[:3, :3]) for d in deform]
    skew = rotations[-1] - rotations[-1].T
    spin = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    spin = spin / np.linalg.norm(spin) if np.linalg.norm(spin) > 1e-9 else np.array([1.0, 0.0, 0.0])
    angles = [np.degrees(np.arctan2((np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) @ spin) / 2,
                                    (np.trace(r) - 1) / 2)) for r in rotations]
    centre = rest.mean(0)
    offaxis = (rest - centre) - np.outer((rest - centre) @ axis_dir, axis_dir)
    rod = rest - offaxis * (1 - radial)
    pieces = []
    for verts, end in ((rest, along.min()), (rod, along.max())):
        anchor = centre + axis_dir * (end - (centre - bone_origin) @ axis_dir)
        keys = [((d @ np.append(anchor, 1))[:3], a) for d, a in zip(deform, angles)]
        pieces.append((anchor, verts - anchor, keys))
    return spin, pieces


def flex_prefix(block):
    """The block's lines up to and including the wing-flex chain's closing ANIM_trans (object frame)."""
    last = max(i for i, l in enumerate(block) if "a380x/xp12/wing/flex" in l)
    end = next(i for i in range(last, len(block)) if block[i].strip() == "ANIM_rotate_end")
    assert block[end + 1].split()[0] == "ANIM_trans", "flex chain closes with its pivot return"
    return block[1:end + 2]


def patch(text: str, gltf: Gltf, clip_name: str, engine: int):
    """Returns (new text, actuators patched)."""
    if MARKER in text:
        return text, 0
    lines = text.split("\n")
    obj = parse_obj8(lines)
    pos = np.asarray(obj.positions).reshape(-1, 3)
    vt_lines = [i for i, l in enumerate(lines) if l.startswith("VT")]
    normals = np.array([[float(x) for x in lines[i].split()[4:7]] for i in vt_lines])
    uv = np.asarray(obj.uv).reshape(-1, 2)
    draws = {d.source_line - 1: d for d in obj.draws}
    dataref = f"{REVERSE}[{engine}]"

    clip = gltf.clip(clip_name)
    times = clip[next(k for k in clip if k[1] == "scale")][0]
    bones = sorted({n for n, p in clip if p == "scale"})
    candidates = {i: d for i, d in draws.items() if d.count == 276}
    new_vt, new_idx, blocks, drop = [], [], {}, set()
    base = len(pos)
    for bone in bones:
        child = next(c for c in gltf.nodes[bone]["children"] if "mesh" in gltf.nodes[c])
        w0 = gltf.world(child, clip, 0.0)
        fbw = gltf.positions(child) @ w0[:3, :3].T + w0[:3, 3]
        line, draw = min(candidates.items(), key=lambda kv: np.linalg.norm(
            (pos[obj.indices[kv[1].start:kv[1].start + 276]] @ np.asarray(kv[1].matrix).reshape(4, 4)[:3, :3].T).mean(0)
            + np.asarray(kv[1].matrix).reshape(4, 4)[:3, 3] - fbw.mean(0)))
        m = np.asarray(draw.matrix).reshape(4, 4)
        ids = np.asarray(obj.indices[draw.start:draw.start + 276])
        rest = pos[ids] @ m[:3, :3].T + m[:3, 3]
        assert np.linalg.norm(rest.mean(0) - fbw.mean(0)) < 0.1, f"{gltf.nodes[child]['name']}: no OBJ match"
        del candidates[line]
        b0 = gltf.world(bone, clip, 0.0)
        shift = rest.mean(0) - fbw.mean(0)                     # the export's rest pose is ~2-5 cm off FBW's
        lift = np.eye(4); lift[:3, 3] = shift
        deform = [lift @ gltf.world(child, clip, t) @ np.linalg.inv(w0) @ np.linalg.inv(lift) for t in times]
        radial = float(clip[(bone, "scale")][1][-1][0])
        spin, pieces = telescope(rest, b0[:3, 1], b0[:3, 3] + shift, deform, radial)
        nrm = normals[ids] @ m[:3, :3].T
        block_start = max(i for i in range(line) if lines[i].strip() == "ANIM_begin")
        prefix = flex_prefix(lines[block_start:line + 1])
        out = []
        for anchor, verts, keys in pieces:
            first = base + len(new_vt)
            new_vt += [(v, n, t) for v, n, t in zip(verts, nrm, uv[ids])]
            new_idx += [first + k for k in range(len(verts))]
            out += ["ANIM_begin", *prefix, f"\tANIM_trans_begin\t{dataref}"]
            out += [f"\tANIM_trans_key\t{fmt(t / times[-1])}\t" + "\t".join(fmt(c) for c in p) for t, (p, _) in zip(times, keys)]
            out += ["\tANIM_trans_end", f"\tANIM_rotate_begin\t" + "\t".join(fmt(c) for c in spin) + f"\t{dataref}"]
            out += [f"\tANIM_rotate_key\t{fmt(t / times[-1])}\t{fmt(a)}" for t, (_, a) in zip(times, keys)]
            out += ["\tANIM_rotate_end", f"\tTRIS\t{len(obj.indices) + len(new_idx) - len(verts)}\t{len(verts)}", "ANIM_end"]
        blocks[next(i for i in range(line, len(lines)) if lines[i].strip() == "ANIM_end")] = out
        drop.add(line)
    assert not new_vt or len(bones) == 12, "12 actuators per reverser"

    last_vt, last_idx = vt_lines[-1], max(i for i, l in enumerate(lines) if l.startswith(("IDX\t", "IDX10\t", "IDX ", "IDX10 ")))
    result = []
    for i, l in enumerate(lines):
        if i in drop:
            continue
        if l.startswith("POINT_COUNTS"):
            f = l.split()
            l = "\t".join(["POINT_COUNTS", str(int(f[1]) + len(new_vt)), f[2], f[3], str(int(f[4]) + len(new_idx))])
        result.append(l)
        if i == last_vt:
            result += ["VT\t" + "\t".join(fmt(c) for c in (*v, *(n / (np.linalg.norm(n) or 1)), *t)) for v, n, t in new_vt]
        elif i == last_idx:
            result += ["IDX10\t" + "\t".join(map(str, new_idx[k:k + 10])) for k in range(0, len(new_idx) - len(new_idx) % 10, 10)]
            result += [f"IDX\t{k}" for k in new_idx[len(new_idx) - len(new_idx) % 10:]]
        elif i in blocks:
            if MARKER not in result:
                result.append(MARKER)
            result += blocks[i]
    return "\n".join(result), len(drop)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    ap.add_argument("--fbw", type=Path, required=True, help="FBW a380_exterior.gltf")
    args = ap.parse_args()
    gltf = Gltf(args.fbw)
    for name, clip, engine in SIDES:
        path = args.aircraft / OBJ_DIR / name
        text, n = patch(path.read_text(encoding="latin-1"), gltf, clip, engine)
        if n:
            tmp = path.with_name(path.name + ".tmp-telescope")
            tmp.write_text(text, encoding="latin-1")
            tmp.replace(path)
        print(f"{name}: {n} actuators telescoping")


if __name__ == "__main__":
    main()
