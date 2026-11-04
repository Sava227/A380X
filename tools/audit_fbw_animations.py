#!/usr/bin/env python3
"""Which FBW flight-deck animation clips move their geometry in the X-Plane objects?

For every clip of A380_COCKPIT_LOD00.gltf: the visible primitives of each animated node's subtree are looked
up in the panel OBJs (by material and world position, legends also at their 2 mm lifted position) and every
triangle found is classified as animated (inside an ANIM block with a keyed transform) or static. Meshes the
generic export wrote in node-local coordinates are recognised by their block's ANIM_trans at the node origin.

    python3 tools/audit_fbw_animations.py --aircraft <aircraft> [--json report.json]
Status per clip: animated | static | partial | not exported (no OBJ geometry for its materials).
"""
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

from cockpit_feedback import (DEFAULT_SOURCE, EYE, LEGEND_LIFT, LIFT_TOLERANCE, PANEL_DIRS, SHELL_TOLERANCE,
                              TOLERANCE, Source, _cross, _dot, _sub, _unit)

KEYED = ("ANIM_rotate_begin", "ANIM_trans_begin")


class ObjIndex:
    """Vertices, indices and the animation state of every TRIS triangle of one OBJ."""

    def __init__(self, path: Path):
        self.name = path.name
        lines = path.read_bytes().decode("latin-1").splitlines()
        self.vertices = [tuple(map(float, l.split()[1:4])) for l in lines if l.startswith("VT")]
        self.indices = [int(v) for l in lines if l.startswith(("IDX\t", "IDX10\t", "IDX ", "IDX10 ")) for v in l.split()[1:]]
        self.tri_state = {}           # first index of a triangle -> (keyed, static_pivots)
        stack = []                     # per open block: [keyed?, pivots]
        for line in lines:
            w = line.split()
            if not w:
                continue
            if w[0] == "ANIM_begin":
                stack.append([False, []])
            elif w[0] == "ANIM_end" and stack:
                stack.pop()
            elif stack and w[0] in KEYED:
                stack[-1][0] = True
            elif stack and w[0] in ("ANIM_rotate", "ANIM_trans") and len(w) >= 9 and w[-1] not in ("none", "0"):
                try:
                    if float(w[-3]) != float(w[-2]):
                        stack[-1][0] = True
                except ValueError:
                    pass
            if stack and w[0] == "ANIM_trans" and len(w) >= 4:
                try:
                    stack[-1][1].append(tuple(map(float, w[1:4])))
                except ValueError:
                    pass
            if w[0] == "TRIS":
                keyed = any(b[0] for b in stack)
                pivots = [p for b in stack for p in b[1]]
                start, count = int(w[1]), int(w[2])
                for k in range(start, start + count, 3):
                    self.tri_state[k] = (keyed, pivots)
        self.grid = defaultdict(list)
        for i, p in enumerate(self.vertices):
            self.grid[tuple(int(math.floor(v / 0.003)) for v in p)].append(i)
        self.by_vertex = defaultdict(list)
        for k in self.tri_state:
            for v in self.indices[k:k + 3]:
                self.by_vertex[v].append(k)

    def near(self, p, radius):
        key = tuple(int(math.floor(v / 0.003)) for v in p)
        reach = int(math.ceil(radius / 0.003))
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                for dz in range(-reach, reach + 1):
                    yield from self.grid.get((key[0] + dx, key[1] + dy, key[2] + dz), ())


def mesh_points(source, child):
    out = []
    for material, triangles, _count in source.primitives(child):
        points = sorted({p for tri in triangles for p in tri})
        normal = (0.0, 0.0, 0.0)
        for tri in triangles:
            normal = tuple(a + b for a, b in zip(normal, _cross(_sub(tri[1], tri[0]), _sub(tri[2], tri[0]))))
        normal = _unit(normal)
        centre = tuple(sum(p[a] for p in points) / len(points) for a in range(3))
        if _dot(normal, _sub(EYE, centre)) < 0:
            normal = tuple(-n for n in normal)
        out.append((material, points, normal))
    return out


def classify(obj: ObjIndex, points, normal, lifted_ok):
    """(animated triangles, static triangles) touching this primitive's vertices."""
    picked = set()
    for p in points:
        lifted = tuple(p[a] + LEGEND_LIFT * normal[a] for a in range(3))
        for i in obj.near(p, LEGEND_LIFT + LIFT_TOLERANCE):
            v = obj.vertices[i]
            if max(abs(a - b) for a, b in zip(v, p)) <= TOLERANCE or (lifted_ok and (
                    max(abs(a - b) for a, b in zip(v, lifted)) <= LIFT_TOLERANCE
                    or abs(math.dist(v, p) - LEGEND_LIFT) <= SHELL_TOLERANCE)):
                picked.add(i)
    tris = {k for i in picked for k in obj.by_vertex[i] if all(v in picked for v in obj.indices[k:k + 3])}
    animated = sum(1 for k in tris if obj.tri_state[k][0])
    return animated, len(tris) - animated


def local_export(obj: ObjIndex, origin):
    """Keyed triangles whose block is placed at this node origin (geometry in node-local coordinates)."""
    keyed = static = 0
    for keyed_flag, pivots in obj.tri_state.values():
        if any(max(abs(a - b) for a, b in zip(origin, q)) <= 0.0002 for q in pivots):
            if keyed_flag:
                keyed += 1
            else:
                static += 1
    return keyed, static


def audit(aircraft: Path, source_path: Path):
    from build_cockpit_controls import transform_point
    source = Source(source_path)
    objs = {}
    for d in PANEL_DIRS:
        for p in sorted((aircraft / d).glob("*.obj")):
            if not p.name.startswith("._"):
                objs[p.name] = p
    cache = {}

    def obj_for(material):
        for name, path in objs.items():
            if name.endswith(material + ".obj"):
                if name not in cache:
                    cache[name] = ObjIndex(path)
                return cache[name]
        return None

    rows = []
    for clip in source.doc["animations"]:
        name = clip["name"].strip()
        targets = sorted({c["target"]["node"] for c in clip["channels"]})
        animated = static = 0
        materials, exported = set(), False
        for index in targets:
            origin = transform_point(source.world(index), (0, 0, 0))
            for child in source.descendants(index):
                for material, points, normal in mesh_points(source, child):
                    materials.add(material)
                    obj = obj_for(material)
                    if obj is None:
                        continue
                    exported = True
                    a, s = classify(obj, points, normal, child != index)
                    if a + s == 0:
                        a, s = local_export(obj, origin)
                    animated += a
                    static += s
        if not exported:
            status = "not exported"
        elif animated and not static:
            status = "animated"
        elif animated:
            status = "partial"
        elif static:
            status = "static"
        else:
            status = "not found"
        rows.append(dict(clip=name, nodes=[source.nodes[i].get("name") for i in targets], status=status,
                         animated_triangles=animated, static_triangles=static, materials=sorted(materials)))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    rows = audit(args.aircraft, args.source)
    counts = defaultdict(int)
    for r in rows:
        counts[r["status"]] += 1
    print(dict(counts))
    for r in rows:
        if r["status"] != "animated":
            print(f'{r["status"]:13s} {r["clip"]:40s} {r["nodes"][:2]} a={r["animated_triangles"]} s={r["static_triangles"]}')
    if args.json:
        args.json.write_text(json.dumps(rows, indent=1) + "\n")


if __name__ == "__main__":
    main()
