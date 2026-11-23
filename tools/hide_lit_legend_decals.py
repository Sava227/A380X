#!/usr/bin/env python3
"""Hide each FBW legend decal face while its lit lamp quad is on (in-sim finding 2026-11-04).

The lit legend quads live in a380x_cockpit_controls.obj (_obja 119). The FBW legend decal objects
(decal_*.obj, _obja 120-125) are drawn after it and the quads use ATTR_poly_os, so X-Plane painted the
unlit decal over a lit quad: with a lamp dataref at 1.0 (Log.txt: PACK 1 OFF 1.00, APU MASTER ON 1.00)
nothing glowed on the overhead. The FCU lamps that work sit on objects drawn before 119.

Each decal face that carries a lamp quad is now wrapped in ANIM_hide on that lamp's dataref above its
dark threshold (the quad's own ANIM_hide upper bound), so the lit quad - legend art plus glow - replaces
the decal while lit, and the decal shows as before while dark. Lamps (comment -> face):
  # OVHD LAMP lamp/<id>          tools/ovhd_lamps_contract.py lamps()
  # ELEC LAMP lamp/<id>          tools/elec_panel_contract.py lamps()
  # ADIRS ANNUNCIATOR <face>, # FCU ANNUNCIATOR <face>, # A380X FEEDBACK V1 LAMP <control> <face>

Faces match at their FBW position or lifted LEGEND_LIFT (the legend export), at any ANIM depth (the hide
nests inside an existing motion block). Only those lines change; vertex / index / texture payloads are
untouched; re-running is a no-op.

    python3 tools/hide_lit_legend_decals.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" [--dry-run]
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

from cockpit_feedback import (DEFAULT_SOURCE, EYE, LEGEND_LIFT, LIFT_TOLERANCE, TOLERANCE, Source, _cross, _dot, _Grid,
                              _sub, _unit)
from patch_efis_animations import vertex_payload

CONTROLS_OBJ = "objects/interior/cockpit-controls/a380x_cockpit_controls.obj"
DECAL_DIR = "objects/interior/cockpit-panels"
MARKER = "# A380X LIT DECAL HIDE"


def lamp_faces(controls_text: str) -> dict[str, tuple[str, float]]:
    """face node -> (lamp dataref, dark threshold) for every lit quad of the controls object."""
    from elec_panel_contract import lamps as elec_lamps
    from ovhd_lamps_contract import lamps as ovhd_lamps
    by_ref = {row[0]: row[4] for row in (*ovhd_lamps(), *elec_lamps())}
    lines = controls_text.splitlines()
    out = {}
    for i, line in enumerate(lines):
        face = None
        if line.startswith(("# OVHD LAMP lamp/", "# ELEC LAMP lamp/")):
            face = "?"
        elif line.startswith(("# ADIRS ANNUNCIATOR ", "# FCU ANNUNCIATOR ")):
            face = line.split()[-1]
        elif line.startswith("# A380X FEEDBACK V1 LAMP "):
            face = line.split()[-1]
        if face is None:
            continue
        for follow in lines[i + 1:i + 12]:
            w = follow.split()
            if w[:1] == ["TRIS"]:
                break
            if w[:1] == ["ANIM_hide"] and len(w) == 4 and "avionics_power" not in w[3]:
                ref, threshold = w[3], float(w[2])
                if face == "?":
                    face = by_ref.get(ref)
                if face:
                    out.setdefault(face, (ref, threshold))
                break
    return out


def face_points(source: Source, face: str):
    """(material, points, lifted points) of a legend face."""
    out = []
    for material, triangles, _count in source.primitives(source.names[face]):
        points = sorted({p for tri in triangles for p in tri})
        normal = _unit(tuple(sum(c) for c in zip(*[_cross(_sub(t[1], t[0]), _sub(t[2], t[0])) for t in triangles])))
        centre = tuple(sum(p[a] for p in points) / len(points) for a in range(3))
        if _dot(normal, _sub(EYE, centre)) < 0:
            normal = tuple(-n for n in normal)
        lifted = [tuple(p[a] + LEGEND_LIFT * normal[a] for a in range(3)) for p in points]
        out.append((material, points, lifted))
    return out


def _mul(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def _translate(v):
    return [[1, 0, 0, v[0]], [0, 1, 0, v[1]], [0, 0, 1, v[2]], [0, 0, 0, 1]]


def _rotate(axis, degrees):
    x, y, z = _unit(axis)
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    k = 1 - c
    return [[c + x * x * k, x * y * k - z * s, x * z * k + y * s, 0], [y * x * k + z * s, c + y * y * k, y * z * k - x * s, 0],
            [z * x * k - y * s, z * y * k + x * s, c + z * z * k, 0], [0, 0, 0, 1]]


IDENTITY = _translate((0.0, 0.0, 0.0))


def tris_matrices(lines) -> dict[int, list]:
    """Line index of every TRIS -> its rest transform (every dataref 0; key tables at their value-0 pose),
    for blocks exported in node-local coordinates."""
    out, stack, m, i = {}, [], IDENTITY, 0
    while i < len(lines):
        w = lines[i].split()
        op = w[0] if w else ""
        if op == "ANIM_begin":
            stack.append(m)
        elif op == "ANIM_end" and stack:
            m = stack.pop()
        elif op == "ANIM_trans" and len(w) >= 7:
            a, b = [float(v) for v in w[1:4]], [float(v) for v in w[4:7]]
            f = 0.0
            if len(w) >= 10 and float(w[7]) != float(w[8]):
                f = min(1.0, max(0.0, -float(w[7]) / (float(w[8]) - float(w[7]))))
            m = _mul(m, _translate([a[k] + f * (b[k] - a[k]) for k in range(3)]))
        elif op == "ANIM_rotate" and len(w) >= 6:
            r1, r2, r = float(w[4]), float(w[5]), float(w[4])
            if len(w) >= 9 and float(w[6]) != float(w[7]):
                r = r1 + min(1.0, max(0.0, -float(w[6]) / (float(w[7]) - float(w[6])))) * (r2 - r1)
            m = _mul(m, _rotate([float(v) for v in w[1:4]], r))
        elif op in ("ANIM_trans_begin", "ANIM_rotate_begin"):
            axis = [float(v) for v in w[1:4]] if op == "ANIM_rotate_begin" else None
            keys, i = [], i + 1
            while lines[i].split()[:1] in (["ANIM_trans_key"], ["ANIM_rotate_key"], ["ANIM_keyframe_loop"]):
                k = lines[i].split()
                if k[0] != "ANIM_keyframe_loop":
                    keys.append((float(k[1]), [float(v) for v in k[2:]]))
                i += 1
            keys.sort(key=lambda kv: kv[0])
            pose = keys[0][1]
            for (a, pa), (b, pb) in zip(keys, keys[1:]):
                if a <= 0.0 <= b and b > a:
                    pose = [pa[k] + (0.0 - a) / (b - a) * (pb[k] - pa[k]) for k in range(len(pa))]
                    break
            else:
                if keys[-1][0] < 0.0:
                    pose = keys[-1][1]
            m = _mul(m, _translate(pose[:3]) if axis is None else _rotate(axis, pose[0]))
            continue
        elif op == "TRIS":
            out[i] = m
        i += 1
    return out


def _apply(m, p):
    return tuple(m[r][0] * p[0] + m[r][1] * p[1] + m[r][2] * p[2] + m[r][3] for r in range(3))


def patch_decal(text: str, targets: list[tuple[str, list, None, list, list]]):
    """targets: (face, [(dataref, threshold)], None, points, lifted); a face hides while any of its lamps
    is lit. Returns (text, {face: triangles wrapped})."""
    targets = [t for t in targets if f"{MARKER} {t[0]}\n" not in text]
    if not targets:
        return text, {}
    lines = text.splitlines(keepends=True)
    vertices = [tuple(map(float, l.split()[1:4])) for l in lines if l.startswith("VT")]
    indices = [int(v) for l in lines if l.startswith(("IDX\t", "IDX10\t", "IDX ", "IDX10 ")) for v in l.split()[1:]]
    matrices = tris_matrices(lines)
    grids = [(_Grid(points), _Grid(lifted)) for _f, _r, _t, points, lifted in targets]

    def owners_of(p, tol):
        return frozenset(k for k, (g0, g1) in enumerate(grids)
                         if any(max(abs(a - b) for a, b in zip(p, q)) <= tol for q in g0.near(p, tol))
                         or any(max(abs(a - b) for a, b in zip(p, q)) <= LIFT_TOLERANCE + tol
                                for q in g1.near(p, LIFT_TOLERANCE + tol)))
    cache = {}

    def vertex_owners(index, m):
        key = (index, id(m))
        if key not in cache:
            if m is IDENTITY:
                cache[key] = owners_of(vertices[index], TOLERANCE)
            else:      # local-frame export: rest pose, with a looser match for the transform rounding
                cache[key] = owners_of(_apply(m, vertices[index]), 0.00005)
        return cache[key]
    counts, output = {}, []
    for line_index, line in enumerate(lines):
        w = line.split()
        if w[:1] != ["TRIS"]:
            output.append(line)
            continue
        back = line_index - 1
        while back > 0 and lines[back].split()[:1] == ["ANIM_hide"]:
            back -= 1
        if lines[back].split()[:1] == ["ANIM_begin"] and lines[back - 1].lstrip().startswith(MARKER):
            output.append(line)          # already inside one of our hide blocks
            continue
        start, count = int(w[1]), int(w[2])
        m = matrices.get(line_index, IDENTITY)
        if m == IDENTITY:
            m = IDENTITY
        runs = []
        for pos in range(start, start + count, 3):
            a, b, c = (vertex_owners(v, m) for v in indices[pos:pos + 3])
            common = a & b & c
            match = min(common) if common else -1
            if runs and runs[-1][0] == match:
                runs[-1][2] += 3
            else:
                runs.append([match, pos, 3])
        if len(runs) == 1 and runs[0][0] < 0:
            output.append(line)
            continue
        indent = line[:len(line) - len(line.lstrip())]
        for match, run_start, run_count in runs:
            if match < 0:
                output.append(f"{indent}TRIS\t{run_start}\t{run_count}\n")
                continue
            face, conditions = targets[match][0], targets[match][1]
            counts[face] = counts.get(face, 0) + run_count // 3
            output += [f"{indent}{MARKER} {face}\n", f"{indent}ANIM_begin\n",
                       *(f"{indent}ANIM_hide\t{threshold + 0.0001:g}\t1000000\t{ref}\n" for ref, threshold in conditions),
                       f"{indent}TRIS\t{run_start}\t{run_count}\n", f"{indent}ANIM_end\n"]
    patched = "".join(output)
    if vertex_payload(text) != vertex_payload(patched):
        raise ValueError("decal payload changed")
    return patched, counts


def _plan(aircraft: Path, source: Source, conditions: dict):
    """{decal path: (original text, patched text, {face: triangles})} for these face conditions."""
    by_material = {}
    for face, conds in conditions.items():
        for material, points, lifted in face_points(source, face):
            by_material.setdefault(material, []).append((face, conds, None, points, lifted))
    out = {}
    for path in sorted((aircraft / DECAL_DIR).glob("decal_*.obj")):
        targets = [t for m, ts in by_material.items() if path.name.endswith(m + ".obj") for t in ts]
        if not targets:
            continue
        text = path.read_bytes().decode("latin-1")
        done = {t[0]: 0 for t in targets if f"{MARKER} {t[0]}\n" in text}
        patched, counts = patch_decal(text, targets)
        out[path] = (text, patched, {**done, **counts})
    return out


def run(aircraft: Path, source_path: Path, dry_run: bool):
    source = Source(source_path)
    controls = (aircraft / CONTROLS_OBJ).read_bytes().decode("latin-1")
    faces = {f: v for f, v in lamp_faces(controls).items() if f in source.names}
    # pass 1: which lamp faces exist in the decal objects
    first = _plan(aircraft, source, {face: [cond] for face, cond in faces.items()})
    present = {face for _t, _p, counts in first.values() for face in counts}
    # a face missing from the decal objects (FBW: ENG 2 BLEED's upper face covers the whole cap) hides
    # its sibling face instead
    conditions = {face: [faces[face]] for face in present}
    for face in faces:
        if face not in present and face[-5:-1] == "_SEQ":
            sibling = face[:-1] + ("1" if face.endswith("2") else "2")
            if sibling in conditions:
                conditions[sibling].append(faces[face])
    plan = _plan(aircraft, source, conditions) if conditions != {f: [faces[f]] for f in present} else first
    report = {}
    for path, (text, patched, counts) in plan.items():
        new = {f: n for f, n in counts.items() if n}
        if new:
            report[path.name] = new
            if not dry_run:
                tmp = path.with_name(path.name + ".tmp-litdecal")
                tmp.write_bytes(patched.encode("latin-1"))
                tmp.replace(path)
    return report, sorted(set(faces) - present), len(faces)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    report, missing, total = run(args.aircraft, args.source, args.dry_run)
    faces = sum(len(c) for c in report.values())
    print(f"{total} lamp faces; {faces} decal faces {'would be' if args.dry_run else ''} wrapped in "
          f"{len(report)} decal objects")
    for name, counts in report.items():
        print(f"  {name}: {len(counts)} faces, {sum(counts.values())} triangles")
    if missing:
        print(f"  not found in any decal object ({len(missing)}): {' '.join(missing[:20])}")


if __name__ == "__main__":
    main()
