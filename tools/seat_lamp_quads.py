#!/usr/bin/env python3
"""Seat the lit-legend quads on their legends, so the lights do not hover in front of the buttons.

The overhead / ELEC / ADIRS lamp quads (tools/ovhd_cockpit_lamps.py, elec_cockpit_panel.py) were placed
0.4 mm in front of the FBW legend faces as authored. But the exported caps and legend decals are animated
in the button's local frame, and their key-0 pose carries a rest offset, so in the sim the legend sits
about 1.2 mm deeper than the authored face: the lit quads hovered about 1.6 mm in front of the legend
(1.2 mm in front of the cap). The legend decal hides while its lamp is lit (hide_lit_legend_decals.py),
so the right place for a quad is just in front of where the legend really is.

For every quad whose legend face carries the '# A380X LIT DECAL HIDE <face>' marker, both are evaluated
with their animations at rest (every dataref 0); a quad further than TOLERANCE from TARGET in front of
the legend is moved along the legend normal to TARGET. Only VT positions of the quad's own vertices change
(checked: not shared with other triangles); quads under a rotation are reported, not moved. Re-running
is a no-op.

    python3 tools/seat_lamp_quads.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" [--dry-run]
Needs numpy. Run after hide_lit_legend_decals.py.
"""
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import numpy as np

from cockpit_feedback import EYE
from hide_lit_legend_decals import lamp_faces
from obj8_numbers import fmt

CONTROLS_OBJ = "objects/interior/cockpit-controls/a380x_cockpit_controls.obj"
DECAL_DIR = "objects/interior/cockpit-panels"
TARGET = 0.00006          # m in front of the legend (as cockpit_feedback.py LAMP_LIFT)
TOLERANCE = 0.00005


def _rot(axis, deg):
    x, y, z = np.array(axis, float) / (np.linalg.norm(axis) or 1)
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg)); k = 1 - c
    m = np.eye(4)
    m[:3, :3] = [[c + x * x * k, x * y * k - z * s, x * z * k + y * s], [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
                 [z * x * k - y * s, z * y * k + x * s, c + z * z * k]]
    return m


def _tr(v):
    m = np.eye(4); m[:3, 3] = v
    return m


class Obj:
    """TRIS lines with the transform at rest (every dataref 0) and whether a rotation is involved."""

    def __init__(self, path: Path):
        self.path = path
        self.lines = path.read_bytes().decode("latin-1").splitlines(keepends=True)
        self.vt = [i for i, l in enumerate(self.lines) if l.startswith("VT")]
        self.V = np.array([[float(x) for x in self.lines[i].split()[1:4]] for i in self.vt])
        self.I = np.array([int(v) for l in self.lines if l.startswith(("IDX\t", "IDX10\t", "IDX ", "IDX10 "))
                           for v in l.split()[1:]], dtype=np.int64)
        self.tris = {}
        stack, mats, rotated, m, r, i = [], [], [], np.eye(4), False, 0
        while i < len(self.lines):
            w = self.lines[i].split()
            op = w[0] if w else ""
            if op == "ANIM_begin":
                stack.append((m, r))
            elif op == "ANIM_end" and stack:
                m, r = stack.pop()
            elif op == "ANIM_trans" and len(w) >= 7:
                a, b = np.array(w[1:4], float), np.array(w[4:7], float)
                f = 0.0
                if len(w) >= 10 and float(w[7]) != float(w[8]):
                    f = min(1.0, max(0.0, -float(w[7]) / (float(w[8]) - float(w[7]))))
                m = m @ _tr(a + f * (b - a))
            elif op == "ANIM_rotate" and len(w) >= 6:
                ang = float(w[4])
                if len(w) >= 9 and float(w[6]) != float(w[7]):
                    ang += min(1.0, max(0.0, -float(w[6]) / (float(w[7]) - float(w[6])))) * (float(w[5]) - float(w[4]))
                m, r = m @ _rot([float(x) for x in w[1:4]], ang), True
            elif op in ("ANIM_trans_begin", "ANIM_rotate_begin"):
                axis = [float(x) for x in w[1:4]] if op == "ANIM_rotate_begin" else None
                keys, i = [], i + 1
                while self.lines[i].split()[:1] in (["ANIM_trans_key"], ["ANIM_rotate_key"], ["ANIM_keyframe_loop"]):
                    k = self.lines[i].split()
                    if k[0] != "ANIM_keyframe_loop":
                        keys.append((float(k[1]), [float(x) for x in k[2:]]))
                    i += 1
                keys.sort(key=lambda kv: kv[0])
                pose = keys[0][1] if keys[0][0] >= 0 else keys[-1][1]
                for (a, pa), (b, pb) in zip(keys, keys[1:]):
                    if a <= 0.0 <= b and b > a:
                        pose = [pa[k] + (0.0 - a) / (b - a) * (pb[k] - pa[k]) for k in range(len(pa))]
                        break
                if axis is None:
                    m = m @ _tr(pose[:3])
                else:
                    m, r = m @ _rot(axis, pose[0]), True
                continue
            elif op == "TRIS":
                self.tris[i] = (int(w[1]), int(w[2]), m.copy(), r)
            i += 1

    def world(self, line):
        start, count, m, _r = self.tris[line]
        return (np.c_[self.V[self.I[start:start + count]], np.ones(count)] @ m.T)[:, :3]


def first_tris(lines, at, span):
    return next((j for j in range(at, min(len(lines), at + span)) if lines[j].lstrip().startswith("TRIS")), None)


def plan(aircraft: Path):
    ctl = Obj(aircraft / CONTROLS_OBJ)
    faces = lamp_faces("".join(ctl.lines))
    by_ref = {ref: face for face, (ref, _t) in faces.items()}
    quads = {}
    for i, line in enumerate(ctl.lines):
        face = None
        if line.startswith(("# ADIRS ANNUNCIATOR ", "# FCU ANNUNCIATOR ")):     # cockpit_feedback.py seats its own
            face = line.split()[-1]
        elif line.startswith(("# OVHD LAMP lamp/", "# ELEC LAMP lamp/")):
            tail = "/lamp/" + line.split("lamp/", 1)[1].strip()
            face = next((f for r, f in by_ref.items() if r.endswith(tail)), None)
        j = first_tris(ctl.lines, i, 30) if face else None
        if j is not None:
            quads.setdefault(face, j)
    legends = {}
    for path in sorted((aircraft / DECAL_DIR).glob("decal_*.obj")):
        if path.name.startswith("._"):
            continue
        obj = Obj(path)
        for i, line in enumerate(obj.lines):
            m = re.match(r"\s*# A380X LIT DECAL HIDE (\S+)", line)
            if m and (j := first_tris(obj.lines, i, 12)) is not None:
                legends.setdefault(m.group(1), []).append((obj, j))
    moves, skipped = {}, []
    for face, ql in sorted(quads.items()):
        if face not in legends:
            continue
        q = ctl.world(ql)
        d = np.concatenate([obj.world(j) for obj, j in legends[face]])
        normal = np.sum([np.cross(d[k + 1] - d[k], d[k + 2] - d[k]) for k in range(0, len(d) - 2, 3)], axis=0)
        normal /= np.linalg.norm(normal) or 1
        if np.dot(normal, np.array(EYE) - d.mean(0)) < 0:
            normal = -normal
        gap = float(np.dot(q.mean(0) - d.mean(0), normal))
        if gap <= TARGET + TOLERANCE:
            continue          # on the legend, or behind a legend the export raised 2 mm: already on the button
        start, count, _m, rotated = ctl.tris[ql]
        if rotated:
            skipped.append((face, gap))
            continue
        moves[face] = (ql, (TARGET - gap) * normal, gap)
    return ctl, moves, skipped


def apply(ctl: Obj, moves):
    owners = {}
    for line, (start, count, _m, _r) in ctl.tris.items():
        for v in set(ctl.I[start:start + count].tolist()):
            owners.setdefault(v, set()).add(line)
    out = list(ctl.lines)
    for face, (line, shift, _gap) in moves.items():
        start, count, _m, _r = ctl.tris[line]
        for v in sorted(set(ctl.I[start:start + count].tolist())):
            if owners[v] != {line}:
                raise ValueError(f"{face}: quad vertex {v} is shared with other triangles")
            words = out[ctl.vt[v]].rstrip("\n").split("\t") if "\t" in out[ctl.vt[v]] else out[ctl.vt[v]].split()
            p = np.array([float(x) for x in words[1:4]]) + shift
            out[ctl.vt[v]] = "\t".join([words[0], *(fmt(x) for x in p), *words[4:]]) + "\n"
    return "".join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    ctl, moves, skipped = plan(args.aircraft)
    gaps = sorted(g for _l, _s, g in moves.values())
    print(f"{len(moves)} lamp quads {'would be ' if args.dry_run else ''}seated on their legends"
          + (f" (were {gaps[0] * 1000:+.2f} .. {gaps[-1] * 1000:+.2f} mm in front, now {TARGET * 1000:+.2f} mm)" if gaps else ""))
    for face, gap in skipped:
        print(f"  not moved (under a rotation): {face} {gap * 1000:+.2f} mm")
    if moves and not args.dry_run:
        text = apply(ctl, moves)
        tmp = ctl.path.with_name(ctl.path.name + ".tmp-seat")
        tmp.write_bytes(text.encode("latin-1"))
        tmp.replace(ctl.path)


if __name__ == "__main__":
    main()
