#!/usr/bin/env python3
"""Motion, click targets and lamps for the FBW cockpit controls the port left static.

Driven by tools/cockpit_feedback_contract.py (SWITCHING knobs, ECAM control panel keys, SURV panel,
KCCU key caps, ENG MASTER lever decals, glareshield side switches). For every control:

* panel OBJs (objects/interior/cockpit-panels, objects/interior/cockpit): the visible FBW meshes of the
  animated node's subtree are wrapped in the authored clip (translation and/or single-axis rotation,
  keyed on the control's dataref). Only TRIS ranges that are not animated yet are wrapped (ANIM depth
  0), and VT / IDX / UV / texture payloads stay byte-identical (checked). Lamp faces that live in the
  emissive decal object get their own ATTR_light_level and are hidden while dark.
* a380x_cockpit_controls.obj (append-only, after MARKER): draw-disabled click boxes that move with the
  control, and lit quads (fcu_labels atlas cell) over lamp faces whose decal has no LIT texture.

Re-running on a patched aircraft is a no-op. X-Plane must be closed while the installed aircraft is
patched; --aircraft may also be the repository copy (output/A380X-XP12).

    python3 tools/cockpit_feedback.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" \\
        --source ~/Developer/fbw-cockpit-source/a380-cockpit/A380_COCKPIT_LOD00.gltf [--dry-run]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from build_cockpit_controls import (FCU_LABEL_COLUMNS, FCU_LABEL_ROWS, accessor_values, load_buffer,
                                    local_matrix, matrix_identity, matrix_multiply, transform_point)
from cockpit_feedback_contract import COMMANDS, CONTROLS, DATAREFS, Control, click_manipulator, hooked_commands
from obj8_numbers import fmt as obj8_fmt
from patch_center_right_animations import rotation_keys, transform_vector
from patch_efis_animations import vertex_payload

MARKER = "# A380X FEEDBACK V1"
CONTROLS_OBJ = "objects/interior/cockpit-controls/a380x_cockpit_controls.obj"
PROVIDER_MANIFEST = Path("avionics-providers/fbw-avionics-v1.json")
PANEL_DIRS = ("objects/interior/cockpit-panels", "objects/interior/cockpit")
EYE = (0.0, 3.2, 31.9)          # between the two seats (OBJ coordinates), lamps face this way
LAMP_LIFT = 0.00006             # m in front of the authored face
CLICK_MARGIN = 0.0005
TOLERANCE = 0.000012            # vertex match, as patch_center_right_animations
LEGEND_LIFT = 0.002             # the exported legend decals sit 2 mm out along their face normal
SHELL_TOLERANCE = 0.00005       # a vertex exactly LEGEND_LIFT from a source point
LIFT_TOLERANCE = 0.0003         # curved legend faces: per-vertex normals differ slightly from the face
DEFAULT_SOURCE = Path.home() / "Developer/fbw-cockpit-source/a380-cockpit/A380_COCKPIT_LOD00.gltf"


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(v):
    length = math.sqrt(_dot(v, v)) or 1.0
    return tuple(x / length for x in v)


def _vec(values):
    return "\t".join(obj8_fmt(v) for v in values)


class Source:
    """The FBW cockpit glTF: world matrices, clips and primitive geometry."""

    def __init__(self, path: Path):
        self.path = path
        self.doc = json.loads(path.read_text())
        self.buffers = [load_buffer(path, row["uri"]) for row in self.doc["buffers"]]
        self.nodes = self.doc["nodes"]
        self.names = {node["name"]: i for i, node in enumerate(self.nodes) if "name" in node}
        self.parents = {child: i for i, node in enumerate(self.nodes) for child in node.get("children", [])}
        self.clips = {a["name"].strip(): a for a in self.doc["animations"]}
        self._world = {}

    def world(self, index):
        if index not in self._world:
            local = local_matrix(self.nodes[index])
            self._world[index] = (matrix_multiply(self.world(self.parents[index]), local)
                                  if index in self.parents else local)
        return self._world[index]

    def parent_world(self, index):
        return self.world(self.parents[index]) if index in self.parents else matrix_identity()

    def descendants(self, index):
        yield index
        for child in self.nodes[index].get("children", []):
            yield from self.descendants(child)

    def primitives(self, index):
        """(material name, world triangles, index count) of a node's own mesh."""
        node = self.nodes[index]
        if "mesh" not in node:
            return []
        out = []
        for primitive in self.doc["meshes"][node["mesh"]]["primitives"]:
            points = accessor_values(self.doc, self.buffers, primitive["attributes"]["POSITION"])
            indices = [int(r[0]) for r in accessor_values(self.doc, self.buffers, primitive["indices"])]
            packed = primitive.get("extras", {}).get("ASOBO_primitive", {})
            if packed:
                start, count = packed.get("StartIndex", 0), packed["PrimitiveCount"] * 3
                base = packed.get("BaseVertexIndex", 0)
                indices = [base + i for i in indices[start:start + count]]
            world = self.world(index)
            triangles = [tuple(transform_point(world, points[i]) for i in indices[k:k + 3])
                         for k in range(0, len(indices) - 2, 3)]
            material = self.doc["materials"][primitive["material"]]["name"].lower()
            out.append((material, triangles, len(indices)))
        return out


def _extras(control: Control):
    """(clip, dataref, kind, low, high) of the main clip and every extra motion."""
    lo, hi = (control.minimum, control.maximum) if control.kind == "follow" else (0.0, 1.0)
    out = [(control.clip, control.dataref, control.kind, lo, hi)]
    for extra in control.extra:
        clip, dataref, *span = extra
        out.append((clip, dataref, "follow" if span else "press", *(span or (0.0, 1.0))))
    return out


def clip_span(source: Source, clip: dict):
    times = [row[0] for channel in clip["channels"]
             for row in accessor_values(source.doc, source.buffers, clip["samplers"][channel["sampler"]]["input"])]
    return min(times), max(times)


def compile_motion(source: Source, control: Control) -> dict:
    """The clip(s) as OBJ stages keyed on the control's datarefs, per animated node (outermost first).
    `stages` (the main node's chain, ancestors included) is completed by link_motions()."""
    index = source.names[control.node]
    motion = dict(control=control, index=index, pivot=transform_point(source.world(index), (0, 0, 0)),
                  stages=[], nodes={})
    if control.kind == "static":
        return motion
    for clip_name, dataref, kind, lo, hi in _extras(control):
        clip = source.clips.get(clip_name)
        if clip is None:
            raise ValueError(f"{control.id}: no FBW clip {clip_name}")
        if control.all_nodes:
            nodes = sorted({c["target"]["node"] for c in clip["channels"]})
        else:
            nodes = [index, *(source.names[n] for n in control.also)]
        span = clip_span(source, clip) if control.all_nodes or control.also else None
        for node in nodes:
            stages = compile_clip(source, control, clip_name, dataref, kind, node, span, lo, hi,
                                  required=not control.all_nodes)["stages"]
            if stages:
                motion["nodes"].setdefault(node, []).extend(stages)
    if not motion["nodes"]:
        raise ValueError(f"{control.id}: clip {control.clip} moves nothing")
    if index in motion["nodes"]:
        motion["stages"] = list(motion["nodes"][index])
    return motion


def _quat_matrix(q):
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def _unwrap(angles):
    out = []
    for a in angles:
        if out:
            while a - out[-1] > 180:
                a -= 360
            while a - out[-1] < -180:
                a += 360
        out.append(a)
    return out


def euler_stages(quats, rest, parent, pivot):
    """A rotation that changes axis during the clip: world Z, Y, X tables (exact at every key)."""
    from patch_efis_animations import quaternion_multiply
    columns = [_unit([parent[r][c] for r in range(3)]) for c in range(3)]
    p = [[columns[c][r] for c in range(3)] for r in range(3)]            # parent rotation, scale removed
    inverse = tuple(-v if i < 3 else v for i, v in enumerate(rest))
    zs, ys, xs = [], [], []
    for q in quats:
        m = _quat_matrix(quaternion_multiply(q, inverse))
        d = [[sum(p[r][i] * m[i][j] * p[c][j] for i in range(3) for j in range(3)) for c in range(3)] for r in range(3)]
        ys.append(math.degrees(math.asin(max(-1.0, min(1.0, -d[2][0])))))
        xs.append(math.degrees(math.atan2(d[2][1], d[2][2])))
        zs.append(math.degrees(math.atan2(d[1][0], d[0][0])))
    return [(axis, _unwrap(angles)) for axis, angles in (((0.0, 0.0, 1.0), zs), ((0.0, 1.0, 0.0), ys),
                                                          ((1.0, 0.0, 0.0), xs)) if max(map(abs, angles)) > 1e-4]


def compile_clip(source: Source, control: Control, clip_name: str, dataref: str, kind: str, index=None,
                 span=None, lo=None, hi=None, required=True) -> dict:
    """One node's channels of a clip as stages. `span` (the whole clip's time range) keys a node whose own
    channels cover only part of it; no-op channels (constant keys, scale) give no stage."""
    if index is None:
        index = source.names[control.node]
    clip = source.clips.get(clip_name)
    if clip is None:
        raise ValueError(f"{control.id}: no FBW clip {clip_name}")
    channels = [c for c in clip["channels"] if c["target"]["node"] == index]
    if not channels and required:
        raise ValueError(f"{control.id}: clip {clip_name} does not animate {source.nodes[index].get('name')}")
    samples = {}
    for channel in channels:
        sampler = clip["samplers"][channel["sampler"]]
        times = [row[0] for row in accessor_values(source.doc, source.buffers, sampler["input"])]
        values = accessor_values(source.doc, source.buffers, sampler["output"])
        samples[channel["target"]["path"]] = (times, values)
    node = source.nodes[index]
    parent = source.parent_world(index)
    pivot = transform_point(source.world(index), (0, 0, 0))
    stages = []
    if "translation" in samples:
        ts, poses = samples["translation"]
        rest = node.get("translation", (0, 0, 0))
        deltas = [transform_vector(parent, _sub(p, rest)) for p in poses]
        if max(abs(v) for d in deltas for v in d) > 1e-6:
            stages.append(dict(kind="translation", times=ts, poses=deltas, dataref=dataref))
    if "rotation" in samples:
        ts, quats = samples["rotation"]
        rest = node.get("rotation", (0, 0, 0, 1))
        try:
            parts = [rotation_keys(quats, rest, parent)]
        except ValueError:
            parts = euler_stages(quats, rest, parent, pivot)
        for axis, angles in parts:
            if max(map(abs, angles)) > 1e-4:
                stages.append(dict(kind="rotation", times=ts, axis=axis, poses=angles, pivot=pivot, dataref=dataref))
    if not stages:
        if required and not channels:
            raise ValueError(f"{control.id}: clip {clip_name} has no translation or rotation")
        return dict(control=control, index=index, pivot=pivot, stages=[])
    times = sorted({t for s in stages for t in s["times"]})
    t0, t1 = span if span else (times[0], times[-1])
    if t1 <= t0:
        raise ValueError(f"{control.id}: clip {clip_name} has no duration")
    key_value = {}
    if kind in ("press", "follow", "toggle", "rotary"):
        if lo is None:
            lo, hi = (control.minimum, control.maximum) if kind == "follow" else (0.0, 1.0)
        key_value = {t: lo + (hi - lo) * (t - t0) / (t1 - t0) for t in times}
    elif kind == "selector":
        if len(times) != control.positions:
            raise ValueError(f"{control.id}: {len(times)} keys for {control.positions} positions")
        key_value = {t: float(i) for i, t in enumerate(times)}
    elif kind == "switch":
        # value 0 = the key whose lever tip is lowest (crew view), upward from there
        tip = max((p for _m, tris, _n in source.primitives(index) for tri in tris for p in tri),
                  key=lambda p: _dot(_sub(p, pivot), _sub(p, pivot)))
        heights = []
        for t in times:
            point = tip
            for stage in reversed(stages):
                point = apply_stage(stage, t, point)
            heights.append((point[1], t))
        order = [t for _h, t in sorted(heights)]
        if control.positions == len(times):
            key_value = {t: float(rank) for rank, t in enumerate(order)}
        elif control.positions == 2 and len(times) == 3:
            key_value = {t: rank / 2.0 for rank, t in enumerate(order)}
        else:
            raise ValueError(f"{control.id}: {len(times)} keys for {control.positions} positions")
    for stage in stages:
        pairs = sorted(zip((key_value[t] for t in stage["times"]), stage["poses"]))
        stage["values"] = [v for v, _p in pairs]
        stage["poses"] = [p for _v, p in pairs]
    return dict(control=control, index=index, pivot=pivot, stages=stages)


def link_motions(source: Source, motions: list[dict]):
    """Compose the motion of nested animated nodes: every node's chain is the stages of all its animated
    ancestors (outermost first) and its own. Sets motion["stages"] (main node) and motion["click_stages"]."""
    node_stages, owner = {}, {}
    for motion in motions:
        for node, stages in motion["nodes"].items():
            node_stages.setdefault(node, []).extend(stages)
            owner.setdefault(node, motion)

    def chain(index):
        path = [index]
        while path[-1] in source.parents:
            path.append(source.parents[path[-1]])
        return [s for node in reversed(path) for s in node_stages.get(node, [])]

    for motion in motions:
        control = motion["control"]
        motion["chain"] = chain
        motion["animated"] = node_stages
        motion["owner"] = owner
        if control.kind != "static":
            motion["stages"] = chain(motion["index"])
        click = source.names[control.click_node] if control.click_node else motion["index"]
        motion["click_index"] = click
        motion["click_stages"] = chain(click) if control.kind != "static" else []
    return motions


def apply_stage(stage, t, point):
    i = stage["times"].index(t) if t in stage["times"] else min(range(len(stage["times"])),
                                                                 key=lambda k: abs(stage["times"][k] - t))
    if stage["kind"] == "translation":
        return tuple(a + b for a, b in zip(point, stage["poses"][i]))
    angle = math.radians(stage["poses"][i])
    x, y, z = stage["axis"]
    c, s = math.cos(angle), math.sin(angle)
    r = _sub(point, stage["pivot"])
    rotated = (r[0] * (c + x * x * (1 - c)) + r[1] * (x * y * (1 - c) - z * s) + r[2] * (x * z * (1 - c) + y * s),
               r[0] * (y * x * (1 - c) + z * s) + r[1] * (c + y * y * (1 - c)) + r[2] * (y * z * (1 - c) - x * s),
               r[0] * (z * x * (1 - c) - y * s) + r[1] * (z * y * (1 - c) + x * s) + r[2] * (c + z * z * (1 - c)))
    return tuple(a + b for a, b in zip(stage["pivot"], rotated))


def stage_lines(motion) -> list[str]:
    lines = []
    for stage in (motion["stages"] if isinstance(motion, dict) else motion):
        ref = stage["dataref"]
        if stage["kind"] == "translation":
            lines.append(f"ANIM_trans_begin\t{ref}")
            lines += [f"ANIM_trans_key\t{obj8_fmt(v)}\t{_vec(p)}" for v, p in zip(stage["values"], stage["poses"])]
            lines.append("ANIM_trans_end")
        else:
            pivot, inverse = _vec(stage["pivot"]), _vec(-v for v in stage["pivot"])
            lines += [f"ANIM_trans\t{pivot}\t{pivot}", f"ANIM_rotate_begin\t{_vec(stage['axis'])}\t{ref}"]
            lines += [f"ANIM_rotate_key\t{obj8_fmt(v)}\t{obj8_fmt(p)}" for v, p in zip(stage["values"], stage["poses"])]
            lines += ["ANIM_rotate_end", f"ANIM_trans\t{inverse}\t{inverse}"]
    return lines


def mesh_targets(source: Source, motion: dict) -> list[dict]:
    """Every visible primitive below the control's animated nodes (down to the next animated node, which
    gets its own chain). Children (legends, lamp faces) match either at their FBW position or lifted
    LEGEND_LIFT along their face normal, as the legend export placed them."""
    control = motion["control"]
    if control.kind == "static" or not control.patch:
        return []
    emis_lamps = {lamp.face: lamp for lamp in control.lamps if lamp.cell is None}
    animated, owner, chain = motion["animated"], motion["owner"], motion["chain"]
    out = []
    for root in motion["nodes"]:
        if owner[root] is not motion:
            continue
        pending = [root]
        while pending:
            child = pending.pop()
            pending.extend(c for c in source.nodes[child].get("children", []) if c not in animated)
            name = source.nodes[child].get("name", "")
            for material, triangles, count in source.primitives(child):
                points = sorted({p for tri in triangles for p in tri})
                lifted = []
                if child != root:
                    normal = (0.0, 0.0, 0.0)
                    for tri in triangles:
                        normal = tuple(a + b for a, b in zip(normal, _cross(_sub(tri[1], tri[0]), _sub(tri[2], tri[0]))))
                    normal = _unit(normal)
                    centre = tuple(sum(p[a] for p in points) / len(points) for a in range(3))
                    if _dot(normal, _sub(EYE, centre)) < 0:
                        normal = tuple(-n for n in normal)
                    lifted = [tuple(p[a] + LEGEND_LIFT * normal[a] for a in range(3)) for p in points]
                every = points + lifted
                margin = LEGEND_LIFT + LIFT_TOLERANCE if lifted else TOLERANCE
                low = [min(p[a] for p in every) - margin for a in range(3)]
                high = [max(p[a] for p in every) + margin for a in range(3)]
                out.append(dict(motion=motion, name=name, material=material, count=count, low=low, high=high,
                                points=points, lifted=lifted, lamp=emis_lamps.get(name), stages=chain(child)))
    return out


def _depths(lines):
    depth = 0
    for line in lines:
        words = line.split()
        word = words[0] if words else ""
        if word == "ANIM_begin":
            depth += 1
        elif word == "ANIM_end":
            depth -= 1
        yield depth


class _Grid:
    """Points bucketed in CELL cubes, for neighbourhood queries."""
    CELL = 0.003

    def __init__(self, points):
        self.cells = {}
        for q in points:
            self.cells.setdefault(tuple(int(math.floor(v / self.CELL)) for v in q), []).append(q)

    def near(self, p, radius):
        if not self.cells:
            return
        key = [int(math.floor(v / self.CELL)) for v in p]
        reach = int(math.ceil(radius / self.CELL))
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                for dz in range(-reach, reach + 1):
                    yield from self.cells.get((key[0] + dx, key[1] + dy, key[2] + dz), ())


def patch_panel(text: str, filename: str, targets: list[dict]):
    """Wrap the static draws of every applicable target; returns (text, report rows)."""
    applicable = [t for t in targets if filename.endswith(t["material"] + ".obj")
                  and (not t["motion"]["control"].files or filename in t["motion"]["control"].files)]
    # A control already wrapped by an earlier run is left alone (later contract rows patch on top).
    applicable = [t for t in applicable if f"{MARKER} {t['motion']['control'].id} {t['name']}\n" not in text]
    if not applicable:
        return text, []
    lines = text.splitlines(keepends=True)
    vertices = [tuple(map(float, line.split()[1:4])) for line in lines if line.startswith("VT")]
    indices = [int(v) for line in lines if line.startswith(("IDX\t", "IDX10\t", "IDX ", "IDX10 "))
               for v in line.split()[1:]]
    chosen = []
    for target in applicable:
        picked = set()
        low, high = target["low"], target["high"]
        points, lifted = _Grid(target["points"]), _Grid(target["lifted"])
        for i, p in enumerate(vertices):
            if not all(low[a] <= p[a] <= high[a] for a in range(3)):
                continue
            if any(max(abs(a - b) for a, b in zip(p, q)) <= TOLERANCE for q in points.near(p, TOLERANCE)) or \
                    any(max(abs(a - b) for a, b in zip(p, q)) <= LIFT_TOLERANCE for q in lifted.near(p, LIFT_TOLERANCE)):
                picked.add(i)
            elif target["lifted"] and any(abs(math.dist(p, q) - LEGEND_LIFT) <= SHELL_TOLERANCE
                                          for q in points.near(p, LEGEND_LIFT + SHELL_TOLERANCE)):
                picked.add(i)     # curved decal (knob pointer): lifted along its own vertex normal
        chosen.append(picked)
    owners = {}
    for k, picked in enumerate(chosen):
        for v in picked:
            owners.setdefault(v, []).append(k)
    counts = [0] * len(applicable)
    shared = [0] * len(applicable)
    skipped_animated = [0] * len(applicable)
    light_state = "ATTR_light_level_reset"
    output = []
    for line, depth in zip(lines, _depths(lines)):
        words = line.split()
        if words and words[0] in ("ATTR_light_level", "ATTR_light_level_reset"):
            light_state = line.strip()
        if not words or words[0] != "TRIS":
            output.append(line)
            continue
        start, count = int(words[1]), int(words[2])
        runs = []
        for position in range(start, start + count, 3):
            tri = indices[position:position + 3]
            matches = [k for k in owners.get(tri[0], ()) if tri[1] in chosen[k] and tri[2] in chosen[k]]
            if len(matches) > 1:
                # coincident meshes are fine when they move the same way
                motions = {"\n".join(stage_lines(applicable[k]["stages"])) for k in matches}
                if len(motions) > 1:
                    # a part resting on another (key cap on the keyboard base) shares a few vertices with
                    # it: the triangle goes to the smaller, more specific mesh
                    matches = sorted(matches, key=lambda k: applicable[k]["count"])
                    shared[matches[0]] += 3
                matches = matches[:1]
            match = matches[0] if matches else -1
            if match >= 0 and depth > 0:
                skipped_animated[match] += 3
                match = -1
            if runs and runs[-1][0] == match:
                runs[-1][2] += 3
            else:
                runs.append([match, position, 3])
        if len(runs) == 1 and runs[0][0] == -1:
            output.append(line)
            continue
        indent = line[:len(line) - len(line.lstrip())]
        for match, run_start, run_count in runs:
            if match < 0:
                output.append(f"{indent}TRIS\t{run_start}\t{run_count}\n")
                continue
            target = applicable[match]
            counts[match] += run_count
            control, lamp = target["motion"]["control"], target["lamp"]
            block = [f"{MARKER} {control.id} {target['name']}"]
            if lamp:
                block.append(f"ATTR_light_level\t0\t1\t{lamp.dataref}")
            block.append("ANIM_begin")
            if lamp:
                block.append(f"ANIM_hide\t-1\t0.001\t{lamp.dataref}")
            block += stage_lines(target["stages"]) + [f"TRIS\t{run_start}\t{run_count}", "ANIM_end"]
            if lamp:
                block.append(light_state)
            output.extend(indent + row + "\n" for row in block)
    patched = "".join(output)
    if vertex_payload(text) != vertex_payload(patched):
        raise ValueError(f"{filename}: VT/IDX/texture payload changed")
    report = []
    for target, count, animated, share in zip(applicable, counts, skipped_animated, shared):
        report.append(dict(control=target["motion"]["control"].id, mesh=target["name"], wrapped=count,
                           source_indices=target["count"], already_animated=animated, shared=share,
                           lamp=target["lamp"].dataref if target["lamp"] else None))
    return patched, report


def oriented_box(points, matrix, margin=CLICK_MARGIN):
    """12-triangle box over node-local points placed by matrix (OBJ8 clockwise fronts)."""
    low = [min(p[a] for p in points) - margin for a in range(3)]
    high = [max(p[a] for p in points) + margin for a in range(3)]
    x, y, z = low
    X, Y, Z = high
    local = ((x, y, z), (X, y, z), (X, Y, z), (x, Y, z), (x, y, Z), (X, y, Z), (X, Y, Z), (x, Y, Z))
    world = [transform_point(matrix, p) for p in local]
    centre = tuple(sum(p[a] for p in world) / 8 for a in range(3))
    faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (3, 7, 6, 2), (0, 4, 7, 3), (1, 2, 6, 5))
    vertices, indices = [], []
    for face in faces:
        a, b, c = (world[k] for k in face[:3])
        normal = _unit(_cross(_sub(b, a), _sub(c, a)))
        face_centre = tuple(sum(world[k][axis] for k in face) / 4 for axis in range(3))
        if _dot(normal, _sub(face_centre, centre)) < 0:
            normal = tuple(-n for n in normal)
            order = (0, 1, 2, 0, 2, 3)
        else:
            order = (0, 2, 1, 0, 3, 2)
        base = len(vertices)
        vertices.extend("VT\t" + _vec((*world[k], *normal, 0.0, 0.0)) for k in face)
        indices.extend(base + k for k in order)
    return vertices, indices


def cap_box(source: Source, motion: dict, margin=CLICK_MARGIN):
    """Push target over the front of a knob cap: the central half of the knob seen along its rotation
    axis, from 30 % of its depth to 0.6 mm in front of it (in front of the rotary target, so it wins)."""
    index = motion["index"]
    world = source.world(index)
    rotation = next((s for s in motion["stages"] if s["kind"] == "rotation"), None)
    points = local_points(source, index)
    low = [min(p[a] for p in points) for a in range(3)]
    high = [max(p[a] for p in points) for a in range(3)]
    if rotation is None:
        return oriented_box(points, world, margin)
    inverse = _invert(world)
    axis = _unit(tuple(sum(inverse[r][c] * rotation["axis"][c] for c in range(3)) for r in range(3)))
    k = max(range(3), key=lambda a: abs(axis[a]))
    centre = [(lo + hi) / 2 for lo, hi in zip(low, high)]
    front_high = transform_point(world, [high[a] if a == k else centre[a] for a in range(3)])
    front_low = transform_point(world, [low[a] if a == k else centre[a] for a in range(3)])
    front_is_high = _dot(_sub(EYE, front_high), _sub(EYE, front_high)) < _dot(_sub(EYE, front_low), _sub(EYE, front_low))
    depth = high[k] - low[k]
    lo2, hi2 = list(low), list(high)
    for a in range(3):
        if a != k:
            half = (high[a] - low[a]) / 4
            lo2[a], hi2[a] = centre[a] - half, centre[a] + half
    if front_is_high:
        lo2[k], hi2[k] = high[k] - 0.3 * depth, high[k] + 0.0006
    else:
        lo2[k], hi2[k] = low[k] - 0.0006, low[k] + 0.3 * depth
    corners = [tuple(lo2), tuple(hi2)]
    return oriented_box(corners, world, 0.0)


def local_points(source: Source, index: int):
    """Points of a node's subtree in that node's local frame."""
    inverse = _invert(source.world(index))
    out = []
    for child in source.descendants(index):
        for _material, triangles, _count in source.primitives(child):
            out.extend(transform_point(inverse, p) for tri in triangles for p in tri)
    return out


def _invert(m):
    r = [[m[j][i] for j in range(3)] for i in range(3)]        # rotation (with scale) inverse via adjugate
    a, b, c = m[0][:3]
    d, e, f = m[1][:3]
    g, h, i = m[2][:3]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    inv = [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
           [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
           [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]]
    del r
    t = [m[0][3], m[1][3], m[2][3]]
    it = [-sum(inv[row][k] * t[k] for k in range(3)) for row in range(3)]
    return [inv[0] + [it[0]], inv[1] + [it[1]], inv[2] + [it[2]], [0, 0, 0, 1]]


def decal_offset(aircraft: Path, source: Source, face: str) -> float:
    """Where the exported decal of a lamp face sits along its normal toward the crew (0, or the 2 mm of
    the legend export), measured in the panel OBJ of the face's material."""
    index = source.names[face]
    for material, triangles, _count in source.primitives(index):
        path = next((p for d in PANEL_DIRS for p in sorted((aircraft / d).glob("*.obj"))
                     if p.name.endswith(material + ".obj") and not p.name.startswith("._")), None)
        if path is None:
            continue
        points = sorted({p for tri in triangles for p in tri})
        normal = _unit(tuple(sum(c) for c in zip(*[_cross(_sub(t[1], t[0]), _sub(t[2], t[0])) for t in triangles])))
        centre = tuple(sum(p[a] for p in points) / len(points) for a in range(3))
        if _dot(normal, _sub(EYE, centre)) < 0:
            normal = tuple(-n for n in normal)
        grid = _Grid(tuple(map(float, line.split()[1:4])) for line in path.read_bytes().decode("latin-1").splitlines()
                     if line.startswith("VT"))
        offsets = []
        for p in points:
            near = list(grid.near(p, LEGEND_LIFT + LIFT_TOLERANCE))
            if near:
                q = min(near, key=lambda q: math.dist(p, q))
                offsets.append(_dot(_sub(q, p), normal))
        if offsets:
            offsets.sort()
            return max(0.0, offsets[len(offsets) // 2])
    return 0.0


def lamp_geometry(source: Source, lamp, cell: int, lift: float = 0.0):
    """Lit triangles over a lamp face, lifted toward the crew (LAMP_LIFT in front of its decal, which
    sits `lift` in front of the authored face), sampling one atlas cell."""
    index = source.names[lamp.face]
    u = (cell % FCU_LABEL_COLUMNS + .5) / FCU_LABEL_COLUMNS
    v = 1 - (cell // FCU_LABEL_COLUMNS + .5) / FCU_LABEL_ROWS
    vertices, indices = [], []
    for _material, triangles, _count in source.primitives(index):
        for tri in triangles:
            centre = tuple(sum(p[a] for p in tri) / 3 for a in range(3))
            normal = _unit(_cross(_sub(tri[1], tri[0]), _sub(tri[2], tri[0])))
            view = _unit(_sub(EYE, centre))
            if _dot(normal, view) > 0:          # OBJ8 front = clockwise: the cross product points away
                tri = (tri[0], tri[2], tri[1])
                normal = tuple(-n for n in normal)
            facing = tuple(-n for n in normal)  # toward the crew
            base = len(vertices)
            for p in tri:
                lifted = tuple(p[a] + (lift + LAMP_LIFT) * facing[a] for a in range(3))
                vertices.append("VT\t" + _vec((*lifted, *facing, u, v)))
            indices.extend((base, base + 1, base + 2))
    if not indices:
        raise ValueError(f"lamp face {lamp.face} has no triangles")
    return vertices, indices


def prune_clicks(text: str, motions: list[dict]) -> tuple[str, list[str]]:
    """Remove the click blocks of controls that no longer have a click target (their vertices stay,
    unreferenced). A block runs from its '# ... CLICK <id>' marker to its ANIM_end."""
    gone = {m["control"].id for m in motions if click_manipulator(m["control"]) is None}
    lines, out, removed, i = text.split("\n"), [], [], 0
    while i < len(lines):
        words = lines[i].split()
        if lines[i].startswith(f"{MARKER} CLICK ") and len(words) >= 6 and words[5] in gone:
            removed.append(words[5])
            i += 1
            while i < len(lines) and lines[i] != "ANIM_end":
                i += 1
            i += 1
            continue
        out.append(lines[i])
        i += 1
    return "\n".join(out), removed


def append_controls(text: str, source: Source, motions: list[dict], lifts: dict | None = None):
    """Append one block with the click targets and lamp quads not present yet (re-runs add nothing)."""
    clicks, lamps, report = [], [], []
    for motion in motions:
        control = motion["control"]
        manip = click_manipulator(control)
        if manip and f"{MARKER} CLICK {control.id}\n" in text:
            manip = None
        if manip:
            if control.push_cap:   # the push cap first: drawn and hit-tested in front of the body
                vertices, indices = cap_box(source, motion)
                clicks.append((control, f"ATTR_manip_command\tbutton\t{control.push_cap}\t{control.label}",
                               vertices, indices, motion))
            click = motion.get("click_index", motion["index"])
            vertices, indices = oriented_box(local_points(source, click), source.world(click))
            clicks.append((control, manip, vertices, indices, motion))
        for lamp in control.lamps:
            if lamp.cell is not None and f"{MARKER} LAMP {control.id} {lamp.face}\n" not in text:
                vertices, indices = lamp_geometry(source, lamp, lamp.cell, (lifts or {}).get(lamp.face, 0.0))
                lamps.append((control, lamp, vertices, indices, motion))
    if not clicks and not lamps:
        return text, []
    batch = text.count(MARKER + " CONTROLS") + 1
    lines = text.splitlines()
    row = next(i for i, line in enumerate(lines) if line.startswith("POINT_COUNTS"))
    counts = list(map(int, lines[row].split()[1:]))
    if sum(line.startswith("VT") for line in lines) != counts[0]:
        raise ValueError("controls OBJ vertex count differs from POINT_COUNTS")
    all_vertices, all_indices, commands = [], [], []
    def add(vertices, indices):
        base, start = counts[0] + len(all_vertices), counts[3] + len(all_indices)
        all_vertices.extend(vertices)
        all_indices.extend(base + i for i in indices)
        return start, len(indices)
    commands += ["ATTR_manip_none", "ATTR_no_cockpit", "ATTR_draw_enable", "ATTR_poly_os\t2", "ATTR_no_blend\t0.5"]
    for control, lamp, vertices, indices, motion in lamps:
        start, count = add(vertices, indices)
        commands += [f"{MARKER} LAMP {control.id} {lamp.face}", f"ATTR_light_level\t0\t1\t{lamp.dataref}",
                     "ANIM_begin", f"ANIM_hide\t-1\t0.001\t{lamp.dataref}", *stage_lines(motion),
                     f"TRIS\t{start}\t{count}", "ANIM_end", "ATTR_light_level_reset"]
        report.append(dict(lamp=lamp.dataref, face=lamp.face, cell=lamp.cell, triangles=count // 3))
    commands += ["ATTR_blend", "ATTR_poly_os\t0", "ATTR_draw_disable"]
    for control, manip, vertices, indices, motion in clicks:
        start, count = add(vertices, indices)
        marker = f"{MARKER} CLICK {control.id}" + (" CAP" if manip.startswith("ATTR_manip_command\t") and control.push_cap else "")
        stages = motion.get("click_stages", motion["stages"])
        if manip.startswith("ATTR_manip_command\t") and control.push_cap:
            stages = motion["stages"]
        commands += [marker, "ANIM_begin", *stage_lines(stages), manip,
                     f"TRIS\t{start}\t{count}", "ATTR_manip_none", "ANIM_end"]
        report.append(dict(click=control.id, manipulator=manip.split("\t")[0], triangles=count // 3))
    commands += ["ATTR_manip_none", "ATTR_draw_enable"]
    at = max(i for i, line in enumerate(lines) if line.startswith("VT")) + 1
    lines[at:at] = all_vertices
    at = max(i for i, line in enumerate(lines) if line.startswith(("IDX\t", "IDX10\t", "IDX ", "IDX10 "))) + 1
    lines[at:at] = ["IDX\t" + str(i) for i in all_indices]
    lines[row] = "POINT_COUNTS\t" + "\t".join(map(str, (counts[0] + len(all_vertices), counts[1], counts[2],
                                                          counts[3] + len(all_indices))))
    header = MARKER + " CONTROLS" + ("" if batch == 1 else f" {batch}")
    return "\n".join([*lines, "", header, *commands, ""]), report


SCREENS_OBJ = "objects/interior/cockpit-panels/a380x_mfd_screens.obj"
SCREEN_LIFT = 0.0005             # m in front of the authored (black) screen face
RMP_MARKER = "# A380X RMP DEVICES V1"


def screen_quad(source: Source, node: str, lift=SCREEN_LIFT):
    """Rectangle over a screen mesh, lifted toward the crew: (BL, TL, TR, BR) corners seen from the seat,
    up = toward the windshield (+z), right = crew right (-x), and its facing normal."""
    points = sorted({p for _m, tris, _n in source.primitives(source.names[node]) for tri in tris for p in tri})
    centre = tuple(sum(p[a] for p in points) / len(points) for a in range(3))
    normal = (0.0, 0.0, 0.0)
    for _m, tris, _n in source.primitives(source.names[node]):
        for tri in tris:
            normal = tuple(a + b for a, b in zip(normal, _cross(_sub(tri[1], tri[0]), _sub(tri[2], tri[0]))))
    normal = _unit(normal)
    if _dot(normal, _sub(EYE, centre)) < 0:
        normal = tuple(-n for n in normal)
    def project(v):
        d = _dot(v, normal)
        return _unit(tuple(a - d * n for a, n in zip(v, normal)))
    up0 = project((0.0, 0.0, 1.0))
    side0 = _unit(_cross(up0, normal))
    # principal in-plane axes of the screen outline (the overhead RMP is turned in its plane)
    xs = [_dot(_sub(p, centre), side0) for p in points]
    ys = [_dot(_sub(p, centre), up0) for p in points]
    sxx = sum(x * x for x in xs); syy = sum(y * y for y in ys); sxy = sum(x * y for x, y in zip(xs, ys))
    angle = 0.5 * math.atan2(2 * sxy, sxx - syy)             # long axis
    long_axis = tuple(math.cos(angle) * a + math.sin(angle) * b for a, b in zip(side0, up0))
    right = long_axis if _dot(long_axis, (-1.0, 0.0, 0.0)) >= 0 else tuple(-a for a in long_axis)   # crew right is -x
    up = _unit(_cross(normal, right))
    if _dot(up, up0) < 0:
        up = tuple(-a for a in up)
    us = [_dot(_sub(p, centre), right) for p in points]
    vs = [_dot(_sub(p, centre), up) for p in points]
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    def corner(u, v):
        return tuple(centre[a] + u * right[a] + v * up[a] + lift * normal[a] for a in range(3))
    return [corner(u0, v0), corner(u0, v1), corner(u1, v1), corner(u1, v0)], normal, right, up


def append_screens(text: str, source: Source):
    """One full-UV device quad per RMP in the visible screen object (no manipulators there)."""
    from cockpit_feedback_contract import RMP_CHANNELS, RMP_DEVICES, RMP_SCREEN_NODES
    if RMP_MARKER in text:
        return text, []
    lines = text.splitlines()
    row = next(i for i, line in enumerate(lines) if line.startswith("POINT_COUNTS"))
    counts = list(map(int, lines[row].split()[1:]))
    vertices, indices, commands, report = [], [], [RMP_MARKER, "ATTR_no_cockpit", "ATTR_poly_os\t2", "ATTR_draw_enable"], []
    for n, (device, channel, node) in enumerate(zip(RMP_DEVICES, RMP_CHANNELS, RMP_SCREEN_NODES), 1):
        corners, normal, right, up = screen_quad(source, node)
        base = counts[0] + len(vertices)
        start = counts[3] + len(indices)
        # OBJ8 fronts are clockwise seen from the crew: BL, TL, TR / BL, TR, BR
        for corner, uv in zip(corners, ((0, 0), (0, 1), (1, 1), (1, 0))):
            vertices.append("VT\t" + _vec((*corner, *normal, *uv)))
        tri = [0, 1, 2, 0, 2, 3]
        a, b, c = (corners[k] for k in tri[:3])
        if _dot(_cross(_sub(b, a), _sub(c, a)), normal) > 0:   # cross points away from the viewer for fronts
            tri = [0, 2, 1, 0, 3, 2]
        indices.extend(base + k for k in tri)
        commands += ["ANIM_begin", f"ANIM_hide\t-0.5\t0.5\ta380x/xp12/rmp/{n}/device_active",
                     f"ATTR_cockpit_device\t{device}\t1\t{channel}\t1", f"TRIS\t{start}\t6", "ATTR_no_cockpit",
                     "ANIM_end"]
        width = math.dist(corners[0], corners[3])
        height = math.dist(corners[0], corners[1])
        report.append(dict(device=device, width_mm=round(width * 1000, 1), height_mm=round(height * 1000, 1)))
    commands += ["ATTR_poly_os\t0"]
    at = max(i for i, line in enumerate(lines) if line.startswith("VT")) + 1
    lines[at:at] = vertices
    at = max(i for i, line in enumerate(lines) if line.startswith(("IDX\t", "IDX10\t", "IDX ", "IDX10 "))) + 1
    lines[at:at] = ["IDX\t" + str(i) for i in indices]
    lines[row] = "POINT_COUNTS\t" + "\t".join(map(str, (counts[0] + len(vertices), counts[1], counts[2],
                                                          counts[3] + len(indices))))
    return "\n".join([*lines, "", *commands, ""]), report


def sha(text: str) -> str:
    return hashlib.sha256(text.encode("latin-1")).hexdigest()


def run(aircraft: Path, source_path: Path, dry_run: bool, controls=CONTROLS) -> dict:
    source = Source(source_path)
    motions = link_motions(source, [compile_motion(source, c) for c in controls])
    targets = [t for m in motions for t in mesh_targets(source, m)]
    staged, report = {}, {"panels": [], "controls": []}
    for directory in PANEL_DIRS:
        for path in sorted((aircraft / directory).glob("*.obj")):
            if path.name.startswith("._"):
                continue
            original = path.read_bytes().decode("latin-1")
            patched, rows = patch_panel(original, path.name, targets)
            if patched != original:
                staged[path] = (original, patched)
            report["panels"] += [dict(file=path.name, **r) for r in rows]
    # Every visible source mesh must be wrapped exactly, unless it already moved before this pass.
    # Legend children may differ by a few triangles from the FBW mesh (the export re-triangulated some
    # legend art): at least 80 % wrapped is accepted, and reported as partial. More than the FBW count is
    # fine up to double-sided duplicates (the export writes both faces of two-sided materials).
    for r in report["panels"]:
        r["partial"] = 0 < r["wrapped"] < r["source_indices"]
    # Coincident meshes of one control count once (the triangle goes to the first): totals per control and file.
    totals = {}
    for r in report["panels"]:
        total = totals.setdefault((r["file"], r["control"]), dict(file=r["file"], control=r["control"], wrapped=0,
                                                                  source_indices=0, already_animated=0))
        for key in ("wrapped", "source_indices", "already_animated"):
            total[key] += r[key]
    problems = [r for r in report["panels"] if r["wrapped"] == 0 and r["already_animated"] == 0]
    problems += [r for r in totals.values() if 0 < r["wrapped"] < 0.8 * (r["source_indices"] - r["already_animated"])
                 or r["wrapped"] > 2.2 * r["source_indices"] + 6]
    controls_path = aircraft / CONTROLS_OBJ
    original = controls_path.read_bytes().decode("latin-1")
    lifts = {lamp.face: decal_offset(aircraft, source, lamp.face) for m in motions
             for lamp in m["control"].lamps if lamp.cell is not None
             and f"{MARKER} LAMP {m['control'].id} {lamp.face}\n" not in original}
    pruned, report["removed_clicks"] = prune_clicks(original, motions)
    patched, rows = append_controls(pruned, source, motions, lifts)
    if patched != original:
        staged[controls_path] = (original, patched)
    report["controls"] = rows
    screens_path = aircraft / SCREENS_OBJ
    original = screens_path.read_bytes().decode("latin-1")
    patched, report["screens"] = append_screens(original, source)
    if patched != original:
        staged[screens_path] = (original, patched)
    report["problems"] = problems
    report["files"] = [dict(path=str(p.relative_to(aircraft)), before=sha(o), after=sha(n)) for p, (o, n) in staged.items()]
    if not dry_run and not problems:
        for path, (_original, text) in staged.items():
            tmp = path.with_name(path.name + ".tmp-feedback")
            tmp.write_bytes(text.encode("latin-1"))
            tmp.replace(path)
    manifest_path = aircraft / PROVIDER_MANIFEST
    if manifest_path.is_file():
        document = json.loads(manifest_path.read_text())
        from cockpit_feedback_contract import RMP_COMMANDS, RMP_DATAREFS, RMP_DEVICES
        section = {"commands": list(COMMANDS), "datarefs": list(DATAREFS),
                   "hooked_commands": list(hooked_commands()), "rmp_commands": list(RMP_COMMANDS),
                   "rmp_datarefs": list(RMP_DATAREFS), "cockpit_devices": list(RMP_DEVICES), "runtime_tested": False}
        from avionics_provider import AVIONICS_COMMANDS, AVIONICS_DATAREFS
        # the overhead pushbutton positions extend the lamp contract, part of the main provider lists
        if (document.get("cockpit_feedback") != section or document.get("commands") != list(AVIONICS_COMMANDS)
                or document.get("datarefs") != list(AVIONICS_DATAREFS)):
            document["cockpit_feedback"] = section
            document["commands"], document["datarefs"] = list(AVIONICS_COMMANDS), list(AVIONICS_DATAREFS)
            report["files"].append(dict(path=str(PROVIDER_MANIFEST)))
            if not dry_run and not problems:
                manifest_path.write_text(json.dumps(document, indent=2) + "\n")
    report["written"] = not dry_run and not problems
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    report = run(args.aircraft.resolve(), args.source.resolve(), args.dry_run)
    if args.report:
        args.report.write_text(json.dumps(report, indent=1) + "\n")
    wrapped = sum(1 for r in report["panels"] if r["wrapped"])
    print(f"{len(report['files'])} OBJ files {'would change' if args.dry_run else 'changed' if report['written'] else 'NOT written'}, "
          f"{wrapped} meshes wrapped, {sum(1 for r in report['controls'] if 'click' in r)} click targets, "
          f"{sum(1 for r in report['controls'] if 'lamp' in r)} lamp quads, {len(report['problems'])} problems")
    for row in report["problems"][:40]:
        print("  problem:", row)
    return 1 if report["problems"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
