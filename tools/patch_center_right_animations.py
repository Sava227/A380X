#!/usr/bin/env python3
"""Add authored center-right motion to existing material exports.

Source glTF nodes identify the exact meshes, pivots and poses. Only OBJ draw
commands change; vertices, indices, texture references and UVs are preserved.
The landing-gear lever already has a source animation and is only retargeted.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from build_cockpit_controls import (
    accessor_values, load_buffer, local_matrix, matrix_identity,
    matrix_multiply, transform_point,
)
from patch_efis_animations import quaternion_multiply, vertex_payload
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation


MOTIONS = (
    ("KNOB_SWITCHING_BRK", "KNOB_SWITCHING_BRK", "center_right/autobrake_selected", 5),
    ("SWITCH_AUTOBKR_ASKID", "SWITCH_AUTOBKR_ASKID", "center_right/antiskid_on", 1),
    ("PUSH_RTO_ARM", "PUSH_RTO_ARM", "center_right/rto_pressed", 1),
    ("Accu_Press", "BRK_ACCU_PRESS_ARC", "center_right/body_accu_psi", 5000),
    ("Press_Arc_L", "BRK_PRESS_ARC_L", "center_right/brake_left_ratio_milli", 1000),
    ("Press_Arc_R", "BRK_PRESS_ARC_R", "center_right/brake_right_ratio_milli", 1000),
    ("KNOB_CHRONO_GPS", "KNOB_CHRONO_GPS", "clock/mode", 2),
    ("KNOB_CHRONO_RUN", "KNOB_CHRONO_RUN", "clock/et_selector", 2),
    ("PUSH_CHRONO_CHR", "PUSH_CHRONO_CHR", "clock/chr_held", 1),
    ("PUSH_CHRONO_RST", "PUSH_CHRONO_RST", "clock/rst_held", 1),
    ("PUSH_CHRONO_SET", "PUSH_CHRONO_SET", "clock/date_held", 1),
)
PREFIX = "a380x/xp12/"
MARKER = "# CENTER RIGHT MOTION "
OLD_GEAR_REF = "sim/cockpit2/controls/gear_handle_down"
GEAR_REF = PREFIX + "center_right/gear_lever"


def unit(vector):
    length = math.sqrt(sum(v * v for v in vector))
    if length < 1e-12:
        raise ValueError("Zero rotation axis")
    return tuple(v / length for v in vector)


def transform_vector(matrix, vector):
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3))
                 for row in range(3))


def rotation_keys(quaternions, rest, parent):
    """Continuous signed angles relative to source rest, in source world space."""
    inverse = tuple(-v if i < 3 else v for i, v in enumerate(rest))
    previous = rest
    deltas = []
    for quaternion in quaternions:
        # glTF permits antipodal quaternion representations within one clip.
        if sum(a * b for a, b in zip(previous, quaternion)) < 0:
            quaternion = tuple(-v for v in quaternion)
        deltas.append(quaternion_multiply(quaternion, inverse))
        previous = quaternion
    axis = next((unit(delta[:3]) for delta in deltas
                 if sum(v * v for v in delta[:3]) > 1e-8), (0., 0., 1.))
    angles = []
    for delta in deltas:
        signed = sum(a * b for a, b in zip(delta[:3], axis))
        residual = math.sqrt(sum((v - signed * a) ** 2 for v, a in zip(delta[:3], axis)))
        if residual > 2e-5:
            raise ValueError("Authored motion is not a single-axis rotation")
        angle = math.degrees(2 * math.atan2(signed, delta[3]))
        if angles:
            while angle - angles[-1] > 180:
                angle -= 360
            while angle - angles[-1] < -180:
                angle += 360
        angles.append(angle)
    return unit(transform_vector(parent, axis)), angles


def primitive_points(document, buffers, primitive):
    points = accessor_values(document, buffers, primitive["attributes"]["POSITION"])
    indices = [int(row[0]) for row in accessor_values(document, buffers, primitive["indices"])]
    packed = primitive.get("extras", {}).get("ASOBO_primitive", {})
    if packed:
        start = packed.get("StartIndex", 0)
        count = packed["PrimitiveCount"] * 3
        base = packed.get("BaseVertexIndex", 0)
        indices = [base + i for i in indices[start:start + count]]
    return [points[i] for i in sorted(set(indices))], len(indices)


def source_targets(source, include_gear=False, motions=None, include_child_points=False):
    document = json.loads(source.read_text())
    buffers = [load_buffer(source, row["uri"]) for row in document["buffers"]]
    nodes = document["nodes"]
    names = {node["name"]: i for i, node in enumerate(nodes) if "name" in node}
    parents = {child: i for i, node in enumerate(nodes) for child in node.get("children", [])}
    matrices = {}

    def world(index):
        if index not in matrices:
            local = local_matrix(nodes[index])
            matrices[index] = matrix_multiply(world(parents[index]), local) if index in parents else local
        return matrices[index]

    def descendants(index):
        yield index
        for child in nodes[index].get("children", []):
            yield from descendants(child)

    result = []
    motions = MOTIONS if motions is None else motions
    if include_gear:
        motions += (("lever_landing_gear", "Plane.131", "center_right/gear_lever", 1),)
    for animation_name, node_name, ref, maximum in motions:
        index = names[node_name]
        animation = next(a for a in document["animations"] if a["name"] == animation_name)
        channel = next((c for c in animation["channels"] if c["target"]["node"] == index),
                       animation["channels"][0])
        animated_index = channel["target"]["node"]
        if index not in set(descendants(animated_index)):
            raise ValueError(f"{animation_name} does not animate {node_name}")
        sampler = animation["samplers"][channel["sampler"]]
        times = [row[0] for row in accessor_values(document, buffers, sampler["input"])]
        poses = accessor_values(document, buffers, sampler["output"])
        values = [maximum * (time - times[0]) / (times[-1] - times[0]) for time in times]
        parent = world(parents[animated_index]) if animated_index in parents else matrix_identity()
        motion = dict(dataref=PREFIX + ref, pivot=transform_point(world(animated_index), (0, 0, 0)),
                      values=values, animation=animation_name)
        if channel["target"]["path"] == "rotation":
            axis, angles = rotation_keys(poses, nodes[animated_index].get("rotation", (0, 0, 0, 1)), parent)
            motion.update(kind="rotation", axis=axis, poses=angles)
        elif channel["target"]["path"] == "translation":
            rest = nodes[animated_index].get("translation", (0, 0, 0))
            motion.update(kind="translation", poses=[transform_vector(parent, tuple(v - r for v, r in zip(p, rest)))
                                                       for p in poses])
        else:
            raise ValueError(f"Unsupported center-right animation: {animation_name}")
        for child in descendants(animated_index):
            node = nodes[child]
            for primitive in document["meshes"][node["mesh"]]["primitives"] if "mesh" in node else ():
                points, count = primitive_points(document, buffers, primitive)
                points = [transform_point(world(child), point) for point in points]
                low = tuple(min(p[a] for p in points) for a in range(3))
                high = tuple(max(p[a] for p in points) for a in range(3))
                material = document["materials"][primitive["material"]]["name"].lower()
                # The retained decal repair lifts the front captions slightly.
                if child != index and child != animated_index:
                    low = tuple(v + delta for v, delta in zip(low, (-.00004, -.00004, -.004)))
                    high = tuple(v + delta for v, delta in zip(high, (.00004, .0015, .00004)))
                result.append(dict(motion, name=node["name"], material=material,
                                   low=low, high=high, count=count,
                                   points=points if child == index or include_child_points else None))
    return result


def animation_block(target, start, count, marker=MARKER):
    def vec(values):
        return "\t".join(f"{obj8_fmt(value)}" for value in values)

    lines = [marker + target["name"], "ANIM_begin"]
    if target["kind"] == "rotation":
        pivot = vec(target["pivot"])
        inverse = vec([-v for v in target["pivot"]])
        lines += [f"ANIM_trans\t{pivot}\t{pivot}",
                  f'ANIM_rotate_begin\t{vec(target["axis"])}\t{target["dataref"]}']
        lines += [f"ANIM_rotate_key\t{obj8_fmt(value)}\t{obj8_fmt(pose)}"
                  for value, pose in zip(target["values"], target["poses"])]
        lines += ["ANIM_rotate_end", f"ANIM_trans\t{inverse}\t{inverse}"]
    else:
        lines += [f'ANIM_trans_begin\t{target["dataref"]}']
        lines += [f"ANIM_trans_key\t{obj8_fmt(value)}\t{vec(pose)}"
                  for value, pose in zip(target["values"], target["poses"])]
        lines += ["ANIM_trans_end"]
    lines += [f"TRIS\t{start}\t{count}", "ANIM_end"]
    return lines


def wrap_geometry(text, filename, targets, marker=MARKER):
    applicable = [t for t in targets if filename.endswith(t["material"] + ".obj")]
    gear_count = text.count(OLD_GEAR_REF)
    text = text.replace(OLD_GEAR_REF, GEAR_REF)
    if not applicable:
        return text, [], gear_count
    # Each material file is patched atomically. This marker makes reruns safe.
    if marker in text:
        missing = [t["name"] for t in applicable if marker + t["name"] not in text]
        if missing:
            raise ValueError(f"Incomplete previous center-right patch: {missing}")
        return text, [], gear_count
    # A future complete export already owns these source nodes' motion.
    present = {t["dataref"] for t in applicable if t["dataref"] in text}
    if present:
        if present != {t["dataref"] for t in applicable}:
            raise ValueError("Partial native center-right export requires a clean rebuild")
        return text, [], gear_count
    lines = text.splitlines(keepends=True)
    vertices = [tuple(map(float, line.split()[1:4])) for line in lines if line.startswith("VT\t")]
    indices = [int(value) for line in lines if line.startswith(("IDX\t", "IDX10\t"))
               for value in line.split()[1:]]
    selected = []
    for target in applicable:
        points = target.get("points")
        grid = {}
        tolerance = .000012
        if points:
            for point in points:
                key = tuple(round(v / tolerance) for v in point)
                grid.setdefault(key, []).append(point)
        chosen = set()
        for i, point in enumerate(vertices):
            if not all(target["low"][a] - tolerance <= point[a] <= target["high"][a] + tolerance
                       for a in range(3)):
                continue
            if points:
                key = tuple(round(v / tolerance) for v in point)
                candidates = (p for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)
                              for p in grid.get((key[0] + x, key[1] + y, key[2] + z), ()))
                if not any(max(abs(a - b) for a, b in zip(point, p)) <= tolerance for p in candidates):
                    continue
            chosen.add(i)
        selected.append(chosen)
    counts = [0] * len(applicable)
    output = []
    for line in lines:
        fields = line.split()
        if not fields or fields[0] != "TRIS":
            output.append(line)
            continue
        start, count = map(int, fields[1:3])
        runs = []
        for position in range(start, start + count, 3):
            matches = [i for i, chosen in enumerate(selected)
                       if all(vertex in chosen for vertex in indices[position:position + 3])]
            if len(matches) > 1:
                raise ValueError(f"Ambiguous animation geometry in {filename} at index {position}: "
                                 f"{[applicable[i]['name'] for i in matches]}")
            match = matches[0] if matches else -1
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
            else:
                counts[match] += run_count
                output.extend(indent + row + "\n" for row in animation_block(applicable[match], run_start, run_count, marker))
    for target, count in zip(applicable, counts):
        if count != target["count"]:
            raise ValueError(f'{filename}: {target["name"]} expected {target["count"]} indices, found {count}')
    return "".join(output), [dict(name=t["name"], dataref=t["dataref"], indices=c)
                            for t, c in zip(applicable, counts)], gear_count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--directory", required=True, action="append", type=Path)
    parser.add_argument("--backup", type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not args.dry_run and args.backup is None:
        parser.error("--backup is required unless --dry-run is used")
    targets = source_targets(args.source)
    staged = []
    for directory in args.directory:
        for path in sorted(directory.glob("*.obj")):
            if path.name.startswith("._"):
                continue
            original = path.read_text()
            changed, meshes, gear_count = wrap_geometry(original, path.name, targets)
            if original == changed:
                continue
            if vertex_payload(original) != vertex_payload(changed):
                raise ValueError("Geometry, UV, or texture payload changed unexpectedly")
            staged.append((path, original, changed, meshes, gear_count))
    report = []
    # No file mutation until all expected triangle counts have been checked.
    for path, original, changed, meshes, gear_count in staged:
        digest = hashlib.sha256(original.encode()).hexdigest()
        backup = args.backup / (digest[:16] + "-" + path.name) if args.backup else None
        if not args.dry_run:
            backup.parent.mkdir(parents=True, exist_ok=True)
            if not backup.exists():
                backup.write_text(original)
            path.write_text(changed)
        report.append(dict(path=str(path), meshes=meshes, gear_blocks=gear_count,
                           before_sha256=digest, backup=str(backup) if backup else None))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    summary_targets = [{key: value for key, value in target.items() if key != "points"} for target in targets]
    args.report.write_text(json.dumps(dict(dry_run=args.dry_run, files=report, source_targets=summary_targets), indent=2) + "\n")
    print(json.dumps(dict(files=len(report), meshes=sum(len(row["meshes"]) for row in report),
                          gear_blocks=sum(row["gear_blocks"] for row in report), report=str(args.report))))


if __name__ == "__main__":
    main()
