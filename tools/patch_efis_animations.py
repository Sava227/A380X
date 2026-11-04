#!/usr/bin/env python3
"""Patch only existing EFIS animation blocks and authored BARO unit selectors.

Vertex/index/UV/texture/legend data stay unchanged. This is a bounded migration
for existing cockpit exports: future exports use animation_datarefs.json.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import re
from pathlib import Path

from build_cockpit_controls import accessor_values, load_buffer, local_matrix, matrix_multiply, transform_point
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation


def legacy_mappings():
    result = {}
    for side, suffix, pressure in (("captain", "", "pilot"), ("first_officer", "_copilot", "copilot")):
        prefix = f"a380x/xp12/efis/{side}/"
        for stock, field in (("data", "cstr"), ("fix", "wpt"), ("vor", "vor1"),
                             ("ndb", "ndb"), ("airport", "arpt"), ("weather", "wx_requested"),
                             ("terrain", "terr"), ("tcas", "traf")):
            result[f"sim/cockpit2/EFIS/EFIS_{stock}_on{suffix}"] = (prefix + field, None)
        result[f"sim/cockpit2/EFIS/map_mode{suffix}"] = (prefix + "mode", None)
        result[f"sim/cockpit2/EFIS/map_range{suffix}"] = (prefix + "range_index", (1., 6., 0., 7.))
        result[f"sim/cockpit2/gauges/actuators/barometer_setting_in_hg_{pressure}"] = (
            prefix + "baro_preset_hpa", (28., 31., 745., 1100.))
    return result


def patch_legacy(text):
    mapping = legacy_mappings()
    lines, active, changed = [], None, 0
    for line in text.splitlines(keepends=True):
        fields = line.split()
        if fields and fields[0] in ("ANIM_rotate_begin", "ANIM_trans_begin"):
            active = mapping.get(fields[-1])
            if active:
                line = line.replace(fields[-1], active[0])
                changed += 1
        elif active and fields and fields[0] in ("ANIM_rotate_key", "ANIM_trans_key") and active[1]:
            old_min, old_max, new_min, new_max = active[1]
            value = (float(fields[1]) - old_min) / (old_max - old_min) * (new_max - new_min) + new_min
            line = re.sub(r"(ANIM_(?:rotate|trans)_key\s+)\S+", lambda m: m[1] + f"{obj8_fmt(value)}", line)
        elif fields and fields[0] in ("ANIM_rotate_end", "ANIM_trans_end"):
            active = None
        lines.append(line)
    return "".join(lines), changed


def quaternion_multiply(a, b):
    x,y,z,w=a; X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def unit_targets(source):
    document = json.loads(source.read_text())
    buffers = [load_buffer(source, row["uri"]) for row in document["buffers"]]
    nodes = document["nodes"]
    parents = {child: index for index, node in enumerate(nodes) for child in node.get("children", [])}
    matrices = {}
    def world(index):
        if index not in matrices:
            matrix = local_matrix(nodes[index])
            matrices[index] = matrix_multiply(world(parents[index]), matrix) if index in parents else matrix
        return matrices[index]
    result = []
    for short, side in (("CS", "captain"), ("FO", "first_officer")):
        name = f"KNOB_EFIS_{short}_BARO2"
        index = next(i for i,n in enumerate(nodes) if n.get("name") == name)
        animation = next(a for a in document["animations"] if a["name"] == name)
        channel = next(c for c in animation["channels"] if c["target"]["path"] == "rotation")
        sampler = animation["samplers"][channel["sampler"]]
        quaternions = accessor_values(document, buffers, sampler["output"])
        q = quaternion_multiply(quaternions[-1], tuple(-v if i < 3 else v for i,v in enumerate(quaternions[0])))
        length = math.sqrt(sum(v*v for v in q[:3]))
        axis = tuple(v/length for v in q[:3])
        angle = math.degrees(2*math.atan2(length, q[3]))
        pivot = transform_point(world(index), (0, 0, 0))
        for child in (index, *nodes[index].get("children", [])):
            for primitive in document["meshes"][nodes[child]["mesh"]]["primitives"]:
                points = [transform_point(world(child), point)
                          for point in accessor_values(document, buffers, primitive["attributes"]["POSITION"])]
                material = document["materials"][primitive["material"]]["name"].lower()
                low = tuple(min(p[a] for p in points) for a in range(3))
                high = tuple(max(p[a] for p in points) for a in range(3))
                if child != index:
                    # Existing legend repair lifted emissive decals toward the
                    # camera. Accept that 0..4 mm normal separation while still
                    # requiring exactly the authored indicator triangle count.
                    low = tuple(v + delta for v, delta in zip(low, (-.00004, -.00004, -.004)))
                    high = tuple(v + delta for v, delta in zip(high, (.00004, .0015, .00004)))
                result.append(dict(name=nodes[child]["name"], side=side, material=material,
                    low=low, high=high,
                    count=len(accessor_values(document, buffers, primitive["indices"])),
                    pivot=pivot, axis=axis, angle=angle))
    return result


def wrap_unit_geometry(text, filename, targets):
    applicable = [target for target in targets if filename.endswith(target["material"] + ".obj")]
    if not applicable or "# EFIS BARO UNIT " in text:
        return text, 0
    lines = text.splitlines(keepends=True)
    vertices = [tuple(map(float, line.split()[1:4])) for line in lines if line.startswith("VT\t")]
    indices = [int(value) for line in lines if line.startswith(("IDX\t", "IDX10\t")) for value in line.split()[1:]]
    selected = {}
    for index, target in enumerate(applicable):
        selected[index] = {i for i, point in enumerate(vertices)
                           if all(target["low"][a] - 0.000006 <= point[a] <= target["high"][a] + 0.000006
                                  for a in range(3))}
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
            matches = [index for index, selected_vertices in selected.items()
                       if all(vertex in selected_vertices for vertex in indices[position:position + 3])]
            if len(matches) > 1:
                raise ValueError("Ambiguous BARO unit geometry")
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
                continue
            target = applicable[match]
            counts[match] += run_count
            pivot = "\t".join(f"{obj8_fmt(v)}" for v in target["pivot"])
            inverse = "\t".join(f"{obj8_fmt(-v)}" for v in target["pivot"])
            axis = "\t".join(f"{obj8_fmt(v)}" for v in target["axis"])
            ref = f'a380x/xp12/efis/{target["side"]}/baro_inhg'
            # Source rest pose selects inHg. hPa is the final authored pose.
            block = [f'# EFIS BARO UNIT {target["name"]}', "ANIM_begin",
                f"ANIM_trans\t{pivot}\t{pivot}", f"ANIM_rotate_begin\t{axis}\t{ref}",
                f'ANIM_rotate_key\t0\t{obj8_fmt(target["angle"])}', "ANIM_rotate_key\t1\t0",
                "ANIM_rotate_end", f"ANIM_trans\t{inverse}\t{inverse}",
                f"TRIS\t{run_start}\t{run_count}", "ANIM_end"]
            output.extend(indent + row + "\n" for row in block)
    for target, count in zip(applicable, counts):
        if count != target["count"]:
            raise ValueError(f'{filename}: {target["name"]} expected {target["count"]} indices, found {count}')
    return "".join(output), sum(bool(count) for count in counts)


def vertex_payload(text):
    return "".join(line for line in text.splitlines(keepends=True)
                   if line.startswith(("VT\t", "IDX\t", "IDX10\t", "TEXTURE", "POINT_COUNTS")))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--directory", required=True, action="append", type=Path)
    parser.add_argument("--backup", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    targets = unit_targets(args.source)
    report, staged = [], []
    for directory in args.directory:
        for path in sorted(directory.glob("*.obj")):
            if path.name.startswith("._"):
                continue
            original = path.read_text()
            changed, blocks = patch_legacy(original)
            changed, units = wrap_unit_geometry(changed, path.name, targets)
            if changed == original:
                continue
            if vertex_payload(original) != vertex_payload(changed):
                raise ValueError("Geometry, UV, or texture directives changed unexpectedly")
            staged.append((path, original, changed, blocks, units))
    # Preflight every target before the first write: an unexpected export aborts
    # without leaving a partly migrated aircraft.
    for path, original, changed, blocks, units in staged:
            digest = hashlib.sha256(original.encode()).hexdigest()
            backup = args.backup / (digest[:16] + "-" + path.name)
            backup.parent.mkdir(parents=True, exist_ok=True)
            if not backup.exists():
                backup.write_text(original)
            path.write_text(changed)
            report.append(dict(path=str(path), legacy_blocks=blocks, unit_meshes=units,
                               before_sha256=digest, backup=str(backup)))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(dict(files=len(report), legacy_blocks=sum(row["legacy_blocks"] for row in report),
                         unit_meshes=sum(row["unit_meshes"] for row in report), report=str(args.report))))

if __name__ == "__main__":
    main()
