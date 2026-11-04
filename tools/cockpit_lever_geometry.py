#!/usr/bin/env python3
"""Shared geometry for detented cockpit levers (thrust levers first; flaps lever and speed brake next).

Works on the pinned FBW cockpit glTF and the existing material OBJ exports, never on new meshes:

* ``clip_motion``: an authored glTF clip as an X-Plane motion (rotation about the node pivot or
  translation in the parent frame), its keys mapped linearly onto 0..maximum of a plugin dataref,
  exactly as tools/patch_center_right_animations.py source_targets does;
* ``part_points`` / ``wrap_parts``: wraps the exact source triangle ranges of glTF parts in (nested)
  ANIM blocks, e.g. a button inside its lever. VT/IDX/UV, texture lines and the drawn index ranges stay
  identical; a part is only accepted at ANIM depth 0 and with its exact source index count;
* ``motion_open`` / ``drag_rotate_lines``: draw-disabled click boxes animated with their part and a
  click-hold-drag ``ATTR_manip_drag_rotate`` about the same pivot and axis as the authored motion, with
  ``ATTR_manip_keyframe`` rows for a non-linear dataref-to-angle map and optional native
  ``ATTR_axis_detent_range`` rows (lift detents, for levers that are physically lifted over a gate).

X-Plane OBJ8 reference: developer.x-plane.com/article/obj8-file-format-specification/
(ATTR_manip_drag_rotate, ATTR_manip_keyframe, ATTR_axis_detent_range).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from build_cockpit_controls import accessor_values, matrix_identity, transform_point
from grvty_cockpit_panel import Source, anim_depths, non_draw_lines, oriented_box, tris_coverage  # noqa: F401
from patch_center_right_animations import primitive_points, rotation_keys, transform_vector
from patch_efis_animations import vertex_payload  # noqa: F401
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

TOLERANCE = 0.000012  # metres; as patch_center_right_animations.wrap_geometry


def _num(value: float) -> str:
    """OBJ number; float noise below 1e-9 (axis components, rest poses) is written as 0."""
    return f'{obj8_fmt(0.0 if abs(value) < 1e-9 else value)}'


def _vec(values) -> str:
    return '\t'.join(_num(value) for value in values)


def clip_motion(source: Source, animation_name: str, node_name: str, dataref: str, maximum: float) -> dict:
    """The authored clip of ``node_name`` as keys over 0..maximum of ``dataref`` (world/OBJ frame)."""
    document = source.document
    index = source.names[node_name]
    animation = next(a for a in document['animations'] if a['name'] == animation_name)
    channel = next((c for c in animation['channels'] if c['target']['node'] == index), None)
    if channel is None:
        raise ValueError(f'{animation_name} does not animate {node_name}')
    sampler = animation['samplers'][channel['sampler']]
    times = [row[0] for row in accessor_values(document, source.buffers, sampler['input'])]
    poses = accessor_values(document, source.buffers, sampler['output'])
    if times[-1] <= times[0]:
        raise ValueError(f'{animation_name}: degenerate clip')
    values = [maximum * (t - times[0]) / (times[-1] - times[0]) for t in times]
    parent = source.world(source.parents[index]) if index in source.parents else matrix_identity()
    motion = dict(node=node_name, animation=animation_name, dataref=dataref, values=values,
                  pivot=transform_point(source.world(index), (0, 0, 0)))
    if channel['target']['path'] == 'rotation':
        axis, angles = rotation_keys(poses, source.nodes[index].get('rotation', (0, 0, 0, 1)), parent)
        motion.update(kind='rotation', axis=axis, poses=angles)
    elif channel['target']['path'] == 'translation':
        rest = source.nodes[index].get('translation', (0, 0, 0))
        motion.update(kind='translation', poses=[transform_vector(parent, tuple(v - r for v, r in zip(p, rest)))
                                                 for p in poses])
    else:
        raise ValueError(f'Unsupported clip path in {animation_name}')
    # Duplicate end keys (glTF clips often repeat the last frame) would make ANIM tables non-monotonic.
    keep = [k for k in range(len(values)) if k == 0 or values[k] > values[k - 1] + 1e-12]
    motion['values'] = [values[k] for k in keep]
    motion['poses'] = [motion['poses'][k] for k in keep]
    if abs(motion['values'][0]) > 1e-9 or (motion['kind'] == 'rotation' and abs(motion['poses'][0]) > 1e-3):
        raise ValueError(f'{animation_name}: the source rest pose is not the clip start')
    return motion


def motion_open(motion: dict) -> list[str]:
    """ANIM_begin and the transforms of one motion (the caller closes it with ANIM_end)."""
    lines = ['ANIM_begin']
    if motion['kind'] == 'rotation':
        pivot = motion['pivot']
        lines += [f'ANIM_trans\t{_vec(pivot)}\t{_vec(pivot)}',
                  f'ANIM_rotate_begin\t{_vec(motion["axis"])}\t{motion["dataref"]}']
        lines += [f'ANIM_rotate_key\t{_num(v)}\t{_num(p)}' for v, p in zip(motion['values'], motion['poses'])]
        inverse = [-v for v in pivot]
        lines += ['ANIM_rotate_end', f'ANIM_trans\t{_vec(inverse)}\t{_vec(inverse)}']
    else:
        lines += [f'ANIM_trans_begin\t{motion["dataref"]}']
        lines += [f'ANIM_trans_key\t{_num(v)}\t{_vec(p)}' for v, p in zip(motion['values'], motion['poses'])]
        lines += ['ANIM_trans_end']
    return lines


def nested_block(chain: list[dict], body: list[str]) -> list[str]:
    """Outer motion first; ``body`` (TRIS / manipulator rows) inside the innermost one."""
    lines = []
    for motion in chain:
        lines += motion_open(motion)
    return lines + body + ['ANIM_end'] * len(chain)


def motion_angle(motion: dict, value: float) -> float:
    """Pose (degrees) of a rotation motion at a dataref value (X-Plane interpolates keys linearly)."""
    values, poses = motion['values'], motion['poses']
    if value <= values[0]:
        return poses[0]
    for k in range(1, len(values)):
        if value <= values[k]:
            return poses[k - 1] + (poses[k] - poses[k - 1]) * (value - values[k - 1]) / (values[k] - values[k - 1])
    return poses[-1]


def drag_rotate_lines(motion: dict, dataref: str, mapping: list[tuple[float, float]], tooltip: str,
                      cursor: str = 'hand', detent_ranges: tuple = (), lift: float = 0.0,
                      lift_dataref: str = 'none', lift_range: tuple = (0.0, 0.0)) -> list[str]:
    """Click-hold-drag about the motion's own pivot and axis. ``mapping`` is (manipulator value,
    motion dataref value) from the lowest to the highest value: the end rows give angle1/angle2,
    the inner rows ATTR_manip_keyframe. Pivot/axis are on the rotation axis, so they are the same
    whether X-Plane reads them in object space or in the animated frame."""
    if motion['kind'] != 'rotation' or len(mapping) < 2:
        raise ValueError('drag_rotate needs a rotation and at least two mapping rows')
    if any(b[0] <= a[0] for a, b in zip(mapping, mapping[1:])):
        raise ValueError('mapping must increase')
    angles = [motion_angle(motion, value) for _, value in mapping]
    (v_min, _), (v_max, _) = mapping[0], mapping[-1]
    line = '\t'.join(['ATTR_manip_drag_rotate', cursor, _vec(motion['pivot']), _vec(motion['axis']),
                      _num(angles[0]), _num(angles[-1]), _num(lift), _num(v_min), _num(v_max),
                      _num(lift_range[0]), _num(lift_range[1]), dataref, lift_dataref, tooltip])
    rows = [line]
    rows += [f'ATTR_manip_keyframe\t{_num(value)}\t{_num(angle)}' for (value, _), angle in zip(mapping[1:-1], angles[1:-1])]
    rows += [f'ATTR_axis_detent_range\t{_num(start)}\t{_num(end)}\t{_num(height)}' for start, end, height in detent_ranges]
    return rows


def part_points(source: Source, node_name: str, material: str) -> tuple[list[tuple], int]:
    """World points and index count of the node's primitives of one material (case-insensitive)."""
    document = source.document
    index = source.names[node_name]
    node = document['nodes'][index]
    points, count = [], 0
    for primitive in document['meshes'][node['mesh']]['primitives'] if 'mesh' in node else ():
        if document['materials'][primitive['material']]['name'].lower() != material.lower():
            continue
        local, n = primitive_points(document, source.buffers, primitive)
        world = source.world(index)
        points += [transform_point(world, p) for p in local]
        count += n
    if not count:
        raise ValueError(f'{node_name} has no {material} primitive')
    return points, count


def _grid(points):
    grid = {}
    for p in points:
        grid.setdefault(tuple(round(v / TOLERANCE) for v in p), []).append(p)
    return grid


def _in_grid(grid, point) -> bool:
    key = tuple(round(v / TOLERANCE) for v in point)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                for p in grid.get((key[0] + dx, key[1] + dy, key[2] + dz), ()):
                    if max(abs(a - b) for a, b in zip(point, p)) <= TOLERANCE:
                        return True
    return False


def _non_draw(text: str, marker: str) -> list[str]:
    """Every line a motion wrap must leave in place and in order."""
    return [line for line in text.splitlines()
            if not line.lstrip().startswith(('TRIS', 'ANIM_', marker))]


def wrap_parts(text: str, parts: list[dict], marker: str) -> tuple[str, list[dict]]:
    """Wrap each part's exact static triangle runs in its nested motion chain.

    ``parts``: dict(name, points, count, chain). Raises on ambiguity, nesting, count mismatch or any
    change of the vertex/index payload or of a non-draw line."""
    lines = text.splitlines(keepends=True)
    vertices = [tuple(map(float, line.split()[1:4])) for line in lines if line.startswith('VT\t')]
    indices = [int(v) for line in lines if line.startswith(('IDX\t', 'IDX10\t')) for v in line.split()[1:]]
    grids = [_grid(part['points']) for part in parts]
    boxes = [([min(p[a] for p in part['points']) - TOLERANCE for a in range(3)],
              [max(p[a] for p in part['points']) + TOLERANCE for a in range(3)]) for part in parts]
    owners = []
    for point in vertices:
        found = 0
        for k, (low, high) in enumerate(boxes):
            if all(low[a] <= point[a] <= high[a] for a in range(3)) and _in_grid(grids[k], point):
                found |= 1 << k
        owners.append(found)
    counts = [0] * len(parts)
    output, depth = [], 0
    for line in lines:
        fields = line.split()
        word = fields[0] if fields else ''
        if word == 'ANIM_begin':
            depth += 1
        elif word == 'ANIM_end':
            depth -= 1
        if word != 'TRIS':
            output.append(line)
            continue
        start, count = map(int, fields[1:3])
        runs = []
        for position in range(start, start + count, 3):
            shared = owners[indices[position]] & owners[indices[position + 1]] & owners[indices[position + 2]]
            if shared & (shared - 1):
                raise ValueError(f'Ambiguous part geometry at index {position}: '
                                 f'{[parts[k]["name"] for k in range(len(parts)) if shared >> k & 1]}')
            match = shared.bit_length() - 1 if shared else -1
            if runs and runs[-1][0] == match:
                runs[-1][2] += 3
            else:
                runs.append([match, position, 3])
        if len(runs) == 1 and runs[0][0] == -1:
            output.append(line)
            continue
        if depth:
            raise ValueError(f'Part geometry already inside an animation at TRIS {start} {count}')
        indent = line[:len(line) - len(line.lstrip())]
        for match, run_start, run_count in runs:
            if match < 0:
                output.append(f'{indent}TRIS\t{run_start}\t{run_count}\n')
                continue
            counts[match] += run_count
            part = parts[match]
            block = [f'{marker}{part["name"]}'] + nested_block(part['chain'], [f'TRIS\t{run_start}\t{run_count}'])
            output.extend(indent + row + '\n' for row in block)
    for part, count in zip(parts, counts):
        if count != part['count']:
            raise ValueError(f'{part["name"]}: expected {part["count"]} indices, found {count}')
    patched = ''.join(output)
    if vertex_payload(text) != vertex_payload(patched):
        raise ValueError('visible VT/IDX/UV/texture payload changed')
    if _non_draw(text, marker) != _non_draw(patched, marker):
        raise ValueError('lines other than the wrapped TRIS ranges changed')
    if tris_coverage(text) != tris_coverage(patched):
        raise ValueError('drawn index ranges changed')
    rows = [dict(name=p['name'], indices=c, chain=[m['dataref'] for m in p['chain']]) for p, c in zip(parts, counts)]
    return patched, rows


def describe(motion: dict) -> dict:
    return {k: (list(v) if isinstance(v, tuple) else v) for k, v in motion.items()}


def dumps(value) -> str:
    return json.dumps(value, indent=2) + '\n'


def unit_axis(axis) -> tuple:
    length = math.sqrt(sum(a * a for a in axis)) or 1.0
    return tuple(a / length for a in axis)


__all__ = ['Source', 'clip_motion', 'motion_open', 'nested_block', 'motion_angle', 'drag_rotate_lines',
           'part_points', 'wrap_parts', 'oriented_box', 'describe', 'dumps', 'Path', 'vertex_payload',
           'non_draw_lines', 'tris_coverage', 'anim_depths', 'unit_axis']
