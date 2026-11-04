#!/usr/bin/env python3
"""Stage the L/G GRVTY panel geometry: striped master cover, two switch guards, 3-position switch.

Pinned FBW clips and meshes (A380_COCKPIT_LOD00.gltf, behaviour mip.xml lines ~122-188):

* GRVTY_LDG rotates SWITCH_GRVTY with its whole subtree (lever bar GRVTY_LDG, heads Cylinder.105/106,
  SWITCH_OVHD_EXTLT_WING.001) through three keys: RESET (source rest pose), OFF, DOWN.
  Driven by a380x/xp12/gear/grvty_switch_anim (0 RESET .. 2 DOWN).
* GRVTY_LDG_LOCK2 / LOCK3 rotate the left / right switch guard (L:A32NX_LG_GRVTY_SWITCH_GUARD_1 / _2),
  driven by grvty_guard_1_anim / grvty_guard_2_anim (0 closed .. 1 open).
* GRVTY_LDG_LOCK1 rotates Plane.160 and its children GRVTY_LDG_LOCK_COVER (MIP03 material) and the
  authored INVISBLE_CLICKSPOTS mesh GRVTY_LDG_LOCK1, driven by grvty_master_guard_anim.

Staged changes (nothing else changes; the input aircraft is never written):

* panel_a380_cockpit_mip_knobs_02_4k.obj / panel_a380_cockpit_mip03_4k.obj: the exact source
  triangle ranges are wrapped in ANIM blocks; VT/IDX/UV/texture payload and every other line stay
  identical (checked), the wrapped ranges are static (ANIM depth 0) before the patch.
* a380x_cockpit_controls.obj: append-only (after MARKER) draw-disabled click boxes, each moving with
  its part: the authored cover click spot, both guards, and the switch (up/down manipulator, one
  detent per click). Guards and switch are only clickable while the master cover is open; the
  plugin enforces every FBW interlock, so a refused click changes nothing and animates nothing.

    python3 tools/grvty_cockpit_panel.py --aircraft output/A380X-XP12 \\
        --source <A380_COCKPIT_LOD00.gltf> --stage build/grvty-geometry-<timestamp>
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from avionics_provider import GRVTY_COMMANDS, GRVTY_DATAREFS
from build_cockpit_controls import load_buffer, local_matrix, matrix_multiply, transform_point
from isis_cockpit_device import _append
from patch_center_right_animations import (OLD_GEAR_REF, animation_block, primitive_points, source_targets,
                                           wrap_geometry)
from patch_efis_animations import vertex_payload
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

PREFIX = 'a380x/xp12/gear/'
MARKER = '# A380X LG GRVTY V1'
MOTION_MARKER = MARKER + ' MOTION '
CLICK_MARKER = MARKER + ' CLICK '
CONTROLS = 'objects/interior/cockpit-controls/a380x_cockpit_controls.obj'
KNOBS = 'objects/interior/cockpit-panels/panel_a380_cockpit_mip_knobs_02_4k.obj'
MIP03 = 'objects/interior/cockpit-panels/panel_a380_cockpit_mip03_4k.obj'
# (glTF animation, animated node, dataref suffix, maximum): keys map linearly onto 0..maximum.
MOTIONS = (
    ('GRVTY_LDG', 'SWITCH_GRVTY', 'gear/grvty_switch_anim', 2),
    ('GRVTY_LDG_LOCK1', 'Plane.160', 'gear/grvty_master_guard_anim', 1),
    ('GRVTY_LDG_LOCK2', 'GRVTY_LDG_LOCK2', 'gear/grvty_guard_1_anim', 1),
    ('GRVTY_LDG_LOCK3', 'GRVTY_LDG_LOCK3', 'gear/grvty_guard_2_anim', 1),
)
# Visible meshes each panel object must animate (source triangle counts are checked as well).
EXPECTED = {
    KNOBS: {'SWITCH_GRVTY', 'GRVTY_LDG', 'Cylinder.105', 'Cylinder.106', 'SWITCH_OVHD_EXTLT_WING.001',
            'GRVTY_LDG_LOCK2', 'GRVTY_LDG_LOCK3'},
    MIP03: {'Plane.160', 'GRVTY_LDG_LOCK_COVER'},
}
CLICK_MARGIN = 0.0005  # m around the authored meshes
MASTER_STATE = PREFIX + 'grvty_master_guard'
# (name, nodes in the box frame of nodes[0], animation, manipulator, hidden while the cover is closed)
CLICKS = (
    ('cover', ('GRVTY_LDG_LOCK1',), 'GRVTY_LDG_LOCK1',
     f'ATTR_manip_command\thand\t{PREFIX}grvty_master_guard_toggle\tL/G GRVTY COVER - open / close', False),
    ('guard_1', ('GRVTY_LDG_LOCK2',), 'GRVTY_LDG_LOCK2',
     f'ATTR_manip_command\thand\t{PREFIX}grvty_guard_1_toggle\tL/G GRVTY GUARD 1 (left) - open / close', True),
    ('guard_2', ('GRVTY_LDG_LOCK3',), 'GRVTY_LDG_LOCK3',
     f'ATTR_manip_command\thand\t{PREFIX}grvty_guard_2_toggle\tL/G GRVTY GUARD 2 (right) - open / close', True),
    ('switch', ('GRVTY_LDG', 'Cylinder.105', 'Cylinder.106'), 'GRVTY_LDG',
     f'ATTR_manip_command_switch_up_down\tup_down\t{PREFIX}grvty_switch_up\t{PREFIX}grvty_switch_down\t'
     'L/G GRVTY - RESET / OFF / DOWN (DOWN: cover and both guards open)', True),
)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def motion_targets(source: Path) -> list[dict]:
    return source_targets(source, motions=MOTIONS, include_child_points=True)


class Source:
    """Pinned FBW cockpit glTF: node world matrices and packed primitive points."""

    def __init__(self, path: Path):
        self.document = json.loads(path.read_text())
        self.buffers = [load_buffer(path, row['uri']) for row in self.document['buffers']]
        self.nodes = self.document['nodes']
        self.names = {node['name']: i for i, node in enumerate(self.nodes) if 'name' in node}
        self.parents = {child: i for i, node in enumerate(self.nodes) for child in node.get('children', [])}

    def world(self, index: int):
        local = local_matrix(self.nodes[index])
        return matrix_multiply(self.world(self.parents[index]), local) if index in self.parents else local

    def local_points(self, name: str) -> list[tuple[float, float, float]]:
        node = self.nodes[self.names[name]]
        return [tuple(p) for primitive in self.document['meshes'][node['mesh']]['primitives']
                for p in primitive_points(self.document, self.buffers, primitive)[0]]

    def points_in_frame(self, nodes: tuple[str, ...]) -> list[tuple[float, float, float]]:
        """Points of nodes[0] and of its direct children nodes[1:], in nodes[0]'s local frame."""
        frame = self.names[nodes[0]]
        points = list(self.local_points(nodes[0]))
        for name in nodes[1:]:
            child = self.names[name]
            if self.parents.get(child) != frame:
                raise ValueError(f'{name} is not a child of {nodes[0]}')
            matrix = local_matrix(self.nodes[child])
            points.extend(transform_point(matrix, p) for p in self.local_points(name))
        return points


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(v):
    length = math.sqrt(sum(x * x for x in v)) or 1.0
    return tuple(x / length for x in v)


def oriented_box(points, matrix, margin=CLICK_MARGIN):
    """12-triangle box over local points, placed by matrix; OBJ8 clockwise fronts, outward normals."""
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
        # OBJ8 fronts are clockwise seen from outside: (b - a) x (c - a) of every emitted triangle
        # points into the box. The quad order's own winding is outward unless the matrix mirrors.
        if sum(n * d for n, d in zip(normal, _sub(face_centre, centre))) < 0:
            normal = tuple(-n for n in normal)
            order = (0, 1, 2, 0, 2, 3)
        else:
            order = (0, 2, 1, 0, 3, 2)
        base = len(vertices)
        vertices.extend('VT\t' + '\t'.join(f'{obj8_fmt(v)}' for v in (*world[k], *normal, 0.0, 0.0)) for k in face)
        indices.extend(base + k for k in order)
    return vertices, indices, dict(corner_min_local=list(transform_point(matrix, low)),
                                   corner_max_local=list(transform_point(matrix, high)))


def click_geometry(source_path: Path, targets: list[dict]):
    """(vertices, indices, commands, report rows) for the controls object, relative to 0."""
    source = Source(source_path)
    motion = {t['name']: t for t in targets}
    vertices, indices = [], []
    commands = ['ATTR_manip_none', 'ATTR_no_cockpit', 'ATTR_draw_disable']
    hidden_open = False
    rows = []
    for name, nodes, animated, manip, guarded in CLICKS:
        box_vertices, box_indices, bounds = oriented_box(source.points_in_frame(nodes),
                                                         source.world(source.names[nodes[0]]))
        start, base = len(indices), len(vertices)
        vertices.extend(box_vertices)
        indices.extend(base + i for i in box_indices)
        if guarded and not hidden_open:
            # Under the closed cover nothing is reachable (FBW verifies the master guard first).
            commands += ['ANIM_begin', f'ANIM_hide\t-1\t0.5\t{MASTER_STATE}']
            hidden_open = True
        if not guarded and hidden_open:
            raise ValueError('Unguarded click targets must precede the guarded ones')
        block = animation_block(motion[animated], start, len(box_indices), CLICK_MARKER)
        tris = next(i for i, row in enumerate(block) if row.startswith('TRIS'))
        commands += [*block[:tris], manip, block[tris], 'ATTR_manip_none', *block[tris + 1:]]
        rows.append(dict(name=name, nodes=list(nodes), animation=animated, dataref=motion[animated]['dataref'],
                         manipulator=manip.split('\t')[0], commands=[w for w in manip.split('\t') if w.startswith(PREFIX)],
                         hidden_while_cover_closed=guarded, triangles=len(box_indices) // 3, **bounds))
    if hidden_open:
        commands.append('ANIM_end')
    commands.append('ATTR_draw_enable')
    return vertices, indices, commands, rows


def augment_controls(text: str, vertices, indices, commands) -> str:
    if MARKER in text:
        return text
    return _append(text, vertices, indices, commands, MARKER)


def anim_depths(lines):
    depth = 0
    for line in lines:
        word = line.split()[0] if line.split() else ''
        if word == 'ANIM_begin':
            depth += 1
        elif word == 'ANIM_end':
            depth -= 1
        yield line, depth


def tris_coverage(text: str):
    ranges = sorted((int(w[1]), int(w[2])) for w in (line.split() for line in text.splitlines())
                    if w and w[0] == 'TRIS')
    merged = []
    for start, count in ranges:
        if merged and start <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], start + count)
        else:
            merged.append([start, start + count])
    return merged


def non_draw_lines(text: str):
    """Every line that the motion patch must leave in place and in order."""
    return [line for line in text.splitlines()
            if not line.lstrip().startswith(('TRIS', 'ANIM_', MOTION_MARKER))]


def patch_panel(text: str, relative: str, targets: list[dict]):
    if OLD_GEAR_REF in text:
        raise ValueError(f'{relative}: un-retargeted stock gear lever; run patch_center_right_animations first')
    applicable = [t for t in targets if relative.endswith(t['material'] + '.obj')]
    if {t['name'] for t in applicable} != EXPECTED[relative]:
        raise ValueError(f'{relative}: unexpected source meshes {sorted(t["name"] for t in applicable)}')
    if MOTION_MARKER in text:
        patched, matched, _ = wrap_geometry(text, relative, targets, MOTION_MARKER)
        if patched != text:
            raise ValueError(f'{relative}: inconsistent previous L/G GRVTY patch')
        return text, []
    patched, matched, gear = wrap_geometry(text, relative, targets, MOTION_MARKER)
    if gear:
        raise ValueError(f'{relative}: unexpected gear-lever retarget')
    counts = {row['name']: row['indices'] for row in matched}
    expected_counts = {t['name']: t['count'] for t in applicable}
    if counts != expected_counts:
        raise ValueError(f'{relative}: animated index counts {counts} != source {expected_counts}')
    if vertex_payload(text) != vertex_payload(patched):
        raise ValueError(f'{relative}: visible VT/IDX/UV/texture payload changed')
    if non_draw_lines(text) != non_draw_lines(patched):
        raise ValueError(f'{relative}: lines other than the wrapped TRIS ranges changed')
    if tris_coverage(text) != tris_coverage(patched):
        raise ValueError(f'{relative}: drawn index ranges changed')
    for line, depth in anim_depths(patched.splitlines()):
        if line.startswith(MOTION_MARKER) and depth != 0:
            raise ValueError(f'{relative}: source range already inside an animation ({line})')
    return patched, matched


def stage(aircraft: Path, source: Path, destination: Path) -> dict:
    if destination.exists():
        raise ValueError('Stage directory must not exist')
    targets = motion_targets(source)
    used_refs = {t['dataref'] for t in targets} | {MASTER_STATE}
    if not used_refs <= set(GRVTY_DATAREFS):
        raise ValueError(f'Datarefs outside the native contract: {sorted(used_refs - set(GRVTY_DATAREFS))}')
    staged, motions = {}, {}
    for relative in (KNOBS, MIP03):
        patched, matched = patch_panel((aircraft / relative).read_text(), relative, targets)
        staged[relative] = patched
        motions[relative] = matched
    vertices, indices, commands, clicks = click_geometry(source, targets)
    used_commands = {c for row in clicks for c in row['commands']}
    if used_commands != set(GRVTY_COMMANDS):
        raise ValueError(f'Click targets do not cover the native command set: {sorted(used_commands)}')
    staged[CONTROLS] = augment_controls((aircraft / CONTROLS).read_text(), vertices, indices, commands)
    destination.mkdir(parents=True)
    files = []
    for relative, text in staged.items():
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text)
        files.append(dict(path=relative, before_sha256=sha(aircraft / relative), after_sha256=sha(target),
                          changed=sha(aircraft / relative) != sha(target)))
    report = dict(format=1, aircraft=str(aircraft), source=str(source), source_sha256=sha(source),
                  marker=MARKER, files=files,
                  motions={relative: rows for relative, rows in motions.items()},
                  source_targets=[{k: v for k, v in t.items() if k != 'points'} for t in targets],
                  click_targets=clicks, commands=list(GRVTY_COMMANDS),
                  datarefs_used=sorted(used_refs),
                  visible_vertex_index_uv_payload_preserved=True, controls_append_only=True,
                  simulator_tested=False)
    (destination / 'staging-report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--aircraft', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--stage', type=Path, required=True)
    args = parser.parse_args()
    report = stage(args.aircraft.resolve(), args.source.resolve(), args.stage.resolve())
    print(json.dumps({'stage': str(args.stage.resolve()), 'files': report['files'],
                      'motions': {k: [(m['name'], m['indices']) for m in v] for k, v in report['motions'].items()},
                      'click_targets': [c['name'] for c in report['click_targets']]}, indent=2))


if __name__ == '__main__':
    main()
