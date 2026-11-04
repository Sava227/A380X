#!/usr/bin/env python3
"""Stage two continuous FCU LCD faces without rebuilding cockpit input meshes.

Only the two retained LCD quads are expanded to their authored opening bounds.
The two redundant half-face draws are removed. Unused vertex/index slots are
deliberately retained so every other control, animation and draw keeps its ID.
This tool never launches X-Plane or writes into the installed aircraft.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re

from avionics_panel_layout import FCU_PANEL_RECT, PANEL_SIZE
from build_cockpit_controls import FCU_LEGACY_SCREEN_QUADS, FCU_SCREEN_QUADS
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

MARKER = '# A380X FCU SHARED DISPLAYS V1 SPD+HDG ALT+VS'
CONTROLS = 'objects/interior/cockpit-controls/a380x_cockpit_controls.obj'
REVIEWED_FILES = (
    CONTROLS,
    'objects/interior/cockpit-panels/panel_a380_cockpit_mip_knob01_4k.obj',
    'objects/interior/cockpit-panels/decal_a380x_cockpit_decal_emis.obj',
    'cockpit_3d/-PANELS-/Panel.png',
    'cockpit_3d/fcu_labels.png',
    'cockpit_3d/fcu_labels_LIT.png',
)


def quad_vertices(bounds):
    x0, x1, u0, u1 = bounds
    height = math.hypot(3.0021 - 2.9828, 32.7603 - 32.7532)
    cy = (2.9828 + 3.0021) / 2 + .347 * .002
    cz = (32.7532 + 32.7603) / 2 - .938 * .002
    left, bottom, right, top = FCU_PANEL_RECT
    low_u = (left + u1 * (right-left)) / PANEL_SIZE[0]
    high_u = (left + u0 * (right-left)) / PANEL_SIZE[0]
    return (
        (x0, cy-.938*height/2, cz-.347*height/2, 0, .347, -.938, low_u, bottom/PANEL_SIZE[1]),
        (x1, cy-.938*height/2, cz-.347*height/2, 0, .347, -.938, high_u, bottom/PANEL_SIZE[1]),
        (x1, cy+.938*height/2, cz+.347*height/2, 0, .347, -.938, high_u, top/PANEL_SIZE[1]),
        (x0, cy+.938*height/2, cz+.347*height/2, 0, .347, -.938, low_u, top/PANEL_SIZE[1]),
    )


def patch_shared_displays(original):
    lines = original.splitlines(keepends=True)
    vertices = [(at, tuple(map(float, line.split()[1:])))
                for at, line in enumerate(lines) if line.startswith('VT\t')]
    indices = [int(v) for line in lines if line.startswith(('IDX\t', 'IDX10\t'))
               for v in line.split()[1:]]
    draws = [(at, *map(int, line.split()[1:])) for at, line in enumerate(lines)
             if line.startswith('TRIS\t')]

    def locate(bounds):
        points = quad_vertices(bounds)
        matches = [i for i, (_, v) in enumerate(vertices[:-3])
                   if abs(v[0]-points[0][0]) < 2e-6
                   and all(max(abs(a-b) for a, b in zip(vertices[i+c][1], p)) < 2e-6
                           for c, p in enumerate(points))]
        if len(matches) != 1:
            raise ValueError(f'FCU face/UV contract mismatch: {bounds}: {matches}')
        base = matches[0]
        expected = [base, base+1, base+2, base, base+2, base+3]
        face_draws = [(at, start) for at, start, count in draws
                      if count == 6 and indices[start:start+6] == expected]
        if len(face_draws) != 1:
            raise ValueError('FCU LCD must have exactly one independent draw')
        used = set(expected)
        if any(used.intersection(indices[start:start+count]) for at, start, count in draws
               if at != face_draws[0][0]):
            raise ValueError('FCU LCD vertices are shared with another cockpit draw')
        return base, face_draws[0][0]

    if MARKER in original:
        for bounds in FCU_SCREEN_QUADS:
            locate(bounds)
        return original, dict(shared_screens=2, already_applied=True)

    legacy = [locate(bounds) for bounds in FCU_LEGACY_SCREEN_QUADS]
    allowed_vertex_lines = set()
    removed_draw_lines = set()
    for pair, bounds in enumerate(FCU_SCREEN_QUADS):
        base, draw_line = legacy[pair*2]
        for corner, values in enumerate(quad_vertices(bounds)):
            at = vertices[base+corner][0]
            allowed_vertex_lines.add(at)
            lines[at] = 'VT\t' + '\t'.join(f'{obj8_fmt(v)}' for v in values) + '\n'
        removed = legacy[pair*2+1][1]
        removed_draw_lines.add(removed)
        lines[removed] = ''
        if pair == 0:
            lines[draw_line] = MARKER + '\n' + lines[draw_line]

    unchanged = all(a == b for at, (a, b) in enumerate(zip(original.splitlines(keepends=True), lines))
                    if at not in allowed_vertex_lines | removed_draw_lines | {legacy[0][1]})
    if not unchanged:
        raise ValueError('Non-FCU geometry or interaction changed')
    changed = ''.join(lines)
    # Re-enter the strict two-face validation on the actual output.
    patch_shared_displays(changed)
    return changed, dict(shared_screens=2, fields=[['SPD', 'HDG'], ['ALT', 'V/S']],
                         removed_half_face_draws=2, preserved_index_ids=True,
                         preserved_all_manipulators_and_animations=True,
                         preserved_non_fcu_payload=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--aircraft', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    aircraft, output = args.aircraft.resolve(), args.output.resolve()
    if output.is_relative_to(aircraft):
        parser.error('Stage outside the installed aircraft')
    report = dict(scope='Offline shared FCU LCD surfaces; aircraft untouched', files=[])
    for relative in REVIEWED_FILES:
        installed = aircraft / relative
        before = installed.read_bytes()
        after, checks = before, dict(unchanged=True)
        if relative == CONTROLS:
            text, checks = patch_shared_displays(before.decode())
            after = text.encode()
        staged = output / 'aircraft' / relative
        staged.parent.mkdir(parents=True, exist_ok=True)
        staged.write_bytes(after)
        report['files'].append(dict(installed=str(installed), staged=str(staged), relative=relative,
            before_sha256=hashlib.sha256(before).hexdigest(), after_sha256=hashlib.sha256(after).hexdigest(),
            checks=checks))
    (output / 'staging-report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(files=len(report['files']), report=str(output / 'staging-report.json'))))


if __name__ == '__main__':
    main()
