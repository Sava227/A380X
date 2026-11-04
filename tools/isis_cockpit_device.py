#!/usr/bin/env python3
"""Append the ISIS 1 cockpit device and its physical controls to the INSTALLED cockpit objects.

Nothing existing is regenerated, re-ordered or rewritten: vertices/indices are appended after
the last VT/IDX rows (POINT_COUNTS updated) and the draw commands after a MARKER, exactly like
pfd_cockpit_devices.py / sd_cockpit_device.py. Re-running is a no-op once the MARKER exists.

* a380x_mfd_screens.obj: one visible full-UV quad on the authored SCREEN_ISIS_1 face
  (ATTR_cockpit_device a380x.xp12.isis.1, lighting channel 21), hidden by
  a380x/xp12/isis/device_active while the plugin device is unavailable (the powered-off
  dark face underneath then stays visible).
* a380x_cockpit_controls.obj: the draw-disabled touch twin (ATTR_manip_device: click toggles
  the pop-up) and 12-triangle hit boxes over the authored PUSH_ISIS_1_* / KNOB_ISIS_1_SET_SEL
  meshes with ATTR_manip_command / ATTR_manip_command_knob.

Geometry comes from the pinned FBW cockpit glTF (source coordinates before the aircraft
attachment's 180-degree rotation; +X is the crew's left). `python3 tools/isis_cockpit_device.py
--check <A380_COCKPIT_LOD00.gltf>` re-measures every constant below from the glTF.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

from avionics_panel_layout import ISIS_SCREEN_SURFACE, ISIS2_SCREEN_SURFACE
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

ISIS_DEVICE = 'a380x.xp12.isis.1'
ISIS2_DEVICE = 'a380x.xp12.isis.2'
ISIS_DEVICES = (ISIS_DEVICE, ISIS2_DEVICE)
ISIS_CHANNEL = 21
ISIS_CHANNELS = {ISIS_DEVICE: 21, ISIS2_DEVICE: 22}
ISIS_PREFIXES = ('a380x/xp12/isis/', 'a380x/xp12/isis2/')
ISIS_COMMAND_NAMES = ('mode', 'ls', 'plus', 'minus', 'menu', 'set_push', 'set_up', 'set_down', 'popup')
ISIS_DATAREF_NAMES = ('powered', 'state', 'page', 'device_active', 'device_bound', 'draw_count', 'brightness',
                      'ls_active', 'baro_std', 'baro_unit_inhg', 'metric_altitude', 'init_seconds', 'fail_att',
                      'role', 'mode_pressed', 'ls_pressed', 'plus_pressed', 'minus_pressed', 'menu_pressed',
                      'set_sel_axial', 'set_sel_rotation')
ISIS_COMMANDS = tuple(prefix + name for prefix in ISIS_PREFIXES for name in ISIS_COMMAND_NAMES)
ISIS_DATAREFS = tuple(prefix + name for prefix in ISIS_PREFIXES for name in ISIS_DATAREF_NAMES)
ISIS_ACTIVE_DATAREF = 'a380x/xp12/isis/device_active'
MARKER = '# A380X NATIVE ISIS 1 V1'
MARKER2 = '# A380X NATIVE ISIS 2 V1'
SCREEN_LIFT = 0.002   # visible quad in front of the authored face (the ISIS frame lip is ~8 mm deep)
TOUCH_LIFT = 0.003

# Axis-aligned bounds (min xyz, max xyz) of the authored meshes, node + legend children.
# The knob push is the centre cap: only its central 55 % (x/y) is a push target so the
# rest of the knob face keeps the rotate (click left/right, mouse wheel) manipulator.
CONTROL_NODES = (
    ('PUSH_ISIS_1_MODE', 'command', ('a380x/xp12/isis/mode',), 'ISIS 1 - MODE (swap SFD / SND)'),
    ('PUSH_ISIS_1_LS_DIR', 'command', ('a380x/xp12/isis/ls',), 'ISIS - LS (localizer / glideslope scales)'),
    ('PUSH_ISIS_1_PLUS', 'command', ('a380x/xp12/isis/plus',), 'ISIS - + (brightness / previous)'),
    ('PUSH_ISIS_1_MINUS', 'command', ('a380x/xp12/isis/minus',), 'ISIS - - (brightness / next)'),
    ('PUSH_ISIS_1_MENU', 'command', ('a380x/xp12/isis/menu',), 'ISIS - MENU (hold 2 s: ATT RST)'),
    ('PUSH_ISIS_1_SET_SEL', 'command', ('a380x/xp12/isis/set_push',), 'ISIS - SET/SEL push (baro STD / select)'),
    ('KNOB_ISIS_1_SET_SEL', 'knob', ('a380x/xp12/isis/set_up', 'a380x/xp12/isis/set_down'),
     'ISIS - SET/SEL (baro / bugs / menu)'),
)
PUSH_CORE = 0.55
CONTROL_BOUNDS = {
    'PUSH_ISIS_1_MODE': ((0.16587, 2.79541, 32.9586), (0.18199, 2.80156, 32.96332)),
    'PUSH_ISIS_1_LS_DIR': ((0.13651, 2.79534, 32.9586), (0.15263, 2.80156, 32.9635)),
    'PUSH_ISIS_1_PLUS': ((0.19402, 2.76918, 32.94911), (0.19955, 2.78514, 32.95752)),
    'PUSH_ISIS_1_MINUS': ((0.19405, 2.74179, 32.93916), (0.19957, 2.75785, 32.94765)),
    'PUSH_ISIS_1_MENU': ((0.16551, 2.72522, 32.93306), (0.18175, 2.73129, 32.93755)),
    'PUSH_ISIS_1_SET_SEL': ((0.1233, 2.73654, 32.91499), (0.13556, 2.74847, 32.92238)),
    'KNOB_ISIS_1_SET_SEL': ((0.12172, 2.72878, 32.916), (0.13715, 2.74998, 32.93956)),
}
# ISIS 2 (lower unit). Its "-" legend PUSH_ISIS_2_MINUS_SEQ1 is a root node in the FBW model.
CONTROL_BOUNDS_2 = {
    'PUSH_ISIS_2_MODE': ((0.16596, 2.7087, 32.92706), (0.18208, 2.71476, 32.93155)),
    'PUSH_ISIS_2_LS_DIR': ((0.1366, 2.70861, 32.92705), (0.15272, 2.71476, 32.93178)),
    'PUSH_ISIS_2_PLUS': ((0.19411, 2.68249, 32.91757), (0.19965, 2.69834, 32.92568)),
    'PUSH_ISIS_2_MINUS': ((0.19415, 2.6552, 32.90762), (0.19966, 2.67105, 32.91553)),
    'PUSH_ISIS_2_MENU': ((0.1656, 2.63853, 32.90152), (0.18184, 2.64449, 32.90572)),
    'PUSH_ISIS_2_SET_SEL': ((0.12345, 2.64989, 32.88299), (0.13571, 2.66183, 32.89037)),
    'KNOB_ISIS_2_SET_SEL': ((0.12186, 2.64214, 32.884), (0.1373, 2.66334, 32.90757)),
}
CONTROL_NODES_2 = (
    ('PUSH_ISIS_2_MODE', 'command', ('a380x/xp12/isis2/mode',), 'ISIS 2 - MODE (swap SFD / SND)'),
    ('PUSH_ISIS_2_LS_DIR', 'command', ('a380x/xp12/isis2/ls',), 'ISIS 2 - LS (localizer / glideslope scales, SFD)'),
    ('PUSH_ISIS_2_PLUS', 'command', ('a380x/xp12/isis2/plus',), 'ISIS 2 - + (brightness / previous)'),
    ('PUSH_ISIS_2_MINUS', 'command', ('a380x/xp12/isis2/minus',), 'ISIS 2 - - (brightness / next)'),
    ('PUSH_ISIS_2_MENU', 'command', ('a380x/xp12/isis2/menu',), 'ISIS 2 - MENU (SFD; hold 2 s: ATT RST)'),
    ('PUSH_ISIS_2_SET_SEL', 'command', ('a380x/xp12/isis2/set_push',), 'ISIS 2 - SET/SEL push (baro STD / select, SFD)'),
    ('KNOB_ISIS_2_SET_SEL', 'knob', ('a380x/xp12/isis2/set_up', 'a380x/xp12/isis2/set_down'),
     'ISIS 2 - SET/SEL (baro / bugs / menu, SFD)'),
)
UNITS = {
    1: dict(device=ISIS_DEVICE, channel=21, marker=MARKER, surface=ISIS_SCREEN_SURFACE, bounds=CONTROL_BOUNDS,
            nodes=None, active='a380x/xp12/isis/device_active', label='A380X ISIS 1'),
    2: dict(device=ISIS2_DEVICE, channel=22, marker=MARKER2, surface=ISIS2_SCREEN_SURFACE, bounds=CONTROL_BOUNDS_2,
            nodes=CONTROL_NODES_2, active='a380x/xp12/isis2/device_active', label='A380X ISIS 2'),
}
# ISIS 1 V1 was installed when MODE opened the FBW BUGS page; MODE now swaps SFD / SND.
STALE_LABELS = (('ISIS - MODE (bugs page)', 'ISIS 1 - MODE (swap SFD / SND)'),)


def _normal(corners):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = corners[0], corners[1], corners[2]
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    # Point toward the crew (-Z, the seated crew looks toward +Z).
    sign = -1.0 if nz > 0 else 1.0
    return (sign * nx / length, sign * ny / length, sign * nz / length)


def screen_quad(lift, surface=ISIS_SCREEN_SURFACE):
    """Seated-view BL, TL, TR, BR corners (UV 0,0 / 0,1 / 1,1 / 1,0) lifted along the normal."""
    x0, x1, y0, z0, y1, z1 = surface
    corners = ((x1, y0, z0), (x1, y1, z1), (x0, y1, z1), (x0, y0, z0))
    normal = _normal(corners)
    rows = []
    for point, uv in zip(corners, ((0, 0), (0, 1), (1, 1), (1, 0))):
        vertex = (*(point[i] + normal[i] * lift for i in range(3)), *normal, *uv)
        rows.append('VT\t' + '\t'.join(f'{obj8_fmt(v)}' for v in vertex))
    return rows


def _box_rows(lo, hi):
    x, y, z = lo
    X, Y, Z = hi
    box = ((x, y, z), (X, y, z), (X, Y, z), (x, Y, z), (x, y, Z), (X, y, Z), (X, Y, Z), (x, Y, Z))
    faces = (((0, 3, 2, 1), (0, 0, -1)), ((4, 5, 6, 7), (0, 0, 1)), ((0, 1, 5, 4), (0, -1, 0)),
             ((3, 7, 6, 2), (0, 1, 0)), ((0, 4, 7, 3), (-1, 0, 0)), ((1, 2, 6, 5), (1, 0, 0)))
    vertices, indices = [], []
    for face, normal in faces:
        base = len(vertices)
        vertices.extend('VT\t' + '\t'.join(f'{obj8_fmt(v)}' for v in (*box[k], *normal, 0, 0)) for k in face)
        indices.extend(base + k for k in (0, 2, 1, 0, 3, 2))  # OBJ8 clockwise fronts (sd_cockpit_device)
    return vertices, indices


def control_bounds(node):
    lo, hi = {**CONTROL_BOUNDS, **CONTROL_BOUNDS_2}[node]
    if node.endswith('_SET_SEL') and node.startswith('PUSH_'):
        centre = [(a + b) / 2 for a, b in zip(lo, hi)]
        half = [(b - a) / 2 * PUSH_CORE for a, b in zip(lo, hi)]
        lo = (centre[0] - half[0], centre[1] - half[1], lo[2])
        hi = (centre[0] + half[0], centre[1] + half[1], hi[2])
    return lo, hi


def _append(text, vertices, indices, commands, marker=MARKER):
    lines = text.splitlines()
    row = next(i for i, line in enumerate(lines) if line.startswith('POINT_COUNTS'))
    count = list(map(int, lines[row].split()[1:]))
    if len(count) != 4:
        raise ValueError('Invalid OBJ point counts')
    previous_vertices = sum(line.startswith('VT\t') for line in lines)
    previous_indices = sum(len(line.split()) - 1 for line in lines if re.match(r'^IDX(?:10)?\s', line))
    if previous_vertices != count[0] or previous_indices != count[3]:
        raise ValueError('Existing cockpit geometry is inconsistent; refusing to rewrite it')
    at = max(i for i, line in enumerate(lines) if line.startswith('VT\t')) + 1
    lines[at:at] = vertices
    at = max(i for i, line in enumerate(lines) if re.match(r'^IDX(?:10)?\s', line)) + 1
    lines[at:at] = ['IDX\t' + str(count[0] + n) for n in indices]
    lines[row] = 'POINT_COUNTS\t' + '\t'.join(map(str, (count[0] + len(vertices), count[1], count[2],
                                                        count[3] + len(indices))))
    adjusted = []
    for line in commands:
        words = line.split()
        if words and words[0] == 'TRIS':
            line = f'TRIS\t{int(words[1]) + count[3]}\t{words[2]}'
        adjusted.append(line)
    return '\n'.join([*lines, '', marker, *adjusted, ''])


def augment_screen(text, unit=1):
    """Visible ISIS quad in a380x_mfd_screens.obj (no manipulators in that object)."""
    spec = UNITS[unit]
    if spec['marker'] in text:
        return text
    return _append(text, screen_quad(SCREEN_LIFT, spec['surface']), [0, 1, 2, 0, 2, 3], [
        'ATTR_no_cockpit', 'ATTR_poly_os\t2', 'ATTR_draw_enable',
        'ANIM_begin', f'ANIM_hide\t-0.5\t0.5\t{spec["active"]}',
        f'ATTR_cockpit_device\t{spec["device"]}\t1\t{spec["channel"]}\t1',
        'TRIS\t0\t6', 'ATTR_no_cockpit', 'ANIM_end', 'ATTR_poly_os\t0'], spec['marker'])


def augment_controls(text, unit=1):
    """Touch twin + physical ISIS controls in a380x_cockpit_controls.obj."""
    spec = UNITS[unit]
    if spec['marker'] in text:
        for old, new in STALE_LABELS:  # in-place tooltip upgrade of the installed ISIS 1 V1 block
            text = text.replace('\t' + old + '\n', '\t' + new + '\n')
        return text
    vertices, indices = screen_quad(TOUCH_LIFT, spec['surface']), [0, 1, 2, 0, 2, 3]
    tooltip = 'A380X ISIS - click to open/close the pop-up' if unit == 1 else f'{spec["label"]} - click to open/close the pop-up'
    commands = ['ATTR_manip_none', 'ATTR_no_cockpit', 'ATTR_poly_os\t2', 'ATTR_draw_disable',
                'ANIM_begin', f'ANIM_hide\t-0.5\t0.5\t{spec["active"]}',
                f'ATTR_cockpit_device\t{spec["device"]}\t1\t{spec["channel"]}\t1',
                f'ATTR_manip_device\thand\t{spec["device"]}\t{tooltip}',
                'TRIS\t0\t6', 'ATTR_manip_none', 'ATTR_no_cockpit', 'ANIM_end', 'ATTR_poly_os\t0']
    for node, kind, names, label in spec['nodes'] or CONTROL_NODES:
        box_vertices, box_indices = _box_rows(*control_bounds(node))
        start = len(indices)
        indices.extend(len(vertices) + i for i in box_indices)
        vertices.extend(box_vertices)
        if kind == 'knob':
            commands.append(f'ATTR_manip_command_knob\trotate_medium\t{names[0]}\t{names[1]}\t{label}')
        else:
            commands.append(f'ATTR_manip_command\tbutton\t{names[0]}\t{label}')  # pushbutton: pointing finger
        commands.extend([f'TRIS\t{start}\t36', 'ATTR_manip_none'])
    commands.append('ATTR_draw_enable')
    return _append(text, vertices, indices, commands, spec['marker'])


def augment_screens_all(text):
    return augment_screen(augment_screen(text, 1), 2)


def augment_controls_all(text):
    return augment_controls(augment_controls(text, 1), 2)


def measure(source: Path) -> dict:
    """Re-measure the ISIS 1/2 screens and control bounds from the authored glTF."""
    sys.dont_write_bytecode = True
    import build_cockpit_controls as bcc
    document = json.loads(source.read_text(encoding='utf-8'))
    buffers = [bcc.load_buffer(source, buffer['uri']) for buffer in document['buffers']]
    nodes = document['nodes']
    names = {node.get('name'): index for index, node in enumerate(nodes) if node.get('name')}
    parents = {child: index for index, node in enumerate(nodes) for child in node.get('children', [])}
    materials = {material['name']: index for index, material in enumerate(document['materials'])}
    cache = {}

    def world(index):
        if index not in cache:
            local = bcc.local_matrix(nodes[index])
            cache[index] = bcc.matrix_multiply(world(parents[index]), local) if index in parents else local
        return cache[index]

    def points(index, material=None):
        node = nodes[index]
        out = []
        for primitive in document['meshes'][node['mesh']]['primitives'] if 'mesh' in node else ():
            if material is not None and primitive.get('material') != materials[material]:
                continue
            positions = bcc.accessor_values(document, buffers, primitive['attributes']['POSITION'])
            used = range(len(positions))
            if material is not None:
                # MSFS shares one vertex/index buffer between primitives; ASOBO_primitive gives the range.
                extras = primitive.get('extras', {}).get('ASOBO_primitive', {})
                base = extras.get('BaseVertexIndex', 0)
                used = range(base, base + extras.get('VertexCount', len(positions) - base))
            out.extend(bcc.transform_point(world(index), positions[i]) for i in used)
        return out

    pfd = points(names['SCREEN_DU_PFDL'])  # method proof: must reproduce PFD_SCREEN_SURFACES[0]
    result = {'pfd': (min(p[0] for p in pfd), max(p[0] for p in pfd), min(p[1] for p in pfd),
                      min(p[2] for p in pfd), max(p[1] for p in pfd), max(p[2] for p in pfd))}
    for key, material in (('screen', 'SCREEN_ISIS_1'), ('screen2', 'SCREEN_ISIS_2')):
        screen = points(names['DYN_SCREEN'], material)
        result[key] = (min(p[0] for p in screen), max(p[0] for p in screen), min(p[1] for p in screen),
                       min(p[2] for p in screen), max(p[1] for p in screen), max(p[2] for p in screen))
    for node, *_ in (*CONTROL_NODES, *CONTROL_NODES_2):
        index = names[node]
        found = points(index)
        if node.startswith('PUSH_'):
            for child in nodes[index].get('children', []):
                found += points(child)
        if node == 'PUSH_ISIS_2_MINUS':
            found += points(names['PUSH_ISIS_2_MINUS_SEQ1'])
        result[node] = (tuple(min(p[k] for p in found) for k in range(3)),
                        tuple(max(p[k] for p in found) for k in range(3)))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', type=Path, required=True, help='A380_COCKPIT_LOD00.gltf')
    args = parser.parse_args()
    measured = measure(args.check)
    errors = []
    from avionics_panel_layout import PFD_SCREEN_SURFACES
    if max(abs(a - b) for a, b in zip(measured['pfd'], PFD_SCREEN_SURFACES[0])) > 5e-6:
        errors.append(f'method does not reproduce SCREEN_DU_PFDL: {measured["pfd"]}')
    if max(abs(a - b) for a, b in zip(measured['screen'], ISIS_SCREEN_SURFACE)) > 5e-6:
        errors.append(f'screen {measured["screen"]} != {ISIS_SCREEN_SURFACE}')
    if max(abs(a - b) for a, b in zip(measured['screen2'], ISIS2_SCREEN_SURFACE)) > 5e-6:
        errors.append(f'screen2 {measured["screen2"]} != {ISIS2_SCREEN_SURFACE}')
    for node, (lo, hi) in {**CONTROL_BOUNDS, **CONTROL_BOUNDS_2}.items():
        got = measured[node]
        if max(abs(a - b) for a, b in zip((*lo, *hi), (*got[0], *got[1]))) > 1e-5 + 5e-6:
            errors.append(f'{node} {got} != {(lo, hi)}')
    print(json.dumps({'passed': not errors, 'errors': errors, 'measured': measured}, indent=2))
    sys.exit(1 if errors else 0)


if __name__ == '__main__':
    main()
