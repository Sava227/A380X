"""Cockpit geometry of the web EFB tablets (docs/EFB_DEVELOPMENT.md).

The two tablets become X-Plane plugin cockpit devices a380x.xp12.efb.captain / .first_officer
(1440 x 1000 texels, twice the old 720 x 500 panel-atlas region and the same aspect). This module:

* retires the two panel-atlas EFB faces (cockpit-region TRIS sampling EFB_PANEL_RECT) and the 34
  legacy atlas touch manipulators (ten page tabs, six actions and power per tablet) of the
  cockpit-controls object, the way retire_display_overlays.py retired the PFD/ND faces;
* appends one draw-disabled ATTR_manip_device touch quad per tablet to the cockpit-controls object
  and one visible ATTR_cockpit_device quad per tablet to the screen object (a380x_mfd_screens.obj).

The quads use the measured EFB_SCREEN_SURFACES corners (bottom-left, top-left, top-right, bottom-right
as seen on each tablet) with UV (0,0)..(1,1); device texel (x, y) = (u * 1440, v * 1000), origin
bottom-left, exactly like the PFD/ND/MFD devices. Both edits are idempotent (MARKER) and never shift
an existing vertex, index or draw: new vertices and indices are appended.

The a380x/xp12/efb/... commands themselves stay registered for keyboard/joystick bindings.
"""
from __future__ import annotations

import re

from avionics_panel_layout import EFB_PANEL_RECT, EFB_SCREEN_SURFACES, PANEL_SIZE
from build_cockpit_controls import mfd_surface_normal
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

EFB_DEVICES = ('a380x.xp12.efb.captain', 'a380x.xp12.efb.first_officer')
EFB_CHANNELS = (24, 25)          # instrument_brightness_ratio slots (14..23 are taken)
EFB_DEVICE_SIZE = (1440, 1000)
EFB_COMMANDS = ('a380x/xp12/efb/slides_arm_all', 'a380x/xp12/efb/slides_disarm_all',
                'a380x/xp12/efb/captain/popup', 'a380x/xp12/efb/first_officer/popup')
EFB_DATAREFS = tuple('a380x/xp12/efb/' + name for name in (
    'slide_armed', 'slides_armed_count', 'doors_open_count', 'devices_active',
    'captain/device_bound', 'first_officer/device_bound', 'captain/web_live', 'first_officer/web_live',
    'actions_processed', 'actions_rejected', 'touch_events'))
MARKER = '# A380X NATIVE EFB DEVICE QUADS V1'
RETIRED = '# A380X EFB DEVICE V1 RETIRED '
VISIBLE_LIFT = .003              # the retired atlas face's offset from the authored glass
TOUCH_LIFT = .004                # 1 mm in front of the visible screen (MFD convention)
LEGACY_TOUCH = re.compile(r'^ATTR_manip_command\tbutton\ta380x/xp12/efb/(page/[a-z]+|action_[1-6]|power)\t')
LEGACY_TOUCH_COUNT = 34
SEATS = ('CAPT', 'F/O')


def _geometry(lines):
    vertices, indices = [], []
    for line in lines:
        words = line.split()
        if not words:
            continue
        if words[0] == 'VT':
            vertices.append(tuple(map(float, words[1:])))
        elif words[0] in ('IDX', 'IDX10'):
            indices.extend(map(int, words[1:]))
    return vertices, indices


def _retire_atlas_faces(lines):
    """Comment out the cockpit-region TRIS whose UVs cover exactly the EFB atlas rectangle."""
    vertices, indices = _geometry(lines)
    region, retired = False, 0
    for k, line in enumerate(lines):
        words = line.split()
        if words and words[0] == 'ATTR_cockpit_region':
            region = True
        elif words and words[0] in ('ATTR_no_cockpit', 'ATTR_cockpit_device'):
            region = False
        if region and words and words[0] == 'TRIS':
            start, count = map(int, words[1:3])
            uvs = [vertices[i][6:8] for i in indices[start:start + count]]
            if count == 6 and uvs:
                bounds = (min(u for u, _ in uvs) * PANEL_SIZE[0], min(v for _, v in uvs) * PANEL_SIZE[1],
                          max(u for u, _ in uvs) * PANEL_SIZE[0], max(v for _, v in uvs) * PANEL_SIZE[1])
                if all(abs(a - b) < .01 for a, b in zip(bounds, EFB_PANEL_RECT)):
                    lines[k] = RETIRED + 'atlas EFB face (the a380x.xp12.efb device owns this screen): ' + line
                    retired += 1
    return retired


def _retire_legacy_touch(lines):
    """Comment out each legacy tablet manipulator and its TRIS (an invisible, inert quad is left out)."""
    retired = []
    for k, line in enumerate(lines):
        if LEGACY_TOUCH.match(line):
            if k + 1 >= len(lines) or not lines[k + 1].startswith('TRIS\t'):
                raise ValueError('Legacy EFB manipulator without its TRIS: ' + line)
            retired.append(line.split('\t')[2])
            lines[k] = RETIRED + line
            lines[k + 1] = RETIRED + lines[k + 1]
    return retired


def device_quads(lift):
    """(side, device, four VT rows BL/TL/TR/BR with normal and UV) for both tablets."""
    quads = []
    for side, corners in enumerate(EFB_SCREEN_SURFACES):
        normal = mfd_surface_normal(corners)
        rows = []
        for point, uv in zip(corners, ((0, 0), (0, 1), (1, 1), (1, 0))):
            rows.append((*(point[i] + normal[i] * lift for i in range(3)), *normal, *uv))
        quads.append((side, EFB_DEVICES[side], rows))
    return quads


def _append_quads(lines, touch):
    row = next(i for i, line in enumerate(lines) if line.startswith('POINT_COUNTS'))
    counts = list(map(int, lines[row].split()[1:]))
    vertices, indices, draws = [], [], []
    for side, device, rows in device_quads(TOUCH_LIFT if touch else VISIBLE_LIFT):
        base = counts[0] + len(vertices)
        draws.append((side, device, counts[3] + len(indices)))
        vertices.extend('VT\t' + '\t'.join(obj8_fmt(v) for v in vertex) for vertex in rows)
        indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))
    lines[row] = 'POINT_COUNTS\t' + '\t'.join(map(str, (counts[0] + len(vertices), counts[1], counts[2],
                                                         counts[3] + len(indices))))
    at = max(i for i, line in enumerate(lines) if line.startswith('VT\t')) + 1
    lines[at:at] = vertices
    at = max(i for i, line in enumerate(lines) if re.match(r'^IDX(?:10)?\s', line)) + 1
    lines[at:at] = ['IDX\t' + str(n) for n in indices]
    # The visible screen object must carry no manipulator attribute at all (validate_aircraft.py).
    lines.extend(['', MARKER, *(['ATTR_manip_none'] if touch else []), 'ATTR_no_cockpit', 'ATTR_poly_os\t2',
                  'ATTR_draw_disable' if touch else 'ATTR_draw_enable'])
    for side, device, start in draws:
        lines.extend(['ANIM_begin', f'ATTR_cockpit_device\t{device}\t1\t{EFB_CHANNELS[side]}\t1'])
        if touch:
            lines.append(f'ATTR_manip_device\tbutton\t{device}\tA380X EFB {SEATS[side]} - tap and drag')
        lines.append(f'TRIS\t{start}\t6')
        if touch:
            lines.append('ATTR_manip_none')
        lines.extend(['ATTR_no_cockpit', 'ANIM_end'])
    lines.extend(['ATTR_draw_enable', 'ATTR_poly_os\t0', ''])


def augment_controls(text):
    """Cockpit-controls object: retire the atlas tablet, add the two touch quads. Returns (text, stats)."""
    if MARKER in text:
        return text, {'already_applied': True}
    lines = text.splitlines()
    faces = _retire_atlas_faces(lines)
    touch = _retire_legacy_touch(lines)
    if faces != 2:
        raise ValueError(f'Expected the two panel-atlas EFB faces, found {faces}')
    if len(touch) != LEGACY_TOUCH_COUNT:
        raise ValueError(f'Expected {LEGACY_TOUCH_COUNT} legacy EFB touch regions, found {len(touch)}')
    _append_quads(lines, True)
    return '\n'.join(lines), {'retired_atlas_faces': faces, 'retired_touch_regions': len(touch), 'touch_quads': 2}


def augment_screen(text):
    """Visible screen object: one full-UV device quad per tablet. Returns (text, stats)."""
    if MARKER in text:
        return text, {'already_applied': True}
    lines = text.splitlines()
    _append_quads(lines, False)
    return '\n'.join(lines), {'visible_quads': 2}


def check(controls_text, screen_text):
    """Problems of an installed or staged pair (empty when consistent)."""
    problems = []
    if MARKER not in controls_text or MARKER not in screen_text:
        return ['EFB device quads missing']
    for device in EFB_DEVICES:
        if controls_text.count(f'ATTR_manip_device\tbutton\t{device}\t') != 1:
            problems.append('expected one touch quad for ' + device)
        if screen_text.count(f'ATTR_cockpit_device\t{device}\t') != 1:
            problems.append('expected one visible quad for ' + device)
    live = [line for line in controls_text.splitlines() if LEGACY_TOUCH.match(line)]
    if live:
        problems.append(f'{len(live)} legacy EFB touch regions still active')
    if 'ATTR_manip_' in screen_text:
        problems.append('visible screen object carries manipulators')
    return problems
