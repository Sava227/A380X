#!/usr/bin/env python3
"""Build invisible X-Plane click meshes from named FBW cockpit controls.

The source model already contains accurate button/switch geometry and painted
legends. This tool reuses only the named control meshes as invisible mouse
targets, adds a native X-Plane command, and exposes the configured label as the
hover tooltip.
FCU engagement bars are fitted to the authored button fronts; glareshield and
gear/RTO lamps reuse authored emissive faces. All use native state.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import re
import struct
import zlib
from dataclasses import dataclass, replace
from pathlib import Path
from urllib.parse import unquote_to_bytes

from avionics_provider import ADIRS_COMMANDS, CENTER_RIGHT_COMMANDS, CLOCK_COMMANDS, DU_COMMANDS, EFIS_COMMANDS, FCU_COMMANDS, GLARESHIELD_COMMANDS
from avionics_provider import KCCU_COMMANDS, MFD_COMMANDS, SD_COMMANDS

from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation
from avionics_panel_layout import (
    CLOCK_CHR_PANEL_RECT,
    CLOCK_UTC_PANEL_RECT,
    CLOCK_ET_PANEL_RECT,
    CLOCK_SCREEN_SURFACES,
    EFB_ACTION_TOUCH_RECTS,
    EFB_PAGE_NAMES,
    EFB_PAGE_TOUCH_RECTS,
    EFB_PANEL_RECT,
    EFB_POWER_TOUCH_RECT,
    EFB_SCREEN_SURFACES,
    EFIS_BARO_PANEL_RECTS,
    EFIS_BARO_SCREEN_SURFACES,
    EFIS_LOWER_PANEL_RECTS,
    EFIS_LOWER_SCREEN_X_BOUNDS,
    EFIS_LOWER_SCREEN_YZ,
    EFIS_UPPER_PANEL_RECTS,
    EFIS_UPPER_SCREEN_X_BOUNDS,
    EFIS_UPPER_SCREEN_YZ,
    EWD_PANEL_RECT,
    EWD_SCREEN_SURFACE,
    FCU_PANEL_RECT,
    MFD_AUTO_ADJUST,
    MFD_DEVICE_BUS,
    MFD_DEVICE_IDS,
    MFD_LIGHTING_CHANNELS,
    MFD_SCREEN_CORNERS,
    ND_PANEL_RECTS,
    ND_SCREEN_SURFACES,
    PANEL_SIZE,
    PFD_PANEL_RECTS,
    PFD_SCREEN_SURFACES,
)


COMPONENTS = {
    5120: ('b', 1),
    5121: ('B', 1),
    5122: ('h', 2),
    5123: ('H', 2),
    5125: ('I', 4),
    5126: ('f', 4),
}
TYPE_LENGTHS = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


# The source material already carries correctly positioned daylight legends.
# Keep the atlas metadata available to the panel builder, but do not overlay
# duplicate legend quads: those obscured the authored labels and appeared
# inverted in X-Plane. The live windows and state-gated engagement bars
# are separate from these unused daylight label cells.
FCU_LABEL_COLUMNS = 8
FCU_LABEL_ROWS = 8
FCU_LABELS = (
    # label, centre x/y/z, width, height
    ('MACH  SPD', 0.0901, 3.0256, 32.7651, 0.030, 0.0052),
    ('TRUE  MAG', 0.0407, 3.0256, 32.7651, 0.030, 0.0052),
    ('HDG  V/S', 0.0000, 3.0190, 32.7626, 0.029, 0.0052),
    ('TRK  FPA', 0.0000, 2.9972, 32.7546, 0.027, 0.0050),
    ('METER', -0.0408, 3.0256, 32.7651, 0.021, 0.0052),
    ('FD', 0.0000, 2.9833, 32.7510, 0.011, 0.0058),
    ('AP1', 0.0101, 2.9632, 32.7435, 0.013, 0.0058),
    ('AP2', -0.0099, 2.9632, 32.7435, 0.013, 0.0058),
    ('A/THR', 0.0000, 2.9443, 32.7364, 0.016, 0.0055),
    ('LOC', 0.0410, 2.9420, 32.7356, 0.013, 0.0055),
    ('ALT', -0.0411, 2.9420, 32.7356, 0.013, 0.0055),
    ('APPR', -0.0906, 2.9420, 32.7356, 0.016, 0.0055),
    ('100', -0.0535, 2.9938, 32.7470, 0.012, 0.0046),
    ('1000', -0.0265, 2.9938, 32.7470, 0.015, 0.0046),
    ('UP', -0.0775, 2.9848, 32.7442, 0.010, 0.0048),
    ('DN', -0.0775, 2.9507, 32.7316, 0.010, 0.0048),
)

# SEQ2 nodes identify the authored white text, not an engagement lamp. Use their
# parent button meshes to place separate green bars above the untouched legends.
FCU_ANNUNCIATORS = (
    ('AP1', 'PUSH_FCU_AP1_SEQ2', 'a380x/xp12/fcu/ap1_light'),
    ('AP2', 'PUSH_FCU_AP2_SEQ2', 'a380x/xp12/fcu/ap2_light'),
    ('FD', 'PUSH_FCU_FD_SEQ2', 'a380x/xp12/fcu/fd_light'),
    ('A/THR', 'PUSH_FCU_ATHR_SEQ2', 'a380x/xp12/fcu/athr_light'),
    ('LOC', 'PUSH_FCU_LOC_SEQ2', 'a380x/xp12/fcu/loc_light'),
    ('ALT', 'PUSH_FCU_ALT_SEQ2', 'a380x/xp12/fcu/altitude_hold_light'),
    ('APPR', 'PUSH_FCU_APPR_SEQ2', 'a380x/xp12/fcu/approach_light'),
)
FCU_ANNUNCIATOR_FIRST_CELL = len(FCU_LABELS)
GLARE_ANNUNCIATORS = (
    ('ATC', 'PUSH_GLARESHIELD_CS_ATCMSG_SEQ1', 'a380x/xp12/glareshield/atc_lit', (255, 180, 45, 255)),
    ('LAND', 'PUSH_GLARESHIELD_CS_AUTOLAND_SEQ1', 'a380x/xp12/glareshield/autoland_lit', (255, 55, 45, 255)),
    ('WARN', 'PUSH_GLARESHIELD_CS_MASTERWARN_SEQ1', 'a380x/xp12/glareshield/master_warning_lit', (255, 55, 45, 255)),
    ('CAUT', 'PUSH_GLARESHIELD_CS_MASTERCAUT_SEQ1', 'a380x/xp12/glareshield/master_caution_lit', (255, 180, 45, 255)),
    ('CAPT', 'PUSH_GLARESHIELD_CS_SIDESTICK_SEQ1', 'a380x/xp12/glareshield/priority_green_left', (65, 255, 95, 255)),
    ('ARROW', 'PUSH_GLARESHIELD_CS_SIDESTICK_SEQ2', 'a380x/xp12/glareshield/priority_arrow_left', (255, 55, 45, 255)),
    ('ATC', 'PUSH_GLARESHIELD_FO_ATCMSG_SEQ1', 'a380x/xp12/glareshield/atc_lit', (255, 180, 45, 255)),
    ('LAND', 'PUSH_GLARESHIELD_FO_AUTOLAND_SEQ1', 'a380x/xp12/glareshield/autoland_lit', (255, 55, 45, 255)),
    ('WARN', 'PUSH_GLARESHIELD_FO_MASTERWARN_SEQ1', 'a380x/xp12/glareshield/master_warning_lit', (255, 55, 45, 255)),
    ('CAUT', 'PUSH_GLARESHIELD_FO_MASTERCAUT_SEQ1', 'a380x/xp12/glareshield/master_caution_lit', (255, 180, 45, 255)),
    ('F/O', 'PUSH_GLARESHIELD_FO_SIDESTICK_SEQ1', 'a380x/xp12/glareshield/priority_green_right', (65, 255, 95, 255)),
    ('ARROW', 'PUSH_GLARESHIELD_FO_SIDESTICK_SEQ2', 'a380x/xp12/glareshield/priority_arrow_right', (255, 55, 45, 255)),
    ('UNLK', 'GEAR_WING_LEFT_RED', 'a380x/xp12/center_right/gear_wing_left_red', (255, 55, 45, 255)),
    ('GEAR', 'GEAR_WING_LEFT_GREEN', 'a380x/xp12/center_right/gear_wing_left_green', (65, 255, 95, 255)),
    ('UNLK', 'GEAR_WING_RIGHT_RED', 'a380x/xp12/center_right/gear_wing_right_red', (255, 55, 45, 255)),
    ('GEAR', 'GEAR_WING_RIGHT_GREEN', 'a380x/xp12/center_right/gear_wing_right_green', (65, 255, 95, 255)),
    ('UNLK', 'PUSH_AUTOBKR_LDGGEAR_2_SEQ1', 'a380x/xp12/center_right/gear_nose_red', (255, 55, 45, 255)),
    ('GEAR', 'PUSH_AUTOBKR_LDGGEAR_2_SEQ2', 'a380x/xp12/center_right/gear_nose_green', (65, 255, 95, 255)),
    ('UNLK', 'GEAR_BODY_LEFT_RED', 'a380x/xp12/center_right/gear_body_left_red', (255, 55, 45, 255)),
    ('GEAR', 'GEAR_BODY_LEFT_GREEN', 'a380x/xp12/center_right/gear_body_left_green', (65, 255, 95, 255)),
    ('UNLK', 'GEAR_BODY_RIGHT_RED', 'a380x/xp12/center_right/gear_body_right_red', (255, 55, 45, 255)),
    ('GEAR', 'GEAR_BODY_RIGHT_GREEN', 'a380x/xp12/center_right/gear_body_right_green', (65, 255, 95, 255)),
    ('ARM', 'PUSH_RTO_ARM_SEQ2', 'a380x/xp12/center_right/rto_arm', (65, 255, 95, 255)),
    ('↓', 'GEAR_ARROW', 'a380x/xp12/center_right/gear_arrow', (255, 55, 45, 255)),
)
GLARE_ANNUNCIATOR_FIRST_CELL = FCU_ANNUNCIATOR_FIRST_CELL + len(FCU_ANNUNCIATORS)
ADIRS_ANNUNCIATORS = tuple(
    (label, f'PUSH_OVHD_ADIRS_{suffix}_SEQ{sequence}',
     f'a380x/xp12/adirs/{channel}{unit}_{field}', color)
    for unit in range(1, 4)
    for channel, suffix in (('ir', f'IR{unit}'), ('adr', str(unit)))
    for label, sequence, field, color in (
        ('FAULT', 1, 'fault', (255, 180, 45, 255)),
        ('OFF', 2, 'off', (240, 245, 255, 255)),
    )
)
ADIRS_ANNUNCIATOR_FIRST_CELL = GLARE_ANNUNCIATOR_FIRST_CELL + len(GLARE_ANNUNCIATORS)
# ELEC overhead legends (tools/elec_cockpit_panel.py). FAULT and OFF reuse the ADIRS cells;
# these four follow them in the previously empty last row of the atlas.
ELEC_LEGEND_FIRST_CELL = ADIRS_ANNUNCIATOR_FIRST_CELL + 12
ELEC_LEGEND_CELLS = {legend: ELEC_LEGEND_FIRST_CELL + i for i, legend in enumerate(('altn', 'disc', 'avail', 'on'))}
EFIS_ANNUNCIATORS = tuple(
    (label, f'PUSH_EFIS_{source}_{label}_SEQ2', f'a380x/xp12/efis/{side}/{label.lower()}')
    for source, side in (('CS', 'captain'), ('FO', 'first_officer'))
    for label in ('LS', 'VV')
)
EFIS_SELECTORS = tuple(
    (f'EFIS_{source}_{selector}_{label}_SEQ1', f'KNOB_EFIS_{source}_{selector}',
     f'a380x/xp12/efis/{side}/{field}', value, label == 'ZOOM')
    for source, side in (('CS', 'captain'), ('FO', 'first_officer'))
    for selector, field, labels in (
        ('ROSE', 'mode', ('LS', 'VOR', 'NAV', 'ARC', 'PLAN')),
        ('RANGE', 'range_index', ('ZOOM', '10', '20', '40', '80', '160', '320', '640')),
    ) for value, label in enumerate(labels)
)

# One continuous surface per physical LCD; each contains two target fields.
FCU_SCREEN_QUADS = (
    (0.014283556, 0.115242831, 0.00, 0.50),  # SPD + HDG
    (-0.115033157, -0.016629022, 0.50, 1.00),  # ALT + V/S
)

# Historical split faces, used only by the preservation-aware migration.
FCU_LEGACY_SCREEN_QUADS = (
    (0.064763194, 0.115242831, 0.00, 0.25),
    (0.014283556, 0.064763194, 0.25, 0.50),
    (-0.065831089, -0.016629022, 0.50, 0.75),
    (-0.115033157, -0.065831089, 0.75, 1.00),
)


def fcu_engagement_bar(positions: list[tuple[float, float, float]]) -> tuple[
    tuple[float, float, float], float, float
]:
    """Fit a thin upper bar to a button's front, clear of its lower text decal."""
    length = math.hypot(0.347, 0.938)
    normal = (0.0, 0.347 / length, -0.938 / length)
    vertical = (0.0, 0.938 / length, 0.347 / length)

    def projection(point: tuple[float, float, float], axis: tuple[float, float, float]) -> float:
        return sum(point[index] * axis[index] for index in range(3))

    front_depth = max(projection(point, normal) for point in positions)
    front = [point for point in positions if projection(point, normal) >= front_depth - 0.0003]
    x0, x1 = min(point[0] for point in front), max(point[0] for point in front)
    bottom = min(projection(point, vertical) for point in front)
    top = max(projection(point, vertical) for point in front)
    width, height = x1 - x0, top - bottom
    if width <= 0 or height <= 0:
        raise ValueError('FCU button has no measurable front face')
    bar_height = bottom + 0.75 * height
    # A physical 0.3 mm separation prevents z-fighting without floating the bar.
    bar_depth = front_depth + 0.0003
    center = ((x0 + x1) / 2,
              vertical[1] * bar_height + normal[1] * bar_depth,
              vertical[2] * bar_height + normal[2] * bar_depth)
    return center, 0.63 * width, 0.13 * height


# Plugin cockpit devices (docs/mfd-spec/08-xplane-cockpit-device.md section 2.5,
# docs/mfd-spec/09-port-integration.md section 3.3). The visible screens live in
# a separate object with an opaque black albedo; the cockpit object carries only
# the draw-disabled touch twins. Both are generated from MFD_SCREEN_CORNERS.
MFD_SEATS = ('CAPT', 'F/O')
MFD_SCREEN_TEXTURE = 'mfd_screen_black.png'
MFD_SCREEN_TEXTURE_PATH = '../../../textures/interior/cockpit-panels/' + MFD_SCREEN_TEXTURE
MFD_SCREEN_LIFT = 0.002   # visible quad, like the PFD/ND overlays
MFD_TOUCH_LIFT = 0.003    # touch twin 1 mm IN FRONT of the visible screen
# UV (0,0) is device texel (0,0), the bottom-left as seen from the seat.
MFD_CORNER_UVS = ((0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0))   # BL TL TR BR


def mfd_surface_normal(corners: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float]:
    """cross(BR - BL, TL - BL), normalised: the face normal toward the crew."""
    bottom_left, top_left, _top_right, bottom_right = corners
    horizontal = tuple(bottom_right[axis] - bottom_left[axis] for axis in range(3))
    vertical = tuple(top_left[axis] - bottom_left[axis] for axis in range(3))
    cross = (
        horizontal[1] * vertical[2] - horizontal[2] * vertical[1],
        horizontal[2] * vertical[0] - horizontal[0] * vertical[2],
        horizontal[0] * vertical[1] - horizontal[1] * vertical[0],
    )
    length = math.sqrt(sum(value * value for value in cross))
    return tuple(value / length for value in cross)


def mfd_device_quads(lift: float) -> list[tuple[int, str, list[tuple[float, ...]]]]:
    """Return (side, device ID, four BL/TL/TR/BR vertices) raised along the normal.

    Each vertex is (x, y, z, nx, ny, nz, u, v). Indices (0,1,2) and (0,2,3)
    over this order are clockwise front faces, as OBJ8 expects.
    """
    quads = []
    for side, (corners, device) in enumerate(zip(MFD_SCREEN_CORNERS, MFD_DEVICE_IDS)):
        normal = mfd_surface_normal(corners)
        quads.append((side, device, [
            (*(point[axis] + normal[axis] * lift for axis in range(3)), *normal, *uv)
            for point, uv in zip(corners, MFD_CORNER_UVS)
        ]))
    return quads


def mfd_device_attribute(side: int, device: str) -> str:
    return (f'ATTR_cockpit_device\t{device}\t{MFD_DEVICE_BUS}\t'
            f'{MFD_LIGHTING_CHANNELS[side]}\t{MFD_AUTO_ADJUST}')


def build_mfd_screens_obj() -> str:
    """Visible MFD device screens: a separate, non-cockpit attached object.

    The device image replaces the lit texture only; the albedo still comes
    from the object TEXTURE, so it must be opaque black (spec 08 section 2.2).
    """
    vertices, indices, draws = [], [], []
    for side, device, quad in mfd_device_quads(MFD_SCREEN_LIFT):
        base, start = len(vertices), len(indices)
        vertices.extend(quad)
        indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))
        draws.append((side, device, start))
    lines = [
        'I', '800', 'OBJ', '',
        f'TEXTURE\t{MFD_SCREEN_TEXTURE_PATH}',
        'GLOBAL_cockpit_lit',
        f'POINT_COUNTS\t{len(vertices)}\t0\t0\t{len(indices)}',
        '',
    ]
    for side, device, quad in mfd_device_quads(MFD_SCREEN_LIFT):
        lines.append(f'# {MFD_SEATS[side]} MFD {device}. Corners as seen from the seat: '
                     'BL, TL, TR, BR. UV (0,0) = device bottom-left.')
        lines.extend('VT\t' + '\t'.join(f'{obj8_fmt(value)}' for value in vertex) for vertex in quad)
    lines.append('')
    packed_count = len(indices) - len(indices) % 10
    for offset in range(0, packed_count, 10):
        lines.append('IDX10\t' + '\t'.join(map(str, indices[offset:offset + 10])))
    lines.extend(f'IDX\t{index}' for index in indices[packed_count:])
    lines.extend(['', 'ATTR_poly_os\t2'])
    for side, device, start in draws:
        lines.extend([mfd_device_attribute(side, device), f'TRIS\t{start}\t6', 'ATTR_no_cockpit'])
    lines.extend(['ATTR_poly_os\t0', '',
                  '# Generated from avionics_panel_layout.MFD_SCREEN_CORNERS by build_cockpit_controls.py.'])
    return '\n'.join(lines) + '\n'


def mfd_screen_black_png() -> bytes:
    """Opaque 8x8 black RGB PNG used as the visible MFD albedo."""
    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack('>I', len(data)) + kind + data
                + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff))
    width = height = 8
    raw = (b'\0' + b'\0' * (width * 3)) * height
    return (b'\x89PNG\r\n\x1a\n'
            + chunk(b'IHDR', struct.pack('>2I5B', width, height, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


@dataclass(frozen=True)
class Control:
    node: str
    command: str | None
    label: str
    dataref: str | None = None
    axis: tuple[float, float, float] | list[float] | None = None
    value_min: float = 0.0
    value_max: float = 1.0
    command_up: str | None = None
    command_down: str | None = None
    cursor: str = 'rotate_medium'
    toggle_values: tuple[float, float] | list[float] | None = None
    command_push: str | None = None
    command_pull: str | None = None
    push_label: str | None = None
    pull_label: str | None = None
    command_cursor: str = 'hand'
    switch_axis: str | None = None
    rest_rotation: tuple[float, float, float, float] | list[float] | None = None
    visible_dataref: str | None = None
    visible_value: float | None = None


def matrix_identity() -> list[list[float]]:
    return [[1.0 if row == column else 0.0 for column in range(4)] for row in range(4)]


def matrix_multiply(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][inner] * right[inner][column] for inner in range(4)) for column in range(4)]
        for row in range(4)
    ]


def local_matrix(node: dict) -> list[list[float]]:
    if 'matrix' in node:
        values = node['matrix']
        return [[float(values[column * 4 + row]) for column in range(4)] for row in range(4)]
    tx, ty, tz = map(float, node.get('translation', (0, 0, 0)))
    x, y, z, w = map(float, node.get('rotation', (0, 0, 0, 1)))
    sx, sy, sz = map(float, node.get('scale', (1, 1, 1)))
    rotation = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
        [0, 0, 0, 1],
    ]
    scale = [[sx, 0, 0, 0], [0, sy, 0, 0], [0, 0, sz, 0], [0, 0, 0, 1]]
    translation = matrix_identity()
    translation[0][3], translation[1][3], translation[2][3] = tx, ty, tz
    return matrix_multiply(translation, matrix_multiply(rotation, scale))


def transform_point(matrix: list[list[float]], point: tuple[float, ...]) -> tuple[float, float, float]:
    values = (*point[:3], 1.0)
    return tuple(sum(matrix[row][column] * values[column] for column in range(4)) for row in range(3))


def inverse_transpose_3x3(matrix: list[list[float]]) -> list[list[float]]:
    a, b, c = matrix[0][:3]
    d, e, f = matrix[1][:3]
    g, h, i = matrix[2][:3]
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(determinant) < 1e-12:
        raise ValueError('Control has a singular world transform')
    inverse = [
        [(e * i - f * h) / determinant, (c * h - b * i) / determinant, (b * f - c * e) / determinant],
        [(f * g - d * i) / determinant, (a * i - c * g) / determinant, (c * d - a * f) / determinant],
        [(d * h - e * g) / determinant, (b * g - a * h) / determinant, (a * e - b * d) / determinant],
    ]
    return [[inverse[column][row] for column in range(3)] for row in range(3)]


def transform_normal(matrix: list[list[float]], normal: tuple[float, ...]) -> tuple[float, float, float]:
    normal_matrix = inverse_transpose_3x3(matrix)
    transformed = tuple(sum(normal_matrix[row][column] * normal[column] for column in range(3)) for row in range(3))
    length = math.sqrt(sum(value * value for value in transformed))
    if length < 1e-12:
        raise ValueError('Control contains a zero-length normal')
    return tuple(value / length for value in transformed)


def load_buffer(path: Path, uri: str) -> bytes:
    if uri.startswith('data:'):
        header, payload = uri.split(',', 1)
        return base64.b64decode(payload) if ';base64' in header else unquote_to_bytes(payload)
    source = path.parent / uri
    if source.is_file():
        return source.read_bytes()
    parts = sorted(source.parent.glob(source.name + '.part[0-9][0-9]'))
    if not parts:
        raise FileNotFoundError(source)
    return b''.join(part.read_bytes() for part in parts)


def normalize_component(value: int | float, component_type: int) -> float:
    if component_type == 5120:
        return max(float(value) / 127.0, -1.0)
    if component_type == 5121:
        return float(value) / 255.0
    if component_type == 5122:
        return max(float(value) / 32767.0, -1.0)
    if component_type == 5123:
        return float(value) / 65535.0
    return float(value)


def accessor_values(document: dict, buffers: list[bytes], accessor_index: int) -> list[tuple[float, ...]]:
    accessor = document['accessors'][accessor_index]
    if 'sparse' in accessor:
        raise ValueError('Sparse glTF accessors are not supported for cockpit controls')
    component_type = accessor['componentType']
    format_character, component_size = COMPONENTS[component_type]
    components = TYPE_LENGTHS[accessor['type']]
    view = document['bufferViews'][accessor['bufferView']]
    stride = view.get('byteStride', component_size * components)
    start = view.get('byteOffset', 0) + accessor.get('byteOffset', 0)
    unpack = struct.Struct('<' + format_character * components)
    result = []
    for index in range(accessor['count']):
        values = unpack.unpack_from(buffers[view['buffer']], start + index * stride)
        if accessor.get('normalized'):
            values = tuple(normalize_component(value, component_type) for value in values)
        result.append(tuple(float(value) for value in values))
    return result


def adirs_lamp_face(positions, matrix):
    """Preserve a tilted overhead legend's authored rectangular face and roll.

    Source pushbutton legends lie in local X/Z with +Y facing the crew; local
    -Z is the top of the lettering. A generic glareshield quad loses both the
    overhead tilt and its side-to-side slope.
    """
    low = tuple(min(p[axis] for p in positions) for axis in range(3))
    high = tuple(max(p[axis] for p in positions) for axis in range(3))
    if high[1] - low[1] > .00002:
        raise ValueError('ADIRS annunciator is not an authored planar X/Z face')
    normal = tuple(matrix[axis][1] for axis in range(3))
    length = math.sqrt(sum(value * value for value in normal))
    normal = tuple(value / length for value in normal)
    y = (low[1] + high[1]) / 2
    corners = [transform_point(matrix, p) for p in (
        (low[0], y, high[2]), (high[0], y, high[2]),
        (high[0], y, low[2]), (low[0], y, low[2]),
    )]
    corners = [tuple(p[axis] + .0004 * normal[axis] for axis in range(3)) for p in corners]
    return corners, normal


def load_controls(path: Path) -> list[Control]:
    raw = json.loads(path.read_text(encoding='utf-8'))
    controls = [Control(**item) for item in raw]
    if not controls:
        raise ValueError('Cockpit control configuration is empty')
    nodes = [control.node for control in controls]
    if len(nodes) != len(set(nodes)):
        raise ValueError('Cockpit control node names must be unique')
    for control in controls:
        if not control.node or not control.label.strip():
            raise ValueError(f'Invalid cockpit control: {control}')
        has_knob = control.command_up is not None and control.command_down is not None
        has_drag = control.dataref is not None and control.axis is not None
        has_toggle = control.dataref is not None and control.toggle_values is not None
        if (control.command_up is None) != (control.command_down is None):
            raise ValueError(f'Knob must define both command_up and command_down: {control.node}')
        if (control.command_push is None) != (control.command_pull is None):
            raise ValueError(f'Knob must define both command_push and command_pull: {control.node}')
        if control.command_push is not None and not has_knob:
            raise ValueError(f'Push/pull requires a rotary command knob: {control.node}')
        if sum((control.command is not None, control.dataref is not None, has_knob)) != 1:
            raise ValueError(f'Control must define exactly one command, dataref, or command knob: {control.node}')
        if control.command is not None and not control.command.startswith(('sim/', 'a380x/')):
            raise ValueError(f'Invalid cockpit command: {control.node}')
        if control.command is not None and control.command_cursor not in {'hand', 'button', 'arrow'}:
            raise ValueError(f'Invalid cockpit command cursor: {control.node}')
        if has_knob:
            knob_commands = (control.command_up, control.command_down, control.command_push, control.command_pull)
            if any(command is not None and not command.startswith(('sim/', 'a380x/'))
                   for command in knob_commands):
                raise ValueError(f'Invalid cockpit knob command: {control.node}')
            if control.cursor not in {'rotate_small', 'rotate_medium', 'rotate_large', 'up_down'}:
                raise ValueError(f'Invalid cockpit knob cursor: {control.node}')
        if control.switch_axis not in (None, 'up_down') or (control.switch_axis and not has_knob):
            raise ValueError(f'Invalid cockpit switch axis: {control.node}')
        if control.rest_rotation is not None:
            if len(control.rest_rotation) != 4 or not all(math.isfinite(float(v)) for v in control.rest_rotation):
                raise ValueError(f'Invalid cockpit rest rotation: {control.node}')
            if abs(sum(float(v) ** 2 for v in control.rest_rotation) - 1.0) > 0.01:
                raise ValueError(f'Cockpit rest rotation must be a unit quaternion: {control.node}')
        if control.dataref is not None:
            if not control.dataref.startswith('sim/') or has_drag == has_toggle:
                raise ValueError(f'Dataref control must define exactly one drag axis or toggle pair: {control.node}')
            if has_drag:
                if len(control.axis) != 3:
                    raise ValueError(f'Invalid cockpit drag control: {control.node}')
                values = (*control.axis, control.value_min, control.value_max)
                if not all(math.isfinite(float(value)) for value in values):
                    raise ValueError(f'Cockpit drag control contains a non-finite value: {control.node}')
                if math.sqrt(sum(float(value) ** 2 for value in control.axis)) < 1e-9:
                    raise ValueError(f'Cockpit drag control has a zero-length axis: {control.node}')
            else:
                if len(control.toggle_values) != 2 or not all(
                    math.isfinite(float(value)) for value in control.toggle_values
                ) or float(control.toggle_values[0]) == float(control.toggle_values[1]):
                    raise ValueError(f'Invalid cockpit toggle values: {control.node}')
                if control.cursor not in {'button', 'rotate_small', 'rotate_medium', 'rotate_large'}:
                    raise ValueError(f'Invalid cockpit toggle cursor: {control.node}')
        if any(character in label for label in (control.label, control.push_label or '', control.pull_label or '')
               for character in '\r\n\t'):
            raise ValueError(f'Control label must fit on one OBJ line: {control.node}')
    return controls


def command_catalog(path: Path) -> set[str]:
    return {
        line.split(maxsplit=1)[0]
        for line in path.read_text(encoding='utf-8', errors='replace').splitlines()
        if line.startswith('sim/')
    }


def build_obj(
    source: Path,
    controls: list[Control],
    known_commands: set[str] | None = None,
    known_datarefs: set[str] | None = None,
    fcu_overlay: bool = False,
    additional_sources: list[tuple[Path, list[Control]]] | None = None,
) -> tuple[str, dict]:
    source_groups = [(source, controls), *(additional_sources or [])]
    configured_commands = {
        control.command
        for _group_source, group_controls in source_groups
        for control in group_controls
        if control.command is not None
    }
    configured_commands.update(
        control.command_up
        for _group_source, group_controls in source_groups
        for control in group_controls
        if control.command_up is not None
    )
    configured_commands.update(
        control.command_down
        for _group_source, group_controls in source_groups
        for control in group_controls
        if control.command_down is not None
    )
    configured_commands.update(
        command
        for _group_source, group_controls in source_groups
        for control in group_controls
        for command in (control.command_push, control.command_pull)
        if command is not None
    )
    # Aircraft-local commands are registered by the native avionics plugin and
    # packaged FlyWithLua modules,
    # therefore do not appear in X-Plane's static Commands.txt catalog.
    catalog_commands = {command for command in configured_commands if command.startswith('sim/')}
    missing_commands = sorted(catalog_commands - known_commands) if known_commands is not None else []
    if missing_commands:
        raise ValueError('Commands absent from X-Plane catalog: ' + ', '.join(missing_commands))
    unregistered_fcu = sorted(command for command in configured_commands
                              if command.startswith('a380x/xp12/fcu/') and command not in FCU_COMMANDS)
    if unregistered_fcu:
        raise ValueError('Commands absent from native FCU provider: ' + ', '.join(unregistered_fcu))
    unregistered_efis = sorted(command for command in configured_commands
        if command.startswith('a380x/xp12/efis/') and command not in EFIS_COMMANDS)
    if unregistered_efis:
        raise ValueError('Commands absent from native EFIS provider: ' + ', '.join(unregistered_efis))
    unregistered_du = sorted(command for command in configured_commands
        if command.startswith('a380x/xp12/du/') and command not in DU_COMMANDS)
    if unregistered_du:
        raise ValueError('Commands absent from native DU provider: ' + ', '.join(unregistered_du))
    unregistered_glare = sorted(command for command in configured_commands
        if command.startswith('a380x/xp12/glareshield/') and command not in GLARESHIELD_COMMANDS)
    if unregistered_glare:
        raise ValueError('Commands absent from native glareshield provider: ' + ', '.join(unregistered_glare))
    unregistered_clock = sorted(command for command in configured_commands
        if command.startswith('a380x/xp12/clock/') and command not in CLOCK_COMMANDS)
    if unregistered_clock:
        raise ValueError('Commands absent from native clock provider: ' + ', '.join(unregistered_clock))
    unregistered_center_right = sorted(command for command in configured_commands
        if command.startswith('a380x/xp12/center_right/') and command not in CENTER_RIGHT_COMMANDS)
    if unregistered_center_right:
        raise ValueError('Commands absent from native center-right provider: ' + ', '.join(unregistered_center_right))
    unregistered_adirs = sorted(command for command in configured_commands
        if command.startswith('a380x/xp12/adirs/') and command not in ADIRS_COMMANDS)
    if unregistered_adirs:
        raise ValueError('Commands absent from native ADIRS provider: ' + ', '.join(unregistered_adirs))
    unregistered_kccu = sorted(command for command in configured_commands
        if command.startswith(('a380x/xp12/kccu/', 'a380x/xp12/mfd/'))
        and command not in (*KCCU_COMMANDS, *MFD_COMMANDS))
    if unregistered_kccu:
        raise ValueError('Commands absent from native KCCU/MFD provider: ' + ', '.join(unregistered_kccu))
    unregistered_sd = sorted(command for command in configured_commands
        if command.startswith('a380x/xp12/ecam/sd/') and command not in SD_COMMANDS)
    if unregistered_sd:
        raise ValueError('Commands absent from native ECAM provider: ' + ', '.join(unregistered_sd))
    configured_datarefs = {
        re.sub(r'\[\d+\]$', '', control.dataref)
        for _group_source, group_controls in source_groups
        for control in group_controls
        if control.dataref is not None
    }
    missing_datarefs = sorted(configured_datarefs - known_datarefs) if known_datarefs is not None else []
    if missing_datarefs:
        raise ValueError('Datarefs absent from X-Plane catalog: ' + ', '.join(missing_datarefs))

    vertices: list[tuple[float, float, float, float, float, float]] = []
    indices: list[int] = []
    draws: list[tuple[Path, Control, int, int, int]] = []
    annunciator_faces: dict[str, tuple[tuple[float, float, float], float, float]] = {}
    selector_faces: dict[str, tuple[list[tuple[float, float, float]], tuple[float, float, float]]] = {}
    control_motions = {}
    adirs_faces = {}
    for group_source, group_controls in source_groups:
        document = json.loads(group_source.read_text(encoding='utf-8'))
        buffers = [load_buffer(group_source, buffer['uri']) for buffer in document['buffers']]
        nodes = document['nodes']
        names = {node.get('name'): index for index, node in enumerate(nodes) if node.get('name')}
        if 'KNOB_SWITCHING_BRK' in names and 'LEVER_LANDINGGEAR' in names:
            # Match the hit volumes and illuminated button faces to the same
            # authored motion used by the retained textured cockpit meshes.
            from patch_center_right_animations import source_targets, animation_block
            for target in source_targets(group_source, include_gear=True):
                control_motions[(group_source, target['name'])] = target
        if 'KNOB_OVHD_ADIRS_1' in names:
            from patch_center_right_animations import animation_block
            from patch_adirs_animations import adirs_targets
            for target in adirs_targets(group_source):
                control_motions[(group_source, target['name'])] = target
        if 'KNOB_FCU_ALT' in names and 'PUSH_FCU_AP1' in names:
            from patch_center_right_animations import animation_block
            from patch_fcu_animations import fcu_touch_targets
            for target in fcu_touch_targets(group_source):
                control_motions[(group_source, target['name'])] = target
        parents = {child: index for index, node in enumerate(nodes) for child in node.get('children', [])}
        matrices: dict[int, list[list[float]]] = {}

        def world_matrix(index: int) -> list[list[float]]:
            if index not in matrices:
                local = local_matrix(nodes[index])
                matrices[index] = matrix_multiply(world_matrix(parents[index]), local) if index in parents else local
            return matrices[index]

        if fcu_overlay:
            for _label, lamp_node, _dataref, _color in ADIRS_ANNUNCIATORS:
                if lamp_node not in names:
                    continue
                index = names[lamp_node]
                lamp = nodes[index]
                positions = [point
                    for primitive in document['meshes'][lamp['mesh']]['primitives']
                    for point in accessor_values(document, buffers, primitive['attributes']['POSITION'])]
                adirs_faces[lamp_node] = adirs_lamp_face(positions, world_matrix(index))
            fcu_lamp_nodes = {row[1] for row in (*FCU_ANNUNCIATORS, *EFIS_ANNUNCIATORS)}
            for lamp_node in [row[1] for row in (*FCU_ANNUNCIATORS, *EFIS_ANNUNCIATORS)] + [row[1] for row in GLARE_ANNUNCIATORS]:
                gear_pair = lamp_node.startswith(('GEAR_WING_', 'GEAR_BODY_'))
                if gear_pair:
                    # Each source mesh contains the wing and body lamp on
                    # one side, with a physical gap between the two lenses.
                    gear_group = '1' if '_LEFT_' in lamp_node else '3'
                    gear_sequence = 'SEQ2' if lamp_node.endswith('_GREEN') else 'SEQ1'
                    source_node = f'PUSH_AUTOBKR_LDGGEAR_{gear_group}_{gear_sequence}'
                else:
                    source_node = lamp_node.removesuffix('_SEQ2') if lamp_node in fcu_lamp_nodes else lamp_node
                if source_node not in names:
                    continue
                lamp_index = names[source_node]
                lamp = nodes[lamp_index]
                if 'mesh' not in lamp:
                    raise ValueError(f'FCU annunciator has no authored face: {lamp_node}')
                positions = [
                    transform_point(world_matrix(lamp_index), point)
                    for primitive in document['meshes'][lamp['mesh']]['primitives']
                    for point in accessor_values(document, buffers, primitive['attributes']['POSITION'])
                ]
                if gear_pair:
                    middle_x = (min(p[0] for p in positions) + max(p[0] for p in positions)) / 2
                    high_x = (lamp_node.startswith('GEAR_WING_LEFT_')
                              or lamp_node.startswith('GEAR_BODY_RIGHT_'))
                    positions = [p for p in positions if (p[0] > middle_x) == high_x]
                if lamp_node in fcu_lamp_nodes:
                    annunciator_faces[lamp_node] = fcu_engagement_bar(positions)
                    continue
                low = tuple(min(point[axis] for point in positions) for axis in range(3))
                high = tuple(max(point[axis] for point in positions) for axis in range(3))
                center = tuple((low[axis] + high[axis]) / 2 for axis in range(3))
                height = math.hypot(high[1] - low[1], high[2] - low[2])
                annunciator_faces[lamp_node] = (center, high[0] - low[0], height)
            for marker, knob, _dataref, _value, _square in EFIS_SELECTORS:
                if marker not in names or knob not in names:
                    continue
                positions = [transform_point(world_matrix(names[marker]), point)
                    for primitive in document['meshes'][nodes[names[marker]]['mesh']]['primitives']
                    for point in accessor_values(document, buffers, primitive['attributes']['POSITION'])]
                knob_positions = [transform_point(world_matrix(names[knob]), point)
                    for primitive in document['meshes'][nodes[names[knob]]['mesh']]['primitives']
                    for point in accessor_values(document, buffers, primitive['attributes']['POSITION'])]
                knob_center = tuple((min(point[axis] for point in knob_positions)
                    + max(point[axis] for point in knob_positions)) / 2 for axis in range(3))
                selector_faces[marker] = (positions, knob_center)

        for control in group_controls:
            if control.node not in names:
                raise ValueError(f'Cockpit control node not found in {group_source}: {control.node}')
            node_index = names[control.node]
            node = nodes[node_index]
            if 'mesh' not in node:
                raise ValueError(f'Cockpit control has no mesh: {control.node}')
            if control.rest_rotation is None:
                matrix = world_matrix(node_index)
            else:
                rest_node = dict(node, rotation=control.rest_rotation)
                local = local_matrix(rest_node)
                matrix = (matrix_multiply(world_matrix(parents[node_index]), local)
                          if node_index in parents else local)
            draw_start = len(indices)
            vertex_start = len(vertices)
            control_triangles = 0
            for primitive in document['meshes'][node['mesh']]['primitives']:
                if primitive.get('mode', 4) != 4 or 'indices' not in primitive:
                    raise ValueError(f'Cockpit control must use indexed triangles: {control.node}')
                positions = accessor_values(document, buffers, primitive['attributes']['POSITION'])
                normals = accessor_values(document, buffers, primitive['attributes']['NORMAL'])
                source_indices = [int(value[0]) for value in accessor_values(document, buffers, primitive['indices'])]
                if len(positions) != len(normals) or len(source_indices) % 3:
                    raise ValueError(f'Malformed cockpit control mesh: {control.node}')
                base = len(vertices)
                transformed_positions = [transform_point(matrix, point) for point in positions]
                transformed_normals = [transform_normal(matrix, normal) for normal in normals]
                vertices.extend((*point, *normal) for point, normal in zip(transformed_positions, transformed_normals))
                for offset in range(0, len(source_indices), 3):
                    triangle = source_indices[offset:offset + 3]
                    a, b, c = (transformed_positions[index] for index in triangle)
                    ab = tuple(b[axis] - a[axis] for axis in range(3))
                    ac = tuple(c[axis] - a[axis] for axis in range(3))
                    cross = (
                        ab[1] * ac[2] - ab[2] * ac[1],
                        ab[2] * ac[0] - ab[0] * ac[2],
                        ab[0] * ac[1] - ab[1] * ac[0],
                    )
                    normal = tuple(sum(transformed_normals[index][axis] for index in triangle) for axis in range(3))
                    if sum(cross[axis] * normal[axis] for axis in range(3)) > 0:
                        triangle[1], triangle[2] = triangle[2], triangle[1]
                    indices.extend(base + index for index in triangle)
                control_triangles += len(source_indices) // 3
            if control.command_push is None:
                draws.append((group_source, control, draw_start, len(indices) - draw_start, control_triangles))
            else:
                # Use separate, coplanar hit regions on the front of the knob.
                # Keeping the entire old mesh clickable underneath overlapping
                # push/pull volumes makes picking depend on camera angle. The
                # outer quarters rotate; upper/lower centre regions push/pull.
                knob_points = [vertex[:3] for vertex in vertices[vertex_start:]]
                del vertices[vertex_start:]
                del indices[draw_start:]
                normal_length = math.hypot(0.347, 0.938)
                normal = (0.0, 0.347 / normal_length, -0.938 / normal_length)
                vertical = (0.0, -normal[2], normal[1])
                left = min(point[0] for point in knob_points)
                right = max(point[0] for point in knob_points)
                low = min(sum(point[a] * vertical[a] for a in range(3)) for point in knob_points)
                high = max(sum(point[a] * vertical[a] for a in range(3)) for point in knob_points)
                front = max(sum(point[a] * normal[a] for a in range(3)) for point in knob_points) + 0.001
                if right - left < 1e-6 or high - low < 1e-6:
                    raise ValueError(f'Push/pull knob has no usable front surface: {control.node}')
                centre_left = left + (right - left) * 0.25
                centre_right = right - (right - left) * 0.25
                middle = (low + high) / 2
                rotary = replace(control, command_push=None, command_pull=None, push_label=None, pull_label=None)
                zones = (
                    (rotary, left, low, centre_left, high),
                    (rotary, centre_right, low, right, high),
                    (Control(control.node + ':push', control.command_push,
                             control.push_label or control.label + ' - PUSH', command_cursor='button'),
                     centre_left, middle, centre_right, high),
                    (Control(control.node + ':pull', control.command_pull,
                             control.pull_label or control.label + ' - PULL'), centre_left, low, centre_right, middle),
                )
                for zone_control, x0, y0, x1, y1 in zones:
                    start, base = len(indices), len(vertices)
                    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                        vertices.append((x, vertical[1] * y + normal[1] * front,
                                         vertical[2] * y + normal[2] * front, *normal))
                    # Same winding convention as the source click meshes.
                    indices.extend((base, base + 2, base + 1, base, base + 3, base + 2))
                    draws.append((group_source, zone_control, start, 6, 2))

    overlay_draws: list[tuple[str, int, int]] = []
    device_touch_draws: list[tuple[int, str, int]] = []
    if fcu_overlay:
        normal = (0.0, 0.347, -0.938)
        vertical = (0.0, 0.938, 0.347)
        # The source gear panel has a painted arrow but no separate lamp mesh.
        # Put a small live marker on its front face, beside the gear lever.
        annunciator_faces['GEAR_ARROW'] = ((-0.146, 2.635, 32.90152), 0.011, 0.020)

        def add_quad(
            name: str,
            center: tuple[float, float, float],
            width: float,
            height: float,
            uv: tuple[float, float, float, float],
            surface_offset: float = 0.002,
            face_normal: tuple[float, float, float] = normal,
            face_vertical: tuple[float, float, float] = vertical,
        ) -> None:
            start = len(indices)
            base = len(vertices)
            cx, cy, cz = center
            cx += face_normal[0] * surface_offset
            cy += face_normal[1] * surface_offset
            cz += face_normal[2] * surface_offset
            half_width, half_height = width / 2, height / 2
            u0, v0, u1, v1 = uv
            points = (
                (cx - half_width, cy - face_vertical[1] * half_height, cz - face_vertical[2] * half_height, u0, v0),
                (cx + half_width, cy - face_vertical[1] * half_height, cz - face_vertical[2] * half_height, u1, v0),
                (cx + half_width, cy + face_vertical[1] * half_height, cz + face_vertical[2] * half_height, u1, v1),
                (cx - half_width, cy + face_vertical[1] * half_height, cz + face_vertical[2] * half_height, u0, v1),
            )
            vertices.extend((*point[:3], *face_normal, point[3], point[4]) for point in points)
            indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))
            overlay_draws.append((name, start, 6))

        screen_y0, screen_z0 = 2.9828, 32.7532
        screen_y1, screen_z1 = 3.0021, 32.7603
        center_y, center_z = (screen_y0 + screen_y1) / 2, (screen_z0 + screen_z1) / 2
        screen_height = math.hypot(screen_y1 - screen_y0, screen_z1 - screen_z0)
        fcu_left, fcu_bottom, fcu_right, fcu_top = FCU_PANEL_RECT
        fcu_width = fcu_right - fcu_left
        panel_v1 = fcu_top / PANEL_SIZE[1]
        panel_v0 = fcu_bottom / PANEL_SIZE[1]
        for screen_index, (x0, x1, u0, u1) in enumerate(FCU_SCREEN_QUADS):
            add_quad(
                f'screen:{screen_index}',
                ((x0 + x1) / 2, center_y, center_z),
                abs(x1 - x0), screen_height,
                # The native renderer uses the same panel coordinate system
                # as the PFD/ND. Source X is reversed by the aircraft's
                # 180-degree attachment; preserve bottom-to-top V order.
                ((fcu_left + u1 * fcu_width) / PANEL_SIZE[0], panel_v0,
                 (fcu_left + u0 * fcu_width) / PANEL_SIZE[0], panel_v1),
                0.002,
            )

        def add_efis_screen(name: str, surface: tuple[float, ...], rect: tuple[int, ...]) -> None:
            x0, x1, y0, z0, y1, z1 = surface
            left, bottom, right, top = rect
            add_quad(name, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                     x1 - x0, math.hypot(y1 - y0, z1 - z0),
                     (right / PANEL_SIZE[0], bottom / PANEL_SIZE[1],
                      left / PANEL_SIZE[0], top / PANEL_SIZE[1]))

        for side in range(2):
            add_efis_screen(f'efis:{side}:baro', EFIS_BARO_SCREEN_SURFACES[side],
                            EFIS_BARO_PANEL_RECTS[side])
            lower, upper = EFIS_UPPER_SCREEN_X_BOUNDS[side]
            column_width = (upper - lower) / 5
            for column in range(5):
                # Increasing display column moves toward negative source X.
                x1 = upper - column * column_width
                add_efis_screen(f'efis:{side}:upper:{column}',
                    (x1 - column_width, x1, *EFIS_UPPER_SCREEN_YZ),
                    EFIS_UPPER_PANEL_RECTS[side][column])
                add_efis_screen(f'efis:{side}:lower:{column}',
                    (*EFIS_LOWER_SCREEN_X_BOUNDS[side][column], *EFIS_LOWER_SCREEN_YZ),
                    EFIS_LOWER_PANEL_RECTS[side][column])

        for name, surface, rect in zip(('chr', 'utc', 'et'), CLOCK_SCREEN_SURFACES,
                (CLOCK_CHR_PANEL_RECT, CLOCK_UTC_PANEL_RECT, CLOCK_ET_PANEL_RECT)):
            x0, x1, y0, z0, y1, z1 = surface
            left, bottom, right, top = rect
            add_quad(f'clock:{name}', ((x0+x1)/2, (y0+y1)/2, (z0+z1)/2),
                     x1-x0-.0006, math.hypot(y1-y0,z1-z0)-.0006,
                     (right/PANEL_SIZE[0], bottom/PANEL_SIZE[1],
                      left/PANEL_SIZE[0], top/PANEL_SIZE[1]), .0004,
                     (0., .34202, -.93969), (0., .93969, .34202))

        for screen_index, (surface, panel_rect) in enumerate(zip(PFD_SCREEN_SURFACES, PFD_PANEL_RECTS)):
            x0, x1, y0, z0, y1, z1 = surface
            panel_left, panel_bottom, panel_right, panel_top = panel_rect
            # Source X is reversed by the aircraft attachment's 180-degree
            # heading rotation, so map the lower source X to the panel's right.
            # The authored screen's lower and upper edges retain normal panel
            # V order. Reversing V here turns the live PFD upside down.
            add_quad(
                f'pfd:{screen_index}',
                ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                abs(x1 - x0),
                math.hypot(y1 - y0, z1 - z0),
                (
                    panel_right / PANEL_SIZE[0],
                    panel_bottom / PANEL_SIZE[1],
                    panel_left / PANEL_SIZE[0],
                    panel_top / PANEL_SIZE[1],
                ),
            )

        for screen_index, (surface, panel_rect) in enumerate(zip(ND_SCREEN_SURFACES, ND_PANEL_RECTS)):
            x0, x1, y0, z0, y1, z1 = surface
            panel_left, panel_bottom, panel_right, panel_top = panel_rect
            add_quad(
                f'nd:{screen_index}',
                ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                abs(x1 - x0),
                math.hypot(y1 - y0, z1 - z0),
                (
                    panel_right / PANEL_SIZE[0],
                    panel_bottom / PANEL_SIZE[1],
                    panel_left / PANEL_SIZE[0],
                    panel_top / PANEL_SIZE[1],
                ),
            )

        x0, x1, y0, z0, y1, z1 = EWD_SCREEN_SURFACE
        left, bottom, right, top = EWD_PANEL_RECT
        add_quad(
            'ewd:0',
            ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
            abs(x1 - x0), math.hypot(y1 - y0, z1 - z0),
            (right / PANEL_SIZE[0], bottom / PANEL_SIZE[1],
             left / PANEL_SIZE[0], top / PANEL_SIZE[1]),
        )

        def surface_normal(
            corners: tuple[tuple[float, float, float], ...],
        ) -> tuple[float, float, float]:
            bottom_left, top_left, _top_right, bottom_right = corners
            horizontal = tuple(bottom_right[axis] - bottom_left[axis] for axis in range(3))
            vertical = tuple(top_left[axis] - bottom_left[axis] for axis in range(3))
            cross = (
                horizontal[1] * vertical[2] - horizontal[2] * vertical[1],
                horizontal[2] * vertical[0] - horizontal[0] * vertical[2],
                horizontal[0] * vertical[1] - horizontal[1] * vertical[0],
            )
            length = math.sqrt(sum(value * value for value in cross))
            return tuple(value / length for value in cross)

        def surface_point(
            corners: tuple[tuple[float, float, float], ...], u: float, v: float,
        ) -> tuple[float, float, float]:
            bottom_left, top_left, top_right, bottom_right = corners
            bottom = tuple(
                bottom_left[axis] + u * (bottom_right[axis] - bottom_left[axis])
                for axis in range(3)
            )
            top = tuple(
                top_left[axis] + u * (top_right[axis] - top_left[axis])
                for axis in range(3)
            )
            return tuple(bottom[axis] + v * (top[axis] - bottom[axis]) for axis in range(3))

        def add_efb_surface(
            name: str,
            corners: tuple[tuple[float, float, float], ...],
        ) -> None:
            start = len(indices)
            base = len(vertices)
            normal = surface_normal(corners)
            left, bottom, right, top = EFB_PANEL_RECT
            # These explicit corners are already bottom-left, top-left,
            # top-right, bottom-right as seen by the seated pilot. Unlike the
            # PFD/ND bounds, they need no U reversal; V increases with physical
            # height on both tablets, as it does in the panel render target.
            texture = (
                (left / PANEL_SIZE[0], bottom / PANEL_SIZE[1]),
                (left / PANEL_SIZE[0], top / PANEL_SIZE[1]),
                (right / PANEL_SIZE[0], top / PANEL_SIZE[1]),
                (right / PANEL_SIZE[0], bottom / PANEL_SIZE[1]),
            )
            for point, uv in zip(corners, texture):
                raised = tuple(point[axis] + normal[axis] * 0.003 for axis in range(3))
                vertices.append((*raised, *normal, *uv))
            indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))
            overlay_draws.append((name, start, 6))

        def add_efb_touch(
            side: int,
            corners: tuple[tuple[float, float, float], ...],
            rect: tuple[int, int, int, int],
            command: str,
            label: str,
            node: str,
        ) -> None:
            panel_left, panel_bottom, panel_right, panel_top = EFB_PANEL_RECT
            left, bottom, right, top = rect
            u0 = (left - panel_left) / (panel_right - panel_left)
            u1 = (right - panel_left) / (panel_right - panel_left)
            v0 = (bottom - panel_bottom) / (panel_top - panel_bottom)
            v1 = (top - panel_bottom) / (panel_top - panel_bottom)
            # Use the same direct pixel-to-surface map as the visible quad.
            corners_uv = (
                (u0, v0), (u0, v1), (u1, v1), (u1, v0),
            )
            normal = surface_normal(corners)
            start = len(indices)
            base = len(vertices)
            for u, v in corners_uv:
                point = surface_point(corners, u, v)
                raised = tuple(point[axis] + normal[axis] * 0.006 for axis in range(3))
                vertices.append((*raised, *normal))
            indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))
            draws.append((
                source,
                Control(f'EFB_{side}_{node}', command, label, command_cursor='button'),
                start,
                6,
                2,
            ))

        for side, corners in enumerate(EFB_SCREEN_SURFACES, start=1):
            add_efb_surface(f'efb:{side}', corners)
            seat = 'CAPT' if side == 1 else 'FO'
            visible_pages = (
                'Aircraft', 'Loading', 'Ground', 'Perfo', 'Nav', 'Radios',
                'Faults', 'Checks', 'States', 'Settings',
            )
            for page, visible_page, rect in zip(EFB_PAGE_NAMES, visible_pages, EFB_PAGE_TOUCH_RECTS):
                add_efb_touch(
                    side, corners, rect,
                    f'a380x/xp12/efb/page/{page}',
                    f'{seat} A380 panel - {visible_page}',
                    f'PAGE_{page.upper()}',
                )
            for action, rect in enumerate(EFB_ACTION_TOUCH_RECTS, start=1):
                add_efb_touch(
                    side, corners, rect,
                    f'a380x/xp12/efb/action_{action}',
                    f'{seat} A380 panel - action {action}',
                    f'ACTION_{action}',
                )
            add_efb_touch(
                side, corners, EFB_POWER_TOUCH_RECT,
                'a380x/xp12/efb/power',
                f'{seat} A380 panel - power / standby',
                'POWER',
            )

        # PLAN scroll chevrons occupy the two bottom corners of each ND. They
        # are clickable only in PLAN, matching the native ND footer controls.
        for side, (surface, rect) in enumerate(zip(ND_SCREEN_SURFACES, ND_PANEL_RECTS)):
            x0, x1, y0, z0, y1, z1 = surface
            left, bottom, right, top = rect
            for direction, u0, u1 in (('previous', 0., .2), ('next', .8, 1.)):
                height_fraction = 18 / (top - bottom)
                center_fraction = height_fraction / 2
                source_center_x = x1 - (u0 + u1) / 2 * (x1 - x0)
                name = f'nd-plan-touch:{side}:{direction}'
                add_quad(name, (source_center_x, y0 + (y1 - y0) * center_fraction,
                               z0 + (z1 - z0) * center_fraction),
                         (x1 - x0) * (u1 - u0),
                         math.hypot(y1 - y0, z1 - z0) * height_fraction,
                         (0., 0., 0., 0.), .006)
                _name, start, count = overlay_draws[-1]
                seat = 'captain' if side == 0 else 'first_officer'
                control = Control(f'ND_{side}_PLAN_{direction.upper()}',
                    f'a380x/xp12/efis/{seat}/plan_{direction}',
                    f'{seat.upper()} ND PLAN - {direction} waypoint',
                    command_cursor='button', visible_dataref=f'a380x/xp12/efis/{seat}/mode',
                    visible_value=4.)
                draws.append((source, control, start, count, 2))

        for lamp_index, (_label, lamp_node, _dataref) in enumerate(FCU_ANNUNCIATORS):
            if lamp_node not in annunciator_faces:
                continue
            center, width, height = annunciator_faces[lamp_node]
            cell = FCU_ANNUNCIATOR_FIRST_CELL + lamp_index
            column, row = cell % FCU_LABEL_COLUMNS, cell // FCU_LABEL_COLUMNS
            u0, u1 = column / FCU_LABEL_COLUMNS, (column + 1) / FCU_LABEL_COLUMNS
            v0, v1 = 1 - (row + 1) / FCU_LABEL_ROWS, 1 - row / FCU_LABEL_ROWS
            # fcu_engagement_bar already separates the bar from the front face.
            add_quad(f'lamp:{lamp_node}', center, width, height, (u1, v0, u0, v1), 0.0)
        for lamp_index, (_label, lamp_node, _dataref, _color) in enumerate(GLARE_ANNUNCIATORS):
            if lamp_node not in annunciator_faces:
                continue
            center, width, height = annunciator_faces[lamp_node]
            cell = GLARE_ANNUNCIATOR_FIRST_CELL + lamp_index
            column, row = cell % FCU_LABEL_COLUMNS, cell // FCU_LABEL_COLUMNS
            u0, u1 = column / FCU_LABEL_COLUMNS, (column + 1) / FCU_LABEL_COLUMNS
            v0, v1 = 1 - (row + 1) / FCU_LABEL_ROWS, 1 - row / FCU_LABEL_ROWS
            add_quad(f'lamp:{lamp_node}', center, width, height, (u1, v0, u0, v1), 0.0004)
        for lamp_index, (_label, lamp_node, _dataref, _color) in enumerate(ADIRS_ANNUNCIATORS):
            if lamp_node not in adirs_faces:
                continue
            corners, face_normal = adirs_faces[lamp_node]
            cell = ADIRS_ANNUNCIATOR_FIRST_CELL + lamp_index
            column, row = cell % FCU_LABEL_COLUMNS, cell // FCU_LABEL_COLUMNS
            u0, u1 = column / FCU_LABEL_COLUMNS, (column + 1) / FCU_LABEL_COLUMNS
            v0, v1 = 1 - (row + 1) / FCU_LABEL_ROWS, 1 - row / FCU_LABEL_ROWS
            start, base = len(indices), len(vertices)
            for point, uv in zip(corners, ((u0, v0), (u1, v0), (u1, v1), (u0, v1))):
                vertices.append((*point, *face_normal, *uv))
            indices.extend((base, base + 2, base + 1, base, base + 3, base + 2))
            overlay_draws.append((f'adirs-lamp:{lamp_node}', start, 6))
        # Reuse an existing solid green atlas cell. LS/VV stay unlit unless
        # their own side is active; the authored white button legends remain.
        column, row = FCU_ANNUNCIATOR_FIRST_CELL % FCU_LABEL_COLUMNS, FCU_ANNUNCIATOR_FIRST_CELL // FCU_LABEL_COLUMNS
        green_uv = ((column + .5) / FCU_LABEL_COLUMNS, 1 - (row + .5) / FCU_LABEL_ROWS)
        for _label, lamp_node, _dataref in EFIS_ANNUNCIATORS:
            if lamp_node in annunciator_faces:
                center, width, height = annunciator_faces[lamp_node]
                add_quad(f'lamp:{lamp_node}', center, width, height,
                         (*green_uv, *green_uv), 0.0)
        for marker, _knob, _dataref, _value, square in EFIS_SELECTORS:
            if marker not in selector_faces:
                continue
            positions, knob_center = selector_faces[marker]
            low = tuple(min(point[axis] for point in positions) for axis in range(3))
            high = tuple(max(point[axis] for point in positions) for axis in range(3))
            center = tuple((low[axis] + high[axis]) / 2 for axis in range(3))
            width, height = high[0] - low[0], math.hypot(high[1] - low[1], high[2] - low[2])
            if square:
                add_quad(f'selector:{marker}', center, width * .8, height * .8,
                         (*green_uv, *green_uv), .0005)
                continue
            # Point away from the knob, like the source selected-mode arrows.
            dx = center[0] - knob_center[0]
            dy = sum((center[axis] - knob_center[axis]) * vertical[axis] for axis in range(3))
            direction_length = math.hypot(dx, dy)
            dx, dy = dx / direction_length, dy / direction_length
            radius = min(width, height) * .48
            start, base = len(indices), len(vertices)
            for across, up in ((dx, dy), (-dx * .6 - dy * .85, -dy * .6 + dx * .85),
                               (-dx * .6 + dy * .85, -dy * .6 - dx * .85)):
                point = (center[0] + radius * across,
                         center[1] + radius * up * vertical[1] + .0005 * normal[1],
                         center[2] + radius * up * vertical[2] + .0005 * normal[2])
                vertices.append((*point, *normal, *green_uv))
            indices.extend((base, base + 1, base + 2))
            overlay_draws.append((f'selector:{marker}', start, 3))

        # MFD plugin-device touch quads go last, so no existing overlay
        # offset changes. Explicit seat-view corners (BL, TL, TR, BR) with
        # UV (0,0)..(1,1): never add_quad's 20-degree main-panel basis, the
        # MFD faces are tilted 54 degrees back (spec 09b section 2.3).
        for side, device, quad in mfd_device_quads(MFD_TOUCH_LIFT):
            start, base = len(indices), len(vertices)
            vertices.extend(quad)
            indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))
            device_touch_draws.append((side, device, start))

    lines = [
        'I',
        '800',
        'OBJ',
        '# A380X FLYPAD ORIENTATION: UV0 TOUCH0 A380_CONTROL_UI_V3',
        '',
        *(
            [
                'TEXTURE\t../../../cockpit_3d/fcu_labels.png',
                'TEXTURE_LIT\t../../../cockpit_3d/fcu_labels_LIT.png',
                f'COCKPIT_REGION\t0\t0\t{PANEL_SIZE[0]}\t{PANEL_SIZE[1]}',
                '',
            ]
            if fcu_overlay else []
        ),
        f'POINT_COUNTS\t{len(vertices)}\t0\t0\t{len(indices)}',
        '',
    ]
    lines.extend(
        f'VT\t{obj8_fmt(x)}\t{obj8_fmt(y)}\t{obj8_fmt(z)}\t{obj8_fmt(nx)}\t{obj8_fmt(ny)}\t{obj8_fmt(nz)}\t{obj8_fmt(u)}\t{obj8_fmt(v)}'
        for x, y, z, nx, ny, nz, *texture_uv in vertices
        for u, v in (texture_uv if texture_uv else (0.0, 0.0),)
    )
    lines.append('')
    packed_count = len(indices) - len(indices) % 10
    for offset in range(0, packed_count, 10):
        lines.append('IDX10\t' + '\t'.join(map(str, indices[offset:offset + 10])))
    lines.extend(f'IDX\t{index}' for index in indices[packed_count:])
    lines.extend(['', 'ATTR_draw_disable'])
    for _group_source, control, start, count, _triangles in draws:
        if control.visible_dataref is not None:
            value = control.visible_value
            lines.extend(['ANIM_begin',
                f'ANIM_hide\t-100\t{obj8_fmt(value - .5)}\t{control.visible_dataref}',
                f'ANIM_hide\t{obj8_fmt(value + .5)}\t100\t{control.visible_dataref}'])
        if control.command is not None:
            lines.append(f'ATTR_manip_command\t{control.command_cursor}\t{control.command}\t{control.label}')
        elif control.command_up is not None and control.command_down is not None:
            manipulator = ('command_switch_up_down' if control.switch_axis == 'up_down'
                           else 'command_knob')
            lines.append(
                f'ATTR_manip_{manipulator}\t{control.cursor}\t{control.command_up}\t'
                f'{control.command_down}\t{control.label}'
            )
        elif control.toggle_values is not None:
            low, high = (float(value) for value in control.toggle_values)
            lines.append(
                f'ATTR_manip_toggle\t{control.cursor}\t{obj8_fmt(low)}\t{obj8_fmt(high)}\t'
                f'{control.dataref}\t{control.label}'
            )
        else:
            axis = '\t'.join(f'{obj8_fmt(float(value))}' for value in control.axis or ())
            lines.append(
                f'ATTR_manip_drag_axis\thand\t{axis}\t{obj8_fmt(float(control.value_min))}\t'
                f'{obj8_fmt(float(control.value_max))}\t{control.dataref}\t{control.label}'
            )
        motion_node = control.node.split(':', 1)[0] if control.node.startswith('KNOB_FCU_') else control.node
        motion = control_motions.get((_group_source, motion_node))
        if motion:
            lines.extend(animation_block(motion, start, count))
        else:
            lines.append(f'TRIS\t{start}\t{count}')
        lines.append('ATTR_manip_none')
        if control.visible_dataref is not None:
            lines.append('ANIM_end')
    # Device tag -> device manipulator -> TRIS -> manip_none -> no_cockpit
    # (spec 08 section 2.5 b). Never hidden: input is gated in the plugin.
    for side, device, start in device_touch_draws:
        lines.extend([
            mfd_device_attribute(side, device),
            f'ATTR_manip_device\tarrow\t{device}\t{MFD_SEATS[side]} MFD',
            f'TRIS\t{start}\t6',
            'ATTR_manip_none',
            'ATTR_no_cockpit',
        ])
    lines.append('ATTR_draw_enable')
    if fcu_overlay:
        screen_draws = [
            draw for draw in overlay_draws
            if any(draw[0].startswith(prefix) for prefix in ('screen:', 'pfd:', 'nd:', 'ewd:', 'efb:', 'efis:', 'clock:', 'accu:'))
        ]
        lines.extend(['ATTR_cockpit_region\t0', 'ATTR_poly_os\t2'])
        lines.extend(f'TRIS\t{start}\t{count}' for _name, start, count in screen_draws)
        lines.extend(['ATTR_no_cockpit', 'ATTR_poly_os\t0'])
        lamp_draws = {name.removeprefix('lamp:'): (start, count)
                      for name, start, count in overlay_draws if name.startswith('lamp:')}
        if lamp_draws:
            lines.append('ATTR_poly_os\t2')
            fcu_lamp_nodes = {row[1] for row in FCU_ANNUNCIATORS}
            for _label, lamp_node, dataref in [*FCU_ANNUNCIATORS, *EFIS_ANNUNCIATORS,
                *((label,node,ref) for label,node,ref,_color in GLARE_ANNUNCIATORS)]:
                if lamp_node not in lamp_draws:
                    continue
                start, count = lamp_draws[lamp_node]
                lines.append(f'# FCU ANNUNCIATOR {lamp_node}')
                if lamp_node in fcu_lamp_nodes:
                    # FCU light values already include power, mode state and
                    # integral dimming. A dim but active 0.25 lamp stays lit.
                    lines.extend([f'ATTR_light_level\t0\t1\t{dataref}', 'ANIM_begin',
                                  f'ANIM_hide\t-0.0001\t0.0001\t{dataref}'])
                else:
                    # Preserve the existing EFIS/glareshield electrical gate
                    # and binary state thresholds independently of FCU dimming.
                    lines.extend(['ATTR_light_level\t0\t1\tsim/cockpit2/switches/avionics_power_on',
                                  'ANIM_begin', 'ANIM_hide\t0\t0\tsim/cockpit2/switches/avionics_power_on',
                                  f'ANIM_hide\t-0.5\t0.5\t{dataref}'])
                motion = next((target for (_path, name), target in control_motions.items()
                               if name == lamp_node), None)
                lines.extend(animation_block(motion, start, count) if motion
                             else [f'TRIS\t{start}\t{count}'])
                lines.extend(['ANIM_end', 'ATTR_light_level_reset'])
            lines.append('ATTR_poly_os\t0')
        adirs_draws = {name.removeprefix('adirs-lamp:'): (start, count)
                       for name, start, count in overlay_draws if name.startswith('adirs-lamp:')}
        if adirs_draws:
            lines.append('ATTR_poly_os\t2')
            for _label, lamp_node, dataref, _color in ADIRS_ANNUNCIATORS:
                if lamp_node not in adirs_draws:
                    continue
                start, count = adirs_draws[lamp_node]
                # The ADIRS runtime owns electrical gating, including battery
                # operation. The general avionics switch must not mask it.
                lines.extend([f'# ADIRS ANNUNCIATOR {lamp_node}',
                              f'ATTR_light_level\t0\t1\t{dataref}', 'ANIM_begin',
                              f'ANIM_hide\t-0.5\t0.5\t{dataref}'])
                motion = next((target for (_path, name), target in control_motions.items()
                               if name == lamp_node), None)
                lines.extend(animation_block(motion, start, count) if motion
                             else [f'TRIS\t{start}\t{count}'])
                lines.extend(['ANIM_end', 'ATTR_light_level_reset'])
            lines.append('ATTR_poly_os\t0')
        selector_draws = {name.removeprefix('selector:'): (start, count)
                          for name, start, count in overlay_draws if name.startswith('selector:')}
        if selector_draws:
            lines.extend(['ATTR_poly_os\t2',
                          'ATTR_light_level\t0\t1\tsim/cockpit2/switches/avionics_power_on'])
            for marker, _knob, dataref, value, _square in EFIS_SELECTORS:
                if marker not in selector_draws:
                    continue
                start, count = selector_draws[marker]
                lines.extend([f'# EFIS SELECTOR {marker}', 'ANIM_begin',
                    'ANIM_hide\t0\t0\tsim/cockpit2/switches/avionics_power_on',
                    f'ANIM_hide\t-100\t{obj8_fmt(value - .5)}\t{dataref}',
                    f'ANIM_hide\t{obj8_fmt(value + .5)}\t100\t{dataref}',
                    f'TRIS\t{start}\t{count}', 'ANIM_end'])
            lines.extend(['ATTR_light_level_reset', 'ATTR_poly_os\t0'])
    lines.extend(['', '# Generated from named FBW cockpit control meshes.'])
    report = {
        'source': str(source),
        'sources': [
            {'source': str(group_source), 'controls': len(group_controls)}
            for group_source, group_controls in source_groups
        ],
        'controls': [
            {
                'source': str(group_source),
                'node': control.node,
                'manipulator': (
                    'command' if control.command is not None
                    else 'command_switch_up_down' if control.switch_axis == 'up_down'
                    else 'command_knob' if control.command_up is not None
                    else 'toggle' if control.toggle_values is not None
                    else 'drag_axis'
                ),
                'command': control.command,
                'command_up': control.command_up,
                'command_down': control.command_down,
                'dataref': control.dataref,
                'toggle_values': control.toggle_values,
                'label': control.label,
                'triangles': triangles,
            }
            for group_source, control, _start, _count, triangles in draws
        ],
        'vertices': len(vertices),
        'indices': len(indices),
        'triangles': len(indices) // 3,
        'fcu_overlay': fcu_overlay,
        'fcu_label_quads': 0,
        'fcu_annunciator_quads': sum(node in annunciator_faces for _label,node,_ref in FCU_ANNUNCIATORS),
        'glareshield_annunciator_quads': sum(node in annunciator_faces for _label,node,_ref,_color in GLARE_ANNUNCIATORS),
        'adirs_annunciator_quads': len(adirs_faces),
        'adirs_annunciators': [
            {'node': node, 'dataref': ref, 'corners': adirs_faces[node][0]}
            for _label, node, ref, _color in ADIRS_ANNUNCIATORS if node in adirs_faces
        ],
        'fcu_annunciators': [
            {'node': node, 'source_button': node.removesuffix('_SEQ2'),
             'dataref': dataref, 'bar_face': annunciator_faces[node]}
            for _label, node, dataref in FCU_ANNUNCIATORS if node in annunciator_faces
        ],
        'fcu_screen_quads': len(FCU_SCREEN_QUADS) if fcu_overlay else 0,
        'efis_screen_quads': 22 if fcu_overlay else 0,
        'clock_screen_quads': 3 if fcu_overlay else 0,
        'gear_annunciators': [
            {'node': node, 'dataref': ref, 'face': annunciator_faces[node]}
            for _label, node, ref, _color in GLARE_ANNUNCIATORS
            if node in annunciator_faces and (node.startswith('GEAR_') or 'LDGGEAR' in node)
        ],
        'efis_annunciator_quads': sum(node in annunciator_faces for _label, node, _ref in EFIS_ANNUNCIATORS),
        'efis_selector_markers': len(selector_faces),
        'nd_plan_touch_targets': 4 if fcu_overlay else 0,
        'pfd_screen_quads': len(PFD_SCREEN_SURFACES) if fcu_overlay else 0,
        'nd_screen_quads': len(ND_SCREEN_SURFACES) if fcu_overlay else 0,
        'ewd_screen_quads': 1 if fcu_overlay else 0,
        'efb_screen_quads': len(EFB_SCREEN_SURFACES) if fcu_overlay else 0,
        'efb_touch_targets': (
            len(EFB_SCREEN_SURFACES)
            * (len(EFB_PAGE_TOUCH_RECTS) + len(EFB_ACTION_TOUCH_RECTS) + 1)
            if fcu_overlay else 0
        ),
        'mfd_device_touch_quads': len(device_touch_draws),
        'mfd_devices': [device for _side, device, _start in device_touch_draws],
        'kccu_targets': sum(control.node.startswith('PUSH_KCCU')
                            for _source, control, _start, _count, _triangles in draws),
        'limitations': [
            'FCU controls are routed exclusively through the aircraft-native FBW-derived guidance plugin.',
            'The source FCU supplies the daylight legends; configured labels also remain available as mouse hover tooltips.',
            'Mapped overhead, EFIS, and engine-master controls use native-state movement; FCU click regions and engagement bars follow independent authored mechanical travel. Retained FCU meshes are animated by patch_fcu_animations.py; throttle travel is not yet animated.',
            'Passenger-door click targets use the authored closed-position handle meshes; native door commands remain bindable when a door is open.',
        ],
    }
    return '\n'.join(lines) + '\n', report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='FBW A380 cockpit glTF')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--commands', type=Path, help='Optional installed X-Plane Commands.txt validation')
    parser.add_argument('--datarefs', type=Path, help='Optional installed X-Plane DataRefs.txt validation')
    parser.add_argument('--fcu-overlay', action='store_true',
                        help='Add four live values across the two authored AFS display windows')
    parser.add_argument('--mfd-screens-output', type=Path,
                        help='Also write the visible MFD device-screen OBJ and its black '
                             f'{MFD_SCREEN_TEXTURE} texture into this OBJ path\'s directory')
    parser.add_argument('--native-nd', action='store_true', help='Add the local xp-kit ND device screens and touch twins')
    parser.add_argument(
        '--additional-source', nargs=2, action='append', default=[], metavar=('SOURCE', 'CONFIG'),
        help='Append named controls from another glTF and JSON configuration to the same cockpit object',
    )
    args = parser.parse_args()
    controls = load_controls(args.config)
    known = command_catalog(args.commands) if args.commands else None
    known_datarefs = command_catalog(args.datarefs) if args.datarefs else None
    additional_sources = [
        (Path(source), load_controls(Path(config)))
        for source, config in args.additional_source
    ]
    encoded, report = build_obj(
        args.source, controls, known, known_datarefs, args.fcu_overlay, additional_sources,
    )
    if args.native_nd:
        from nd_cockpit_devices import augment
        encoded=augment(encoded,True)
        report['native_nd_device_touch_quads']=2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(encoded, encoding='ascii', newline='\n')
    if args.mfd_screens_output:
        args.mfd_screens_output.parent.mkdir(parents=True, exist_ok=True)
        screens=build_mfd_screens_obj()
        if args.native_nd:
            screens=augment(screens,False)
        args.mfd_screens_output.write_text(screens, encoding='ascii', newline='\n')
        (args.mfd_screens_output.parent / MFD_SCREEN_TEXTURE).write_bytes(mfd_screen_black_png())
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
