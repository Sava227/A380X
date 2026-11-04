"""Refresh only the eight MFD touch vertices in an existing cockpit object.

Keeps all other meshes, attributes, indices and user edits byte-for-byte. The
matching device/UV/normal contract is checked before any replacement is returned.
"""
from build_cockpit_controls import MFD_TOUCH_LIFT, mfd_device_quads
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation


def refresh_touch_geometry(text):
    lines = text.splitlines(keepends=True)
    vertex_rows = [i for i, line in enumerate(lines) if line.startswith('VT\t')]
    indices = [int(value) for line in lines if line.startswith(('IDX\t', 'IDX10\t'))
               for value in line.split()[1:]]
    for _side, device, vertices in mfd_device_quads(MFD_TOUCH_LIFT):
        marker = f'ATTR_manip_device\tarrow\t{device}\t'
        matches = [i for i, line in enumerate(lines) if line.startswith(marker)]
        if len(matches) != 1:
            raise ValueError(f'Expected one MFD touch surface for {device}')
        at = matches[0]
        if lines[at - 1].split()[:2] != ['ATTR_cockpit_device', device]:
            raise ValueError(f'Missing matching device tag: {device}')
        command, start, count = lines[at + 1].split()
        start, count = int(start), int(count)
        ids = indices[start:start + count]
        if command != 'TRIS' or count != 6 or len(ids) != 6:
            raise ValueError(f'Invalid MFD touch triangles: {device}')
        base = ids[0]
        if ids != [base, base + 1, base + 2, base, base + 2, base + 3]:
            raise ValueError(f'Unexpected MFD winding: {device}')
        for vertex_id, vertex in zip(range(base, base + 4), vertices):
            row = vertex_rows[vertex_id]
            previous = tuple(map(float, lines[row].split()[1:]))
            if len(previous) != 8 or any(abs(a - b) > 1e-6 for a, b in zip(previous[3:], vertex[3:])):
                raise ValueError(f'MFD UV/normal mismatch: {device}')
            if sum((a - b) ** 2 for a, b in zip(previous[:3], vertex[:3])) > .005 ** 2:
                raise ValueError(f'MFD geometry is not the authored screen: {device}')
            lines[row] = 'VT\t' + '\t'.join(f'{obj8_fmt(value)}' for value in vertex) + '\n'
    return ''.join(lines)
