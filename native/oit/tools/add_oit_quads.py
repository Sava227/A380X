"""Add the two OIT cockpit-device quads to objects/interior/cockpit-panels/a380x_mfd_screens.obj.

The quads lie exactly on the OIT glass of screen_powered_off_display_faces.obj (TRIS 186 captain, 198 F/O),
corners as seen from each seat BL, TL, TR, BR, UV (0,0) = device bottom-left, drawn with ATTR_poly_os 2 over the
dark face like every other screen in this file. Devices: a380x.xp12.oit.captain (bus 1, lighting channel 26) and
a380x.xp12.oit.first_officer (bus 1, channel 27), created by the A380XOIT plugin. Idempotent.
Usage: add_oit_quads.py <a380x_mfd_screens.obj>
"""
import sys

# (device, channel, normal, BL, TL, TR, BR) from the powered-off faces (OBJ frame: +x = captain side).
QUADS = [
    ('a380x.xp12.oit.captain', 26, (-0.523, 0.293, -0.8),
     (1.0240, 2.5305, 32.6737), (1.0431, 2.7145, 32.7310), (0.8324, 2.6955, 32.8621), (0.8139, 2.5112, 32.8059)),
    ('a380x.xp12.oit.first_officer', 27, (0.523, 0.293, -0.8),
     (-0.8140, 2.5111, 32.8060), (-0.8324, 2.6955, 32.8622), (-1.0432, 2.7144, 32.7311), (-1.0241, 2.5304, 32.6739)),
]
UVS = ((0, 0), (0, 1), (1, 1), (1, 0))
MARK = '# A380X OIT DEVICE QUADS V1'


def main(path):
    text = open(path).read()
    if MARK in text:
        print('already present:', path)
        return
    lines = text.split('\n')
    vt = [i for i, l in enumerate(lines) if l.startswith('VT')]
    idx_rows = [i for i, l in enumerate(lines) if l.startswith('IDX')]
    pc = next(i for i, l in enumerate(lines) if l.startswith('POINT_COUNTS'))
    nv, ni = int(lines[pc].split()[1]), int(lines[pc].split()[4])
    assert nv == len(vt), (nv, len(vt))
    new_vt, new_idx, section = [], [], ['', MARK + ' (A380XOIT plugin; tools/add_oit_quads.py)', 'ATTR_no_cockpit',
                                         'ATTR_poly_os\t2', 'ATTR_draw_enable']
    for k, (dev, ch, n, *corners) in enumerate(QUADS):
        base = nv + 4 * k
        for (x, y, z), (u, v) in zip(corners, UVS):
            new_vt.append(f'VT\t{x}\t{y}\t{z}\t{n[0]}\t{n[1]}\t{n[2]}\t{u}\t{v}')
        new_idx += [base, base + 1, base + 2, base, base + 2, base + 3]
        section += [f'ATTR_cockpit_device\t{dev}\t1\t{ch}\t1', f'TRIS\t{ni + 6 * k}\t6', 'ATTR_no_cockpit']
    section += ['ATTR_poly_os\t0', '']
    lines[pc] = f'POINT_COUNTS\t{nv + len(new_vt)}\t0\t0\t{ni + len(new_idx)}'
    last_idx = idx_rows[-1]
    lines[last_idx + 1:last_idx + 1] = [f'IDX\t{i}' for i in new_idx]
    lines[vt[-1] + 1:vt[-1] + 1] = new_vt
    # the section goes at the end of the drawing commands (before trailing blank lines)
    while lines and lines[-1] == '':
        lines.pop()
    lines += section
    open(path, 'w').write('\n'.join(lines) + '\n')
    print(f'added 2 OIT quads: vertices {nv} -> {nv + len(new_vt)}, indices {ni} -> {ni + len(new_idx)}')


if __name__ == '__main__':
    main(sys.argv[1])
