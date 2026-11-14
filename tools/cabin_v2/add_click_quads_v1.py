"""Add click spots to the clickable cockpit object (objects/interior/cockpit-controls/a380x_cockpit_controls.obj, the
only object with Plane Maker's clickable flag; all objects share its frame: z 115.108, psi 180).

  * OIT touch: invisible device quads a380x.xp12.oit.captain / .first_officer with ATTR_manip_device, like the EFB
    quads (the visible OIT quads are in a380x_mfd_screens.obj).
  * Cabin: an invisible quad over every main-deck window (toggles a380x/xp12/cabin/shade[i]) and in front of every
    overhead bin door (its bulge + 15 mm toward the aisle) (toggles a380x/xp12/cabin/bin[j]), from manifest.json of cabin_anim.py.

Idempotent (section markers). Usage: add_click_quads.py <a380x_cockpit_controls.obj> <manifest.json>
"""
import json, math, sys

OIT = [  # device, corners BL TL TR BR (same as the visible quads, tools/add_oit_quads.py of the OIT plugin)
    ('a380x.xp12.oit.captain', 'A380 OIT CAPT - touch', (-0.523, 0.293, -0.8),
     [(1.0240, 2.5305, 32.6737), (1.0431, 2.7145, 32.7310), (0.8324, 2.6955, 32.8621), (0.8139, 2.5112, 32.8059)], 26),
    ('a380x.xp12.oit.first_officer', 'A380 OIT F/O - touch', (0.523, 0.293, -0.8),
     [(-0.8140, 2.5111, 32.8060), (-0.8324, 2.6955, 32.8622), (-1.0432, 2.7144, 32.7311), (-1.0241, 2.5304, 32.6739)], 27),
]
OIT_MARK = '# A380X OIT TOUCH QUADS V1'
CABIN_MARK = '# A380X CABIN CLICK SPOTS V1'
SHADE_X = 3.25      # |x| of the window click plane (inboard of the bezel)
BIN_OFFSET = 0.015  # m toward the aisle from the door chord


def main(obj_path, manifest_path):
    text = open(obj_path).read()
    lines = text.split('\n')
    m = json.load(open(manifest_path))
    vt_rows = [i for i, l in enumerate(lines) if l.startswith('VT')]
    idx_rows = [i for i, l in enumerate(lines) if l.startswith('IDX')]
    pc = next(i for i, l in enumerate(lines) if l.startswith('POINT_COUNTS'))
    p = lines[pc].split()
    nv, ni = int(p[1]), int(p[4])
    assert nv == len(vt_rows)
    new_vt, new_idx, section = [], [], []

    def quad(corners, n):
        base = nv + len(new_vt)
        for (x, y, z), (u, v) in zip(corners, ((0, 0), (0, 1), (1, 1), (1, 0))):
            new_vt.append('VT\t%.5f\t%.5f\t%.5f\t%.4f\t%.4f\t%.4f\t%d\t%d' % (x, y, z, n[0], n[1], n[2], u, v))
        start = ni + len(new_idx)
        new_idx.extend([base, base + 1, base + 2, base, base + 2, base + 3])
        return start

    if OIT_MARK not in text:
        section += ['', OIT_MARK + ' (A380XOIT plugin)', 'ATTR_no_cockpit', 'ATTR_poly_os\t2', 'ATTR_draw_disable']
        for dev, tip, n, corners, ch in OIT:
            s = quad(corners, n)
            section += [f'ATTR_cockpit_device\t{dev}\t1\t{ch}\t1', f'ATTR_manip_device\tbutton\t{dev}\t{tip}',
                        f'TRIS\t{s}\t6', 'ATTR_manip_none', 'ATTR_no_cockpit']
        section += ['ATTR_draw_enable', 'ATTR_poly_os\t0']
    if CABIN_MARK not in text:
        section += ['', CABIN_MARK + ' (A380XDoors plugin; cabin-bins-shades/tools)', 'ATTR_no_cockpit',
                    'ATTR_draw_disable', 'ATTR_no_cull']
        for sh in m['shades']:
            sx = SHADE_X if sh['x'] > 0 else -SHADE_X
            n = (-1 if sx > 0 else 1, 0, 0)
            y0, y1, z0, z1 = sh['y0'], sh['y1'], sh['z0'], sh['z1']
            s = quad([(sx, y0, z0), (sx, y1, z0), (sx, y1, z1), (sx, y0, z1)], n)
            section += [f"ATTR_manip_toggle\thand\t1\t0\ta380x/xp12/cabin/shade[{sh['index']}]\tWindow shade",
                        f'TRIS\t{s}\t6']
        for b in m['bins']:
            hx, hy = b['hinge']; fx, fy = b['free']
            cx, cy = hx - fx, hy - fy
            l = math.hypot(cx, cy)
            px, py = -cy / l, cx / l           # perpendicular to the chord; take the one pointing down (aisle side)
            if py > 0:
                px, py = -px, -py
            off = b.get('bulge', 0.0) + BIN_OFFSET      # in front of the curved door
            ox, oy = px * off, py * off
            z0, z1 = b['z0'] + 0.02, b['z1'] - 0.02
            s = quad([(fx + ox, fy + oy, z0), (hx + ox, hy + oy, z0), (hx + ox, hy + oy, z1), (fx + ox, fy + oy, z1)],
                     (px, py, 0))
            section += [f"ATTR_manip_toggle\thand\t1\t0\ta380x/xp12/cabin/bin[{b['index']}]\tOverhead bin",
                        f'TRIS\t{s}\t6']
        section += ['ATTR_manip_none', 'ATTR_cull', 'ATTR_draw_enable']
    if not section:
        print('already present:', obj_path)
        return
    lines[pc] = f'POINT_COUNTS\t{nv + len(new_vt)}\t{p[2]}\t{p[3]}\t{ni + len(new_idx)}'
    lines[idx_rows[-1] + 1:idx_rows[-1] + 1] = [f'IDX\t{i}' for i in new_idx]
    lines[vt_rows[-1] + 1:vt_rows[-1] + 1] = new_vt
    while lines and lines[-1] == '':
        lines.pop()
    lines += section
    open(obj_path, 'w').write('\n'.join(lines) + '\n')
    print(f'click spots added: vertices {nv} -> {nv + len(new_vt)}, indices {ni} -> {ni + len(new_idx)}, '
          f"shades {len(m['shades'])}, bins {len(m['bins'])}")


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
