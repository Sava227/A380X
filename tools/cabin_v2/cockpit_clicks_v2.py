"""Click spots v2 in the clickable cockpit object (a380x_cockpit_controls.obj), from its pre-cabin original:

  * OIT touch quads (unchanged from v1, tools/add_click_quads_v1.py);
  * cabin: a quad over every window (199, from cabin_anim2.py's manifest 'click' corners) toggling
    a380x/xp12/cabin/shade[i], and in front of every overhead bin door (as v1) toggling a380x/xp12/cabin/bin[j];
  * passenger doors 1-16: an invisible box around the inside handle and one around the outside handle of every door,
    inside a copy of that handle's animation (doors_v2.py's door_clicks.json), so the spot moves with the handle and
    the door; each runs sim/flight_controls/door_toggle_N. The old static door spots (door_toggle_1..16) are retired
    (commented out), the cockpit door (door_toggle_17) is untouched.

Usage: cockpit_clicks_v2.py <original cockpit_controls.obj> <cabin manifest.json> <door_clicks.json> <out.obj>
"""
import json, math, re, sys
import add_click_quads_v1 as v1

DOOR_MARK = '# A380X DOOR CLICK SPOTS V2 (cabin-v2-20261114/tools)'
CABIN_MARK = '# A380X CABIN CLICK SPOTS V2 (A380XDoors plugin; cabin-v2-20261114/tools)'
RETIRED = '# A380X DOOR CLICK V1 RETIRED '


def door_name(k):
    deck, num = ('MAIN DECK', k // 2 + 1) if k < 10 else ('UPPER DECK', (k - 10) // 2 + 1)
    return f'{deck} {num} {"LEFT" if k % 2 == 0 else "RIGHT"}'


def main(src, manifest_path, doors_path, dst):
    lines = open(src).read().split('\n')
    m = json.load(open(manifest_path))
    doors = json.load(open(doors_path))
    assert not any(DOOR_MARK in l or CABIN_MARK in l or v1.OIT_MARK in l for l in lines), 'not the original'
    # retire the static door spots 1..16
    n_ret = 0
    for i, l in enumerate(lines):
        mm = re.match(r'^ATTR_manip_command\thand\tsim/flight_controls/door_toggle_(\d+)\t', l)
        if mm and 1 <= int(mm.group(1)) <= 16:
            assert lines[i + 1].startswith('TRIS')
            lines[i] = RETIRED + l
            lines[i + 1] = RETIRED + lines[i + 1]
            n_ret += 1
    vt_rows = [i for i, l in enumerate(lines) if l.startswith('VT')]
    idx_rows = [i for i, l in enumerate(lines) if l.startswith('IDX')]
    pc = next(i for i, l in enumerate(lines) if l.startswith('POINT_COUNTS'))
    p = lines[pc].split()
    nv, ni = int(p[1]), int(p[4])
    new_vt, new_idx, section = [], [], []

    def verts(pts, n, uvs):
        base = nv + len(new_vt)
        for (x, y, z), (u, v) in zip(pts, uvs):
            new_vt.append('VT\t%.5f\t%.5f\t%.5f\t%.4f\t%.4f\t%.4f\t%d\t%d' % (x, y, z, n[0], n[1], n[2], u, v))
        return base

    def quad(corners, n):
        base = verts(corners, n, ((0, 0), (0, 1), (1, 1), (1, 0)))
        start = ni + len(new_idx)
        new_idx.extend([base, base + 1, base + 2, base, base + 2, base + 3])
        return start

    def box(c8):
        base = verts(c8, (0, 1, 0), [(0, 0)] * 8)
        start = ni + len(new_idx)
        # corner index = 4 i + 2 j + k (x, y, z low/high)
        for a, b, c, d in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):
            new_idx.extend([base + a, base + b, base + c, base + a, base + c, base + d])
        return start

    # OIT (as v1)
    section += ['', v1.OIT_MARK + ' (A380XOIT plugin)', 'ATTR_no_cockpit', 'ATTR_poly_os\t2', 'ATTR_draw_disable']
    for dev, tip, n, corners, ch in v1.OIT:
        s = quad(corners, n)
        section += [f'ATTR_cockpit_device\t{dev}\t1\t{ch}\t1', f'ATTR_manip_device\tbutton\t{dev}\t{tip}',
                    f'TRIS\t{s}\t6', 'ATTR_manip_none', 'ATTR_no_cockpit']
    section += ['ATTR_draw_enable', 'ATTR_poly_os\t0']
    # cabin
    section += ['', CABIN_MARK, 'ATTR_no_cockpit', 'ATTR_draw_disable', 'ATTR_no_cull']
    for sh in m['shades']:
        s = quad([tuple(c) for c in sh['click']], sh['normal'])
        section += [f"ATTR_manip_toggle\thand\t1\t0\ta380x/xp12/cabin/shade[{sh['index']}]\tWindow shade",
                    f'TRIS\t{s}\t6']
    for b in m['bins']:
        hx, hy = b['hinge']; fx, fy = b['free']
        cx, cy = hx - fx, hy - fy
        l = math.hypot(cx, cy)
        px, py = -cy / l, cx / l
        if py > 0:
            px, py = -px, -py
        off = b.get('bulge', 0.0) + v1.BIN_OFFSET
        ox, oy = px * off, py * off
        z0, z1 = b['z0'] + 0.02, b['z1'] - 0.02
        s = quad([(fx + ox, fy + oy, z0), (hx + ox, hy + oy, z0), (hx + ox, hy + oy, z1), (fx + ox, fy + oy, z1)],
                 (px, py, 0))
        section += [f"ATTR_manip_toggle\thand\t1\t0\ta380x/xp12/cabin/bin[{b['index']}]\tOverhead bin",
                    f'TRIS\t{s}\t6']
    section += ['ATTR_manip_none', 'ATTR_cull', 'ATTR_draw_enable']
    # doors
    section += ['', DOOR_MARK, 'ATTR_no_cockpit', 'ATTR_draw_disable', 'ATTR_no_cull']
    for d in sorted(doors, key=lambda d: (d['door'], d['where'])):
        k = d['door']
        s = box(d['box'])
        body = [f"ATTR_manip_command\thand\tsim/flight_controls/door_toggle_{k + 1}\t"
                f"{door_name(k)} door - open / close ({d['where']} handle)", f'TRIS\t{s}\t36', 'ATTR_manip_none']
        for block in reversed(d['chain']):
            body = ['ANIM_begin'] + ['\t' + a for a in block] + ['\t' + x for x in body] + ['ANIM_end']
        section += body
    section += ['ATTR_cull', 'ATTR_draw_enable']
    lines[pc] = f'POINT_COUNTS\t{nv + len(new_vt)}\t{p[2]}\t{p[3]}\t{ni + len(new_idx)}'
    lines[idx_rows[-1] + 1:idx_rows[-1] + 1] = [f'IDX\t{i}' for i in new_idx]
    lines[vt_rows[-1] + 1:vt_rows[-1] + 1] = new_vt
    while lines and lines[-1] == '':
        lines.pop()
    lines += section
    open(dst, 'w').write('\n'.join(lines) + '\n')
    print(f'{dst}: retired {n_ret} static door spots; vertices {nv} -> {nv + len(new_vt)}, indices {ni} -> '
          f"{ni + len(new_idx)}; shades {len(m['shades'])}, bins {len(m['bins'])}, door handle spots {len(doors)}")


if __name__ == '__main__':
    main(*sys.argv[1:5])
