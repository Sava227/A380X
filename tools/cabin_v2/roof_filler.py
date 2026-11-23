"""Close the holes in the upper-deck ceiling (A380X cabin v2).

The upper-deck cabin has no ceiling / upper sidewall around the doors (U1 z 12.3-13.9, U2 z -6.6..-3.2), aft of the
last windows (z -20..-12.8, around and behind U3), and a narrow centre strip whose panels face up (culled from
below), so the sky showed through. cast.json (rays from the upper-deck centreline, x 0 y 5.0, every 0.1 m in z and
2 deg in angle, against the interior objects and against the fuselage skin; tools/roof_cast.py) gives, per ray, the
distance to the first interior surface and whether it faces the viewer.

The cast looks through back faces (X-Plane culls them). Every cell between ZMIN and ZMAX whose ray reaches no
surface although the fuselage skin is behind it (so not a window) gets a quad of a plain light surface; the door
openings (DOORS, beyond DOOR_A from vertical) are never covered, so open doors stay clear. Its distance is filled in smoothly from the surrounding
ceiling (Laplace in-paint over the hole cells), pushed OUT mm behind it so the edges tuck under the existing panels,
and kept at least SKIN_GAP inside the fuselage skin. Facing the cabin. Appended to the upper-deck sidewall object.

Usage: roof_filler.py <cast.json from roof_cast_cull.py> <in fc obj> <aircraft dir> <out fc obj>
"""
import json, math, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cabin_anim2 as c2

AMAX = 112         # deg
DOOR_A = 73        # deg: door openings are beyond this angle from vertical
DOORS = [(13.54, 14.85), (-5.86, -4.55), (-14.75, -13.44)]     # upper-deck door z ranges (closed)
ZMIN, ZMAX = -20.0, 23.4
OUT = 0.02         # m behind the surrounding surface
SKIN_GAP = 0.04    # m inside the fuselage skin
COLOUR = (228.0, 230.0, 232.0)
HANDLE_PHASE = 0.12   # as doors_v2.py: the upper doors stay shut for the first 12 % of the ratio
MARK = '# A380X UPPER DECK CEILING FILLER V1 (cabin-v2-20261114/tools/roof_filler.py)'


def main(cast, src, acf, dst):
    r = json.load(open(cast))
    z, a = np.array(r['z']), np.array(r['a'], float)
    y0 = r['y0']
    din, kind, dex = np.array(r['din'], float), np.array(r['kind']), np.array(r['dex'], float)   # [z, a]
    door = np.zeros_like(kind, bool)
    for z0, z1 in DOORS:
        door |= (z[:, None] > z0 - 0.06) & (z[:, None] < z1 + 0.06) & (np.abs(a)[None, :] >= DOOR_A)
    zone = (np.abs(a)[None, :] <= AMAX) & (z[:, None] >= ZMIN) & (z[:, None] <= ZMAX) & ~door & (dex > 0)
    hole = (kind == 0) & zone
    # dilate one cell inside the zone so the patch edges overlap the existing panels
    d = hole.copy()
    d[1:, :] |= hole[:-1, :]; d[:-1, :] |= hole[1:, :]; d[:, 1:] |= hole[:, :-1]; d[:, :-1] |= hole[:, 1:]
    fill = d & zone
    # Laplace in-paint of the distance over the hole cells from the front hits
    known = (kind == 1)
    R = np.where(known, din, np.nan)
    R0 = np.nanmean(np.where(zone & known, din, np.nan))
    R = np.where(np.isnan(R), R0, R)
    for _ in range(3000):
        P = np.pad(R, 1, mode='edge')
        avg = (P[:-2, 1:-1] + P[2:, 1:-1] + P[1:-1, :-2] + P[1:-1, 2:]) / 4
        R = np.where(known, R, avg)
    rad = R + OUT
    skin = np.where(dex > 0, dex - SKIN_GAP, np.inf)
    clamped = int((rad > skin).sum() * 0 + ((rad > skin) & fill).sum())
    rad = np.minimum(rad, skin)
    # vertices on the ray grid
    o = c2.Obj(src, 'objects/interior/cabin/cabin_a380x_fc_deck_sidwealls.obj')
    uv, score = c2.match_uv(c2.tex_path(acf, o.lines), np.array(COLOUR), o.uvs(20))
    base = len(o.V)
    vid, new_vt, idx = {}, [], []

    def vert(i, j):
        if (i, j) not in vid:
            ar = math.radians(a[j]); s, c = math.sin(ar), math.cos(ar)
            p = (s * rad[i, j], y0 + c * rad[i, j], z[i])
            vid[(i, j)] = base + len(new_vt)
            new_vt.append('VT\t%.5f\t%.5f\t%.5f\t%.4f\t%.4f\t0\t%.6f\t%.6f' % (p[0], p[1], p[2], -s, -c, uv[0], uv[1]))
        return vid[(i, j)]

    def pos(i, j):
        ar = math.radians(a[j])
        return np.array([math.sin(ar) * rad[i, j], y0 + math.cos(ar) * rad[i, j], z[i]])

    # holes inside a closed door's outline (gaps along the door edge) move with that door
    door_hole = np.zeros_like(hole)
    for z0, z1 in DOORS:
        door_hole |= (kind == 0) & (dex > 0) & (z[:, None] > z0 - 0.06) & (z[:, None] < z1 + 0.06) & \
            (np.abs(a)[None, :] >= DOOR_A) & (np.abs(a)[None, :] <= AMAX)
    cells = 0
    door_idx = {}
    for i in range(len(z) - 1):
        for j in range(len(a) - 1):
            q = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
            if not any(door_hole[c] for c in q) or not all(dex[c] > 0 for c in q):
                continue
            zc, ac = (z[i] + z[i + 1]) / 2, (a[j] + a[j + 1]) / 2
            dn = next(n for n, (z0, z1) in enumerate(DOORS) if z0 - 0.07 < zc < z1 + 0.07)
            k = 10 + 2 * dn + (0 if ac > 0 else 1)
            ids = [vert(*c) for c in q]
            P = [pos(*c) for c in q]
            mid = (P[0] + P[2]) / 2
            inward = np.array([-mid[0], y0 - mid[1], 0.0])
            t = [ids[0], ids[1], ids[2], ids[0], ids[2], ids[3]] if c2.is_front(P[0], P[1], P[2], inward) \
                else [ids[0], ids[2], ids[1], ids[0], ids[3], ids[2]]
            door_idx.setdefault(k, []).extend(t)
    n_door = sum(len(v) for v in door_idx.values()) // 6
    for i in range(len(z) - 1):
        for j in range(len(a) - 1):
            if not (fill[i, j] or fill[i + 1, j] or fill[i, j + 1] or fill[i + 1, j + 1]):
                continue
            if not (zone[i, j] and zone[i + 1, j] and zone[i, j + 1] and zone[i + 1, j + 1]):
                continue
            q = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
            ids = [vert(*c) for c in q]
            P = [pos(*c) for c in q]
            mid = (P[0] + P[2]) / 2
            inward = np.array([-mid[0], y0 - mid[1], 0.0])
            if c2.is_front(P[0], P[1], P[2], inward):
                idx += [ids[0], ids[1], ids[2], ids[0], ids[2], ids[3]]
            else:
                idx += [ids[0], ids[2], ids[1], ids[0], ids[3], ids[2]]
            cells += 1
    # write: vertices after the last VT, indices at the end, the draw call last
    lines = o.lines
    I = np.concatenate([o.I, np.array(idx + [x for k in sorted(door_idx) for x in door_idx[k]], dtype=o.I.dtype)])
    M = len(I)
    idx_lines = ['IDX10\t' + '\t'.join(str(x) for x in I[s:s + 10]) for s in range(0, M - M % 10, 10)]
    idx_lines += [f'IDX\t{x}' for x in I[M - M % 10:]]
    last_vt = max(i for i, l in enumerate(lines) if l.startswith('VT'))
    out, done = [], False
    for i, l in enumerate(lines):
        if l.startswith('POINT_COUNTS'):
            p = l.split()
            out.append(f'POINT_COUNTS\t{len(o.V) + len(new_vt)}\t{p[2]}\t{p[3]}\t{M}')
        elif i == last_vt:
            out.append(l); out.extend(new_vt)
        elif l.startswith('IDX'):
            if not done:
                out.extend(idx_lines); done = True
        else:
            out.append(l)
    while out and out[-1].strip() == '':
        out.pop()
    out += ['', MARK, 'ATTR_draw_enable', 'ATTR_cull', f'TRIS\t{len(o.I)}\t{len(idx)}']
    start = len(o.I) + len(idx)
    for k, t in sorted(door_idx.items()):
        sx = 0.55 if k % 2 == 0 else -0.55
        h = HANDLE_PHASE
        out += ['ANIM_begin', f'\tANIM_trans_begin\tsim/flightmodel2/misc/door_open_ratio[{k}]',
                '\tANIM_trans_key\t0\t0\t0\t0', f'\tANIM_trans_key\t{h}\t0\t0\t0',
                f'\tANIM_trans_key\t{h + 0.2 * (1 - h):.6g}\t{sx}\t0\t0', f'\tANIM_trans_key\t1\t{sx}\t0\t1.3',
                '\tANIM_trans_end', f'\tTRIS\t{start}\t{len(t)}', 'ANIM_end']
        start += len(t)
    out += ['']
    open(dst, 'w').write('\n'.join(out))
    print(f'hole cells {int(hole.sum())}, filled quads {cells}, door-edge quads moving with the doors {n_door} {sorted(door_idx)}, vertices +{len(new_vt)}, clamped to the skin {clamped}, '
          f'uv {uv} (score {score:.1f})')


if __name__ == '__main__':
    main(*sys.argv[1:5])
