"""Make the A380X cabin window shades and overhead bin doors movable.

The cabin sidewall objects (FBW model, exported with merged draw calls) contain:
  * window shades: flat 0.35 x 0.57 m panels parked in the wall cavity above each main-deck window;
  * overhead bin doors: curved panels, 1.2-1.3 m long -
      outboard bins  (0.50 x 0.31 m section, |x| ~2.04, with a 66-triangle bottom lip),
      centre bins    (0.58 x 0.29 m section, |x| ~0.64 main deck / ~0.68 upper deck).
Each piece is found as a connected part of a draw call (vertices welded by position). Its triangles are moved to
the end of their draw call's index range (no vertex changes) and drawn inside an animation:
  * shade i:  ANIM_trans_begin a380x/xp12/cabin/shade_ratio[i]   0 -> 1 slides it down over the window;
  * bin j:    ANIM_rotate_begin about its top edge, a380x/xp12/cabin/bin_ratio[j], 0 closed -> 1 open; the door
              swings its lower edge down and over toward the aisle and stops just above horizontal.
The ratios are animated by the A380XDoors plugin from the click targets a380x/xp12/cabin/shade[i] / bin[j], which
tools/add_click_quads.py adds as invisible quads to the clickable cockpit object.

Usage: cabin_anim.py <aircraft dir> <out dir>      (writes the edited objects and manifest.json into <out dir>)
"""
import json, math, os, sys
import numpy as np
from PIL import Image

OBJECTS = ['objects/interior/cabin/cabin_a380x_main_deck_yc_sidewalls.obj',
           'objects/interior/cabin/cabin_a380x_main_deck_pec_sidewalls.obj']
SHADE_DROP = 0.52          # m: parked centre 2.65 -> window centre 2.13
SHADE_OUT = 0.008          # m outboard while lowered: stays behind the window bezel
OPEN_UP_DEG = 10.0         # open doors stop this far above horizontal


def parse(path):
    lines = open(path, errors='replace').read().split('\n')
    V, I, calls, anim = [], [], [], []
    for i, l in enumerate(lines):
        p = l.split()
        if not p:
            continue
        k = p[0]
        if k == 'VT':
            V.append([float(x) for x in p[1:9]])
        elif k in ('IDX10', 'IDX'):
            I.extend(int(x) for x in p[1:])
        elif k == 'ANIM_begin':
            anim.append(1)
        elif k == 'ANIM_end':
            anim.pop()
        elif k == 'TRIS':
            calls.append({'line': i, 'off': int(p[1]), 'n': int(p[2]), 'anim': bool(anim)})
    return lines, np.array(V), np.array(I), calls


def pieces(V, I, call):
    t = I[call['off']:call['off'] + call['n']].reshape(-1, 3)
    key = np.round(V[t.ravel(), :3] / 1e-4).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3)
    parent = np.arange(inv.max() + 1)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b, c in inv:
        ra = find(a)
        parent[find(b)] = ra
        parent[find(c)] = ra
    roots = np.array([find(i) for i in inv[:, 0]])
    for r in np.unique(roots):
        tri = np.nonzero(roots == r)[0]
        P = V[t[tri].ravel(), :3]
        yield tri, P


def classify(P, ntri):
    lo, hi = P.min(0), P.max(0)
    s, c = hi - lo, (hi + lo) / 2
    if s[0] < 0.006 and abs(s[1] - 0.57) < 0.02 and abs(s[2] - 0.35) < 0.02 and ntri <= 8 and abs(c[0]) > 3.0:
        return 'shade'
    if abs(s[0] - 0.50) < 0.05 and abs(s[1] - 0.31) < 0.02 and 1.1 < s[2] < 1.45 and ntri >= 300 and 1.9 < abs(c[0]) < 2.2:
        return 'outboard'
    if abs(s[0] - 0.06) < 0.015 and s[1] < 0.02 and 1.1 < s[2] < 1.45 and 2.25 < abs(c[0]) < 2.4 and abs(c[1] - 2.81) < 0.03:
        return 'lip'
    if abs(s[0] - 0.58) < 0.03 and abs(s[1] - 0.29) < 0.02 and 0.85 < s[2] < 1.45 and ntri >= 1000 and 0.5 < abs(c[0]) < 0.8:
        return 'centre'
    return None


def hinge(P):
    """Hinge (top edge) and free edge (bottom edge) of a door, from its middle cross-section."""
    zc = (P[:, 2].min() + P[:, 2].max()) / 2
    sec = P[np.abs(P[:, 2] - zc) < 0.25]
    if len(sec) < 4:
        sec = P
    h = sec[np.argmax(sec[:, 1])]
    f = sec[np.argmin(sec[:, 1])]
    return h[:2], f[:2]


def open_angle(h, f):
    """Rotation about +z (degrees, right-handed) that swings the free edge down and over toward the aisle (the
    hinge's side of the free edge) to OPEN_UP_DEG above horizontal."""
    r = f - h
    phi0 = math.degrees(math.atan2(r[1], r[0]))
    aisle = 1.0 if h[0] > f[0] else -1.0          # aisle direction along x
    target = OPEN_UP_DEG if aisle > 0 else 180.0 - OPEN_UP_DEG
    if aisle > 0:      # increase the angle through -90 (down) to the target
        theta = target - phi0
        while theta < 0: theta += 360
    else:              # decrease the angle through -90 to the target (-180 + up)
        theta = (target - 360.0) - phi0
        while theta > 0: theta -= 360
    return theta


OUTBOARD_BACK = 2.80      # |x| of the outboard bin back wall (lining)
CENTRE_DIVIDER = 0.005    # |x| of the centre bin back-to-back divider (lining)


def plain_uv(acf_dir, obj_rel, lines, V, door_uvs):
    """A UV inside the bin doors' texture area whose 7x7 neighbourhood is the smoothest light plastic."""
    tex = next(l.split()[1] for l in lines if l.startswith('TEXTURE '))
    img = np.asarray(Image.open(os.path.normpath(os.path.join(acf_dir, os.path.dirname(obj_rel), tex))).convert('L')).astype(float)
    H, W = img.shape
    best, uv_best = 1e9, door_uvs[0]
    for u, v in door_uvs[:: max(1, len(door_uvs) // 400)]:
        x, y = int(u % 1.0 * W), int((1 - v % 1.0) * H)
        blk = img[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
        if blk.size and blk.mean() > 140 and blk.std() < best:
            best, uv_best = blk.std(), (u, v)
    return uv_best


def lining(b):
    """Quads (4 corners each, normals into the cavity) of a bin's lining, from its hinge / free edge / z range."""
    s = 1.0 if b['hinge'][0] > 0 else -1.0
    hx, hy = abs(b['hinge'][0]), b['hinge'][1]
    fx, fy = abs(b['free'][0]), b['free'][1]
    back = OUTBOARD_BACK if b['kind'] == 'outboard' else CENTRE_DIVIDER
    yt, yb = hy + 0.004, fy + 0.004
    z0, z1 = b['z0'] + 0.003, b['z1'] - 0.003
    toward_aisle = -1.0 if b['kind'] == 'outboard' else 1.0      # in |x| terms
    A, Bp, C, D = (hx, yt), (back, yt), (back, yb), (fx, yb)
    quads = [  # (corners in (|x|, y, z), normal in (|x|, y, z))
        ([(A[0], A[1], z0), (Bp[0], Bp[1], z0), (Bp[0], Bp[1], z1), (A[0], A[1], z1)], (0, -1, 0)),       # top
        ([(Bp[0], Bp[1], z0), (C[0], C[1], z0), (C[0], C[1], z1), (Bp[0], Bp[1], z1)], (toward_aisle, 0, 0)),  # back
        ([(C[0], C[1], z0), (D[0], D[1], z0), (D[0], D[1], z1), (C[0], C[1], z1)], (0, 1, 0)),           # bottom
        ([(A[0], A[1], z0), (Bp[0], Bp[1], z0), (C[0], C[1], z0), (D[0], D[1], z0)], (0, 0, 1)),         # end caps
        ([(A[0], A[1], z1), (Bp[0], Bp[1], z1), (C[0], C[1], z1), (D[0], D[1], z1)], (0, 0, -1)),
    ]
    return [([(s * x, y, z) for x, y, z in q], (s * n[0], n[1], n[2])) for q, n in quads]


def main(acf_dir, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    found = []        # (kind, obj index, call index, tri indices within call, P)
    parsed = []
    for oi, rel in enumerate(OBJECTS):
        lines, V, I, calls = parse(os.path.join(acf_dir, rel))
        parsed.append((rel, lines, V, I, calls))
        for ci, call in enumerate(calls):
            if call['anim']:
                continue
            for tri, P in pieces(V, I, call):
                kind = classify(P, len(tri))
                if kind:
                    found.append((kind, oi, ci, tri, P))
    shades = [f for f in found if f[0] == 'shade']
    doors = [f for f in found if f[0] in ('outboard', 'centre')]
    lips = [f for f in found if f[0] == 'lip']
    shades.sort(key=lambda f: (np.sign(f[4][:, 0].mean()), f[4][:, 2].mean()))
    doors.sort(key=lambda f: (round(f[4][:, 1].mean(), 1), f[0], np.sign(f[4][:, 0].mean()), f[4][:, 2].mean()))
    groups = {}       # (obj, call) -> list of (tri indices, anim header lines)
    manifest = {'shades': [], 'bins': []}
    for i, (kind, oi, ci, tri, P) in enumerate(shades):
        side = 1.0 if P[:, 0].mean() > 0 else -1.0
        head = [f'ANIM_trans_begin\ta380x/xp12/cabin/shade_ratio[{i}]', 'ANIM_trans_key\t0\t0\t0\t0',
                f'ANIM_trans_key\t1\t{side * SHADE_OUT:.4f}\t{-SHADE_DROP:.4f}\t0', 'ANIM_trans_end']
        groups.setdefault((oi, ci), []).append((tri, head))
        lo, hi = P.min(0), P.max(0)
        manifest['shades'].append({'index': i, 'x': float(P[:, 0].mean()), 'y0': float(lo[1] - SHADE_DROP),
                                   'y1': float(hi[1] - SHADE_DROP), 'z0': float(lo[2]), 'z1': float(hi[2])})
    used_lips = set()
    door_uvs = {}
    for j, (kind, oi, ci, tri, P) in enumerate(doors):
        h, f = hinge(P)
        theta = open_angle(h, f)
        z0, z1 = P[:, 2].min(), P[:, 2].max()
        head = ['ANIM_trans\t%.5f\t%.5f\t0\t%.5f\t%.5f\t0' % (h[0], h[1], h[0], h[1]),
                f'ANIM_rotate_begin\t0\t0\t1\ta380x/xp12/cabin/bin_ratio[{j}]', 'ANIM_rotate_key\t0\t0',
                f'ANIM_rotate_key\t1\t{theta:.2f}', 'ANIM_rotate_end',
                'ANIM_trans\t%.5f\t%.5f\t0\t%.5f\t%.5f\t0' % (-h[0], -h[1], -h[0], -h[1])]
        groups.setdefault((oi, ci), []).append((tri, head))
        nlip = 0
        if kind == 'outboard':      # the bottom lip of the same door: same object, overlapping z, at the free edge
            for li, (lk, loi, lci, ltri, LP) in enumerate(lips):
                if li in used_lips or loi != oi:
                    continue
                lz0, lz1 = LP[:, 2].min(), LP[:, 2].max()
                overlap = min(z1, lz1) - max(z0, lz0)
                if overlap > 0.9 * min(z1 - z0, lz1 - lz0) and np.sign(LP[:, 0].mean()) == np.sign(P[:, 0].mean()) \
                        and abs(abs(LP[:, 0].mean()) - abs(f[0])) < 0.09:
                    used_lips.add(li)
                    groups.setdefault((loi, lci), []).append((ltri, head))
                    nlip += 1
        t_call = parsed[oi][3][parsed[oi][4][ci]['off']:parsed[oi][4][ci]['off'] + parsed[oi][4][ci]['n']].reshape(-1, 3)
        door_uvs.setdefault(oi, []).extend(map(tuple, parsed[oi][2][t_call[tri].ravel(), 6:8][::7]))
        cvec = h - f; cl = float(np.hypot(*cvec)); nrm = np.array([-cvec[1], cvec[0]]) / cl
        if nrm[1] > 0: nrm = -nrm                   # the chord normal pointing down, to the aisle side
        bulge = float(max(0.0, ((P[:, :2] - f) @ nrm).max()))
        manifest['bins'].append({'index': j, 'kind': kind, 'object': OBJECTS[oi], 'bulge': bulge, 'hinge': [float(h[0]), float(h[1])],
                                 'free': [float(f[0]), float(f[1])], 'z0': float(z0), 'z1': float(z1),
                                 'angle': theta, 'lips': nlip})
    # rewrite each object: reorder the affected calls' indices and wrap the moved ranges
    for oi, (rel, lines, V, I, calls) in enumerate(parsed):
        I = I.copy()
        inserts = {}
        for ci, call in enumerate(calls):
            gl = groups.get((oi, ci))
            if not gl:
                continue
            t = I[call['off']:call['off'] + call['n']].reshape(-1, 3)
            moved = np.zeros(len(t), bool)
            for tri, _ in gl:
                moved[tri] = True
            order = [t[~moved]] + [t[tri] for tri, _ in gl]
            I[call['off']:call['off'] + call['n']] = np.concatenate(order).ravel()
            pos = call['off'] + 3 * int((~moved).sum())
            indent = lines[call['line']][:len(lines[call['line']]) - len(lines[call['line']].lstrip())]
            block = [f'{indent}TRIS\t{call["off"]}\t{3 * int((~moved).sum())}'] if (~moved).any() else []
            for tri, head in gl:
                block += [indent + 'ANIM_begin'] + [indent + '\t' + h for h in head] + \
                         [f'{indent}\tTRIS\t{pos}\t{3 * len(tri)}', indent + 'ANIM_end']
                pos += 3 * len(tri)
            inserts[call['line']] = block
        bins_here = [b for b in manifest['bins'] if b['object'] == rel]
        new_vt, new_idx = [], []
        if bins_here:
            u, v = plain_uv(acf_dir, rel, lines, V, door_uvs[oi])
            base = len(V)
            for b in bins_here:
                for corners, n in lining(b):
                    k = base + len(new_vt)
                    for x, y, z in corners:
                        new_vt.append('VT\t%.5f\t%.5f\t%.5f\t%.4f\t%.4f\t%.4f\t%.6f\t%.6f' % (x, y, z, n[0], n[1], n[2], u, v))
                    new_idx += [k, k + 1, k + 2, k, k + 2, k + 3]
            lining_start = len(I)
            I = np.concatenate([I, np.array(new_idx, dtype=I.dtype)])
            print(f'{os.path.basename(rel)}: lining for {len(bins_here)} bins, uv ({u:.4f}, {v:.4f})')
        out, idx_done, vt_done = [], False, False
        M = len(I)
        idx_lines = ['IDX10\t' + '\t'.join(str(x) for x in I[s:s + 10]) for s in range(0, M - M % 10, 10)]
        idx_lines += [f'IDX\t{x}' for x in I[M - M % 10:]]
        last_vt = max(i for i, l in enumerate(lines) if l.startswith('VT'))
        for i, l in enumerate(lines):
            if l.startswith('POINT_COUNTS') and new_vt:
                p = l.split()
                out.append(f'POINT_COUNTS\t{len(V) + len(new_vt)}\t{p[2]}\t{p[3]}\t{M}')
                continue
            if i == last_vt and new_vt:
                out.append(l)
                out.extend(new_vt)
                continue
            if l.startswith('IDX'):
                if not idx_done:
                    out.extend(idx_lines)
                    idx_done = True
                continue
            if i in inserts:
                out.extend(inserts[i])
                continue
            out.append(l)
        if new_vt:
            while out and out[-1].strip() == '':
                out.pop()
            out += ['', '# A380X CABIN BIN LININGS V1 (tools/cabin_anim.py)', 'ATTR_draw_enable', 'ATTR_no_cull',
                    f'TRIS\t{lining_start}\t{len(new_idx)}', 'ATTR_cull', '']
        dst = os.path.join(out_dir, os.path.basename(rel))
        open(dst, 'w').write('\n'.join(out))
    json.dump(manifest, open(os.path.join(out_dir, 'manifest.json'), 'w'), indent=1)
    kinds = {}
    for b in manifest['bins']:
        kinds[b['kind']] = kinds.get(b['kind'], 0) + 1
    print(f"shades {len(manifest['shades'])}, bins {len(manifest['bins'])} {kinds}, lips attached {len(used_lips)} of {len(lips)}")
    for kind in kinds:
        ex = next(b for b in manifest['bins'] if b['kind'] == kind and b['hinge'][0] > 0)
        print(f"  {kind} (right side): hinge {np.round(ex['hinge'], 3)} free edge {np.round(ex['free'], 3)} open {ex['angle']:.1f} deg")


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
