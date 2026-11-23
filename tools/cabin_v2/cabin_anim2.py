"""A380X cabin v2: openable window shades on every deck, overhead bins with inner skins and moving handles.

Builds on tools/cabin_anim_v1.py (the 2026-11-12 version) and works from the unmodified objects:

  * Window shades, 199 in all (a380x/xp12/cabin/shade_ratio[i], 0 raised -> 1 lowered):
      - economy (main deck, yc sidewalls): the model's own shade panels, parked in the wall above each window, slide
        0.52 m down (85, including the 12 angled ones in the tapering aft cabin that v1 missed);
      - first class (main deck forward, pec sidewalls) and the upper deck (fc deck sidewalls) have no shade panels, so
        one is made per window: a panel in the window reveal 35 mm inboard of the pane, parallel to it, a little
        larger than the pane; it slides up out of the opening along the (tilted) pane plane and is hidden once fully
        raised (ANIM_hide), so it never shows inside the wall.
  * Overhead bins (a380x/xp12/cabin/bin_ratio[j]), as v1, plus:
      - inner skin: the doors are single-sided, so an open door (flipped past vertical) was culled and vanished; each
        door gets an inside surface following its cross-section, 4 mm inside the outer skin, facing into the bin;
      - handle: each door's latch handle (a separate 294-triangle part in cabin_a380x_main_deck_assets.obj) is moved
        into the door's rotation.

Usage: cabin_anim2.py <orig yc obj> <orig pec obj> <orig fc obj> <orig assets obj> <aircraft dir> <out dir>
The aircraft dir is only used to read textures. Writes the four edited objects and manifest.json into <out dir>.
"""
import json, math, os, sys
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cabin_anim_v1 as v1

SHADE_DROP, SHADE_OUT = v1.SHADE_DROP, v1.SHADE_OUT
NEW_SHADE_IN = 0.035        # m inboard of the pane centre plane
NEW_SHADE_GROW = 0.03       # m added to each pane half-size
SHADE_COLOUR = (239.2, 246.1, 255.0)   # mean colour of the economy shade texture patch
SKIN_INSET = 0.004          # m inside the outer door skin
SKIN_PROFILE = 14           # cross-section segments


def tex_path(acf, obj_lines):
    t = next(l.split()[1] for l in obj_lines if l.startswith('TEXTURE '))
    return os.path.normpath(os.path.join(acf, 'objects/interior/cabin', t))


def match_uv(path, target, used_uvs):
    """UV of the smoothest 9x9 texel block, among the UVs the object already uses, closest to a colour."""
    img = np.asarray(Image.open(path).convert('RGB')).astype(float)
    H, W, _ = img.shape
    best, uv_best = 1e18, None
    for u, v in used_uvs:
        x, y = int(u % 1.0 * W), int((1 - v % 1.0) * H)
        blk = img[max(0, y - 4):y + 5, max(0, x - 4):x + 5].reshape(-1, 3)
        if len(blk) < 81:
            continue
        score = np.abs(blk.mean(0) - target).sum() + 4 * blk.std(0).sum()
        if score < best:
            best, uv_best = score, (float(u), float(v))
    return uv_best, best


def is_front(a, b, c, normal):
    """True when triangle a, b, c (in that index order) shows its front face toward normal. X-Plane's front faces
    are clockwise: the right-handed (b-a)x(c-a) normal points to the back (checked on the model's own door skins,
    whose vertex normals point opposite to it)."""
    a, b, c = (np.asarray(p, float) for p in (a, b, c))
    return float(np.dot(np.cross(b - a, c - a), normal)) < 0


def plane_frame(P, side):
    m = P.mean(0)
    _, _, Vt = np.linalg.svd(P - m)
    n = Vt[2] * (1 if Vt[2][0] * side > 0 else -1)          # outward
    up = np.array([0.0, 1.0, 0.0]) - n * n[1]
    up /= np.linalg.norm(up)
    w = np.cross(n, up)
    q = P - m
    return m, n, up, w, (q @ up).min(), (q @ up).max(), (q @ w).min(), (q @ w).max()


def classify2(P, ntri):
    k = v1.classify(P, ntri)
    if k:
        return k
    lo, hi = P.min(0), P.max(0)
    s, c = hi - lo, (hi + lo) / 2
    if ntri == 4 and s[0] < 0.03 and abs(s[1] - 0.57) < 0.02 and abs(s[2] - 0.35) < 0.02 and abs(c[0]) > 3.0:
        return 'shade'            # the angled economy shades of the tapering aft cabin
    return None


class Obj:
    def __init__(self, path, rel):
        self.rel = rel
        self.lines, self.V, self.I, self.calls = v1.parse(path)
        self.groups = {}          # call index -> [(tri indices, anim header lines)]
        self.extra = []           # (anim header lines or None, [VT rows], [triangles as local vertex triples])
        self.used_uv = None

    def uvs(self, step=50):
        if self.used_uv is None:
            self.used_uv = self.V[::step, 6:8]
        return self.used_uv


def main(yc, pec, fc, assets, acf, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    O = {'yc': Obj(yc, 'objects/interior/cabin/cabin_a380x_main_deck_yc_sidewalls.obj'),
         'pec': Obj(pec, 'objects/interior/cabin/cabin_a380x_main_deck_pec_sidewalls.obj'),
         'fc': Obj(fc, 'objects/interior/cabin/cabin_a380x_fc_deck_sidwealls.obj'),
         'assets': Obj(assets, 'objects/interior/cabin/cabin_a380x_main_deck_assets.obj')}
    found = []
    panes = []                # (obj key, centre, P) of window panes without shades
    handles = []              # (call index, tri, P) in assets
    for key, o in O.items():
        for ci, call in enumerate(o.calls):
            if call['anim']:
                continue
            for tri, P in v1.pieces(o.V, o.I, call):
                lo, hi = P.min(0), P.max(0)
                s, c = hi - lo, (hi + lo) / 2
                if key in ('yc', 'pec'):
                    kind = classify2(P, len(tri))
                    if kind:
                        found.append((kind, key, ci, tri, P))
                        continue
                if key == 'pec' and len(tri) == 840 and abs(c[0]) > 3.0 and s[0] < 0.08:
                    panes.append((key, c, P))
                if key == 'fc' and len(tri) == 816 and abs(abs(c[0]) - 2.99) < 0.05 and s[1] < 0.4:
                    panes.append((key, c, P))
                if key == 'assets' and len(tri) == 294 and 0.025 < s[0] < 0.045 and abs(s[1] - 0.047) < 0.006 \
                        and abs(s[2] - 0.18) < 0.01:
                    handles.append((ci, tri, P))
    shades = [f for f in found if f[0] == 'shade']
    doors = [f for f in found if f[0] in ('outboard', 'centre')]
    lips = [f for f in found if f[0] == 'lip']
    doors.sort(key=lambda f: (round(f[4][:, 1].mean(), 1), f[0], np.sign(f[4][:, 0].mean()), f[4][:, 2].mean()))
    manifest = {'shades': [], 'bins': []}

    # ---- shades: one list over all decks, ordered deck, side, z
    items = [('eco', f[1], f) for f in shades] + [('new', p[0], p) for p in panes]

    def sortkey(it):
        P = it[2][4] if it[0] == 'eco' else it[2][2]
        deck = 1 if P[:, 1].mean() > 3.5 else 0
        return (deck, np.sign(P[:, 0].mean()), P[:, 2].mean())
    items.sort(key=sortkey)
    for i, (kind, key, it) in enumerate(items):
        ref = f'a380x/xp12/cabin/shade_ratio[{i}]'
        if kind == 'eco':
            _, _, ci, tri, P = it
            side = 1.0 if P[:, 0].mean() > 0 else -1.0
            m, n, up, w, a0, a1, b0, b1 = plane_frame(P, side)
            off = n * SHADE_OUT
            head = [f'ANIM_trans_begin\t{ref}', 'ANIM_trans_key\t0\t0\t0\t0',
                    f'ANIM_trans_key\t1\t{off[0]:.4f}\t{-SHADE_DROP:.4f}\t{off[2]:.4f}', 'ANIM_trans_end']
            O[key].groups.setdefault(ci, []).append((tri, head))
            lowered = m + np.array([0, -SHADE_DROP, 0])
            hu, hw = (a1 - a0) / 2, (b1 - b0) / 2
        else:
            _, c, P = it
            side = 1.0 if c[0] > 0 else -1.0
            m, n, up, w, a0, a1, b0, b1 = plane_frame(P, side)
            mc = m + up * (a0 + a1) / 2 + w * (b0 + b1) / 2
            hu, hw = (a1 - a0) / 2 + NEW_SHADE_GROW, (b1 - b0) / 2 + NEW_SHADE_GROW
            lowered = mc - n * NEW_SHADE_IN
            rise = up * (2 * hu + 0.02)
            corners = [lowered + up * su * hu + w * sw * hw for su, sw in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            head = [f'ANIM_hide\t0\t0.02\t{ref}',
                    f'ANIM_trans_begin\t{ref}', f'ANIM_trans_key\t0\t{rise[0]:.4f}\t{rise[1]:.4f}\t{rise[2]:.4f}',
                    'ANIM_trans_key\t1\t0\t0\t0', 'ANIM_trans_end']
            O[key].extra.append((head, ('shade', corners, -n)))
        nin = -n
        click = [lowered + nin * 0.01 + up * su * (hu + 0.01) + w * sw * (hw + 0.01)
                 for su, sw in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        manifest['shades'].append({'index': i, 'kind': kind, 'object': O[key].rel, 'x': float(m[0]),
                                   'centre': [float(x) for x in lowered], 'normal': [float(x) for x in nin],
                                   'click': [[float(x) for x in p] for p in click]})

    # ---- bins (as v1) + handles + inner skins
    used_lips, used_handles = set(), set()
    for j, (kind, key, ci, tri, P) in enumerate(doors):
        h, f = v1.hinge(P)
        theta = v1.open_angle(h, f)
        z0, z1 = P[:, 2].min(), P[:, 2].max()
        head = ['ANIM_trans\t%.5f\t%.5f\t0\t%.5f\t%.5f\t0' % (h[0], h[1], h[0], h[1]),
                f'ANIM_rotate_begin\t0\t0\t1\ta380x/xp12/cabin/bin_ratio[{j}]', 'ANIM_rotate_key\t0\t0',
                f'ANIM_rotate_key\t1\t{theta:.2f}', 'ANIM_rotate_end',
                'ANIM_trans\t%.5f\t%.5f\t0\t%.5f\t%.5f\t0' % (-h[0], -h[1], -h[0], -h[1])]
        O[key].groups.setdefault(ci, []).append((tri, head))
        nlip = 0
        if kind == 'outboard':
            for li, (lk, lkey, lci, ltri, LP) in enumerate(lips):
                if li in used_lips or lkey != key:
                    continue
                lz0, lz1 = LP[:, 2].min(), LP[:, 2].max()
                overlap = min(z1, lz1) - max(z0, lz0)
                if overlap > 0.9 * min(z1 - z0, lz1 - lz0) and np.sign(LP[:, 0].mean()) == np.sign(P[:, 0].mean()) \
                        and abs(abs(LP[:, 0].mean()) - abs(f[0])) < 0.09:
                    used_lips.add(li)
                    O[key].groups.setdefault(lci, []).append((ltri, head))
                    nlip += 1
        # handle: same side, inside the door's z range, within 10 cm of the chord
        seg = h - f
        nh = 0
        for hi_, (hci, htri, HP) in enumerate(handles):
            if hi_ in used_handles:
                continue
            c = (HP.min(0) + HP.max(0)) / 2
            if not (z0 + 0.05 < c[2] < z1 - 0.05) or np.sign(c[0]) != np.sign(h[0]):
                continue
            t = float(((c[:2] - f) @ seg) / (seg @ seg))
            if -0.2 < t < 1.2 and np.linalg.norm(c[:2] - (f + np.clip(t, 0, 1) * seg)) < 0.10:
                used_handles.add(hi_)
                O['assets'].groups.setdefault(hci, []).append((htri, head))
                nh += 1
        # inner skin profile: outermost surface across the chord, from the door's own vertices
        cl = float(np.hypot(*seg))
        nrm = np.array([-seg[1], seg[0]]) / cl
        if nrm[1] > 0:
            nrm = -nrm                                       # outward: down / toward the aisle
        q = P[:, :2] - f
        sp, dp = (q @ seg) / (seg @ seg), q @ nrm
        prof = []
        for k in range(SKIN_PROFILE + 1):
            t = k / SKIN_PROFILE
            sel = np.abs(sp - t) < 0.5 / SKIN_PROFILE + 1e-6
            d = dp[sel].max() if sel.any() else 0.0
            prof.append(f + seg * t + nrm * (d - SKIN_INSET))
        prof = np.array(prof)
        # smooth (keeps the ends)
        prof[1:-1] = (prof[:-2] + 2 * prof[1:-1] + prof[2:]) / 4
        O[key].extra.append((head, ('skin', prof, nrm, z0 + 0.012, z1 - 0.012)))
        cvec = h - f
        nrm2 = np.array([-cvec[1], cvec[0]]) / cl
        if nrm2[1] > 0:
            nrm2 = -nrm2
        bulge = float(max(0.0, ((P[:, :2] - f) @ nrm2).max()))
        manifest['bins'].append({'index': j, 'kind': kind, 'object': O[key].rel, 'bulge': bulge,
                                 'hinge': [float(h[0]), float(h[1])], 'free': [float(f[0]), float(f[1])],
                                 'z0': float(z0), 'z1': float(z1), 'angle': theta, 'lips': nlip, 'handles': nh})

    # ---- write
    for key, o in O.items():
        write(o, key, acf, out_dir, manifest)
    json.dump(manifest, open(os.path.join(out_dir, 'manifest.json'), 'w'), indent=1)
    ks = {}
    for s in manifest['shades']:
        d = (s['object'].split('_')[-2], s['kind'])
        ks[d] = ks.get(d, 0) + 1
    print(f"shades {len(manifest['shades'])} {ks}; bins {len(manifest['bins'])}, lips {len(used_lips)}/{len(lips)}, "
          f"handles {len(used_handles)}/{len(handles)}, bins without handle "
          f"{[b['index'] for b in manifest['bins'] if not b['handles']]}")


def write(o, key, acf, out_dir, manifest):
    lines, V, I, calls = o.lines, o.V, o.I.copy(), o.calls
    inserts = {}
    for ci, call in enumerate(calls):
        gl = o.groups.get(ci)
        if not gl:
            continue
        t = I[call['off']:call['off'] + call['n']].reshape(-1, 3)
        moved = np.zeros(len(t), bool)
        for tri, _ in gl:
            moved[tri] = True
        order = [t[~moved]] + [t[tri] for tri, _ in gl]
        I[call['off']:call['off'] + call['n']] = np.concatenate(order).ravel()
        pos = call['off'] + 3 * int((~moved).sum())
        ln = lines[call['line']]
        indent = ln[:len(ln) - len(ln.lstrip())]
        block = [f'{indent}TRIS\t{call["off"]}\t{3 * int((~moved).sum())}'] if (~moved).any() else []
        for tri, head in gl:
            block += [indent + 'ANIM_begin'] + [indent + '\t' + h for h in head] + \
                     [f'{indent}\tTRIS\t{pos}\t{3 * len(tri)}', indent + 'ANIM_end']
            pos += 3 * len(tri)
        inserts[call['line']] = block
    new_vt, new_idx, tail = [], [], []
    base = len(V)

    def vt(p, n, uv):
        new_vt.append('VT\t%.5f\t%.5f\t%.5f\t%.4f\t%.4f\t%.4f\t%.6f\t%.6f' % (p[0], p[1], p[2], n[0], n[1], n[2], uv[0], uv[1]))
        return base + len(new_vt) - 1

    # v1 bin linings (closed cavity behind each door), drawn without culling
    bins_here = [b for b in manifest['bins'] if b['object'] == o.rel]
    lining_uv = None
    if bins_here:
        door_uvs = []
        for ci, gl in o.groups.items():
            call = calls[ci]
            t = o.I[call['off']:call['off'] + call['n']].reshape(-1, 3)
            for tri, head in gl:
                if any('bin_ratio' in h for h in head):
                    door_uvs.extend(map(tuple, V[t[tri].ravel(), 6:8][::7]))
        tpath = tex_path(acf, lines)
        img = np.asarray(Image.open(tpath).convert('L')).astype(float)
        Hh, Ww = img.shape
        best = 1e9
        lining_uv = door_uvs[0]
        for u, v in door_uvs[::max(1, len(door_uvs) // 400)]:
            x, y = int(u % 1.0 * Ww), int((1 - v % 1.0) * Hh)
            blk = img[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
            if blk.size and blk.mean() > 140 and blk.std() < best:
                best, lining_uv = blk.std(), (u, v)
        lin_start = len(I) + len(new_idx)
        for b in bins_here:
            for corners, n in v1.lining(b):
                k0 = vt(corners[0], n, lining_uv)
                for p in corners[1:]:
                    vt(p, n, lining_uv)
                new_idx += [k0, k0 + 1, k0 + 2, k0, k0 + 2, k0 + 3]
        tail += ['', '# A380X CABIN BIN LININGS V1 (tools/cabin_anim2.py)', 'ATTR_draw_enable', 'ATTR_no_cull',
                 f'TRIS\t{lin_start}\t{len(new_idx)}', 'ATTR_cull']
    # extra geometry: inner skins and new shades, each inside its animation
    if o.extra:
        tpath = tex_path(acf, lines)
        shade_uv = None
        if any(e[1][0] == 'shade' for e in o.extra):
            shade_uv, score = match_uv(tpath, np.array(SHADE_COLOUR), o.uvs())
            print(f'{os.path.basename(o.rel)}: shade uv {shade_uv} (score {score:.1f})')
        tail += ['', '# A380X CABIN V2 EXTRA GEOMETRY (tools/cabin_anim2.py): bin inner skins, new window shades',
                 'ATTR_draw_enable', 'ATTR_cull']
        for head, geo in o.extra:
            start = len(I) + len(new_idx)
            if geo[0] == 'skin':
                _, prof, nrm, z0, z1 = geo
                inward = np.array([-nrm[0], -nrm[1], 0.0])
                ids = []
                for p in prof:
                    ids.append((vt((p[0], p[1], z0), inward, lining_uv), vt((p[0], p[1], z1), inward, lining_uv)))
                for k in range(len(prof) - 1):
                    a0, a1 = ids[k]
                    b0, b1 = ids[k + 1]
                    pa0 = np.array([prof[k][0], prof[k][1], z0]); pb0 = np.array([prof[k + 1][0], prof[k + 1][1], z0])
                    pa1 = np.array([prof[k][0], prof[k][1], z1])
                    if is_front(pa0, pb0, pa1, inward):
                        new_idx += [a0, b0, a1, b0, b1, a1]
                    else:
                        new_idx += [a0, a1, b0, b0, a1, b1]
            else:
                _, corners, nin = geo
                ks = [vt(p, nin, shade_uv) for p in corners]
                if is_front(corners[0], corners[1], corners[2], nin):
                    new_idx += [ks[0], ks[1], ks[2], ks[0], ks[2], ks[3]]
                else:
                    new_idx += [ks[0], ks[2], ks[1], ks[0], ks[3], ks[2]]
            n_new = len(I) + len(new_idx) - start
            tail += ['ANIM_begin'] + ['\t' + h for h in head] + [f'\tTRIS\t{start}\t{n_new}', 'ANIM_end']
    I = np.concatenate([I, np.array(new_idx, dtype=I.dtype)]) if new_idx else I
    M = len(I)
    idx_lines = ['IDX10\t' + '\t'.join(str(x) for x in I[s:s + 10]) for s in range(0, M - M % 10, 10)]
    idx_lines += [f'IDX\t{x}' for x in I[M - M % 10:]]
    last_vt = max(i for i, l in enumerate(lines) if l.startswith('VT'))
    out, idx_done = [], False
    for i, l in enumerate(lines):
        if l.startswith('POINT_COUNTS'):
            p = l.split()
            out.append(f'POINT_COUNTS\t{len(V) + len(new_vt)}\t{p[2]}\t{p[3]}\t{M}')
            continue
        if i == last_vt:
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
    while out and out[-1].strip() == '':
        out.pop()
    out += tail + ['']
    open(os.path.join(out_dir, os.path.basename(o.rel)), 'w').write('\n'.join(out))
    print(f'{os.path.basename(o.rel)}: vertices +{len(new_vt)}, indices {len(o.I)} -> {M}, '
          f'anim groups {sum(len(g) for g in o.groups.values())}, extra {len(o.extra)}')


if __name__ == '__main__':
    main(*sys.argv[1:7])
