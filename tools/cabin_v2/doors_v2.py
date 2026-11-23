"""A380X passenger doors v2: upper-deck door fixes, upper-deck handle motion, data for door click spots.

Inputs are the installed objects (backed up in ../original before installing):
  exterior/fuselage/a380x_fuse1..5.obj, a380x_pax_windows.obj,
  interior/cabin/cabin_a380x_pax_doors.obj, cabin_a380x_upper_deck_door_surrounds.obj

Changes:
  1. Upper-deck door windows (a380x_pax_windows.obj): the six window panes were exported outside their doors'
     animation (the six door_open_ratio[10..15] blocks there are empty), so the glass stayed in the door frame
     when the door slid away. Each pane is now wrapped in its door's motion (0.55 m out, then 1.3 m forward).
  2. Upper-deck doors 10-15 get a handle phase like the main-deck doors: every door_open_ratio[10..15] key table
     (fuse2, fuse3, pax_doors, door surrounds, door windows) is re-timed so the door stays shut for the first
     HANDLE_PHASE of the ratio (about 1 s of the 8 s cycle) and then moves exactly as before.
  3. In that phase the handles turn:
       inside  (pax_doors: the lever + grip, 404 + 238 triangles at the door block's origin) swings up and
               inboard UP_IN_DEG about the fore-aft axis through the top of the lever;
       outside (fuse2 / fuse3: the 918-triangle handle) swings out UP_OUT_DEG about the fore-aft axis through
               its top.
     (Main-deck doors already turn both handles in the first 9 % of the ratio; they are unchanged.)
  4. door_clicks.json: for every door 0-15, the inside and outside handle: its animation chain and a padded local
     box, used by cockpit_clicks_v2.py to put clickable copies that move with the doors into the cockpit object.

Usage: doors_v2.py <aircraft dir> <out dir>
"""
import json, os, re, sys
import numpy as np

HANDLE_PHASE = 0.12
UP_IN_DEG = 120.0
UP_OUT_DEG = 90.0
PAD_IN, PAD_OUT = 0.02, 0.02
UPPER = range(10, 16)
REF = 'sim/flightmodel2/misc/door_open_ratio'
FILES = {
    'fuse1': 'objects/exterior/fuselage/a380x_fuse1.obj', 'fuse2': 'objects/exterior/fuselage/a380x_fuse2.obj',
    'fuse3': 'objects/exterior/fuselage/a380x_fuse3.obj', 'fuse4': 'objects/exterior/fuselage/a380x_fuse4.obj',
    'fuse5': 'objects/exterior/fuselage/a380x_fuse5.obj', 'windows': 'objects/exterior/fuselage/a380x_pax_windows.obj',
    'paxdoors': 'objects/interior/cabin/cabin_a380x_pax_doors.obj',
    'surrounds': 'objects/interior/cabin/cabin_a380x_upper_deck_door_surrounds.obj',
}
CHANGED = ('fuse2', 'fuse3', 'windows', 'paxdoors', 'surrounds')


class Obj:
    def __init__(self, path):
        self.lines = open(path, errors='replace').read().split('\n')
        self.V, I = [], []
        for l in self.lines:
            if l.startswith('VT'):
                self.V.append([float(x) for x in l.split()[1:9]])
            elif l.startswith('IDX'):
                I.extend(int(x) for x in l.split()[1:])
        self.V, self.I = np.array(self.V), np.array(I)

    def calls(self):
        """(line index, offset, count, chain) per TRIS; chain = enclosing blocks' ANIM lines (outermost first)."""
        stack, out = [], []
        for i, l in enumerate(self.lines):
            p = l.split()
            if not p:
                continue
            if p[0] == 'ANIM_begin':
                stack.append([])
            elif p[0] == 'ANIM_end':
                stack.pop()
            elif p[0].startswith('ANIM_') and stack:
                stack[-1].append(l.strip())
            elif p[0] == 'TRIS':
                out.append((i, int(p[1]), int(p[2]), [list(b) for b in stack]))
        return out

    def pts(self, off, n):
        return self.V[self.I[off:off + n], :3]


def door_of(chain):
    refs = re.findall(re.escape(REF) + r'\[(\d+)\]', ' '.join(a for b in chain for a in b))
    return int(refs[0]) if refs else None


def block_origin(chain):
    """Closed-position origin of the door block (first key of its ANIM_trans_begin table)."""
    for a in chain[0]:
        if a.startswith('ANIM_trans_key'):
            return np.array([float(x) for x in a.split()[2:5]])
    return np.zeros(3)


def retime(lines):
    """Re-time every door_open_ratio[10..15] key table: hold the closed key for HANDLE_PHASE."""
    out, n, i = [], 0, 0
    while i < len(lines):
        l = lines[i]
        # only the doors' own translation tables (the handle rotations added here are not re-timed)
        m = re.match(r'^(\s*)(ANIM_(trans)_begin)\s.*' + re.escape(REF) + r'\[(1[0-5])\]\s*$', l)
        if not m:
            out.append(l)
            i += 1
            continue
        out.append(l)
        i += 1
        keys = []
        while not lines[i].strip().startswith('ANIM_' + m.group(3) + '_end'):
            keys.append(lines[i])
            i += 1
        ind = keys[0][:len(keys[0]) - len(keys[0].lstrip())]
        parsed = [k.split() for k in keys]
        new = [ind + '\t'.join([parsed[0][0], '0'] + parsed[0][2:])]
        for p in parsed:
            v = float(p[1])
            new.append(ind + '\t'.join([p[0], f'{HANDLE_PHASE + v * (1 - HANDLE_PHASE):.6g}'] + p[2:]))
        out += new
        n += 1
    return out, n


def handle_block(indent, pivot, axis, deg, k):
    px, py, pz = pivot
    return [f'{indent}ANIM_begin',
            f'{indent}\tANIM_trans\t{px:.5f}\t{py:.5f}\t{pz:.5f}\t{px:.5f}\t{py:.5f}\t{pz:.5f}',
            f'{indent}\tANIM_rotate_begin\t{axis[0]}\t{axis[1]}\t{axis[2]}\t{REF}[{k}]',
            f'{indent}\tANIM_rotate_key\t0\t0', f'{indent}\tANIM_rotate_key\t{HANDLE_PHASE}\t{deg:.1f}',
            f'{indent}\tANIM_rotate_key\t1\t{deg:.1f}', f'{indent}\tANIM_rotate_end',
            f'{indent}\tANIM_trans\t{-px:.5f}\t{-py:.5f}\t{-pz:.5f}\t{-px:.5f}\t{-py:.5f}\t{-pz:.5f}']


def box(P, pad):
    lo, hi = P.min(0) - pad, P.max(0) + pad
    lh = (lo, hi)
    return [[float(lh[i][0]), float(lh[j][1]), float(lh[k][2])] for i in (0, 1) for j in (0, 1) for k in (0, 1)]


def main(acf, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    O = {k: Obj(os.path.join(acf, v)) for k, v in FILES.items()}
    clicks = []
    report = []

    # 1. window panes of the upper doors: top-level TRIS -> per-door relative motion
    w = O['windows']
    wl = w.lines
    for (i, off, n, chain) in w.calls():
        if chain:
            continue
        c = w.pts(off, n).mean(0)
        if c[1] < 4.0:
            continue
        zs = {14: 0, -5: 2, -14: 4}
        base = min(zs, key=lambda z: abs(z - c[2]))
        k = 10 + zs[base] + (0 if c[0] > 0 else 1)
        s = 1 if c[0] > 0 else -1
        ind = wl[i][:len(wl[i]) - len(wl[i].lstrip())]
        wl[i] = '\n'.join([f'{ind}ANIM_begin', f'{ind}ANIM_trans_begin\t{REF}[{k}]', f'{ind}ANIM_trans_key\t0\t0\t0\t0',
                           f'{ind}ANIM_trans_key\t0.2\t{0.55 * s:.2f}\t0\t0', f'{ind}ANIM_trans_key\t1\t{0.55 * s:.2f}\t0\t1.3',
                           f'{ind}ANIM_trans_end', f'{ind}\t{wl[i].strip()}', f'{ind}ANIM_end'])
        report.append(f'door window pane at z {c[2]:.2f} x {c[0]:+.2f} -> door {k}')
    w.lines = '\n'.join(wl).split('\n')

    # 2. handles of the upper doors (positions from the unmodified chains)
    edits = {}            # obj key -> {line index: [replacement lines]}
    for key, want, ang, pad, where in (('paxdoors', (238, 404), UP_IN_DEG, PAD_IN, 'inside'),
                                      ('fuse2', (918,), UP_OUT_DEG, PAD_OUT, 'outside'),
                                      ('fuse3', (918,), UP_OUT_DEG, PAD_OUT, 'outside')):
        o = O[key]
        per = {}
        for (i, off, n, chain) in o.calls():
            k = door_of(chain)
            if k not in UPPER or n // 3 not in want or len(chain) != 1:
                continue
            P = o.pts(off, n)
            if key == 'paxdoors' and np.linalg.norm((P.min(0) + P.max(0)) / 2) > 0.4:
                continue
            per.setdefault(k, []).append((i, off, n, chain, P))
        for k, parts in sorted(per.items()):
            side = 1.0 if block_origin(parts[0][3])[0] > 0 else -1.0
            if key == 'paxdoors':
                lever = next(p[4] for p in parts if p[2] // 3 == 404)
                top = lever[lever[:, 1] > lever[:, 1].max() - 0.03]
                pivot = (top[:, 0].mean(), lever[:, 1].max() - 0.015, top[:, 2].mean())
                deg = -side * ang          # grip below the pivot swings inboard and up
            else:
                P = parts[0][4]
                top = P[P[:, 1] > P[:, 1].max() - 0.03]
                pivot = (top[:, 0].mean(), P[:, 1].max() - 0.01, top[:, 2].mean())
                deg = side * ang           # bottom swings outboard
            for (i, off, n, chain, P) in parts:
                ind = o.lines[i][:len(o.lines[i]) - len(o.lines[i].lstrip())]
                edits.setdefault(key, {})[i] = handle_block(ind, pivot, (0, 0, 1), deg, k) + \
                    [f'{ind}\t{o.lines[i].strip()}', f'{ind}ANIM_end']
            allP = np.vstack([p[4] for p in parts])
            hb = handle_block('', pivot, (0, 0, 1), deg, k)
            chain = [list(parts[0][3][0]), [h.strip() for h in hb[1:]]]
            clicks.append({'door': k, 'where': where, 'object': key, 'chain': chain, 'box': box(allP, pad)})
            report.append(f'door {k} {where} handle ({key}): {len(parts)} parts, pivot {np.round(pivot, 3).tolist()}, '
                          f'{deg:+.0f} deg about the fore-aft axis')
    for key, ed in edits.items():
        o = O[key]
        o.lines = [x for i, l in enumerate(o.lines) for x in (ed[i] if i in ed else [l])]

    # 3. main-deck door handles (already animated): click data only
    for key in ('paxdoors', 'fuse1', 'fuse2', 'fuse3', 'fuse4', 'fuse5'):
        o = Obj(os.path.join(acf, FILES[key]))
        groups = {}
        for (i, off, n, chain) in o.calls():
            k = door_of(chain)
            if k is None or k > 9 or len(chain) != 1:
                continue
            rk = [a.split() for a in chain[0] if a.startswith('ANIM_rotate_key')]
            if not any(abs(float(x[1]) - 0.0909) < 0.01 and abs(float(x[2])) > 100 for x in rk):
                continue                  # the handle chains turn > 100 deg in the first 9 %
            if key == 'paxdoors' and n // 3 not in (238, 404, 24):
                continue
            groups.setdefault((k, tuple(chain[0])), []).append(o.pts(off, n))
        for (k, ch), Ps in sorted(groups.items()):
            where = 'inside' if key == 'paxdoors' else 'outside'
            clicks.append({'door': k, 'where': where, 'object': key, 'chain': [list(ch)],
                           'box': box(np.vstack(Ps), PAD_IN if where == 'inside' else PAD_OUT)})
            report.append(f'door {k} {where} handle ({key}): {sum(len(p) for p in Ps) // 3} triangles, animated chain copied')

    # 4. re-time the upper doors and write the changed objects
    for key in CHANGED:
        o = O[key]
        o.lines, n = retime(o.lines)
        report.append(f'{FILES[key]}: {n} upper-door key tables re-timed (hold {HANDLE_PHASE})')
        open(os.path.join(out_dir, os.path.basename(FILES[key])), 'w').write('\n'.join(o.lines))
    # the click chains of the upper doors must use the re-timed tables too
    for c in clicks:
        c['chain'] = [retime(b)[0] for b in c['chain']]
    json.dump(clicks, open(os.path.join(out_dir, 'door_clicks.json'), 'w'), indent=1)
    have = {(c['door'], c['where']) for c in clicks}
    missing = [(k, wh) for k in range(16) for wh in ('inside', 'outside') if (k, wh) not in have]
    print('\n'.join(report))
    print(f'door click entries {len(clicks)}; missing {missing}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
