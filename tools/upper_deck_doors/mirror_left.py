"""Rebuild the left upper-deck door parts as exact mirror images of the right ones.

The left parts (door_open_ratio 11/13/15 and the static left linings/hoods) were made by mirroring the
positions of the right parts only: their winding and normals face the skin. For every left triangle this
finds the right triangle at the mirrored position (x -> -x), takes the right corner normals mirrored
(-nx, ny, nz), and orders the corners so the triangle faces the same way as its right twin.
Left vertices get their own VT rows (positions and UVs kept); TRIS ranges and everything else unchanged.
Usage: mirror_left.py in.obj out.obj"""
import sys, re, numpy as np

src, dst = sys.argv[1], sys.argv[2]
lines = open(src, errors='replace').read().split('\n')
VT = np.array([[float(x) for x in l.split()[1:9]] for l in lines if l.startswith('VT')])
IDX = np.array([int(x) for l in lines if l.startswith('IDX') for x in l.split()[1:]])
STATIC_LEFT = {2, 3, 6, 7, 10, 11}
calls, anim = [], []
for l in lines:
    s = l.strip()
    if s.startswith('ANIM_begin'): anim.append([])
    elif s.startswith('ANIM_end'): anim.pop()
    elif s.startswith(('ANIM_trans_begin', 'ANIM_rotate_begin')): anim[-1].append(s.split()[-1])
    elif s.startswith('TRIS'):
        off, n = (int(x) for x in s.split()[1:3])
        doors = re.findall(r'door_open_ratio\[(\d+)\]', ' '.join(d for lv in anim for d in lv))
        door = int(doors[0]) if doors else None
        left = (door in (11, 13, 15)) or (door is None and len(calls) in STATIC_LEFT)
        calls.append({'off': off, 'n': n, 'door': door, 'left': left})

M = np.array([-1.0, 1.0, 1.0])
def tris(c):
    t = IDX[c['off']:c['off'] + c['n']].reshape(-1, 3)
    return t, VT[t][:, :, :3], VT[t][:, :, 3:6]

new_vt = [list(v) for v in VT]
new_idx = IDX.copy()
report = []
for k, L in enumerate(calls):
    if not L['left']: continue
    tl, Pl, _ = tris(L)
    Cl = Pl.mean(1)
    best = None
    for r, R in enumerate(calls):
        if R['left'] or R['n'] != L['n']: continue
        if L['door'] is not None and R['door'] != L['door'] - 1: continue
        tr_, Pr, Nr = tris(R)
        Cr = Pr.mean(1) * M
        key = {tuple(x): i for i, x in enumerate(np.round(Cr / 0.002).astype(np.int64))}
        hits = sum(tuple(x) in key for x in np.round(Cl / 0.002).astype(np.int64))
        if best is None or hits > best[0]: best = (hits, r, key, Pr, Nr)
    hits, r, key, Pr, Nr = best
    done = 0
    for j in range(len(tl)):
        i = key.get(tuple(np.round(Cl[j] / 0.002).astype(np.int64)))
        if i is None:              # fall back to the nearest mirrored centroid
            i = int(np.argmin(np.linalg.norm(Pr.mean(1) * M - Cl[j], axis=1)))
        # pair each right corner with the left corner at its mirrored position
        order = [int(np.argmin(np.linalg.norm(Pl[j] - Pr[i, a] * M, axis=1))) for a in range(3)]
        if len(set(order)) < 3: continue
        # mirroring flips handedness, so reverse the right corner order: (a0, a2, a1)
        for slot, a in enumerate((0, 2, 1)):
            vi = tl[j, order[a]]
            row = list(VT[vi]); row[3:6] = list(Nr[i, a] * M)
            new_idx[L['off'] + 3 * j + slot] = len(new_vt); new_vt.append(row)
        done += 1
    report.append((k, r, len(tl), hits, done))

# drop VT rows no longer referenced, then renumber
used = np.zeros(len(new_vt), bool); used[new_idx] = True
remap = -np.ones(len(new_vt), np.int64); remap[used] = np.arange(used.sum())
vt_out = [new_vt[i] for i in np.nonzero(used)[0]]
idx_out = remap[new_idx]
N, Mn = len(vt_out), len(idx_out)
idx_lines = ['IDX10\t' + '\t'.join(str(x) for x in idx_out[s:s + 10]) for s in range(0, Mn - Mn % 10, 10)]
idx_lines += [f'IDX\t{x}' for x in idx_out[Mn - Mn % 10:]]
out, vt_done, idx_done = [], False, False
for l in lines:
    if l.startswith('POINT_COUNTS'): out.append(f'POINT_COUNTS\t{N}\t0\t0\t{Mn}'); continue
    if l.startswith('VT'):
        if not vt_done: out.extend('VT\t' + '\t'.join(f'{x:.6f}' for x in v) for v in vt_out); vt_done = True
        continue
    if l.startswith('IDX'):
        if not idx_done: out.extend(idx_lines); idx_done = True
        continue
    out.append(l)
open(dst, 'w').write('\n'.join(out))
for k, r, n, hits, done in report:
    print(f'left call {k:2d} (door {calls[k]["door"]}) <- right call {r:2d}: triangles {n}, centroid matches {hits}, rebuilt {done}')
print('vertices', N, 'indices', Mn)
