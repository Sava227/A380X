"""Per door: share of triangles whose normal faces the cabin (inward) vs the skin (outward), closed state."""
import sys, re, collections, numpy as np
sys.path.insert(0, sys.argv[2])
from xp2wave import rot, tr, interp
verts, idx = [], []
stack, anim, pend = [np.eye(4)], [[]], None
acc = collections.defaultdict(lambda: [0, 0, 0])
for line in open(sys.argv[1], errors='replace'):
    p = line.split()
    if not p: continue
    w = p[0]
    if w == 'VT': verts.append([float(x) for x in p[1:7]])
    elif w in ('IDX10', 'IDX'): idx.extend(int(x) for x in p[1:])
    elif w == 'ANIM_begin': stack.append(stack[-1].copy()); anim.append([])
    elif w == 'ANIM_end': stack.pop(); anim.pop()
    elif w == 'ANIM_trans':
        a = np.array(p[1:4], float); stack[-1] = stack[-1] @ tr(a) if len(p) < 9 or p[8].lower() == 'none' else stack[-1] @ tr(interp([(float(p[7]), a), (float(p[8]), np.array(p[4:7], float))], 0))
    elif w == 'ANIM_trans_begin': pend = ('t', None, []); anim[-1].append(p[-1])
    elif w == 'ANIM_rotate_begin': pend = ('r', [float(x) for x in p[1:4]], []); anim[-1].append(p[-1])
    elif w == 'ANIM_trans_key': pend[2].append((float(p[1]), [float(x) for x in p[2:5]]))
    elif w == 'ANIM_rotate_key': pend[2].append((float(p[1]), [float(p[2])]))
    elif w == 'ANIM_trans_end': stack[-1] = stack[-1] @ tr(interp(pend[2], 0))
    elif w == 'ANIM_rotate_end': stack[-1] = stack[-1] @ rot(pend[1], float(interp(pend[2], 0)[0]))
    elif w == 'TRIS':
        door = (re.findall(r'door_open_ratio\[(\d+)\]', ' '.join(d for lv in anim for d in lv)) or ['static'])[0]
        V = np.array(verts); t = np.array(idx[int(p[1]):int(p[1]) + int(p[2])]).reshape(-1, 3); M = stack[-1]
        pos = (np.c_[V[t].reshape(-1, 6)[:, :3], np.ones(t.size)] @ M.T)[:, :3].reshape(-1, 3, 3)
        nrm = (V[t].reshape(-1, 6)[:, 3:6] @ M[:3, :3].T).reshape(-1, 3, 3).mean(1)
        s = nrm[:, 0] * np.sign(pos.mean(1)[:, 0])
        a = acc[door]; a[0] += len(t); a[1] += int((s < -0.2).sum()); a[2] += int((s > 0.2).sum())
for d in sorted(acc, key=lambda x: (x == 'static', int(x) if x.isdigit() else 99)):
    n, i, o = acc[d]; print(f'door {d:>6}: {n:6d} tris  inward {i / n:.0%}  outward {o / n:.0%}')
