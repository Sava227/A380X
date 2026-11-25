"""Per passenger door (door_open_ratio[0..15]): the closed outer panel's bounding box in each exterior fuselage object."""
import sys, re, json, numpy as np
sys.path.insert(0, sys.argv[1])
import xp2wave3 as xw
REF = 'sim/flightmodel2/misc/door_open_ratio['
def scan(path):
    V, I, calls = [], [], []
    stack = [np.eye(4)]; refs = [set()]; pend = None
    for line in open(path, errors='replace'):
        p = line.split()
        if not p: continue
        k = p[0]
        if k == 'VT': V.append([float(x) for x in p[1:4]])
        elif k in ('IDX10', 'IDX'): I.extend(int(x) for x in p[1:])
        elif k == 'ANIM_begin': stack.append(stack[-1].copy()); refs.append(set(refs[-1]))
        elif k == 'ANIM_end': stack.pop(); refs.pop()
        elif k in ('ANIM_trans_begin', 'ANIM_rotate_begin'):
            pend = ('t' if 'trans' in k else 'r', [float(x) for x in p[1:4]] if 'rotate' in k else None, [], p[-1]); refs[-1].add(p[-1])
        elif k == 'ANIM_trans_key': pend[2].append((float(p[1]), [float(x) for x in p[2:5]]))
        elif k == 'ANIM_rotate_key': pend[2].append((float(p[1]), [float(p[2])]))
        elif k == 'ANIM_trans_end': stack[-1] = stack[-1] @ xw.tr(xw.interp(pend[2], 0.0)); pend = None
        elif k == 'ANIM_rotate_end': stack[-1] = stack[-1] @ xw.rot(pend[1], float(xw.interp(pend[2], 0.0)[0])); pend = None
        elif k == 'ANIM_trans':
            a = np.array(p[1:4], float); refs[-1].add(p[-1]) if len(p) >= 10 else None
            stack[-1] = stack[-1] @ xw.tr(a)
        elif k == 'ANIM_rotate':
            ax = [float(x) for x in p[1:4]]; stack[-1] = stack[-1] @ xw.rot(ax, float(p[4]))
            if len(p) >= 9: refs[-1].add(p[-1])
        elif k == 'TRIS': calls.append((int(p[1]), int(p[2]), stack[-1].copy(), set(refs[-1])))
    V = np.array(V); I = np.array(I)
    out = {}
    for off, cnt, M, rs in calls:
        ds = sorted({int(r[len(REF):-1]) for r in rs if r.startswith(REF)})
        if len(ds) != 1 or ds[0] > 15: continue
        P = np.c_[V[I[off:off + cnt]], np.ones(cnt)] @ M.T
        out.setdefault(ds[0], []).append((cnt, P[:, :3]))
    return out
res = {}
for f in sys.argv[2:]:
    for d, parts in scan(f).items():
        for cnt, P in parts:
            res.setdefault(d, []).append((f.split('/')[-1], cnt, P.min(0).round(3).tolist(), P.max(0).round(3).tolist()))
for d in sorted(res):
    print('door', d)
    for r in sorted(res[d], key=lambda r: -r[1])[:6]: print('   ', r)
