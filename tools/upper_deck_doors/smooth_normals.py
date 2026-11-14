"""Recompute vertex normals of the static lining parts (unanimated TRIS) of an X-Plane OBJ: area-weighted
face normals of the triangles that share a position within the same draw call, only across edges below
CREASE degrees, so real corners stay sharp. Animated parts keep their normals. in.obj out.obj"""
import sys, numpy as np
CREASE = 35.0
src, dst = sys.argv[1], sys.argv[2]
lines = open(src, 'r', errors='replace').read().split('\n')
vt_rows = [i for i, l in enumerate(lines) if l.startswith('VT')]
V = np.array([[float(x) for x in lines[i].split()[1:9]] for i in vt_rows])
I = np.array([int(x) for l in lines if l.startswith(('IDX10', 'IDX ')) or l.startswith('IDX\t') for x in l.split()[1:]])
calls, depth = [], 0
for l in lines:
    s = l.strip()
    if s.startswith('ANIM_begin'): depth += 1
    elif s.startswith('ANIM_end'): depth -= 1
    elif s.startswith('TRIS'):
        p = s.split(); calls.append((int(p[1]), int(p[2]), depth))
cos_crease = np.cos(np.radians(CREASE)); changed = 0
newN = V[:, 3:6].copy()
for off, n, depth in calls:
    if depth: continue                                  # animated door parts: unchanged
    t_all = I[off:off + n].reshape(-1, 3)
    fn0 = -np.cross(V[t_all[:,1],:3]-V[t_all[:,0],:3], V[t_all[:,2],:3]-V[t_all[:,0],:3])
    u0 = fn0 / np.maximum(np.linalg.norm(fn0, axis=1, keepdims=True), 1e-20)
    if np.mean(np.abs(np.einsum('ij,ikj->ik', u0, V[t_all, 3:6])) > 0.9999) < 0.95: continue   # only fully flat-shaded parts
    t = I[off:off + n].reshape(-1, 3)
    a, b, c = V[t[:, 0], :3], V[t[:, 1], :3], V[t[:, 2], :3]
    fn = -np.cross(b - a, c - a)                        # clockwise winding -> outward, length = 2*area
    unit = fn / np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-20)
    key = {}
    for f in range(len(t)):
        for k in range(3):
            key.setdefault(tuple(np.round(V[t[f, k], :3], 5)), []).append(f)
    for f in range(len(t)):
        for k in range(3):
            vi = t[f, k]
            group = key[tuple(np.round(V[vi, :3], 5))]
            acc = sum(fn[g] for g in group if np.dot(unit[g], unit[f]) >= cos_crease)
            ln = np.linalg.norm(acc)
            if ln > 0:
                nn = acc / ln
                if np.dot(nn, V[vi, 3:6]) < 0.9999: changed += 1
                newN[vi] = nn
for row, i in enumerate(vt_rows):
    p = lines[i].split()
    lines[i] = 'VT\t' + '\t'.join(p[1:4]) + '\t' + '\t'.join(f'{x:.6f}' for x in newN[row]) + '\t' + '\t'.join(p[7:9])
open(dst, 'w').write('\n'.join(lines))
print('static calls', sum(1 for c in calls if not c[2]), '| corner normals changed:', changed)
