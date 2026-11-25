"""X-Plane OBJ 800 -> Wavefront OBJ with every draw call placed by its animation chain evaluated at
dataref value VALUE (default 0 = doors closed). Usage: xp2wave.py in.obj out.obj [value]"""
import sys, numpy as np

def rot(axis, deg):
    a = np.asarray(axis, float); n = np.linalg.norm(a)
    if n == 0 or deg == 0: return np.eye(4)
    x, y, z = a / n; c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg)); C = 1 - c
    m = np.eye(4)
    m[:3, :3] = [[c + x*x*C, x*y*C - z*s, x*z*C + y*s], [y*x*C + z*s, c + y*y*C, y*z*C - x*s], [z*x*C - y*s, z*y*C + x*s, c + z*z*C]]
    return m
def tr(v):
    m = np.eye(4); m[:3, 3] = v; return m
def interp(keys, value):
    keys = sorted(keys, key=lambda k: k[0])
    if value <= keys[0][0]: return np.asarray(keys[0][1], float)
    if value >= keys[-1][0]: return np.asarray(keys[-1][1], float)
    for (v0, a0), (v1, a1) in zip(keys, keys[1:]):
        if v0 <= value <= v1:
            t = 0 if v1 == v0 else (value - v0) / (v1 - v0)
            return np.asarray(a0, float) * (1 - t) + np.asarray(a1, float) * t

import os
_OVR = [kv.split('=') for kv in os.environ.get('XPV', '').split(',') if '=' in kv]
def val(ref, default):
    for k, v in _OVR:
        if k in ref: return float(v)
    return default

_BOX = [float(x) for x in os.environ.get('XPBOX', '').split(',')] if os.environ.get('XPBOX') else None

def convert(src, dst, value=0.0):
    verts, idx, out = [], [], []
    stack = [np.eye(4)]; pend = None
    with open(src, 'r', errors='replace') as f:
        for line in f:
            p = line.split()
            if not p: continue
            k = p[0]
            if k == 'VT': verts.append([float(x) for x in p[1:9]])
            elif k in ('IDX10', 'IDX'): idx.extend(int(x) for x in p[1:])
            elif k == 'ANIM_begin': stack.append(stack[-1].copy())
            elif k == 'ANIM_end': stack.pop()
            elif k == 'ANIM_trans':
                a = np.array(p[1:4], float); b = np.array(p[4:7], float)
                if len(p) >= 9 and p[8].lower() != 'none':
                    v0, v1 = float(p[7]), float(p[8]); stack[-1] = stack[-1] @ tr(interp([(v0, a), (v1, b)], val(p[-1], value)))
                else:
                    stack[-1] = stack[-1] @ tr(a)          # static: ANIM_trans x y z x y z [0 0 none]
            elif k == 'ANIM_rotate':
                ax = [float(x) for x in p[1:4]]; r0 = float(p[4]); r1 = float(p[5]) if len(p) > 5 else r0
                v0 = float(p[6]) if len(p) > 7 else 0.0; v1 = float(p[7]) if len(p) > 7 else 1.0
                stack[-1] = stack[-1] @ rot(ax, float(interp([(v0, [r0]), (v1, [r1])], val(p[-1], value))[0]))
            elif k == 'ANIM_trans_begin': pend = ('t', None, [], p[-1])
            elif k == 'ANIM_rotate_begin': pend = ('r', [float(x) for x in p[1:4]], [], p[-1])
            elif k == 'ANIM_trans_key': pend[2].append((float(p[1]), [float(x) for x in p[2:5]]))
            elif k == 'ANIM_rotate_key': pend[2].append((float(p[1]), [float(p[2])]))
            elif k == 'ANIM_trans_end': stack[-1] = stack[-1] @ tr(interp(pend[2], val(pend[3], value))); pend = None
            elif k == 'ANIM_rotate_end': stack[-1] = stack[-1] @ rot(pend[1], float(interp(pend[2], val(pend[3], value))[0])); pend = None
            elif k == 'TRIS': out.append((int(p[1]), int(p[2]), stack[-1].copy()))
    V = np.array(verts); I = np.array(idx)
    with open(dst, 'w') as w:
        w.write(f'# from {src}\n')
        base = 1
        for n, (off, cnt, M) in enumerate(out):
            t = I[off:off + cnt]
            pos = np.c_[V[t, :3], np.ones(len(t))] @ M.T
            if _BOX:   # keep only triangles with a corner inside the OBJ-space box xmin,xmax,ymin,ymax,zmin,zmax
                b = _BOX; inside = ((pos[:, 0] >= b[0]) & (pos[:, 0] <= b[1]) & (pos[:, 1] >= b[2]) & (pos[:, 1] <= b[3]) & (pos[:, 2] >= b[4]) & (pos[:, 2] <= b[5])).reshape(-1, 3).any(1)
                keep = np.repeat(inside, 3); t = t[keep]; pos = pos[keep]
                if not len(t): continue
            nrm = V[t, 3:6] @ M[:3, :3].T
            w.write(f'o call{n:03d}\n')
            for q in pos: w.write(f'v {q[0]:.5f} {-q[2]:.5f} {q[1]:.5f}\n')       # X-Plane (x,y,z) -> Blender (x,-z,y)
            for q in V[t, 6:8]: w.write(f'vt {q[0]:.5f} {q[1]:.5f}\n')
            for q in nrm: w.write(f'vn {q[0]:.5f} {-q[2]:.5f} {q[1]:.5f}\n')
            for j in range(0, len(t), 3):
                a, b, c = base + j, base + j + 1, base + j + 2
                w.write(f'f {a}/{a}/{a} {c}/{c}/{c} {b}/{b}/{b}\n')                # X-Plane clockwise -> Blender CCW
            base += len(t)
    return len(out)

if __name__ == '__main__':
    print(convert(sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 0.0), 'calls')
