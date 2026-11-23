"""Minimal X-Plane OBJ 800 reader: vertices (pos, normal, uv), indices, TRIS draw calls with the
attribute state (ATTR_no_cull / ATTR_cull, ATTR_draw_disable) and the enclosing animation datarefs."""
import numpy as np

def read(path):
    verts, idx, calls = [], [], []
    state = {'cull': True, 'draw': True}
    anim = []          # stack of lists of datarefs
    header = {}
    with open(path, 'r', errors='replace') as f:
        for line in f:
            p = line.split()
            if not p: continue
            k = p[0]
            if k == 'VT':
                verts.append([float(x) for x in p[1:9]])
            elif k == 'IDX10' or k == 'IDX':
                idx.extend(int(x) for x in p[1:])
            elif k == 'TRIS':
                calls.append({'off': int(p[1]), 'n': int(p[2]), 'cull': state['cull'], 'draw': state['draw'],
                              'anim': [d for level in anim for d in level]})
            elif k in ('TEXTURE', 'TEXTURE_LIT', 'TEXTURE_NORMAL'):
                header[k] = p[1] if len(p) > 1 else ''
            elif k == 'TEXTURE_MAP':
                header['MAP_' + p[1]] = p[2] if len(p) > 2 else ''
            elif k == 'ATTR_no_cull': state['cull'] = False
            elif k == 'ATTR_cull': state['cull'] = True
            elif k == 'ATTR_draw_disable': state['draw'] = False
            elif k == 'ATTR_draw_enable': state['draw'] = True
            elif k == 'ANIM_begin': anim.append([])
            elif k == 'ANIM_end': anim and anim.pop()
            elif k in ('ANIM_rotate_begin', 'ANIM_trans_begin') and anim: anim[-1].append(p[-1])
            elif k in ('ANIM_rotate', 'ANIM_trans') and anim: anim[-1].append(p[-1])
    return np.array(verts, dtype=np.float64), np.array(idx, dtype=np.int64), calls, header

def winding_agreement(verts, idx, calls):
    """For each draw call: fraction of triangles whose geometric normal (b-a)x(c-a) agrees with the
    averaged vertex normal. ~1.0 = counter-clockwise front faces, ~0.0 = clockwise."""
    out = []
    for c in calls:
        t = idx[c['off']:c['off'] + c['n']].reshape(-1, 3)
        a, b, cc = verts[t[:, 0], :3], verts[t[:, 1], :3], verts[t[:, 2], :3]
        g = np.cross(b - a, cc - a)
        n = verts[t[:, 0], 3:6] + verts[t[:, 1], 3:6] + verts[t[:, 2], 3:6]
        dots = np.einsum('ij,ij->i', g, n)
        valid = np.abs(dots) > 1e-12
        out.append((c, float((dots[valid] > 0).mean()) if valid.any() else float('nan'), int(len(t))))
    return out
