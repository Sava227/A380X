"""Neutral-pose (installed static pose) point cloud of the FBW wing model with each vertex's flex bone."""
import json, re, sys, numpy as np
from gltfrig import Rig
def static_overrides(r, cfg):
    rules = json.load(open(cfg))['mappings']
    over = {}; used = []
    for name in r.anims:
        for rule in rules:
            if re.fullmatch(rule['pattern'], name):
                if rule.get('static_fraction') is None: break
                t = r.clip_duration(name) * rule['static_fraction']
                for node, v in r.sample(name, t).items(): over.setdefault(node, {}).update(v)
                used.append((name, rule['static_fraction'])); break
    return over, used
def flex_bone(r, i):
    """deepest WING_FLEX_n (n >= 3) at or above node i -> (side, n) or None"""
    while i is not None:
        m = re.fullmatch(r'WING_FLEX_(\d+)\.([LR])', r.nodes[i].get('name', ''))
        if m and int(m.group(1)) >= 3: return (m.group(2), int(m.group(1)))
        i = r.parent.get(i)
    return None
def cloud(path, cfg):
    r = Rig(path); over, used = static_overrides(r, cfg); W = r.world(over)
    pts, bones, meshname = [], [], []
    for ni, n in enumerate(r.nodes):
        if 'mesh' not in n: continue
        for p in r.d['meshes'][n['mesh']]['primitives']:
            P = r.acc(p['attributes']['POSITION']).astype(float); H = np.c_[P, np.ones(len(P))]
            if 'skin' in n and 'JOINTS_0' in p['attributes']:
                sk = r.d['skins'][n['skin']]; joints = sk['joints']
                ibm = r.acc(sk['inverseBindMatrices']).reshape(-1, 4, 4).transpose(0, 2, 1) if 'inverseBindMatrices' in sk else np.tile(np.eye(4), (len(joints), 1, 1))
                J = r.acc(p['attributes']['JOINTS_0']).astype(int); Wt = r.acc(p['attributes']['WEIGHTS_0']).astype(float)
                Wt = np.where(J < len(joints), Wt, 0.0); J = np.where(J < len(joints), J, 0)
                Wt = Wt / np.maximum(Wt.sum(1, keepdims=True), 1e-12)
                M = np.stack([W[j] @ ibm[k] for k, j in enumerate(joints)])
                out = np.zeros((len(P), 4))
                for k in range(4): out += Wt[:, k:k + 1] * np.einsum('nij,nj->ni', M[J[:, k]], H)
                jb = [flex_bone(r, j) for j in joints]
                # dominant flex bone by summed weight
                vb = []
                for v in range(len(P)):
                    acc = {}
                    for k in range(4):
                        if Wt[v, k] > 1e-6: acc[jb[J[v, k]]] = acc.get(jb[J[v, k]], 0) + Wt[v, k]
                    vb.append(max(acc, key=acc.get) if acc else None)
                pts.append(out[:, :3]); bones += vb
            else:
                pts.append((H @ W[ni].T)[:, :3]); bones += [flex_bone(r, ni)] * len(P)
            meshname += [n.get('name')] * len(P)
    return r, np.concatenate(pts), bones, meshname, used
if __name__ == '__main__':
    r, P, b, mn, used = cloud(sys.argv[1], sys.argv[2])
    print('static clips applied:', len(used), used[:12])
    print('points', len(P), 'bbox min', P.min(0).round(2), 'max', P.max(0).round(2))
    import collections; print(collections.Counter(b).most_common(30))
