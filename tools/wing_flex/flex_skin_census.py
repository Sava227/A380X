"""Census of FBW A380 wing skin weights vs the wing-flex joints (how much skin would bend rigidly vs stay frozen)."""
import json, sys, numpy as np, collections
g = sys.argv[1]; d = json.load(open(g)); buf = open(g[:-5] + '.bin', 'rb').read()
CT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}
def acc(i):
    a = d['accessors'][i]; v = d['bufferViews'][a['bufferView']]
    n, c, dt = a['count'], NC[a['type']], np.dtype(CT[a['componentType']])
    off = v.get('byteOffset', 0) + a.get('byteOffset', 0); st = v.get('byteStride', 0) or c * dt.itemsize
    raw = np.frombuffer(buf, np.uint8, count=st * (n - 1) + c * dt.itemsize, offset=off)
    out = np.lib.stride_tricks.as_strided(raw, (n, c * dt.itemsize), (st, 1)).copy().view(dt).reshape(n, c)
    if a.get('normalized'): out = out.astype(np.float64) / np.iinfo(dt).max
    return out
nodes = d['nodes']; parent = {}
for i, n in enumerate(nodes):
    for c in n.get('children', []): parent[c] = i
flex = {}
for a in d['animations']:
    if 'flex' in a['name'].lower():
        for ch in a['channels']: flex.setdefault(ch['target']['node'], set()).add(a['name'])
def chain(i):
    out = []
    while i is not None:
        if i in flex: out.append(i)
        i = parent.get(i)
    return out
tot = collections.Counter()
for ni, n in enumerate(nodes):
    if 'mesh' not in n or 'skin' not in n: continue
    joints = d['skins'][n['skin']]['joints']
    for p in d['meshes'][n['mesh']]['primitives']:
        at = p['attributes']
        if 'JOINTS_0' not in at: continue
        J = acc(at['JOINTS_0']).astype(int); W = acc(at['WEIGHTS_0']).astype(float)
        idx = acc(p['indices']).reshape(-1, 3) if 'indices' in p else np.arange(len(J)).reshape(-1, 3)
        bad = (J >= len(joints)) & (W > 1e-6); tot['(vertex weights on out-of-range joints)'] += int(bad.any(1).sum())
        W = np.where(J >= len(joints), 0.0, W); J = np.where(J >= len(joints), 0, J)
        node_of = np.array(joints)[J]                     # vertex x 4 -> node index
        tw = W.sum(1); dom = W.argmax(1); domw = W.max(1)
        vbone = np.where((tw > 0) & (domw / np.maximum(tw, 1e-12) >= 0.9999), node_of[np.arange(len(J)), dom], -1)
        # flex chain signature of each vertex: weighted set of flex-chain tuples
        def sig(v):
            s = set()
            for k in range(4):
                if W[v, k] > 1e-6: s.add(tuple(chain(node_of[v, k])))
            return frozenset(s)
        sigs = [sig(v) for v in range(len(J))]
        for f in idx:
            b = {vbone[v] for v in f}
            touches_flex = any(any(len(t) for t in sigs[v]) for v in f)
            if not touches_flex: tot['no flex influence'] += 1
            elif len(b) == 1 and -1 not in b: tot['rigid on one bone (bends with flex)'] += 1
            else:
                same = len({sigs[v] for v in f}) == 1 and len(next(iter(sigs[f[0]]))) >= 0 and len(sigs[f[0]]) == 1
                tot['blended, same flex chain (static: offset when bent)' if same else 'blended across flex sections (static: TEARS when bent)'] += 1
print('wing triangles:', dict(tot), 'total', sum(tot.values()))
