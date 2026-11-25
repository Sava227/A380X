"""Minimal glTF rig evaluation (node world matrices at a pose, accessors) for the FBW A380 wing model."""
import json, numpy as np
CT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}
class Rig:
    def __init__(self, path):
        self.d = json.load(open(path)); self.buf = open(path[:-5] + '.bin', 'rb').read()
        self.nodes = self.d['nodes']; self.parent = {}
        for i, n in enumerate(self.nodes):
            for c in n.get('children', []): self.parent[c] = i
        self.byname = {n.get('name'): i for i, n in enumerate(self.nodes)}
        self.anims = {a['name']: a for a in self.d['animations']}
    def acc(self, i):
        d = self.d; a = d['accessors'][i]; v = d['bufferViews'][a['bufferView']]
        n, c, dt = a['count'], NC[a['type']], np.dtype(CT[a['componentType']])
        off = v.get('byteOffset', 0) + a.get('byteOffset', 0); st = v.get('byteStride', 0) or c * dt.itemsize
        raw = np.frombuffer(self.buf, np.uint8, count=st * (n - 1) + c * dt.itemsize, offset=off)
        out = np.lib.stride_tricks.as_strided(raw, (n, c * dt.itemsize), (st, 1)).copy().view(dt).reshape(n, c)
        if a.get('normalized'): out = out.astype(np.float64) / np.iinfo(dt).max
        return out
    def clip_duration(self, name):
        a = self.anims[name]; return max(float(self.acc(s['input']).max()) for s in a['samplers'])
    def sample(self, name, t):
        """-> {node: {path: value}} for clip `name` at time t."""
        a = self.anims[name]; out = {}
        for ch in a['channels']:
            s = a['samplers'][ch['sampler']]; ts = self.acc(s['input'])[:, 0]; vs = self.acc(s['output']).astype(float)
            i = int(np.clip(np.searchsorted(ts, t) - 1, 0, len(ts) - 1)); j = min(i + 1, len(ts) - 1)
            if s.get('interpolation') == 'STEP' or j == i or ts[j] == ts[i]: v = vs[i if t < ts[j] else j]
            else:
                u = (t - ts[i]) / (ts[j] - ts[i]); u = min(max(u, 0.0), 1.0)
                if ch['target']['path'] == 'rotation':
                    q0, q1 = vs[i], vs[j]
                    if np.dot(q0, q1) < 0: q1 = -q1
                    v = slerp(q0, q1, u)
                else: v = vs[i] * (1 - u) + vs[j] * u
            out.setdefault(ch['target']['node'], {})[ch['target']['path']] = v
        return out
    def local(self, i, over=None):
        n = self.nodes[i]
        if 'matrix' in n and not (over and i in over): return np.array(n['matrix'], float).reshape(4, 4).T
        o = (over or {}).get(i, {})
        t = np.array(o.get('translation', n.get('translation', [0, 0, 0])), float)
        r = np.array(o.get('rotation', n.get('rotation', [0, 0, 0, 1])), float)
        s = np.array(o.get('scale', n.get('scale', [1, 1, 1])), float)
        m = np.eye(4); m[:3, :3] = qmat(r) * s; m[:3, 3] = t; return m
    def world(self, over=None):
        W = {}
        def w(i):
            if i in W: return W[i]
            p = self.parent.get(i); m = self.local(i, over); W[i] = m if p is None else w(p) @ m; return W[i]
        for i in range(len(self.nodes)): w(i)
        return W
def qmat(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
def slerp(q0, q1, u):
    d = float(np.clip(np.dot(q0, q1), -1, 1))
    if d > 0.9995: q = q0 + u * (q1 - q0); return q / np.linalg.norm(q)
    th = np.arccos(d); return (np.sin((1 - u) * th) * q0 + np.sin(u * th) * q1) / np.sin(th)
def rot_axis_angle(R):
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    nrm = np.linalg.norm(ax); return (ax / nrm if nrm > 1e-12 else np.array([1.0, 0, 0])), ang
