"""Door-edge seal strips for the upper-deck doors: from each door panel's outer boundary (in the door's own
animated frame), a 3 cm strip outward in the panel plane, 5 mm behind the lining, to close the see-through
gap between door and frame. Returns per door: list of triangles (3 x (pos, normal))."""
import numpy as np, collections

def outer_loop(P):
    """P: (n,3,3) triangle corner positions. Returns the ordered outer boundary loop (positions)."""
    key = lambda q: tuple(np.round(q, 5))
    ids = {}; pts = []
    def vid(q):
        k = key(q)
        if k not in ids: ids[k] = len(pts); pts.append(np.asarray(q))
        return ids[k]
    tri = np.array([[vid(P[f, c]) for c in range(3)] for f in range(len(P))])
    edges = collections.Counter()
    for a, b, c in tri:
        for u, v in ((a, b), (b, c), (c, a)): edges[(min(u, v), max(u, v))] += 1
    bnd = [e for e, n in edges.items() if n == 1]
    adj = collections.defaultdict(list)
    for u, v in bnd: adj[u].append(v); adj[v].append(u)
    seen, loops = set(), []
    for s in adj:
        if s in seen: continue
        loop, prev, cur = [s], None, s; seen.add(s)
        while True:
            nxt = [w for w in adj[cur] if w != prev and w not in seen]
            if not nxt: break
            prev, cur = cur, nxt[0]; loop.append(cur); seen.add(cur)
        loops.append(loop)
    pts = np.array(pts)
    best = max(loops, key=lambda L: np.ptp(pts[L], axis=0).sum())
    return pts[best], pts, tri

def seal(P, N, width=0.03, back=0.005):
    loop, pts, tri = outer_loop(P)
    c = P.reshape(-1, 3).mean(0)
    n_avg = N.reshape(-1, 3).mean(0); n_avg /= np.linalg.norm(n_avg)      # visible side (cabin)
    out_tris = []
    m = len(loop)
    ring = []
    for i in range(m):
        p = loop[i]
        d = p - c; d -= np.dot(d, n_avg) * n_avg; d /= max(np.linalg.norm(d), 1e-9)
        ring.append((p - back * n_avg, p + width * d - back * n_avg))
    for i in range(m):
        a0, a1 = ring[i]; b0, b1 = ring[(i + 1) % m]
        out_tris += [(a0, b0, b1), (a0, b1, a1)]
    return np.array(out_tris), n_avg, len(loop)
