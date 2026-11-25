import numpy as np
class Grid:
    def __init__(self, P, cell=0.01):
        self.P = P; self.cell = cell; self.d = {}
        for i, k in enumerate(map(tuple, np.floor(P / cell).astype(np.int64))): self.d.setdefault(k, []).append(i)
    def nearest(self, q, rad=1):
        k = np.floor(q / self.cell).astype(np.int64); best, bi = 1e9, -1
        for dx in range(-rad, rad + 1):
            for dy in range(-rad, rad + 1):
                for dz in range(-rad, rad + 1):
                    for i in self.d.get((k[0] + dx, k[1] + dy, k[2] + dz), ()):
                        dd = np.sum((self.P[i] - q) ** 2)
                        if dd < best: best, bi = dd, i
        return bi, np.sqrt(best)
