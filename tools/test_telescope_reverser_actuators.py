#!/usr/bin/env python3
"""telescope(): two rigid pieces reproduce a rod stretched x2 along its bone while it swings."""
import unittest

import numpy as np

from telescope_reverser_actuators import telescope


def rot_z(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Telescope(unittest.TestCase):
    def setUp(self):
        # a rod 0.4 m long along +y from (1, 0, 0), radius 0.02 (rings at both ends)
        ring = [(0.02 * np.cos(a), 0, 0.02 * np.sin(a)) for a in np.linspace(0, 2 * np.pi, 8, endpoint=False)]
        self.rest = np.array([(1 + x, y, z) for y in (0.0, 0.4) for x, _, z in ring])
        self.origin = np.array([1.0, 0, 0])
        self.deform = []
        for f in (0.0, 0.5, 1.0):          # swing 60 deg about z through the origin, stretch y up to x2
            s = np.diag([1 - 0.3 * f, 1 + f, 1 - 0.3 * f])
            d = np.eye(4)
            d[:3, :3] = rot_z(60 * f) @ s
            d[:3, 3] = self.origin - d[:3, :3] @ self.origin + [0, 0, -0.5 * f]   # and slide 0.5 m
            self.deform.append(d)

    def pose(self, piece, key, spin):
        anchor, verts, keys = piece
        p, angle = keys[key]
        c, s = np.cos(np.radians(angle)), np.sin(np.radians(angle))
        k = np.array([[0, -spin[2], spin[1]], [spin[2], 0, -spin[0]], [-spin[1], spin[0], 0]])
        return p + verts @ (np.eye(3) + s * k + (1 - c) * k @ k).T

    def test_rest_is_unchanged_and_full_spans_the_stretched_rod(self):
        spin, (barrel, rod) = telescope(self.rest, np.array([0, 1.0, 0]), self.origin, self.deform, 0.7)
        np.testing.assert_allclose(self.pose(barrel, 0, spin), self.rest, atol=1e-9)
        np.testing.assert_allclose(np.abs(spin), [0, 0, 1], atol=1e-9)
        self.assertAlmostEqual(barrel[2][-1][1], 60, places=6)
        target = self.rest @ self.deform[-1][:3, :3].T + self.deform[-1][:3, 3]
        axis = rot_z(60) @ [0, 1, 0]
        b, r = self.pose(barrel, 2, spin), self.pose(rod, 2, spin)
        start = (target - self.origin) @ axis
        self.assertAlmostEqual((b - self.origin).dot(axis).min(), start.min(), places=6)   # near end on FBW's
        self.assertAlmostEqual((r - self.origin).dot(axis).max(), start.max(), places=6)   # far end on FBW's
        self.assertGreaterEqual((b - self.origin).dot(axis).max(), (r - self.origin).dot(axis).min())  # no gap
        radius = lambda q: np.linalg.norm(np.cross(q - q.mean(0), axis), axis=1).max()
        self.assertAlmostEqual(radius(r), 0.7 * radius(b), places=6)                       # rod thinner


if __name__ == "__main__":
    unittest.main()
