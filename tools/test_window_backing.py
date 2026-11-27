#!/usr/bin/env python3
"""window_backing: finds a window hole in a skin panel and builds an inward tube seen from inside."""
import unittest

import numpy as np

from window_backing import DEPTH, add_sleeves, sleeve, window_loops


def panel_with_hole(x=-3.5, n=24):
    """Annulus in the y-z plane at x: inner ellipse 0.34 x 0.26 m (the window), outer 0.8 x 0.7 m."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    inner = np.c_[np.full(n, x), 2.1 + 0.17 * np.cos(a), 10 + 0.13 * np.sin(a)]
    outer = np.c_[np.full(n, x), 2.1 + 0.4 * np.cos(a), 10 + 0.35 * np.sin(a)]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(inner[i], outer[i], outer[j]), (inner[i], outer[j], inner[j])]
    return np.array(tris)


class WindowBacking(unittest.TestCase):
    def test_finds_the_window_not_the_panel_edge(self):
        loops = window_loops(panel_with_hole())
        self.assertEqual(len(loops), 1)
        self.assertAlmostEqual(np.ptp(loops[0][:, 1]), 0.34, places=2)

    def test_tube_goes_inward_and_faces_its_axis(self):
        rim = window_loops(panel_with_hole())[0]
        tris, normals = sleeve(rim, DEPTH)
        self.assertEqual(len(tris), 2 * len(rim))
        self.assertAlmostEqual(tris[..., 0].min(), -3.5, places=6)              # starts at the skin
        self.assertAlmostEqual(tris[..., 0].max(), -3.5 + DEPTH, places=6)      # goes inboard (+x here)
        axis_point = np.array([-3.5, 2.1, 10.0])
        for t, n in zip(tris, normals):
            to_axis = axis_point - t.mean(0)
            to_axis[0] = 0
            self.assertGreater(n[0] @ to_axis, 0)                               # normal towards the axis
            view = -to_axis                                                       # from the axis to the wall
            self.assertGreater(np.cross(t[1] - t[0], t[2] - t[0]) @ view, 0)     # clockwise front seen from it

    def test_appended_to_host(self):
        host = "I\n800\nOBJ\n\nTEXTURE t.png\nPOINT_COUNTS\t3\t0\t0\t3\n\nVT\t0\t0\t0\t0\t0\t1\t0\t0\nVT\t1\t0\t0\t0\t0\t1\t1\t0\n" \
               "VT\t0\t1\t0\t0\t0\t1\t0\t1\nIDX\t0\nIDX\t1\nIDX\t2\n\nTRIS\t0\t3\n"
        rim = window_loops(panel_with_hole())[0]
        out = add_sleeves(host, *sleeve(rim, DEPTH))
        k = 6 * len(rim)
        self.assertIn(f"POINT_COUNTS\t{3 + k}\t0\t0\t{3 + k}", out)
        self.assertIn(f"TRIS\t3\t{k}", out)
        self.assertEqual(sum(l.startswith("VT") for l in out.splitlines()), 3 + k)
        ids = [int(v) for l in out.splitlines() if l.startswith("IDX") for v in l.split()[1:]]
        self.assertEqual(ids, list(range(3 + k)))


if __name__ == "__main__":
    unittest.main()
