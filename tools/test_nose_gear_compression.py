#!/usr/bin/env python3
"""keyed(): OBJ lines that reproduce a rigid swing-about-a-hinge-plus-slide at each dataref key."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
from obj8_preview import parse_obj8  # noqa: E402

from nose_gear_compression import DEFLECTION, SQUASH, STROKE, bulge_band, keyed, squashed, top_begin  # noqa: E402

HEAD = """I
800
OBJ

TEXTURE t.png
POINT_COUNTS\t3\t0\t0\t3
VT\t0.3\t-4\t30\t0\t0\t1\t0\t0
VT\t0.3\t-3\t30\t0\t0\t1\t1\t0
VT\t0.3\t-3\t30.5\t0\t0\t1\t0\t1
IDX\t0
IDX\t1
IDX\t2
"""


def rot_x(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def link_motion(f):
    """Swing f*47 deg about a hinge along x through (0, -2.4, 30.6), and slide f*0.2 m up."""
    d = np.eye(4)
    d[:3, :3] = rot_x(47 * f)
    hinge = np.array([0, -2.4, 30.6])
    d[:3, 3] = hinge - d[:3, :3] @ hinge + [0, 0.2 * f, 0]
    return d


class Keyed(unittest.TestCase):
    def test_reproduces_the_motion_at_every_key(self):
        deform = [np.eye(4)] + [link_motion(f) for f in np.linspace(0, 1, 6)]
        keys = [0.0] + [0.16 + 0.48 * f for f in np.linspace(0, 1, 6)]
        obj = HEAD + "\n".join(["ANIM_begin", *keyed(deform, keys, "\t"), "\tTRIS\t0\t3", "ANIM_end", ""])
        rest = np.array([[0.3, -4, 30], [0.3, -3, 30], [0.3, -3, 30.5]])
        for d, v in zip(deform, keys):
            m = np.asarray(parse_obj8(obj.splitlines(), {DEFLECTION: v}).draws[0].matrix).reshape(4, 4)
            np.testing.assert_allclose(rest @ m[:3, :3].T + m[:3, 3], rest @ d[:3, :3].T + d[:3, 3], atol=1e-6)

    def test_top_begin_finds_the_outermost_block(self):
        lines = ["ANIM_begin", "ANIM_trans 0 0 0 0 0 0", "ANIM_begin", "TRIS 0 3", "ANIM_end", "ANIM_end",
                 "ANIM_begin", "ANIM_begin", "TRIS 3 3", "ANIM_end", "ANIM_end"]
        self.assertEqual(top_begin(lines, 3), 0)
        self.assertEqual(top_begin(lines, 8), 6)


class Squash(unittest.TestCase):
    def test_tyre_flattens_up_to_squash(self):
        full = STROKE - SQUASH
        self.assertAlmostEqual(squashed(0.0), 0.0)
        self.assertAlmostEqual(squashed(full), STROKE)                     # at STROKE the axle is SQUASH short
        self.assertAlmostEqual(squashed(full + 0.1), STROKE + 0.1)         # then follows 1:1
        self.assertLess(squashed(full - 1e-6), squashed(full + 1e-6))


def toy_tyre(radius=0.68, half_width=0.26, n=72):
    """Points on a cylinder tyre around the x axis at the origin (tread and both sidewalls)."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = []
    for r in np.linspace(0.4 * radius, radius, 12):
        for x in (-half_width, half_width):
            pts += [(x, r * np.cos(t), r * np.sin(t)) for t in a]
    pts = np.array(pts)
    return pts, np.zeros((len(pts), 2)), np.c_[np.sign(pts[:, 0]), np.zeros((len(pts), 2))]


class Bulge(unittest.TestCase):
    def test_both_walls_face_out_and_bulge_at_the_bottom(self):
        pts, uv, nrm = toy_tyre()
        tris, uvs, normals = bulge_band(pts, uv, nrm, 0.03)
        for side in (1, -1):
            wall = tris[tris[..., 0].mean(1) * side > 0]
            self.assertGreater(len(wall), 0)
            cross = np.cross(wall[:, 1] - wall[:, 0], wall[:, 2] - wall[:, 0])[:, 0]
            self.assertTrue(np.all(cross * side <= 1e-12))                    # clockwise seen from outside
            out = np.abs(wall[..., 0]) - 0.26
            self.assertGreaterEqual(out.min(), 0.003 - 1e-9)                  # never inside the sidewall
            self.assertAlmostEqual(out.max(), 0.033, delta=0.002)             # full bulge low on the wall
            self.assertLess(wall[..., 1].max(), 0)                            # bottom half only


if __name__ == "__main__":
    unittest.main()
