#!/usr/bin/env python3
"""gear_compression: keyed rigid motion, the wheel-spin anchor, and the sidewall bulge band."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
from obj8_preview import parse_obj8  # noqa: E402

from gear_compression import anchor, bulge_band, chain_lines, keyed, prefix_matrices  # noqa: E402

DREF = "sim/flightmodel2/gear/tire_vertical_deflection_mtr[0]"
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
        deform = [link_motion(f) for f in np.linspace(0, 1, 6)]
        keys = [0.48 * f for f in np.linspace(0, 1, 6)]
        obj = HEAD + "\n".join(["ANIM_begin", *keyed(deform, keys, DREF, "\t"), "\tTRIS\t0\t3", "ANIM_end", ""])
        rest = np.array([[0.3, -4, 30], [0.3, -3, 30], [0.3, -3, 30.5]])
        for d, v in zip(deform, keys):
            m = np.asarray(parse_obj8(obj.splitlines(), {DREF: v}).draws[0].matrix).reshape(4, 4)
            np.testing.assert_allclose(rest @ m[:3, :3].T + m[:3, 3], rest @ d[:3, :3].T + d[:3, 3], atol=1e-6)


SPUN = ["ANIM_begin", "\tANIM_trans\t0\t-4\t0\t0\t-4\t0", "\tANIM_begin", "\t\tANIM_trans\t1\t0\t0\t1\t0\t0", "\tANIM_end",
        "\tANIM_begin", "\t\tANIM_rotate_begin\t1\t0\t0\tsim/flightmodel2/gear/tire_rotation_angle_deg[0]",
        "\t\t\tANIM_rotate_key\t0\t0", "\t\t\tANIM_rotate_key\t360\t360", "\t\tANIM_rotate_end",
        "\t\tTRIS\t0\t3", "\tANIM_end", "\tTRIS\t0\t3", "ANIM_end"]


class Anchor(unittest.TestCase):
    def test_spin_is_the_anchor_and_sibling_blocks_are_not_in_the_chain(self):
        self.assertEqual(anchor(SPUN, 10), (6, True))
        self.assertNotIn(3, chain_lines(SPUN, 10))          # the sibling block's transform does not apply
        self.assertEqual(anchor(SPUN, 12), (12, False))      # no spin above this draw

    def test_prefix_matrix_at_the_anchor(self):
        lines = (HEAD + "\n".join(SPUN)).split("\n")
        spin = next(i for i, l in enumerate(lines) if "tire_rotation" in l)
        m = prefix_matrices(lines, [spin], {})[spin]
        np.testing.assert_allclose(m[:3, 3], [0, -4, 0])


def toy_tyre(radius=0.68, half_width=0.26, n=72):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = np.array([(x, r * np.cos(t), r * np.sin(t)) for r in np.linspace(0.4 * radius, radius, 12)
                    for x in (-half_width, half_width) for t in a])
    return pts, np.zeros((len(pts), 2)), np.c_[np.sign(pts[:, 0]), np.zeros((len(pts), 2))]


class Bulge(unittest.TestCase):
    def test_both_walls_face_out_and_bulge_at_the_bottom(self):
        tris, _, _ = bulge_band(*toy_tyre(), 0.03)
        for side in (1, -1):
            wall = tris[tris[..., 0].mean(1) * side > 0]
            self.assertGreater(len(wall), 0)
            cross = np.cross(wall[:, 1] - wall[:, 0], wall[:, 2] - wall[:, 0])[:, 0]
            self.assertTrue(np.all(cross * side <= 1e-12))
            out = np.abs(wall[..., 0]) - 0.26
            self.assertGreaterEqual(out.min(), 0.003 - 1e-9)
            self.assertAlmostEqual(out.max(), 0.033, delta=0.002)
            self.assertLess(wall[..., 1].max(), 0)


if __name__ == "__main__":
    unittest.main()
