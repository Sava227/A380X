import unittest

import numpy as np
from PIL import Image

from smooth_engine_nacelles import channel_gain, high_pass, smooth_gloss, smooth_normal


class Smooth(unittest.TestCase):
    def test_high_pass_drops_broad_tilt_keeps_panel_line(self):
        y, x = np.mgrid[0:256, 0:256]
        broad = 20 * np.sin(2 * np.pi * x / 256)          # a panel-wide tilt
        line = np.where(x == 128, 40.0, 0.0)              # a one-texel panel line
        rg = np.stack([broad + line, broad], -1)
        out = high_pass(rg, 3)
        self.assertLess(np.abs(out[..., 1]).max(), 2)                    # broad tilt gone
        self.assertGreater(out[100, 128, 0], 30)                         # line kept

    def test_gain_finds_sign_and_scale(self):
        rng = np.random.default_rng(1)
        f = rng.normal(size=(64, 64, 2))
        g = channel_gain(f, f * np.array([0.5, -0.5]))
        np.testing.assert_allclose(g, [0.5, -0.5], atol=1e-9)

    def test_normal_layout_and_gloss_edges(self):
        rng = np.random.default_rng(2)
        fbw = Image.fromarray(np.clip(rng.normal(128, 20, (128, 128, 3)), 0, 255).astype(np.uint8))
        inst = Image.fromarray(np.asarray(fbw.resize((64, 64)))[..., ::-1].copy())
        out = np.asarray(smooth_normal(fbw, inst, 128, 24))
        self.assertEqual(out.shape, (128, 128, 3))
        self.assertTrue((out[..., 2] == 128).all())
        g = np.zeros((32, 32, 4), np.uint8); g[:, 16:, 1] = 200; g[..., 3] = 255
        g[5, 5, 1] = 255                                                  # a speckle
        s = np.asarray(smooth_gloss(Image.fromarray(g, "RGBA")))
        self.assertEqual(s[5, 5, 1], 0)                                   # speckle removed
        self.assertTrue((s[:, 18:, 1] == 200).all() and (s[:, :14, 1] == 0).all())   # edge kept


if __name__ == "__main__":
    unittest.main()
