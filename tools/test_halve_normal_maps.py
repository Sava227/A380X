import unittest

import numpy as np
from PIL import Image

from halve_normal_maps import FLAT_SIZE, halve


class Halve(unittest.TestCase):
    def test_half_size_channels_kept(self):
        rng = np.random.default_rng(1)
        rgba = np.empty((64, 32, 4), np.uint8)
        rgba[..., :2] = rng.integers(64, 192, (64, 32, 2))
        rgba[..., 2], rgba[..., 3] = 128, 255           # FBW layout: X / Y slopes, constant blue / alpha
        out = np.asarray(halve(Image.fromarray(rgba, "RGBA")))
        self.assertEqual(out.shape, (32, 16, 4))
        self.assertTrue((out[..., 2] == 128).all() and (out[..., 3] == 255).all())
        self.assertAlmostEqual(out[..., 0].mean(), rgba[..., 0].mean(), delta=2)

    def test_flat_map_becomes_tiny(self):
        flat = Image.new("RGB", (4096, 4096), (128, 128, 128))
        out = halve(flat)
        self.assertEqual(out.size, (FLAT_SIZE, FLAT_SIZE))
        self.assertEqual(out.getpixel((3, 5)), (128, 128, 128))


if __name__ == "__main__":
    unittest.main()
