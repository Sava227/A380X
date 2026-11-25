import unittest

import numpy as np
from PIL import Image

from restore_fbw_exterior_normals import MAPS, x_plane_normal


class Convert(unittest.TestCase):
    def test_keeps_slopes_resizes_and_sets_blue(self):
        src = np.zeros((64, 64, 3), np.uint8)
        src[..., 0], src[..., 1], src[..., 2] = 140, 120, 255
        out = np.asarray(x_plane_normal(Image.fromarray(src), (16, 16)))
        self.assertEqual(out.shape, (16, 16, 3))
        self.assertTrue((out[..., 0] == 140).all() and (out[..., 1] == 120).all() and (out[..., 2] == 128).all())

    def test_only_exported_maps_no_engines(self):
        self.assertEqual(len(MAPS), 12)
        self.assertFalse(any("eng" in k or "fans" in k for k in MAPS))


if __name__ == "__main__":
    unittest.main()
