import unittest

import numpy as np
from PIL import Image

from clean_fuselage_paint import clean_albedo, clean_gloss


class CleanPaint(unittest.TestCase):
    def setUp(self):
        a = np.full((512, 512, 3), 245, np.uint8)
        a[:, 300:] = (20, 50, 100)                      # blue livery block
        a[100:300, 150:154] = 205                       # streak, 40 darker
        a[360:396, 60:96] = 225                         # dirt patch (0.6x the paint window)
        a[200:203, 20:280] = 10                         # dark door seam
        self.a = a

    def test_weathering_goes_detail_stays(self):
        out = np.asarray(clean_albedo(Image.fromarray(self.a), 80))
        streak = np.r_[out[100:190, 150:154], out[213:300, 150:154]]                   # away from the seam
        self.assertTrue((np.abs(streak.astype(int) - 245) <= 3).all())                  # streak gone
        self.assertTrue((np.abs(out[360:396, 60:96].astype(int) - 245) <= 3).all())     # dirt gone
        self.assertTrue((out[200:203, 30:140] < 40).all())                              # seam kept
        self.assertTrue((out[:, 310:] == (20, 50, 100)).all())
        seam = np.full((512, 512, 3), 245, np.uint8); seam[200:203, :] = 180; seam[:, 100:103] = 180   # outlines
        o = np.asarray(clean_albedo(Image.fromarray(seam), 80)).astype(int)
        self.assertTrue((np.abs(o - 245) <= 3).all())                                    # and are removed whole
        near_edge = out[:, 240:300][self.a[:, 240:300, 0] == 245]
        self.assertTrue((near_edge == 245).all())                                       # no blue haze by the edge

    def test_gloss_even(self):
        g = np.zeros((512, 512, 4), np.uint8); g[..., 1] = 200; g[..., 3] = 255
        g[100:300, 150:154, 1] = 120; g[200:203, :, 1] = 130                            # streak, matte seam band
        out = np.asarray(clean_gloss(Image.fromarray(g, "RGBA")))
        self.assertTrue((np.abs(out[..., 1].astype(int) - 200) <= 3).all())
        self.assertTrue((out[..., 3] == 255).all())

    def test_gloss_parts_kept(self):
        g = np.zeros((512, 512, 4), np.uint8); g[..., 1] = 210; g[..., 3] = 255
        g[300:, :, 1] = 60                                                               # matte black parts
        g[100:102, :, 1] = 40                                                            # dark matte door band (thinner than the closing)
        out = np.asarray(clean_gloss(Image.fromarray(g, "RGBA")))
        self.assertTrue((out[330:, :, 1] == 60).all() and (np.abs(out[100:102, :, 1].astype(int) - 210) <= 3).all())


if __name__ == "__main__":
    unittest.main()
