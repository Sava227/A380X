import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from matte_glareshield_legends import draw_opaque, face_rects, paint

OBJ = """TEXTURE_MAP normal n.png
POINT_COUNTS\t6\t0\t0\t6
VT\t0\t0\t0\t0\t0\t-1\t0.25\t0.75
VT\t0\t0\t0\t0\t0\t-1\t0.5\t0.75
VT\t0\t0\t0\t0\t0\t-1\t0.5\t0.5
VT\t0\t0\t0\t0\t0\t-1\t0\t0
VT\t0\t0\t0\t0\t0\t-1\t0.1\t0
VT\t0\t0\t0\t0\t0\t-1\t0.1\t0.1
IDX10\t0\t1\t2\t3\t4\t5\t0\t0\t0\t0
# A380X LIT DECAL GLOW PUSH_GLARESHIELD_CS_MASTERCAUT_SEQ1
TRIS\t0\t3
# A380X LIT DECAL GLOW PUSH_OVHD_ELEC_BAT1_SEQ1
TRIS\t3\t3
"""


class Matte(unittest.TestCase):
    def test_only_glareshield_rect_changes(self):
        rects = face_rects(OBJ, 8, 8)
        self.assertEqual(rects, {"PUSH_GLARESHIELD_CS_MASTERCAUT_SEQ1": (2, 4, 2, 4)})
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "m.png"
            Image.fromarray(np.full((8, 8, 4), 200, np.uint8), "RGBA").save(p)
            self.assertEqual(paint(p, rects.values(), 1, 20), 4)
            x = np.asarray(Image.open(p))
            self.assertTrue((x[2:4, 2:4, 1] == 20).all() and (x[2:4, 2:4, [0, 2, 3]] == 200).all())
            self.assertEqual(int((x[..., 1] == 20).sum()), 4)
            self.assertEqual(paint(p, rects.values(), 1, 20), 0)    # idempotent


class Opaque(unittest.TestCase):
    def test_glareshield_faces_solid_others_blended(self):
        text, n = draw_opaque(OBJ)
        self.assertEqual(n, 1)
        self.assertIn("PUSH_GLARESHIELD_CS_MASTERCAUT_SEQ1\nATTR_no_blend\t0.5\nTRIS\t0\t3\nATTR_blend\n", text)
        self.assertIn("PUSH_OVHD_ELEC_BAT1_SEQ1\nTRIS\t3\t3\n", text)                     # overhead untouched
        self.assertEqual(draw_opaque(text), (text, 0))                                      # idempotent


if __name__ == "__main__":
    unittest.main()
