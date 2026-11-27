#!/usr/bin/env python3
"""ride_on_compression: the whole drawn body rises by the strut deflection; header untouched; idempotent."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
from obj8_preview import parse_obj8  # noqa: E402

from nose_gear_compression import DATAREF, DEPLOY, DROP, ride_on_compression  # noqa: E402

OBJ = """I
800
OBJ

TEXTURE t.png
POINT_COUNTS\t3\t0\t0\t3
VT\t0\t0\t0\t0\t0\t1\t0\t0
VT\t1\t0\t0\t0\t0\t1\t1\t0
VT\t0\t1\t0\t0\t0\t1\t0\t1
IDX\t0
IDX\t1
IDX\t2

ATTR_no_blend\t0
ANIM_begin
\tANIM_trans\t0\t-4\t0\t0\t-4\t0
\tTRIS\t0\t3
ANIM_end

# Build with Blender"""


def lowest_y(text, deflection, deploy=1.0):
    obj = parse_obj8(text.splitlines(), {DATAREF: deflection, DEPLOY: deploy})
    draw = obj.draws[0]
    return draw.matrix[7]           # y translation of the only draw (row-major 4x4)


class NoseGearCompression(unittest.TestCase):
    def test_lifts_by_the_deflection(self):
        out = ride_on_compression(OBJ)
        self.assertAlmostEqual(lowest_y(out, 0.0), -4.0)
        self.assertAlmostEqual(lowest_y(out, 0.48), -3.52)
        self.assertAlmostEqual(lowest_y(out, 0.0, deploy=0.0), -4.0 + DROP)   # stowed where it was
        self.assertEqual(out.split("POINT_COUNTS")[0], OBJ.split("POINT_COUNTS")[0])

    def test_idempotent(self):
        once = ride_on_compression(OBJ)
        self.assertEqual(ride_on_compression(once), once)


if __name__ == "__main__":
    unittest.main()
