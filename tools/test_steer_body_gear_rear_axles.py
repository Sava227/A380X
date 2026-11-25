import unittest

from steer_body_gear_rear_axles import MARKER, wrap

OBJ = "ANIM_begin\n\t\tATTR_no_blend\t0\n\t\tTRIS\t5 6\n\t\tTRIS\t0 5\n\t\tANIM_begin\nANIM_end\n"


class Wrap(unittest.TestCase):
    def test_wraps_run_once_with_fixed_point_keys(self):
        out = wrap(OBJ, -0.003, 1.6586, 3, ["TRIS\t5 6", "TRIS\t0 5"])
        self.assertIn("\t\tANIM_rotate_begin\t0\t0\t1\tsim/flightmodel2/gear/tire_steer_actual_deg[3]\n", out)
        self.assertIn("ANIM_rotate_key\t-15.0\t30.0\n", out)
        self.assertIn("ANIM_rotate_key\t15.0\t0.0\n", out)
        self.assertIn("ANIM_trans\t-0.00300\t1.65860\t0", out)
        self.assertIn("\t\tTRIS\t0 5\n\t\tANIM_end\n\t\tANIM_begin", out)
        self.assertNotIn("e-", out)
        self.assertEqual(out.count("ANIM_begin"), OBJ.count("ANIM_begin") + 1)
        self.assertEqual(wrap(out, -0.003, 1.6586, 3, ["TRIS\t5 6", "TRIS\t0 5"]), out)   # idempotent
        self.assertEqual(out.count(MARKER), 1)

    def test_missing_run_fails(self):
        with self.assertRaises(ValueError):
            wrap(OBJ, 0, 0, 3, ["TRIS\t9 9"])


if __name__ == "__main__":
    unittest.main()
