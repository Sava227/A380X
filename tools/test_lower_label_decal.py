import unittest

from lower_label_decal import EXPORT_LIFT, LIFT, MARKER, lower

TEXT = "I\n800\nOBJ\n\nPOINT_COUNTS\t1\t0\t0\t0\n\nVT\t1\t2\t3\t0\t0\t1\t0.5\t0.5\n"


class Lower(unittest.TestCase):
    def test_moves_along_the_vertex_normal_once(self):
        out = lower(TEXT)
        vt = [l for l in out.splitlines() if l.startswith("VT")][0].split()
        self.assertEqual([float(v) for v in vt[1:4]], [1.0, 2.0, 3 - (EXPORT_LIFT - LIFT)])
        self.assertEqual(vt[4:], ["0", "0", "1", "0.5", "0.5"])
        self.assertTrue(out.startswith("I\n800\nOBJ\n" + MARKER))
        self.assertEqual(lower(out), out)


if __name__ == "__main__":
    unittest.main()
