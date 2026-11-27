#!/usr/bin/env python3
"""light_switch_animations: LAND keys swapped, NOSE on the three-position switch, once."""
import unittest

from light_switch_animations import MARKER, NOSE_SWITCH, patch

OBJ = """I
800
OBJ

POINT_COUNTS\t3\t0\t0\t3
\t\tANIM_rotate_begin\t-1\t0\t0\tsim/cockpit2/switches/landing_lights_on
\t\tANIM_rotate_key\t0\t24.689378
\t\tANIM_rotate_key\t1\t-26.774377
\t\tANIM_rotate_end
\t\tANIM_rotate_begin\t-1\t0\t0\tsim/cockpit2/switches/taxi_light_on
\t\tANIM_rotate_key\t0\t25.693778
\t\tANIM_rotate_key\t0.5\t0
\t\tANIM_rotate_key\t1\t-25.999971
\t\tANIM_rotate_end
"""


class LightSwitchAnimations(unittest.TestCase):
    def test_patch(self):
        out, n = patch(OBJ)
        self.assertEqual(n, 2)
        lines = out.split("\n")
        i = lines.index("\t\tANIM_rotate_begin\t-1\t0\t0\tsim/cockpit2/switches/landing_lights_on")
        self.assertEqual(lines[i + 1:i + 3], ["\t\tANIM_rotate_key\t0\t-26.774377", "\t\tANIM_rotate_key\t1\t24.689378"])
        i = lines.index(f"\t\tANIM_rotate_begin\t-1\t0\t0\t{NOSE_SWITCH}")
        self.assertEqual(lines[i + 1:i + 4], ["\t\tANIM_rotate_key\t0\t-25.999971", "\t\tANIM_rotate_key\t1\t0",
                                              "\t\tANIM_rotate_key\t2\t25.693778"])
        self.assertIn(MARKER, out)
        self.assertEqual(patch(out), (out, 0))


if __name__ == "__main__":
    unittest.main()
