#!/usr/bin/env python3
"""exterior_lights: flex chain extraction, light lines, the object's blocks and the ACF attachment."""
import unittest

import numpy as np

from exterior_lights import ACF_FILE, LIGHTS, LOGOS, add_to_acf, build, flex_chain, light_lines


def wing(side, bones=13):
    body = ["ANIM_begin"]
    for k in range(bones):
        body += [f"\tANIM_trans\t{k}\t0\t0\t{k}\t0\t0", f"\tANIM_rotate_begin\t0\t0\t1\ta380x/xp12/wing/flex_{side}_inboard",
                 "\tANIM_rotate_key\t-1\t-1", "\tANIM_rotate_key\t1\t1", "\tANIM_rotate_end"]
    return "\n".join(["ANIM_begin", "\tANIM_rotate_begin\t0\t0\t1\ta380x/xp12/wing/flex_left_inboard", "\tANIM_rotate_end",
                      "\tTRIS\t0 3", "ANIM_end", *body, "\tANIM_trans\t-12\t0\t0\t-12\t0\t0", "\tTRIS\t3 3", "ANIM_end"])


class ExteriorLights(unittest.TestCase):
    def test_chain_is_the_deepest_and_ends_at_its_closing_trans(self):
        chain = flex_chain(wing("left"), "left")
        self.assertEqual(sum("ANIM_rotate_begin" in l for l in chain), 13)
        self.assertEqual(chain[-1], "\tANIM_trans\t-12\t0\t0\t-12\t0\t0")
        self.assertNotIn("TRIS", "\n".join(chain))
        with self.assertRaises(AssertionError):
            flex_chain(wing("left", 12), "left")

    def test_light_line(self):
        bb, pm = light_lines("nav", (1, 2, 3), (1, 0, 0), 0, 3000, (0, 0, 1), 60, "")
        self.assertEqual(bb, "LIGHT_PARAM\tairplane_nav_bb 1 2 3 1 0 0 0 3000cd 0 0 1 0.5")
        self.assertTrue(pm.startswith("LIGHT_PARAM\tairplane_nav_pm "))

    def test_object_blocks(self):
        pos = {n: np.zeros(3) for n in [l[0] for l in LIGHTS] + LOGOS}
        text = build(pos, {"L": ["\tANIM_trans\t1\t0\t0\t1\t0\t0"], "R": ["\tANIM_trans\t2\t0\t0\t2\t0\t0"]})
        lines = text.splitlines()
        self.assertEqual(sum(l.strip() == "ANIM_begin" for l in lines), sum(l.strip() == "ANIM_end" for l in lines))
        self.assertIn("\tANIM_hide\t0\t0.99\tsim/flightmodel2/gear/deploy_ratio[0]", lines)
        self.assertEqual(sum(l.lstrip().startswith("LIGHT_PARAM") for l in lines), 2 * len(LIGHTS) + len(LOGOS))
        self.assertNotIn("e-", text)                                # OBJ8 reads no exponent notation

    def test_acf_attachment_once(self):
        acf = "P _obja/0/_obj_flags 24\nP _obja/count 1\nP _other 1"
        once = add_to_acf(acf)
        self.assertIn(f"P _obja/1/_v10_att_file_stl {ACF_FILE}", once)
        self.assertIn("P _obja/count 2", once)
        self.assertEqual(add_to_acf(once), once)


if __name__ == "__main__":
    unittest.main()
