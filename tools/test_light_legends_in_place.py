import unittest

import numpy as np

from light_legends_in_place import DARK, GLOW, LIT_REF, OFF, convert_decal, hide_quads, lit_texture

DECAL = """I\n800\nOBJ\n\nTEXTURE a.png\nPOINT_COUNTS\t3\t0\t0\t6\n\nVT\t0\t0\t0\t0\t0\t1\t0\t0\nIDX10\t0\t1\t2\t0\t1\t2\t0\t1\t2\t0
IDX\t1\nIDX\t2\n
\tANIM_begin
\t\tATTR_shiny_rat\t0.5
\t\t# A380X LIT DECAL HIDE FACE_SEQ1
\t\tANIM_begin
\t\tANIM_hide\t0.0011\t1000000\tlamp/a
\t\tTRIS\t0\t6
\t\tANIM_end
\t\t# A380X LIT DECAL HIDE BOTH_SEQ1
\t\tANIM_begin
\t\tANIM_hide\t0.0011\t1000000\tlamp/b
\t\tANIM_hide\t0.0011\t1000000\tlamp/c
\t\tTRIS\t6\t6
\t\tANIM_end
\tANIM_end
"""
CONTROLS = """# OVHD LAMP lamp/a\nANIM_begin\nANIM_hide\t-1\t0.001\tlamp/a\nTRIS\t0\t6\nANIM_end
# OVHD LAMP lamp/b\nANIM_begin\nANIM_hide\t-1\t0.001\tlamp/b\nTRIS\t6\t6\nANIM_end\n"""


class Convert(unittest.TestCase):
    def test_single_lamp_face_glows_two_lamp_face_keeps_hide(self):
        text, faces = convert_decal(DECAL)
        self.assertEqual(faces, {"FACE_SEQ1": "lamp/a"})
        self.assertIn(f"{GLOW} FACE_SEQ1\n\t\tATTR_light_level\t0\t1\tlamp/a\n\t\tTRIS\t0\t6\n\t\t{DARK}\n\t\t# A380X", text)
        self.assertIn("ANIM_hide\t0.0011\t1000000\tlamp/c\n", text)
        self.assertIn(f"TEXTURE a.png\n{LIT_REF}\n", text)
        self.assertIn(f"IDX\t2\n{DARK}\n\n", text)
        self.assertIn("\t\tATTR_shiny_rat\t0\n", text)                # matte: no windshield mirror
        self.assertEqual(text.count("ANIM_begin"), text.count("ANIM_end"))
        self.assertEqual(convert_decal(text), (text, faces))          # re-running is a no-op

    def test_overlay_quad_hidden_once(self):
        text, hidden = hide_quads(CONTROLS, {"lamp/a"})
        self.assertEqual(hidden, ["lamp/a"])
        self.assertIn(f"lamp/a\n{OFF}lamp/a\nTRIS\t0\t6", text)
        self.assertNotIn(f"{OFF}lamp/b", text)
        self.assertEqual(hide_quads(text, {"lamp/a"})[0], text)

    def test_lit_texture_scales_each_legend_to_full(self):
        rgba = np.zeros((200, 200, 4), np.uint8)
        rgba[20:40, 20:60, 1] = 80                # dim green legend
        rgba[150:170, 150:190, 0] = 40            # dimmer red one far away
        rgba[0:2, 199, 2] = 5                     # noise below the floor
        out = lit_texture(rgba, window=31)
        self.assertEqual(out[30, 30].tolist(), [0, 255, 0])
        self.assertEqual(out[160, 160].tolist(), [255, 0, 0])
        self.assertEqual(out[0, 199].tolist(), [0, 0, 0])


if __name__ == "__main__":
    unittest.main()
