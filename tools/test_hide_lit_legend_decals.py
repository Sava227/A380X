import unittest

from hide_lit_legend_decals import unwrap_bar_lamps

OBJ = """\t\t# CTRL BUTTON MOTION PUSH_EFIS_CS_LS_SEQ2
\t\tANIM_begin
\t\tANIM_trans_begin\ta380x/xp12/efis/captain/ls_pressed
\t\tANIM_trans_end
\t\t# A380X LIT DECAL HIDE PUSH_EFIS_CS_LS_SEQ2
\t\tANIM_begin
\t\tANIM_hide\t0.5001\t1000000\ta380x/xp12/efis/captain/ls
\t\tTRIS\t25137\t12
\t\tANIM_end
\t\tANIM_end
\t\t# A380X LIT DECAL HIDE PUSH_OVHD_APU_MASTER_SEQ1
\t\tANIM_begin
\t\tANIM_hide\t0.5\t1000000\tlamp/apu
\t\tTRIS\t30\t6
\t\tANIM_end
"""


class Unwrap(unittest.TestCase):
    def test_bar_lamp_faces_unwrapped_others_kept(self):
        text, n = unwrap_bar_lamps(OBJ)
        self.assertEqual(n, 1)
        self.assertNotIn("PUSH_EFIS_CS_LS_SEQ2\n\t\tANIM_begin\n\t\tANIM_hide", text)
        self.assertIn("ANIM_trans_end\n\t\tTRIS\t25137\t12\n\t\tANIM_end", text)       # motion block intact
        self.assertIn("HIDE PUSH_OVHD_APU_MASTER_SEQ1", text)                          # overhead lamps kept
        self.assertEqual(text.count("ANIM_begin"), text.count("ANIM_end"))
        self.assertEqual(unwrap_bar_lamps(text), (text, 0))                            # idempotent


if __name__ == "__main__":
    unittest.main()
