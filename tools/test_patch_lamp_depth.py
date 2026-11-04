#!/usr/bin/env python3
"""tools/patch_lamp_depth.py: opaque lamp blocks over the legend decal, APU MASTER decal travel
(synthetic + installed aircraft)."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import patch_lamp_depth as p  # noqa: E402

AIRCRAFT = ROOT / 'output/A380X-XP12'
CONTROLS = '\n'.join([
    'ATTR_poly_os\t2',
    '# FCU ANNUNCIATOR PUSH_FCU_AP1_SEQ2',          # on mip_knob01: stays blended
    'ANIM_begin', 'ANIM_hide\t-0.5\t0.5\ta', 'TRIS\t0\t6', 'ANIM_end',
    '# FCU ANNUNCIATOR PUSH_GLARESHIELD_CS_MASTERWARN_SEQ1',
    'ANIM_begin', 'ANIM_hide\t-0.5\t0.5\tb', 'TRIS\t6\t6', 'ANIM_end',
    '# OVHD LAMP lamp/apu_master_upper',
    'ATTR_light_level\t0\t1\tc', 'ANIM_begin', 'ANIM_hide\t-1\t0.001\tc',
    'ANIM_trans_begin\tsim/cockpit2/electrical/APU_starter_switch',
    'ANIM_trans_key\t0\t0\t0\t0', 'ANIM_trans_key\t1\t0\t0.001\t0.0005', 'ANIM_trans_end',
    'TRIS\t12\t6', 'ANIM_end', 'ATTR_light_level_reset',
    '# ELEC LAMP lamp/bat1_upper',
    'ATTR_light_level\t0\t1\td', 'ANIM_begin', 'ANIM_hide\t-1\t0.001\td',
    '# ELEC LAMP MOTION PUSH_OVHD_ELEC_BAT1', 'ANIM_begin', 'TRIS\t18\t6', 'ANIM_end',
    'ANIM_end', 'ATTR_light_level_reset', 'ATTR_poly_os\t0', ''])


class PatchLampDepth(unittest.TestCase):
    def test_only_decal_backed_blocks_become_opaque(self):
        text, count = p.patch_controls(CONTROLS)
        lines = text.split('\n')
        self.assertEqual(count, 3)
        ap1 = lines.index('# FCU ANNUNCIATOR PUSH_FCU_AP1_SEQ2')
        self.assertEqual(lines[ap1 + 1], 'ANIM_begin')
        for comment in ('# FCU ANNUNCIATOR PUSH_GLARESHIELD_CS_MASTERWARN_SEQ1',
                        '# OVHD LAMP lamp/apu_master_upper', '# ELEC LAMP lamp/bat1_upper'):
            self.assertEqual(lines[lines.index(comment) + 1], p.NO_BLEND)
        # ATTR_blend closes each block after its outer ANIM_end / ATTR_light_level_reset.
        self.assertEqual(lines[lines.index('TRIS\t6\t6') + 2], p.BLEND)
        self.assertEqual(lines[lines.index('TRIS\t12\t6') + 2:lines.index('TRIS\t12\t6') + 4],
                         ['ATTR_light_level_reset', p.BLEND])
        self.assertEqual(lines[lines.index('TRIS\t18\t6') + 3:lines.index('TRIS\t18\t6') + 5],
                         ['ATTR_light_level_reset', p.BLEND])
        self.assertEqual(text.count(p.NO_BLEND), text.count('\n' + p.BLEND + '\n'))
        self.assertEqual(p.patch_controls(text), (text, 0))

    def test_generated_opaque_blocks_are_kept(self):
        generated = CONTROLS.replace('# ELEC LAMP lamp/bat1_upper\n', '# ELEC LAMP lamp/bat1_upper\n' + p.NO_BLEND + '\n')
        text, count = p.patch_controls(generated)
        self.assertEqual(count, 2)
        self.assertEqual(text.count(p.NO_BLEND), 3)

    def test_apu_master_decal_replays_the_lamp_travel(self):
        decal = '\n'.join(['\t\tANIM_begin', '\t\t\tTRIS\t732 6', '\t\tANIM_end',
                           '\t\t\tTRIS\t714 6', '\t\t\tTRIS\t720 6', ''])
        controls = CONTROLS.replace('apu_master_upper', 'apu_master_lower') + '\n' + CONTROLS
        text, faces = p.patch_decal(decal, controls)
        self.assertEqual(faces, 2)
        lines = text.split('\n')
        n = lines.index('\t\t\tTRIS\t714 6')
        self.assertEqual(lines[n - 5:n], ['\t\t\tANIM_begin',
                                          '\t\t\tANIM_trans_begin\tsim/cockpit2/electrical/APU_starter_switch',
                                          '\t\t\tANIM_trans_key\t0\t0\t0\t0',
                                          '\t\t\tANIM_trans_key\t1\t0\t0.001\t0.0005', '\t\t\tANIM_trans_end'])
        self.assertEqual(lines[n + 1], '\t\t\tANIM_end')
        self.assertIn('\t\t\tTRIS\t732 6', lines)   # APU START decal untouched
        self.assertEqual(p.patch_decal(text, controls), (text, 0))

    def test_installed_objects_are_patched_once(self):
        controls_path, decal_path = AIRCRAFT / p.CONTROLS_OBJ, AIRCRAFT / p.DECAL_OBJ
        if not controls_path.exists() or p.MARKER not in controls_path.read_text():
            self.skipTest('lamp depth patch not installed yet')
        controls, decal = controls_path.read_text(), decal_path.read_text()
        self.assertEqual(controls.count(p.MARKER), 1)
        # the cockpit-feedback lamps (tools/cockpit_feedback.py) append their own opaque block after this patch
        self.assertEqual(controls.split('# A380X FEEDBACK V1 CONTROLS')[0].count(p.NO_BLEND), 297)
        self.assertEqual(decal.count(p.DECAL_MARKER), 2)
        self.assertEqual(p.patch_controls(controls), (controls, 0))
        self.assertEqual(p.patch_decal(decal, controls), (decal, 0))


if __name__ == '__main__':
    unittest.main()
