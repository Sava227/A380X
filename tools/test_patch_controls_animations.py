"""EFIS/ISIS/ECAM control motion patch on synthetic OBJ fixtures (no aircraft files touched)."""
import unittest

from patch_controls_animations import (AXIAL_MARKER, BARO_MARKER, BUTTON_MARKER, ISIS_KEYS, LEGACY_KEYS,
                                       NEW_EFIS_KEYS, ROTARY_MARKER, add_baro_shaft, patch_object, retarget_legacy)
from patch_efis_animations import vertex_payload

HEADER = ('I\n800\nOBJ\nTEXTURE t.png\nPOINT_COUNTS\t3\t0\t0\t3\n'
          'VT\t0\t0\t0\t0\t0\t-1\t0.1\t0.2\nVT\t1\t0\t0\t0\t0\t-1\t0.3\t0.4\nVT\t0\t1\t0\t0\t0\t-1\t0.5\t0.6\n'
          'IDX\t0\nIDX\t1\nIDX\t2\n')
LEGACY = ('\tANIM_begin\n\tANIM_trans_begin\ta380x/xp12/efis/captain/cstr\n'
          '\tANIM_trans_key\t0\t0.2\t3.0\t32.7\n\tANIM_trans_key\t1\t0.2\t2.9998\t32.7006\n\tANIM_trans_end\n'
          '\tTRIS\t0 3\n\tANIM_end\n'
          '\tANIM_begin\n\tANIM_trans_begin\ta380x/xp12/efis/captain/wx_requested\n'
          '\tANIM_trans_key\t0\t0\t0\t0\n\tANIM_trans_key\t1\t0\t0\t0\n\tANIM_trans_end\n\tANIM_end\n')
BARO = ('\t\tANIM_begin\n\t\tANIM_trans\t0.2\t2.9\t32.7\t0.2\t2.9\t32.7\n\t\tANIM_rotate\t0\t1\t0\t90\t90\n'
        '\t\tANIM_rotate_begin\t0\t0\t-1\ta380x/xp12/efis/captain/baro_preset_hpa\n'
        '\t\tANIM_rotate_key\t745\t0\n\t\tANIM_rotate_key\t1100\t180\n\t\tANIM_rotate_end\n\t\tTRIS\t0 3\n\t\tANIM_end\n')
SHAFT = {'captain': (0.0, -0.00045, 0.0012), 'first_officer': (0.0, -0.0003, 0.0008)}


class ControlAnimationTests(unittest.TestCase):
    def test_inventory_covers_every_unanimated_efis_and_isis_key(self):
        self.assertEqual({key for _, key in NEW_EFIS_KEYS}, {'vord', 'vor2', 'taxi', 'ls', 'vv'})
        self.assertEqual(set(LEGACY_KEYS.values()), {'cstr', 'wpt', 'ndb', 'arpt', 'vor1', 'wx', 'terr', 'traf'})
        self.assertEqual({node for node, _ in ISIS_KEYS}, {'MODE', 'LS_DIR', 'PLUS', 'MINUS', 'MENU'})

    def test_legacy_latched_keys_become_momentary_only_where_geometry_draws(self):
        text, count = retarget_legacy(HEADER + LEGACY)
        self.assertEqual(count, 1)
        self.assertIn('ANIM_trans_begin\ta380x/xp12/efis/captain/cstr_pressed', text)
        self.assertIn('ANIM_trans_begin\ta380x/xp12/efis/captain/wx_requested', text)  # empty block: untouched
        self.assertEqual(retarget_legacy(text), (text, 0))

    def test_baro_shaft_is_outermost_and_push_pull_symmetric(self):
        text, count = add_baro_shaft(HEADER + BARO, SHAFT)
        self.assertEqual(count, 1)
        lines = [line.strip() for line in text.split('\n')]
        begin = lines.index('ANIM_begin')
        self.assertEqual(lines[begin + 1], BARO_MARKER + 'captain')
        self.assertEqual(lines[begin + 2], 'ANIM_trans_begin\ta380x/xp12/efis/captain/baro_knob_axial')
        self.assertIn('ANIM_trans_key\t-1\t0\t0.00045\t-0.0012', lines)
        self.assertIn('ANIM_trans_key\t1\t0\t-0.00045\t0.0012', lines)
        self.assertEqual(add_baro_shaft(text, SHAFT), (text, 0))

    def test_new_meshes_wrap_once_with_endless_knob_loop(self):
        common = dict(material='test', low=(0, 0, 0), high=(1, 1, 0), count=3,
                      points=[(0, 0, 0), (1, 0, 0), (0, 1, 0)])
        push = dict(common, name='PUSH_ISIS_1_SET_SEL', kind='translation', dataref='a380x/xp12/isis/set_sel_axial',
                    values=[0, 1], poses=[(0, 0, 0), (0, -.0005, .0014)])
        knob = dict(common, name='KNOB_ISIS_1_SET_SEL', kind='rotation', dataref='a380x/xp12/isis/set_sel_rotation',
                    values=[0, 360], poses=[0, 360], axis=(0, -.35, .94), pivot=(.5, .5, 0))
        groups = [(BUTTON_MARKER, []), (AXIAL_MARKER, [push]), (ROTARY_MARKER, [knob])]
        original = HEADER + 'TRIS\t0\t3\n'
        changed, summary = patch_object(original, 'panel_test.obj', groups, SHAFT)
        self.assertEqual(vertex_payload(original), vertex_payload(changed))
        self.assertEqual(len(summary['meshes']), 2)
        self.assertLess(changed.index(AXIAL_MARKER), changed.index(ROTARY_MARKER))
        self.assertEqual(changed.count('ANIM_keyframe_loop\t360'), 1)
        # Laminar/XPlane2Blender order: last line of the key table, before ANIM_rotate_end.
        loop = changed.index('ANIM_keyframe_loop')
        self.assertEqual(changed.index('ANIM_rotate_end'), changed.index('\n', loop) + 1)
        self.assertLess(changed.rindex('ANIM_rotate_key', 0, loop), loop)
        legacy = changed.replace('ANIM_keyframe_loop\t360\nANIM_rotate_end', 'ANIM_rotate_end\nANIM_keyframe_loop\t360')
        self.assertNotEqual(legacy, changed)
        self.assertEqual(patch_object(legacy, 'panel_test.obj', groups, SHAFT)[0], changed)
        repeated, summary = patch_object(changed, 'panel_test.obj', groups, SHAFT)
        self.assertEqual(repeated, changed)
        self.assertEqual(summary['meshes'], [])


if __name__ == '__main__':
    unittest.main()
