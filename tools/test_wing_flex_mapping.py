"""Wing flex clips follow the avionics plugin's FBW bend angles with A380_WINGS.xml's gain / offset."""
import json
from pathlib import Path
import re
import unittest

PROJECT = Path(__file__).resolve().parents[1]
STAGES = {'inboard': ('inboard', 23.25, 55.35), 'inner_midboard': ('inboard_mid', 16.125, 67.0),
          'outer_midboard': ('outboard_mid', 29.41, 31.9), 'outboard': ('outboard', 29.41, 50.0)}


class WingFlexMapping(unittest.TestCase):
    def test_each_clip_maps_fbw_angle_to_its_frame_range(self):
        rules = json.loads((PROJECT / 'config' / 'animation_datarefs.json').read_text())['mappings']
        for side in ('left', 'right'):
            for clip, (name, gain, offset) in STAGES.items():
                with self.subTest(side=side, clip=clip):
                    matches = [r for r in rules if re.fullmatch(r['pattern'], f'{side}_{clip}_flex')]
                    self.assertEqual(len(matches), 1)
                    rule = matches[0]
                    self.assertEqual(rule['dataref'], f'a380x/xp12/wing/flex_{side}_{name}')
                    self.assertAlmostEqual(rule['value_min'] * gain + offset, 0, places=3)
                    self.assertAlmostEqual(rule['value_max'] * gain + offset, 100, places=3)


if __name__ == '__main__':
    unittest.main()
