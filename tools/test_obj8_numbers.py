#!/usr/bin/env python3
"""tools/obj8_numbers.py: fixed-point OBJ8 numbers (X-Plane stops reading a number at its exponent)."""
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import obj8_numbers as n  # noqa: E402

AIRCRAFT = ROOT / 'output/A380X-XP12'


class Format(unittest.TestCase):
    def test_tiny_values_have_no_exponent(self):
        for value in (4.694689366e-07, -3.946580708e-07, 3.187809668e-10, 1e-30, -1e-15, 6.0e-6):
            text = n.fmt(value)
            self.assertNotIn('e', text.lower(), text)
            self.assertAlmostEqual(float(text), value, places=9)

    def test_plain_values(self):
        self.assertEqual(n.fmt(0), '0')
        self.assertEqual(n.fmt(-0.0), '0')
        self.assertEqual(n.fmt(-1e-12), '0')
        self.assertEqual(n.fmt(1), '1')
        self.assertEqual(n.fmt(32.9399147), '32.9399147')
        self.assertEqual(n.fmt(-0.3439709335), '-0.3439709335')
        self.assertEqual(n.fmt(260.0156694), '260.0156694')
        self.assertEqual(n.fmt(1.5e6), '1500000')

    def test_non_finite_rejected(self):
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                n.fmt(value)


class FixLines(unittest.TestCase):
    def test_axis_line(self):
        line = 'ANIM_rotate_begin\t4.694689366e-07\t-0.3439709335\t0.9389802964\ta380x/xp12/du/captain_mfd\n'
        fixed, count = n.fix_line(line)
        self.assertEqual(count, 1)
        self.assertEqual(fixed, 'ANIM_rotate_begin\t0.0000004695\t-0.3439709335\t0.9389802964\ta380x/xp12/du/captain_mfd\n')

    def test_translation_key_and_vertex(self):
        fixed, count = n.fix_line('\tANIM_trans_key\t1\t3.187809668e-10\t-0.0011\t2E-3\n')
        self.assertEqual((fixed, count), ('\tANIM_trans_key\t1\t0.0000000003\t-0.0011\t0.002\n', 2))
        fixed, count = n.fix_line('VT 1e-05 2.5 -3.25E+01 0 0 1 0.5 1.0e-20\n')
        self.assertEqual((fixed, count), ('VT 0.00001 2.5 -32.5 0 0 1 0.5 0\n', 3))

    def test_names_comments_and_paths_untouched(self):
        for line in ('# value 1e-07 in a comment\n', 'TEXTURE ../tex/e1e5.png\n',
                     'ATTR_manip_command button a380x/xp12/rmp/1e3 label 1e3x\n',
                     'ANIM_trans_begin sim/flightmodel/e2e\n'):
            self.assertEqual(n.fix_line(line), (line, 0), line)

    def test_fix_text_is_idempotent(self):
        text = 'I\n800\nOBJ\nVT 1e-7 0 0 0 0 1 0 0\nANIM_rotate_key 0 -2.708071976e-07\n'
        once, count = n.fix_text(text)
        self.assertEqual(count, 2)
        self.assertEqual(n.fix_text(once), (once, 0))

    def test_cli_fix_preserves_other_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            obj = Path(tmp) / 'a.obj'
            body = b'I\r\n800\r\nOBJ\r\n\r\nVT\t1.5e-06\t2\t3\t0\t0\t1\t0\t0\r\n# caf\xe9 1e-5\r\n'
            obj.write_bytes(body)
            sys.argv = ['obj8_numbers.py', '--fix', tmp]
            self.assertEqual(n.main(), 0)
            self.assertEqual(obj.read_bytes(), body.replace(b'1.5e-06', b'0.0000015'))
            sys.argv = ['obj8_numbers.py', '--check', str(obj)]
            self.assertEqual(n.main(), 0)


@unittest.skipUnless((AIRCRAFT / 'objects').is_dir(), 'installable aircraft not present')
class InstalledAircraft(unittest.TestCase):
    def test_no_exponent_notation_in_any_obj(self):
        bad = []
        for obj in n.obj_files([AIRCRAFT / 'objects']):
            with obj.open(encoding='latin-1') as stream:
                for number, line in enumerate(stream, 1):
                    if n.has_exponent(line):
                        bad.append(f'{obj.relative_to(AIRCRAFT)}:{number}')
                        break
        self.assertEqual(bad, [])


if __name__ == '__main__':
    unittest.main()
