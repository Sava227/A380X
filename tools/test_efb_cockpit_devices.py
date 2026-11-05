"""Web EFB tablet geometry and touch mapping (tools/efb_cockpit_devices.py, docs/EFB_DEVELOPMENT.md)."""
import math
from pathlib import Path
import re
import unittest

import efb_cockpit_devices as efb
from avionics_panel_layout import (EFB_ACTION_TOUCH_RECTS, EFB_PAGE_TOUCH_RECTS, EFB_PANEL_RECT,
                                   EFB_POWER_TOUCH_RECT, EFB_SCREEN_SURFACES, PANEL_SIZE)

ROOT = Path(__file__).resolve().parents[1]
LIVE = ROOT / 'output/A380X-XP12/objects/interior'


def synthetic_controls(faces=2, touches=34):
    """A minimal cockpit-controls OBJ with the legacy tablet: atlas faces in the cockpit region and
    34 command manipulators, plus an unrelated FCU face and an unrelated manipulator."""
    vertices, indices, body = [], [], []
    left, bottom, right, top = EFB_PANEL_RECT
    rects = [EFB_PANEL_RECT] * faces + [(0, 1280, 1024, 1492)]
    region = []
    for l, b, r, t in rects:
        base = len(vertices)
        for u, v in ((l, b), (l, t), (r, t), (r, b)):
            vertices.append(f'VT\t0\t0\t0\t0\t1\t0\t{u / PANEL_SIZE[0]:.9g}\t{v / PANEL_SIZE[1]:.9g}')
        region.append(f'TRIS\t{len(indices)}\t6')
        indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))
    names = [f'page/{p}' for p in ('dashboard', 'dispatch', 'ground', 'performance', 'navigation', 'atc',
                                    'failures', 'checklists', 'presets', 'settings')] + [f'action_{i}' for i in range(1, 7)] + ['power']
    for k in range(touches):
        base = len(vertices)
        vertices.extend(['VT\t0\t0\t0\t0\t1\t0\t0\t0'] * 4)
        body += [f'ATTR_manip_command\tbutton\ta380x/xp12/efb/{names[k % 17]}\tCAPT A380 panel', f'TRIS\t{len(indices)}\t6',
                 'ATTR_manip_none']
        indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))
    body += ['ATTR_manip_command\thand\tsim/flight_controls/door_toggle_16\tUPPER DECK 3 RIGHT', f'TRIS\t0\t3', 'ATTR_manip_none']
    return '\n'.join(['I', '800', 'OBJ', '# A380X FLYPAD ORIENTATION: UV0 TOUCH0 A380_CONTROL_UI_V3',
                      f'POINT_COUNTS\t{len(vertices)}\t0\t0\t{len(indices)}', *vertices,
                      *(f'IDX\t{i}' for i in indices), 'ATTR_draw_disable', *body, 'ATTR_draw_enable',
                      'ATTR_cockpit_region\t0', 'ATTR_poly_os\t2', *region, 'ATTR_no_cockpit', 'ATTR_poly_os\t0']) + '\n'


def parse(text):
    lines = text.splitlines()
    vertices = [tuple(map(float, l.split()[1:])) for l in lines if l.startswith('VT\t')]
    indices = [int(v) for l in lines if re.match(r'^IDX(?:10)?\s', l) for v in l.split()[1:]]
    counts = next(list(map(int, l.split()[1:])) for l in lines if l.startswith('POINT_COUNTS'))
    return lines, vertices, indices, counts


def device_quad(text, device):
    lines, vertices, indices, _ = parse(text)
    at = next(i for i, l in enumerate(lines) if l.startswith(f'ATTR_cockpit_device\t{device}\t'))
    tris = next(l for l in lines[at:] if l.startswith('TRIS\t'))
    start, count = map(int, tris.split()[1:])
    ids = indices[start:start + count]
    return [vertices[i] for i in (ids[0], ids[1], ids[2], ids[5])], ids


def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def norm(a): return math.sqrt(dot(a, a))


class Augment(unittest.TestCase):
    def test_retires_atlas_tablet_and_adds_device_quads(self):
        text = synthetic_controls()
        changed, stats = efb.augment_controls(text)
        self.assertEqual(stats, {'retired_atlas_faces': 2, 'retired_touch_regions': 34, 'touch_quads': 2})
        lines, vertices, indices, counts = parse(changed)
        self.assertEqual((counts[0], counts[3]), (len(vertices), len(indices)))
        self.assertFalse([l for l in lines if efb.LEGACY_TOUCH.match(l)])
        self.assertIn('ATTR_manip_command\thand\tsim/flight_controls/door_toggle_16\tUPPER DECK 3 RIGHT', lines)
        self.assertEqual(sum(1 for l in lines if l.startswith(efb.RETIRED + 'TRIS\t')), 34)
        self.assertEqual(sum(1 for l in lines if l.startswith(efb.RETIRED + 'atlas EFB face')), 2)
        region = lines[lines.index('ATTR_cockpit_region\t0'):lines.index('ATTR_no_cockpit')]
        self.assertEqual(sum(1 for l in region if l.startswith('TRIS\t')), 1)          # the FCU face stays
        for side, device in enumerate(efb.EFB_DEVICES):
            at = lines.index(f'ATTR_cockpit_device\t{device}\t1\t{efb.EFB_CHANNELS[side]}\t1')
            self.assertTrue(lines[at + 1].startswith(f'ATTR_manip_device\tbutton\t{device}\t'))
            self.assertTrue(lines[at + 2].startswith('TRIS\t') and lines[at + 3] == 'ATTR_manip_none')
        self.assertEqual(efb.augment_controls(changed), (changed, {'already_applied': True}))   # idempotent
        screen, stats = efb.augment_screen('I\n800\nOBJ\nPOINT_COUNTS\t4\t0\t0\t6\n' + 'VT\t0\t0\t0\t0\t1\t0\t0\t0\n' * 4 +
                                           'IDX\t0\nIDX\t1\nIDX\t2\nIDX\t0\nIDX\t2\nIDX\t3\nTRIS\t0\t6\n')
        self.assertEqual(stats, {'visible_quads': 2})
        self.assertNotIn('ATTR_manip_', screen)
        self.assertEqual(efb.check(changed, screen), [])
        self.assertEqual(efb.augment_screen(screen)[0], screen)

    def test_refuses_an_unexpected_cockpit_object(self):
        with self.assertRaisesRegex(ValueError, 'two panel-atlas EFB faces'):
            efb.augment_controls(synthetic_controls(faces=1))
        with self.assertRaisesRegex(ValueError, '34 legacy EFB touch regions'):
            efb.augment_controls(synthetic_controls(touches=33))
        self.assertEqual(efb.check('x', 'y'), ['EFB device quads missing'])


class Geometry(unittest.TestCase):
    def setUp(self):
        self.controls, _ = efb.augment_controls(synthetic_controls())

    def test_quads_follow_the_measured_tablet_faces(self):
        for side, (corners, (_s, device, rows)) in enumerate(zip(EFB_SCREEN_SURFACES, efb.device_quads(efb.TOUCH_LIFT))):
            quad, ids = device_quad(self.controls, device)
            self.assertEqual(ids, [ids[0], ids[0] + 1, ids[0] + 2, ids[0], ids[0] + 2, ids[0] + 3])
            self.assertEqual([v[6:8] for v in quad], [(0, 0), (0, 1), (1, 1), (1, 0)])   # UV (0,0) = device bottom-left
            normal = quad[0][3:6]
            self.assertAlmostEqual(norm(normal), 1, places=6)
            for point, corner in zip(quad, corners):
                offset = sub(point[:3], corner)
                self.assertAlmostEqual(norm(offset), efb.TOUCH_LIFT, places=6)
                self.assertAlmostEqual(dot(offset, normal), efb.TOUCH_LIFT, places=6)   # along the face normal
            # Same front side as the legacy atlas face (raised toward the crew by +normal).
            horizontal, vertical = sub(corners[3], corners[0]), sub(corners[1], corners[0])
            cross = (horizontal[1] * vertical[2] - horizontal[2] * vertical[1], horizontal[2] * vertical[0] - horizontal[0] * vertical[2],
                     horizontal[0] * vertical[1] - horizontal[1] * vertical[0])
            self.assertGreater(dot(cross, normal), 0)
            # The device texel aspect matches the physical glass within 1.5 % (1440/1000 = old 720/500).
            aspect = norm(horizontal) / norm(vertical)
            self.assertAlmostEqual(aspect / (efb.EFB_DEVICE_SIZE[0] / efb.EFB_DEVICE_SIZE[1]), 1, delta=.015)
        self.assertEqual(efb.EFB_DEVICE_SIZE[0] * (EFB_PANEL_RECT[3] - EFB_PANEL_RECT[1]),
                         efb.EFB_DEVICE_SIZE[1] * (EFB_PANEL_RECT[2] - EFB_PANEL_RECT[0]))
        self.assertLess(efb.VISIBLE_LIFT, efb.TOUCH_LIFT)

    def test_touch_mapping_matches_the_legacy_tablet_regions(self):
        """Every legacy touch region lands on the same physical spot of the glass, and the native
        fallback (a380x_efb_device.inc) maps the device texel back to the same atlas pixel."""
        left, bottom, right, top = EFB_PANEL_RECT
        width, height = efb.EFB_DEVICE_SIZE
        for rect in (*EFB_PAGE_TOUCH_RECTS, *EFB_ACTION_TOUCH_RECTS, EFB_POWER_TOUCH_RECT):
            cx, cy = (rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2
            u, v = (cx - left) / (right - left), (cy - bottom) / (top - bottom)
            x, y = int(u * width), int(v * height)                       # X-Plane device texel (bottom-left)
            back_x = left + x * (right - left) / width                   # efb_device_to_atlas
            back_y = bottom + y * (top - bottom) / height
            self.assertLess(abs(back_x - cx), 1)
            self.assertLess(abs(back_y - cy), 1)
            self.assertTrue(rect[0] <= back_x <= rect[2] and rect[1] <= back_y <= rect[3])
            page_y = height - 1 - y                                       # web page coordinates (top-left)
            self.assertTrue(0 <= page_y < height)

    def test_native_constants_match(self):
        source = (ROOT / 'native/avionics/a380x_efb_device.inc').read_text()
        for device in efb.EFB_DEVICES:
            self.assertIn(f'"{device}"', source)
        self.assertIn('#define EFB_DEVICE_W 1440', source)
        self.assertIn('#define EFB_DEVICE_H 1000', source)
        for name in efb.EFB_DATAREFS:
            self.assertIn(f'"{name}"', source)
        native = (ROOT / 'native/avionics/a380x_efb.c').read_text()
        for name in efb.EFB_COMMANDS:
            self.assertIn(f'"{name}"', native)


class LiveAircraft(unittest.TestCase):
    """Read-only: the generator accepts the current aircraft objects (nothing is written)."""

    def test_live_objects_augment_cleanly(self):
        controls = LIVE / 'cockpit-controls/a380x_cockpit_controls.obj'
        screens = LIVE / 'cockpit-panels/a380x_mfd_screens.obj'
        if not controls.is_file() or not screens.is_file():
            self.skipTest('live aircraft objects not present')
        controls_text, screens_text = controls.read_text(), screens.read_text()
        if efb.MARKER in controls_text:
            self.assertEqual(efb.check(controls_text, screens_text), [])
            return
        changed, stats = efb.augment_controls(controls_text)
        screen, _ = efb.augment_screen(screens_text)
        self.assertEqual(stats['retired_touch_regions'], 34)
        self.assertEqual(efb.check(changed, screen), [])
        _, vertices, indices, counts = parse(changed)
        self.assertEqual((counts[0], counts[3]), (len(vertices), len(indices)))


if __name__ == '__main__':
    unittest.main()
