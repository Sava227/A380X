import json
import math
import re
import struct
import tempfile
import unittest
from pathlib import Path

from avionics_panel_layout import (
    EFB_ACTION_TOUCH_RECTS,
    EFB_PANEL_RECT,
    EFB_PAGE_TOUCH_RECTS,
    EFB_POWER_TOUCH_RECT,
    EFB_SCREEN_SURFACES,
    EFIS_BARO_PANEL_RECTS,
    EFIS_BARO_SCREEN_SURFACES,
    EFIS_UPPER_PANEL_RECTS,
    EFIS_LOWER_PANEL_RECTS,
    EWD_PANEL_RECT,
    EWD_SCREEN_SURFACE,
    FCU_PANEL_RECT,
    MFD_AUTO_ADJUST,
    MFD_DEVICE_BUS,
    MFD_DEVICE_IDS,
    MFD_LIGHTING_CHANNELS,
    MFD_SCREEN_CORNERS,
    MFD_SCREEN_SIZE,
    MFD_SCREEN_SURFACES,
    ND_PANEL_RECTS,
    ND_SCREEN_SURFACES,
    PANEL_SIZE,
    PFD_PANEL_RECTS,
    PFD_SCREEN_SURFACES,
)
from build_cockpit_controls import (Control, FCU_SCREEN_QUADS, FCU_ANNUNCIATORS,
                                    FCU_ANNUNCIATOR_FIRST_CELL, FCU_LABEL_COLUMNS,
                                    FCU_LABEL_ROWS, MFD_SCREEN_TEXTURE_PATH, build_mfd_screens_obj,
                                    build_obj, load_controls, mfd_screen_black_png)
from avionics_provider import EFIS_COMMANDS, FCU_COMMANDS, KCCU_COMMANDS, KCCU_KEYS, MFD_COMMANDS, MFD_DEVICES


class CockpitControlTests(unittest.TestCase):
    def test_fcu_dimming_keeps_other_efis_and_glareshield_lamp_gates(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            binary = Path(directory) / 'mesh.bin'
            binary.write_bytes(struct.pack('<9f', 0, 0, 0, 1, 0, 0, 0, .938, .347)
                               + binary.read_bytes()[36:])
            document = json.loads(source.read_text())
            document['nodes'].extend([
                {'name': 'PUSH_EFIS_CS_LS', 'mesh': 0},
                {'name': 'PUSH_GLARESHIELD_CS_ATCMSG_SEQ1', 'mesh': 0},
            ])
            source.write_text(json.dumps(document))
            encoded, _ = build_obj(source, [Control('PUSH_TEST', 'sim/test/button', 'TEST')],
                                   {'sim/test/button'}, fcu_overlay=True)
            for node, ref in [('PUSH_EFIS_CS_LS_SEQ2', 'a380x/xp12/efis/captain/ls'),
                              ('PUSH_GLARESHIELD_CS_ATCMSG_SEQ1', 'a380x/xp12/glareshield/atc_lit')]:
                block = encoded.split(f'# FCU ANNUNCIATOR {node}\n', 1)[1].split('ATTR_light_level_reset', 1)[0]
                self.assertIn('ATTR_light_level\t0\t1\tsim/cockpit2/switches/avionics_power_on', block)
                self.assertIn('ANIM_hide\t0\t0\tsim/cockpit2/switches/avionics_power_on', block)
                self.assertIn(f'ANIM_hide\t-0.5\t0.5\t{ref}', block)

    def test_nd_surface_bounds_match_the_authored_cockpit_faces(self):
        self.assertEqual(ND_SCREEN_SURFACES, (
            (0.28937626, 0.44796520, 2.59150696, 32.89669418, 2.78888965, 32.96720886),
            (-0.44501489, -0.28624520, 2.59113622, 32.89653778, 2.78925443, 32.96731567),
        ))

    def make_gltf(self, root: Path, node_name: str = 'PUSH_TEST', stem: str = 'test') -> Path:
        positions = struct.pack('<9f', 0, 0, 0, 1, 0, 0, 0, 1, 0)
        normals = struct.pack('<9f', 0, 0, 1, 0, 0, 1, 0, 0, 1)
        indices = struct.pack('<3H', 0, 1, 2)
        (root / 'mesh.bin').write_bytes(positions + normals + indices)
        document = {
            'asset': {'version': '2.0'},
            'buffers': [{'uri': 'mesh.bin', 'byteLength': len(positions + normals + indices)}],
            'bufferViews': [
                {'buffer': 0, 'byteOffset': 0, 'byteLength': len(positions)},
                {'buffer': 0, 'byteOffset': len(positions), 'byteLength': len(normals)},
                {'buffer': 0, 'byteOffset': len(positions + normals), 'byteLength': len(indices)},
            ],
            'accessors': [
                {'bufferView': 0, 'componentType': 5126, 'count': 3, 'type': 'VEC3'},
                {'bufferView': 1, 'componentType': 5126, 'count': 3, 'type': 'VEC3'},
                {'bufferView': 2, 'componentType': 5123, 'count': 3, 'type': 'SCALAR'},
            ],
            'meshes': [{'primitives': [{'attributes': {'POSITION': 0, 'NORMAL': 1}, 'indices': 2}]}],
            'nodes': [{'name': node_name, 'translation': [1, 2, 3], 'mesh': 0}],
        }
        source = root / f'{stem}.gltf'
        source.write_text(json.dumps(document))
        return source

    def test_named_mesh_becomes_invisible_labeled_command_target(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            control = Control('PUSH_TEST', 'sim/test/button', 'TEST - useful sign')
            encoded, report = build_obj(source, [control], {'sim/test/button'})
            self.assertIn('POINT_COUNTS\t3\t0\t0\t3', encoded)
            self.assertIn('VT\t1\t2\t3', encoded)
            self.assertIn('IDX\t0\nIDX\t2\nIDX\t1', encoded)
            self.assertNotIn('IDX10\t0\t2\t1', encoded)
            self.assertIn('ATTR_draw_disable', encoded)
            self.assertIn('ATTR_manip_command\thand\tsim/test/button\tTEST - useful sign', encoded)
            self.assertEqual(report['triangles'], 1)

    def test_momentary_cockpit_door_switch_has_two_click_directions(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory), 'SWITCH_DOOR_LOCK')
            control = Control('SWITCH_DOOR_LOCK', None, 'CKPT DOOR - UNLOCK / NORM / LOCK',
                              command_up='a380x/xp12/cockpit_door/unlock',
                              command_down='a380x/xp12/cockpit_door/lock',
                              cursor='up_down', switch_axis='up_down')
            encoded, report = build_obj(source, [control])
            self.assertIn('ATTR_manip_command_switch_up_down\tup_down\t'
                          'a380x/xp12/cockpit_door/unlock\t'
                          'a380x/xp12/cockpit_door/lock', encoded)
            self.assertEqual(report['controls'][0]['manipulator'], 'command_switch_up_down')

    def test_switch_click_mesh_can_use_its_animated_norm_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory), 'SWITCH_DOOR_LOCK')
            control = Control('SWITCH_DOOR_LOCK', 'sim/test/button', 'DOOR',
                              rest_rotation=[2 ** -.5, 0, 0, 2 ** -.5])
            encoded, _ = build_obj(source, [control])
            self.assertIn('VT\t1\t2\t4', encoded)

    def test_avionics_overlay_adds_fcu_pfd_and_nd_screens_for_both_sides(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            encoded, report = build_obj(
                source,
                [Control('PUSH_TEST', 'sim/test/button', 'TEST')],
                {'sim/test/button'},
                fcu_overlay=True,
            )
            self.assertIn('TEXTURE\t../../../cockpit_3d/fcu_labels.png', encoded)
            self.assertIn('TEXTURE_LIT\t../../../cockpit_3d/fcu_labels_LIT.png', encoded)
            self.assertIn(
                f'COCKPIT_REGION\t0\t0\t{PANEL_SIZE[0]}\t{PANEL_SIZE[1]}',
                encoded,
            )
            self.assertIn('ATTR_cockpit_region\t0', encoded)
            self.assertIn(
                '# A380X FLYPAD ORIENTATION: UV0 TOUCH0 A380_CONTROL_UI_V3',
                encoded,
            )
            self.assertEqual(report['fcu_label_quads'], 0)
            self.assertEqual(report['fcu_screen_quads'], 2)
            self.assertEqual(report['pfd_screen_quads'], 2)
            self.assertEqual(report['nd_screen_quads'], 2)
            self.assertEqual(report['ewd_screen_quads'], 1)
            self.assertEqual(report['efb_screen_quads'], 2)
            self.assertEqual(report['efb_touch_targets'], 34)
            self.assertEqual(report['efis_screen_quads'], 22)
            self.assertEqual(report['triangles'], 151)
            self.assertEqual(report['mfd_device_touch_quads'], 2)
            self.assertEqual(report['mfd_devices'], list(MFD_DEVICE_IDS))
            self.assertEqual(report['kccu_targets'], 0)
            self.assertIn('a380x/xp12/efb/page/dashboard', encoded)
            self.assertIn('a380x/xp12/efb/action_6', encoded)
            self.assertIn('a380x/xp12/efb/power', encoded)
            efb_commands = [line.split('\t') for line in encoded.splitlines()
                            if line.startswith('ATTR_manip_command\t')
                            and '\ta380x/xp12/efb/' in line]
            self.assertEqual(len(efb_commands), 34)
            self.assertTrue(all(command[1] == 'button' for command in efb_commands))
            vertex_lines = [line.split('\t') for line in encoded.splitlines() if line.startswith('VT\t')]
            overlay = vertex_lines[3:]
            self.assertEqual(
                len(overlay),
                4 * (
                    len(FCU_SCREEN_QUADS) + len(PFD_SCREEN_SURFACES)
                    + len(ND_SCREEN_SURFACES) + 1 + len(EFB_SCREEN_SURFACES)
                    + len(EFB_SCREEN_SURFACES)
                    * (len(EFB_PAGE_TOUCH_RECTS) + len(EFB_ACTION_TOUCH_RECTS) + 1)
                    + 3  # separate CHR, UTC, and ET clock windows
                    + 1  # live gear warning arrow
                    + 22  # two baro windows and twenty EFIS label cells
                    + 4  # PLAN waypoint scroll touch regions
                    + 2  # MFD touch quads
                ),
            )
            for screen_index, (x0, x1, u0, u1) in enumerate(FCU_SCREEN_QUADS):
                quad = overlay[screen_index * 4:(screen_index + 1) * 4]
                self.assertAlmostEqual(float(quad[0][1]), x0)
                self.assertAlmostEqual(float(quad[1][1]), x1)
                fcu_left, _, fcu_right, _ = FCU_PANEL_RECT
                self.assertAlmostEqual(float(quad[0][7]),
                                       (fcu_left + u1 * (fcu_right - fcu_left)) / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[1][7]),
                                       (fcu_left + u0 * (fcu_right - fcu_left)) / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[0][8]), FCU_PANEL_RECT[1] / PANEL_SIZE[1])
                self.assertAlmostEqual(float(quad[2][8]), FCU_PANEL_RECT[3] / PANEL_SIZE[1])
                self.assertLess(float(quad[0][8]), float(quad[2][8]))
            efis_overlay = overlay[4 * len(FCU_SCREEN_QUADS):]
            for side in range(2):
                quad = efis_overlay[side * 44:side * 44 + 4]
                left, bottom, right, top = EFIS_BARO_PANEL_RECTS[side]
                self.assertAlmostEqual(float(quad[0][1]), EFIS_BARO_SCREEN_SURFACES[side][0])
                self.assertAlmostEqual(float(quad[0][7]), right / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[1][7]), left / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[0][8]), bottom / PANEL_SIZE[1])
                self.assertAlmostEqual(float(quad[2][8]), top / PANEL_SIZE[1])
                previous_x = math.inf
                for column in range(5):
                    for row, panel_rects in enumerate((EFIS_UPPER_PANEL_RECTS, EFIS_LOWER_PANEL_RECTS)):
                        offset = side * 44 + 4 + column * 8 + row * 4
                        quad = efis_overlay[offset:offset + 4]
                        left, bottom, right, top = panel_rects[side][column]
                        self.assertAlmostEqual(float(quad[0][7]), right / PANEL_SIZE[0])
                        self.assertAlmostEqual(float(quad[1][7]), left / PANEL_SIZE[0])
                        self.assertAlmostEqual(float(quad[0][8]), bottom / PANEL_SIZE[1])
                        self.assertAlmostEqual(float(quad[2][8]), top / PANEL_SIZE[1])
                        if row == 0:
                            # Pilot-visible left-to-right reverses source X.
                            self.assertLess(float(quad[0][1]), previous_x)
                            previous_x = float(quad[0][1])
            pfd_overlay = overlay[4 * (len(FCU_SCREEN_QUADS) + 3 + 22):]
            for screen_index, (surface, rect) in enumerate(zip(PFD_SCREEN_SURFACES, PFD_PANEL_RECTS)):
                x0, x1, *_surface = surface
                left, bottom, right, top = rect
                quad = pfd_overlay[screen_index * 4:(screen_index + 1) * 4]
                self.assertAlmostEqual(float(quad[0][1]), x0)
                self.assertAlmostEqual(float(quad[1][1]), x1)
                self.assertAlmostEqual(float(quad[0][7]), right / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[1][7]), left / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[0][8]), bottom / PANEL_SIZE[1])
                self.assertAlmostEqual(float(quad[2][8]), top / PANEL_SIZE[1])
            nd_overlay = pfd_overlay[4 * len(PFD_SCREEN_SURFACES):]
            for screen_index, (surface, rect) in enumerate(zip(ND_SCREEN_SURFACES, ND_PANEL_RECTS)):
                x0, x1, *_surface = surface
                left, bottom, right, top = rect
                quad = nd_overlay[screen_index * 4:(screen_index + 1) * 4]
                self.assertAlmostEqual(float(quad[0][1]), x0)
                self.assertAlmostEqual(float(quad[1][1]), x1)
                self.assertAlmostEqual(float(quad[0][7]), right / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[1][7]), left / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[0][8]), bottom / PANEL_SIZE[1])
                self.assertAlmostEqual(float(quad[2][8]), top / PANEL_SIZE[1])
            ewd_overlay = nd_overlay[4 * len(ND_SCREEN_SURFACES):]
            self.assertAlmostEqual(float(ewd_overlay[0][1]), EWD_SCREEN_SURFACE[0])
            self.assertAlmostEqual(float(ewd_overlay[0][7]), EWD_PANEL_RECT[2] / PANEL_SIZE[0])
            self.assertAlmostEqual(float(ewd_overlay[0][8]), EWD_PANEL_RECT[1] / PANEL_SIZE[1])
            efb_overlay = ewd_overlay[4:]
            left, bottom, right, top = EFB_PANEL_RECT
            side_stride = 4 * (1 + len(EFB_PAGE_TOUCH_RECTS) + len(EFB_ACTION_TOUCH_RECTS) + 1)
            for side in range(len(EFB_SCREEN_SURFACES)):
                side_start = side * side_stride
                quad = efb_overlay[side_start:side_start + 4]
                self.assertAlmostEqual(float(quad[0][7]), left / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[0][8]), bottom / PANEL_SIZE[1])
                self.assertAlmostEqual(float(quad[1][7]), left / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[1][8]), top / PANEL_SIZE[1])
                self.assertAlmostEqual(float(quad[2][7]), right / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[2][8]), top / PANEL_SIZE[1])
                self.assertAlmostEqual(float(quad[3][7]), right / PANEL_SIZE[0])
                self.assertAlmostEqual(float(quad[3][8]), bottom / PANEL_SIZE[1])
                # Physical up and texture up must agree on both authored faces.
                self.assertGreater(float(quad[1][2]), float(quad[0][2]))
                self.assertGreater(float(quad[1][8]), float(quad[0][8]))

                # Every tab, action and power button must hit the center of
                # its displayed pixels on both captain and first-officer EFBs.
                corners = EFB_SCREEN_SURFACES[side]
                bottom_left, top_left, _top_right, bottom_right = corners
                horizontal = tuple(bottom_right[i] - bottom_left[i] for i in range(3))
                vertical = tuple(top_left[i] - bottom_left[i] for i in range(3))
                normal = (
                    horizontal[1] * vertical[2] - horizontal[2] * vertical[1],
                    horizontal[2] * vertical[0] - horizontal[0] * vertical[2],
                    horizontal[0] * vertical[1] - horizontal[1] * vertical[0],
                )
                length = math.sqrt(sum(value * value for value in normal))
                normal = tuple(value / length for value in normal)
                rects = (*EFB_PAGE_TOUCH_RECTS, *EFB_ACTION_TOUCH_RECTS, EFB_POWER_TOUCH_RECT)
                for touch_index, (rect_left, rect_bottom, rect_right, rect_top) in enumerate(rects):
                    u = ((rect_left + rect_right) * 0.5 - left) / (right - left)
                    v = ((rect_bottom + rect_top) * 0.5 - bottom) / (top - bottom)
                    lower = tuple(
                        corners[0][i] + u * (corners[3][i] - corners[0][i])
                        for i in range(3)
                    )
                    upper = tuple(
                        corners[1][i] + u * (corners[2][i] - corners[1][i])
                        for i in range(3)
                    )
                    expected = tuple(
                        lower[i] + v * (upper[i] - lower[i]) + normal[i] * 0.006
                        for i in range(3)
                    )
                    touch_start = side_start + 4 * (touch_index + 1)
                    touch = efb_overlay[touch_start:touch_start + 4]
                    for axis in range(3):
                        center = sum(float(vertex[axis + 1]) for vertex in touch) / 4
                        self.assertAlmostEqual(center, expected[axis], places=7)

    def test_fcu_annunciators_use_upper_button_bars_and_hide_when_inactive(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            document = json.loads(source.read_text())
            # A button front in the same plane as the authored AFS buttons.
            scale = math.hypot(.938, .347)
            positions = [(x, .938 / scale * y, .347 / scale * y)
                         for x, y in ((0, 0), (.014, 0), (.014, .014), (0, .014))]
            raw = struct.pack('<12f', *(value for point in positions for value in point))
            (Path(directory) / 'button.bin').write_bytes(raw)
            document['buffers'].append({'uri': 'button.bin', 'byteLength': len(raw)})
            document['bufferViews'].append({'buffer': 1, 'byteLength': len(raw)})
            document['accessors'].append({'bufferView': 3, 'componentType': 5126,
                                          'count': 4, 'type': 'VEC3'})
            document['meshes'].append({'primitives': [{'attributes': {'POSITION': 3}}]})
            for index, (_label, node, _dataref) in enumerate(FCU_ANNUNCIATORS):
                document['nodes'].append({'name': node, 'mesh': 0,
                                          'translation': [index * 2, 2, 3]})
                document['nodes'].append({'name': node.removesuffix('_SEQ2'), 'mesh': 1,
                                          'translation': [index * 2, 2, 3]})
            source.write_text(json.dumps(document))
            encoded, report = build_obj(source, [Control('PUSH_TEST', 'sim/test/button', 'TEST')],
                                        {'sim/test/button'}, fcu_overlay=True)
            self.assertEqual(report['fcu_annunciator_quads'], 7)
            self.assertEqual(report['triangles'], 165)
            self.assertEqual(len(report['fcu_annunciators']), 7)
            vertices = [line.split('\t') for line in encoded.splitlines() if line.startswith('VT\t')]
            # The gear arrow follows the seven FCU bars; the two MFD touch quads come last.
            lamp_vertices = vertices[-40:-12]
            for index, (_label, node, dataref) in enumerate(FCU_ANNUNCIATORS):
                self.assertIn(f'# FCU ANNUNCIATOR {node}', encoded)
                self.assertIn(f'ATTR_light_level\t0\t1\t{dataref}', encoded)
                self.assertIn(f'ANIM_hide\t-0.0001\t0.0001\t{dataref}', encoded)
                self.assertNotIn(f'ANIM_hide\t-0.5\t0.5\t{dataref}', encoded)
                self.assertNotIn(f'ANIM_show\t0.5\t1.5\t{dataref}', encoded)
                block = encoded.split(f'# FCU ANNUNCIATOR {node}\n', 1)[1].split('ATTR_light_level_reset', 1)[0]
                self.assertNotIn('sim/cockpit2/switches/avionics_power_on', block)
                quad = lamp_vertices[index * 4:(index + 1) * 4]
                # Thin bar is centred on the body and lies in its upper quarter.
                self.assertAlmostEqual(float(quad[0][1]), index * 2 + .014 * .185)
                self.assertAlmostEqual(float(quad[1][1]), index * 2 + .014 * .815)
                center = [sum(float(vertex[axis + 1]) for vertex in quad) / 4
                          for axis in range(3)]
                up = ((center[1] - 2) * .938 + (center[2] - 3) * .347) / scale
                self.assertAlmostEqual(up, .014 * .75)
                cell = FCU_ANNUNCIATOR_FIRST_CELL + index
                column, row = cell % FCU_LABEL_COLUMNS, cell // FCU_LABEL_COLUMNS
                self.assertAlmostEqual(float(quad[0][7]), (column + 1) / FCU_LABEL_COLUMNS)
                self.assertAlmostEqual(float(quad[1][7]), column / FCU_LABEL_COLUMNS)
                self.assertAlmostEqual(float(quad[0][8]), 1 - (row + 1) / FCU_LABEL_ROWS)
                self.assertAlmostEqual(float(quad[2][8]), 1 - row / FCU_LABEL_ROWS)
            self.assertEqual(encoded.count('ANIM_begin'), encoded.count('ANIM_end'))

    def test_unknown_node_and_command_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            with self.assertRaisesRegex(ValueError, 'absent from X-Plane catalog'):
                build_obj(source, [Control('PUSH_TEST', 'sim/test/missing', 'TEST')], set())
            with self.assertRaisesRegex(ValueError, 'node not found'):
                build_obj(source, [Control('PUSH_MISSING', 'sim/test/button', 'TEST')])

    def test_aircraft_local_command_does_not_require_xplane_catalog_entry(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            encoded, report = build_obj(
                source,
                [Control('PUSH_TEST', 'a380x/xp12/efb/toggle', 'EFB - open / close')],
                set(),
            )
            self.assertIn(
                'ATTR_manip_command\thand\ta380x/xp12/efb/toggle\tEFB - open / close',
                encoded,
            )
            self.assertEqual(report['controls'][0]['command'], 'a380x/xp12/efb/toggle')

    def test_additional_source_controls_share_one_cockpit_object(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cockpit = self.make_gltf(root)
            cabin = self.make_gltf(root, 'DOOR_TEST', 'cabin')
            encoded, report = build_obj(
                cockpit,
                [Control('PUSH_TEST', 'sim/test/button', 'TEST')],
                {'sim/test/button', 'sim/test/door_toggle'},
                additional_sources=[(
                    cabin,
                    [Control('DOOR_TEST', 'sim/test/door_toggle', 'DOOR - open / close')],
                )],
            )
            self.assertIn('POINT_COUNTS\t6\t0\t0\t6', encoded)
            self.assertIn('ATTR_manip_command\thand\tsim/test/door_toggle\tDOOR - open / close', encoded)
            self.assertEqual(len(report['sources']), 2)
            self.assertEqual(len(report['controls']), 2)
            self.assertEqual(report['controls'][1]['source'], str(cabin))

    def test_named_mesh_becomes_drag_axis_target(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            control = Control(
                'PUSH_TEST', None, 'THROTTLE 1 - drag',
                'sim/cockpit2/engine/actuators/throttle_ratio[0]',
                [0.0, -0.002, -0.195], 0.0, 1.0,
            )
            encoded, report = build_obj(
                source,
                [control],
                set(),
                {'sim/cockpit2/engine/actuators/throttle_ratio'},
            )
            self.assertIn(
                'ATTR_manip_drag_axis\thand\t0\t-0.002\t-0.195\t0\t1\t'
                'sim/cockpit2/engine/actuators/throttle_ratio[0]\tTHROTTLE 1 - drag',
                encoded,
            )
            self.assertEqual(report['controls'][0]['manipulator'], 'drag_axis')

    def test_named_mesh_becomes_command_knob_target(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            control = Control(
                'PUSH_TEST', None, 'HEADING - rotate', command_up='sim/test/up',
                command_down='sim/test/down', cursor='rotate_large',
            )
            encoded, report = build_obj(source, [control], {'sim/test/up', 'sim/test/down'})
            self.assertIn(
                'ATTR_manip_command_knob\trotate_large\tsim/test/up\tsim/test/down\tHEADING - rotate',
                encoded,
            )
            self.assertEqual(report['controls'][0]['manipulator'], 'command_knob')

    def test_named_mesh_becomes_two_value_toggle_target(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            control = Control(
                'PUSH_TEST', None, 'ALT increment - toggle 100 / 1000 feet',
                dataref='sim/aircraft/autopilot/alt_step_ft', cursor='button',
                toggle_values=[100, 1000],
            )
            encoded, report = build_obj(
                source, [control], set(), {'sim/aircraft/autopilot/alt_step_ft'},
            )
            self.assertIn(
                'ATTR_manip_toggle\tbutton\t100\t1000\t'
                'sim/aircraft/autopilot/alt_step_ft\tALT increment - toggle 100 / 1000 feet',
                encoded,
            )
            self.assertEqual(report['controls'][0]['manipulator'], 'toggle')

    def test_incomplete_command_knob_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'controls.json'
            config.write_text(json.dumps([{
                'node': 'PUSH_TEST', 'command': None, 'label': 'TEST', 'command_up': 'sim/test/up',
            }]))
            with self.assertRaisesRegex(ValueError, 'both command_up and command_down'):
                load_controls(config)

    def test_push_pull_knob_targets_are_separate_and_cover_the_knob_face(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory), 'KNOB_TEST')
            control = Control(
                'KNOB_TEST', None, 'SPEED - rotate outer edge',
                command_up='a380x/xp12/fcu/speed_up', command_down='a380x/xp12/fcu/speed_down',
                command_push='a380x/xp12/fcu/speed_push', command_pull='a380x/xp12/fcu/speed_pull',
                push_label='SPEED PUSH - managed', pull_label='SPEED PULL - selected',
            )
            encoded, report = build_obj(source, [control], set())
            self.assertEqual(report['vertices'], 16)
            self.assertEqual(report['triangles'], 8)
            self.assertEqual(encoded.count('ATTR_manip_command_knob'), 2)
            self.assertIn('a380x/xp12/fcu/speed_push\tSPEED PUSH - managed', encoded)
            self.assertIn('a380x/xp12/fcu/speed_pull\tSPEED PULL - selected', encoded)
            self.assertEqual([item['node'] for item in report['controls']],
                             ['KNOB_TEST', 'KNOB_TEST', 'KNOB_TEST:push', 'KNOB_TEST:pull'])
            vertices = [tuple(map(float, line.split('\t')[1:4]))
                        for line in encoded.splitlines() if line.startswith('VT\t')]
            # Rotate left/right, push upper-centre, pull lower-centre are
            # adjacent rectangles on one plane, so no hit target masks another.
            norm = math.hypot(0.347, 0.938)
            vertical = (0.0, 0.938 / norm, 0.347 / norm)
            normal = (0.0, vertical[2], -vertical[1])
            rects = []
            depths = []
            for index in range(0, 16, 4):
                points = vertices[index:index + 4]
                ys = [sum(point[a] * vertical[a] for a in range(3)) for point in points]
                rects.append((min(p[0] for p in points), min(ys), max(p[0] for p in points), max(ys)))
                depths.extend(sum(point[a] * normal[a] for a in range(3)) for point in points)
            self.assertLess(max(depths) - min(depths), 1e-7)
            for index, first in enumerate(rects):
                for second in rects[index + 1:]:
                    self.assertTrue(first[2] <= second[0] or second[2] <= first[0]
                                    or first[3] <= second[1] or second[3] <= first[1])
            outer_area = (max(r[2] for r in rects) - min(r[0] for r in rects)) * (
                max(r[3] for r in rects) - min(r[1] for r in rects))
            self.assertAlmostEqual(sum((r[2] - r[0]) * (r[3] - r[1]) for r in rects), outer_area)
            self.assertGreater(rects[2][1], rects[3][1])

    def test_push_pull_controls_validate_all_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            control = Control('PUSH_TEST', None, 'TEST', command_up='sim/test/up',
                              command_down='sim/test/down', command_push='sim/test/push',
                              command_pull='sim/test/pull')
            with self.assertRaisesRegex(ValueError, 'sim/test/pull, sim/test/push'):
                build_obj(source, [control], {'sim/test/up', 'sim/test/down'})

    def test_push_pull_requires_both_actions_and_a_rotary_knob(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'controls.json'
            item = {'node': 'KNOB_TEST', 'command': None, 'label': 'TEST',
                    'command_up': 'a380x/xp12/fcu/speed_up',
                    'command_down': 'a380x/xp12/fcu/speed_down',
                    'command_push': 'a380x/xp12/fcu/speed_push'}
            config.write_text(json.dumps([item]))
            with self.assertRaisesRegex(ValueError, 'both command_push and command_pull'):
                load_controls(config)
            item['command_pull'] = 'a380x/xp12/fcu/speed_pull'
            config.write_text(json.dumps([item]))
            self.assertEqual(len(load_controls(config)), 1)
            item.pop('command_up')
            item.pop('command_down')
            item['command'] = 'a380x/xp12/fcu/speed_push'
            config.write_text(json.dumps([item]))
            with self.assertRaisesRegex(ValueError, 'requires a rotary command knob'):
                load_controls(config)

    def test_unknown_drag_dataref_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            control = Control('PUSH_TEST', None, 'TEST', 'sim/test/missing[0]', [0, 0, 1])
            with self.assertRaisesRegex(ValueError, 'absent from X-Plane catalog'):
                build_obj(source, [control], set(), set())

    def test_dataref_control_requires_one_drag_or_toggle_shape(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'controls.json'
            config.write_text(json.dumps([{
                'node': 'PUSH_TEST', 'command': None, 'label': 'TEST',
                'dataref': 'sim/test/value',
            }]))
            with self.assertRaisesRegex(ValueError, 'exactly one drag axis or toggle pair'):
                load_controls(config)

    def test_duplicate_configuration_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'controls.json'
            item = {'node': 'PUSH_TEST', 'command': 'sim/test/button', 'label': 'TEST'}
            config.write_text(json.dumps([item, item]))
            with self.assertRaisesRegex(ValueError, 'must be unique'):
                load_controls(config)

    def test_shipped_configuration_covers_fcu_efis_overhead_and_engine_masters(self):
        config = Path(__file__).resolve().parents[1] / 'config' / 'cockpit_controls.json'
        controls = load_controls(config)
        self.assertEqual(len(controls), 276)
        nodes = {control.node for control in controls}
        self.assertNotIn('SCREEN_EFB', nodes)
        self.assertNotIn('SCREEN_EFB_FO', nodes)
        self.assertIn('PUSH_FCU_AP1', nodes)
        self.assertIn('PUSH_FCU_SPDMACH', nodes)
        self.assertIn('PUSH_FCU_HDGTRK', nodes)
        self.assertIn('PUSH_FCU_TRUEMAG', nodes)
        self.assertIn('PUSH_FCU_METER', nodes)
        self.assertIn('KNOB_FCU_SPEED', nodes)
        self.assertIn('KNOB_FCU_HDG', nodes)
        self.assertIn('KNOB_FCU_ALT', nodes)
        self.assertIn('KNOB_FCU_SELECTOR_ALT', nodes)
        self.assertIn('KNOB_FCU_UPDN', nodes)
        for index in range(1, 5):
            self.assertIn(f'LIGHTING_Knob_Glareshield_{index}', nodes)
        self.assertIn('PUSH_EFIS_CS_WX', nodes)
        self.assertIn('PUSH_EFIS_CS_LS', nodes)
        self.assertIn('PUSH_EFIS_FO_LS', nodes)
        self.assertIn('PUSH_EFIS_CS_CSTR', nodes)
        self.assertIn('PUSH_EFIS_FO_ARPT', nodes)
        self.assertIn('KNOB_EFIS_CS_RANGE', nodes)
        self.assertIn('KNOB_EFIS_FO_ROSE', nodes)
        self.assertIn('KNOB_EFIS_FO_BARO1', nodes)
        self.assertIn('PUSH_OVHD_ELEC_BAT1', nodes)
        self.assertIn('PUSH_OVHD_APU_MASTERSW', nodes)
        self.assertIn('PUSH_OVHD_ANTIICE_ENG4', nodes)
        self.assertIn('PUSH_OVHD_FUEL_FEEDTK4_MAIN', nodes)
        self.assertIn('SWITCH_OVHD_EXTLT_BEACON', nodes)
        self.assertIn('SWITCH_OVHD_INTLT_SEATBELT', nodes)
        self.assertIn('SWITCH_ENGINES_ENG1', nodes)
        self.assertIn('SWITCH_ENGINES_ENG4', nodes)
        self.assertIn('LEVER_THROTTLE_1_CLICK', nodes)
        self.assertIn('LEVER_THROTTLE_2_REVERSE', nodes)
        self.assertIn('ATHR_DISCONNECT_ENG2', nodes)
        self.assertIn('COCKPITDOOR_OPEN_CLICK', nodes)
        self.assertIn('SWITCH_DOOR_LOCK', nodes)
        switch = next(control for control in controls if control.node == 'SWITCH_DOOR_LOCK')
        self.assertEqual(switch.switch_axis, 'up_down')
        self.assertEqual(switch.command_up, 'a380x/xp12/cockpit_door/unlock')
        self.assertEqual(switch.command_down, 'a380x/xp12/cockpit_door/lock')
        self.assertAlmostEqual(switch.rest_rotation[0], 0.0039186981)

    def test_shipped_fcu_has_no_stock_autopilot_routes(self):
        config = Path(__file__).resolve().parents[1] / 'config' / 'cockpit_controls.json'
        controls = load_controls(config)
        serialized = config.read_text()
        self.assertNotIn('sim/autopilot/', serialized)
        self.assertNotIn('sim/aircraft/autopilot/', serialized)
        self.assertNotIn('sim/cockpit2/autopilot/', serialized)
        for control in controls:
            if '_FCU_' in control.node or control.node.startswith('ATHR_DISCONNECT'):
                for command in (control.command, control.command_up, control.command_down,
                                control.command_push, control.command_pull):
                    if command is not None:
                        self.assertTrue(command.startswith('a380x/xp12/fcu/'))
        knobs = [c for c in controls if c.node in ('KNOB_FCU_SPEED', 'KNOB_FCU_HDG', 'KNOB_FCU_ALT', 'KNOB_FCU_UPDN')]
        self.assertEqual(len(knobs), 4)
        for control in knobs:
            self.assertIsNotNone(control.command_push)
            self.assertIsNotNone(control.command_pull)
        configured = {command for control in controls for command in (
            control.command, control.command_up, control.command_down, control.command_push, control.command_pull)
            if command is not None and command.startswith('a380x/xp12/fcu/')}
        self.assertEqual(configured, set(FCU_COMMANDS) - {
            'a380x/xp12/fcu/ap_disconnect', 'a380x/xp12/fcu/ls_captain_toggle',
            'a380x/xp12/fcu/ls_first_officer_toggle'})

    def test_efis_controls_route_each_physical_button_to_its_correct_native_side(self):
        config = Path(__file__).resolve().parents[1] / 'config' / 'cockpit_controls.json'
        controls = {control.node: control for control in load_controls(config)}
        configured = set()
        for short, side in (('CS', 'captain'), ('FO', 'first_officer')):
            prefix = f'a380x/xp12/efis/{side}/'
            for node, action in (('TAXI', 'taxi'), ('LS', 'ls'), ('VV', 'vv'),
                                 ('VORD', 'vord'), ('VOR', 'vor1'), ('ADF', 'vor2')):
                self.assertEqual(controls[f'PUSH_EFIS_{short}_{node}'].command, prefix + action + '_toggle')
            baro = controls[f'KNOB_EFIS_{short}_BARO1']
            self.assertEqual(baro.command_push, prefix + 'baro_std')
            self.assertEqual(baro.command_pull, prefix + 'baro_qnh')
            self.assertEqual(controls[f'KNOB_EFIS_{short}_BARO2'].command, prefix + 'baro_unit_toggle')
        for control in controls.values():
            configured.update(command for command in (
                control.command, control.command_up, control.command_down, control.command_push, control.command_pull)
                if command and command.startswith('a380x/xp12/efis/'))
        assignable_only = {f'a380x/xp12/efis/{side}/plan_{direction}'
                           for side in ('captain', 'first_officer') for direction in ('next', 'previous')}
        self.assertEqual(configured, set(EFIS_COMMANDS) - assignable_only)

    def test_misspelled_custom_fcu_command_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            with self.assertRaisesRegex(ValueError, 'absent from native FCU provider'):
                build_obj(source, [Control('PUSH_TEST', 'a380x/xp12/fcu/speed_typo', 'TEST')], set())

    def test_mfd_surface_bounds_match_the_authored_cockpit_faces(self):
        self.assertEqual(MFD_SCREEN_SURFACES, (
            (0.11171729, 0.27178246, 2.42302714, 32.69813615, 2.54697620, 32.86913103),
            (-0.27272221, -0.11295360, 2.42319904, 32.69832317, 2.54691513, 32.86900612),
        ))
        self.assertEqual(MFD_SCREEN_SIZE, (768, 1024))
        self.assertEqual(MFD_DEVICE_IDS, tuple(device for device, _name in MFD_DEVICES))
        self.assertEqual((MFD_DEVICE_BUS, MFD_LIGHTING_CHANNELS, MFD_AUTO_ADJUST), (1, (14, 15), 1))
        for (x0, x1, y0, z0, y1, z1), corners in zip(MFD_SCREEN_SURFACES, MFD_SCREEN_CORNERS):
            # Seat-view BL, TL, TR, BR: positive source X is the crew's left.
            self.assertEqual(corners, ((x1, y0, z0), (x1, y1, z1), (x0, y1, z1), (x0, y0, z0)))

    def overlay_build(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            return build_obj(source, [Control('PUSH_TEST', 'sim/test/button', 'TEST')],
                             {'sim/test/button'}, fcu_overlay=True)

    @staticmethod
    def quad_normal(corners):
        bottom_left, top_left, _top_right, bottom_right = corners
        h = [bottom_right[i] - bottom_left[i] for i in range(3)]
        v = [top_left[i] - bottom_left[i] for i in range(3)]
        n = (h[1] * v[2] - h[2] * v[1], h[2] * v[0] - h[0] * v[2], h[0] * v[1] - h[1] * v[0])
        length = math.sqrt(sum(value * value for value in n))
        return tuple(value / length for value in n)

    def test_mfd_touch_quads_are_device_manipulators_inside_the_draw_disabled_block(self):
        encoded, _report = self.overlay_build()
        lines = encoded.splitlines()
        disabled = lines.index('ATTR_draw_disable')
        enabled = lines.index('ATTR_draw_enable')
        indices = [int(value) for line in lines if line.startswith(('IDX\t', 'IDX10\t'))
                   for value in line.split('\t')[1:]]
        vertex_count = sum(line.startswith('VT\t') for line in lines)
        for side, (device, seat) in enumerate(zip(MFD_DEVICE_IDS, ('CAPT', 'F/O'))):
            tag = f'ATTR_cockpit_device\t{device}\t1\t{MFD_LIGHTING_CHANNELS[side]}\t1'
            self.assertEqual(lines.count(tag), 1)
            at = lines.index(tag)
            self.assertTrue(disabled < at < enabled)
            self.assertEqual(lines[at + 1], f'ATTR_manip_device\tarrow\t{device}\t{seat} MFD')
            self.assertRegex(lines[at + 2], r'^TRIS\t\d+\t6$')
            self.assertEqual(lines[at + 3:at + 5], ['ATTR_manip_none', 'ATTR_no_cockpit'])
            first = int(lines[at + 2].split('\t')[1])
            # The touch quads are the last overlay geometry: no earlier offset moves.
            self.assertEqual(first, len(indices) - 12 + 6 * side)
            self.assertEqual(indices[first:first + 6],
                             [vertex_count - 8 + 4 * side + offset for offset in (0, 1, 2, 0, 2, 3)])
        # Device touch quads must never be hidden (spec 08 section 2.4).
        device_block = lines[lines.index('ATTR_cockpit_device\t' + MFD_DEVICE_IDS[0] + '\t1\t14\t1'):enabled]
        self.assertFalse(any(line.startswith(('ANIM_', 'ATTR_cockpit_region')) for line in device_block))
        self.assertEqual(encoded.count('ATTR_manip_device'), 2)

    def test_mfd_touch_quads_use_upright_device_uvs_raised_three_millimetres(self):
        encoded, _report = self.overlay_build()
        vertices = [tuple(map(float, line.split('\t')[1:])) for line in encoded.splitlines()
                    if line.startswith('VT\t')][-8:]
        for side, corners in enumerate(MFD_SCREEN_CORNERS):
            quad = vertices[side * 4:side * 4 + 4]
            normal = self.quad_normal(corners)
            # BL (0,0), TL (0,1), TR (1,1), BR (1,0) as seen from the seat.
            self.assertEqual([vertex[6:8] for vertex in quad],
                             [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)])
            for vertex, corner in zip(quad, corners):
                for axis in range(3):
                    self.assertAlmostEqual(vertex[axis], corner[axis] + 0.003 * normal[axis], places=6)
                    self.assertAlmostEqual(vertex[3 + axis], normal[axis], places=7)
            # Physical up equals texture up; crew-left (larger source X) is u = 0.
            self.assertGreater(quad[1][1], quad[0][1])
            self.assertGreater(quad[1][7], quad[0][7])
            self.assertGreater(quad[0][0], quad[3][0])
            self.assertLess(quad[0][6], quad[3][6])
            # The face points at the crew (crew looks toward +Z) and is tilted 54 degrees back.
            self.assertGreater(normal[1], 0.8)
            self.assertLess(normal[2], -0.58)

    def test_mfd_touch_quads_are_clockwise_front_faces(self):
        encoded, _report = self.overlay_build()
        lines = encoded.splitlines()
        vertices = [tuple(map(float, line.split('\t')[1:7])) for line in lines if line.startswith('VT\t')]
        indices = [int(value) for line in lines if line.startswith(('IDX\t', 'IDX10\t'))
                   for value in line.split('\t')[1:]]
        for offset in range(len(indices) - 12, len(indices), 3):
            a, b, c = (vertices[index] for index in indices[offset:offset + 3])
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            cross = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            # validate_aircraft.py: OBJ8 front faces have cross . normal < 0.
            self.assertLess(sum(cross[i] * (a[3 + i] + b[3 + i] + c[3 + i]) for i in range(3)), -1e-4)

    def test_visible_mfd_object_matches_the_touch_quads(self):
        encoded, _report = self.overlay_build()
        touch = [tuple(map(float, line.split('\t')[1:])) for line in encoded.splitlines()
                 if line.startswith('VT\t')][-8:]
        screens = build_mfd_screens_obj()
        lines = screens.splitlines()
        self.assertEqual(lines[:3], ['I', '800', 'OBJ'])
        self.assertIn(f'TEXTURE\t{MFD_SCREEN_TEXTURE_PATH}', lines)
        self.assertEqual(MFD_SCREEN_TEXTURE_PATH,
                         '../../../textures/interior/cockpit-panels/mfd_screen_black.png')
        self.assertIn('GLOBAL_cockpit_lit', lines)
        self.assertNotIn('ATTR_manip_', screens)
        self.assertNotIn('TEXTURE_LIT', screens)
        self.assertIn('POINT_COUNTS\t8\t0\t0\t12', lines)
        visible = [tuple(map(float, line.split('\t')[1:])) for line in lines if line.startswith('VT\t')]
        self.assertEqual(len(visible), 8)
        for side, corners in enumerate(MFD_SCREEN_CORNERS):
            normal = self.quad_normal(corners)
            for index in range(4):
                screen, hit = visible[side * 4 + index], touch[side * 4 + index]
                self.assertEqual(screen[3:], hit[3:])   # same normal and UV
                for axis in range(3):
                    # VT prints 9 significant digits (0.1 um at z = 32 m).
                    # Visible quad 2 mm above the dark face; the hit surface must
                    # be nearer the crew, not hidden behind that visible quad.
                    self.assertAlmostEqual(screen[axis], corners[index][axis] + 0.002 * normal[axis], places=6)
                    self.assertAlmostEqual(hit[axis] - screen[axis], 0.001 * normal[axis], places=6)
        block = lines[lines.index('ATTR_poly_os\t2'):]
        self.assertEqual(block, [
            'ATTR_poly_os\t2',
            f'ATTR_cockpit_device\t{MFD_DEVICE_IDS[0]}\t1\t14\t1', 'TRIS\t0\t6', 'ATTR_no_cockpit',
            f'ATTR_cockpit_device\t{MFD_DEVICE_IDS[1]}\t1\t15\t1', 'TRIS\t6\t6', 'ATTR_no_cockpit',
            'ATTR_poly_os\t0', '',
            '# Generated from avionics_panel_layout.MFD_SCREEN_CORNERS by build_cockpit_controls.py.',
        ])
        indices = [int(value) for line in lines if line.startswith(('IDX\t', 'IDX10\t'))
                   for value in line.split('\t')[1:]]
        self.assertEqual(indices, [0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7])
        png = mfd_screen_black_png()
        self.assertEqual(png[:8], b'\x89PNG\r\n\x1a\n')
        # 8x8, 8-bit RGB (colour type 2): opaque by construction, every byte black.
        self.assertEqual(struct.unpack('>2I5B', png[16:29]), (8, 8, 8, 2, 0, 0, 0))
        import zlib
        idat_length = struct.unpack('>I', png[33:37])[0]
        self.assertEqual(png[37:41], b'IDAT')
        self.assertEqual(zlib.decompress(png[41:41 + idat_length]), b'\0' * (8 * (1 + 8 * 3)))

    def test_mfd_touch_refresh_is_precise_and_idempotent(self):
        from mfd_cockpit_devices import refresh_touch_geometry
        from obj8_numbers import fmt as obj8_fmt
        from build_cockpit_controls import mfd_device_quads
        encoded, _report = self.overlay_build()
        legacy = encoded
        for (_, _, old), (_, _, new) in zip(mfd_device_quads(.001), mfd_device_quads(.003)):
            for old_vertex, new_vertex in zip(old, new):
                old_line = 'VT\t' + '\t'.join(obj8_fmt(v) for v in old_vertex)
                new_line = 'VT\t' + '\t'.join(obj8_fmt(v) for v in new_vertex)
                self.assertEqual(legacy.count(new_line), 1)
                legacy = legacy.replace(new_line, old_line)
        refreshed = refresh_touch_geometry(legacy)
        self.assertEqual(refreshed, encoded)
        self.assertEqual(refresh_touch_geometry(refreshed), refreshed)
        self.assertEqual(sum(a != b for a, b in zip(legacy.splitlines(), refreshed.splitlines())), 8)
        with self.assertRaises(ValueError):
            refresh_touch_geometry(legacy.replace('ATTR_manip_device', 'ATTR_manip_none', 1))
        with self.assertRaises(ValueError):
            refresh_touch_geometry(legacy.replace('TRIS\t', 'LINES\t'))

    def test_kccu_controls_route_each_key_to_its_own_side(self):
        config = Path(__file__).resolve().parents[1] / 'config' / 'cockpit_controls.json'
        controls = {control.node: control for control in load_controls(config)}
        kccu = {node: control for node, control in controls.items() if node.startswith('PUSH_KCCU')}
        self.assertEqual(len(kccu), 136)
        configured = set()
        for letter, side, seat in (('L', 'captain', 'CAPT'), ('R', 'first_officer', 'F/O')):
            prefix = f'a380x/xp12/kccu/{side}/'
            for key in KCCU_KEYS:
                control = kccu.pop(f'PUSH_KCCU{letter}_{key.upper()}')
                self.assertEqual(control.command, prefix + key)
                self.assertEqual(control.command_cursor, 'button')
                self.assertTrue(control.label.startswith(f'{seat} KCCU - '), control.label)
            wheel = kccu.pop(f'PUSH_KCCU{letter}_WHEEL')
            self.assertEqual((wheel.command, wheel.command_up, wheel.command_down, wheel.cursor),
                             (None, prefix + 'wheel_up', prefix + 'wheel_down', 'rotate_small'))
            self.assertEqual(kccu.pop(f'PUSH_KCCU{letter}_LBUTTON').command, prefix + 'click')
            self.assertEqual(kccu.pop(f'PUSH_KCCU{letter}_KBD_ON_OFF').command, prefix + 'kbd_switch_toggle')
            self.assertEqual(kccu.pop(f'PUSH_KCCU{letter}_CCD_ON_OFF').command, prefix + 'ccd_switch_toggle')
            ball = kccu.pop(f'PUSH_KCCU{letter}_BALL')
            self.assertEqual((ball.command, ball.command_cursor), (f'a380x/xp12/mfd/{side}/popup', 'hand'))
        self.assertEqual(kccu, {})
        for control in controls.values():
            commands = (control.command, control.command_up, control.command_down,
                        control.command_push, control.command_pull)
            configured.update(command for command in commands if command
                              and command.startswith(('a380x/xp12/kccu/', 'a380x/xp12/mfd/')))
            side = {'L': 'captain', 'R': 'first_officer'}.get(control.node[9:10])
            if control.node.startswith('PUSH_KCCU'):
                # Never cross-wired: a KCCU only addresses the MFD on its own side.
                self.assertTrue(all(f'/{side}/' in command for command in commands if command))
            self.assertFalse(control.node.endswith('_SEQ1') or '_BLANK' in control.node
                             or control.node in ('KCCU', 'KCCU_COVER_BALL', 'PUSH_KCCUL_WHEEL.001'))
        popout = {command for command in MFD_COMMANDS if command.endswith('/popout')}
        self.assertEqual(configured, set(KCCU_COMMANDS) | set(MFD_COMMANDS) - popout)
        self.assertEqual(controls['KNOB_EFIS_CS_MFD'].label, 'CAPT MFD DU - brightness and off')
        self.assertEqual(controls['KNOB_EFIS_FO_MFD'].label, 'F/O MFD DU - brightness and off')
        self.assertNotIn('MFD unavailable', config.read_text())

    def test_misspelled_kccu_or_mfd_command_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.make_gltf(Path(directory))
            for command in ('a380x/xp12/kccu/captain/enter', 'a380x/xp12/kccu/copilot/ent',
                            'a380x/xp12/mfd/captain/popup_toggle'):
                with self.subTest(command=command):
                    with self.assertRaisesRegex(ValueError, 'absent from native KCCU/MFD provider'):
                        build_obj(source, [Control('PUSH_TEST', command, 'TEST')], set())
            knob = Control('PUSH_TEST', None, 'TEST', command_up='a380x/xp12/kccu/captain/wheel_up',
                           command_down='a380x/xp12/kccu/captain/wheel_dn', cursor='rotate_small')
            with self.assertRaisesRegex(ValueError, 'wheel_dn'):
                build_obj(source, [knob], set())
            encoded, report = build_obj(source, [Control('PUSH_TEST', KCCU_COMMANDS[0], 'TEST')], set())
            self.assertIn(KCCU_COMMANDS[0], encoded)
            self.assertEqual(report['kccu_targets'], 0)
            self.assertEqual(report['mfd_device_touch_quads'], 0)
            self.assertNotIn('ATTR_manip_device', encoded)

    def test_shipped_cabin_door_configuration_covers_main_deck(self):
        project = Path(__file__).resolve().parents[1]
        controls = load_controls(project / 'config' / 'cabin_door_controls.json')
        self.assertEqual(len(controls), 10)
        for station in range(1, 6):
            for side, channel in (
                ('L', 2 * station - 1),
                ('R', 2 * station),
            ):
                control = next(
                    item for item in controls
                    if item.node == f'ANIM_DOOR_M{station}{side}_CLICK'
                )
                self.assertEqual(
                    control.command,
                    f'sim/flight_controls/door_toggle_{channel}',
                )

    def test_shipped_upper_deck_configuration_covers_all_six_doors(self):
        project = Path(__file__).resolve().parents[1]
        controls = load_controls(project / 'config' / 'upper_deck_door_controls.json')
        self.assertEqual(len(controls), 6)
        for station in range(1, 4):
            for side, channel in (
                ('L', 2 * station + 9),
                ('R', 2 * station + 10),
            ):
                control = next(
                    item for item in controls
                    if item.node == f'PAX_DOOR_U{station}{side}_HANDLE'
                )
                self.assertEqual(
                    control.command,
                    f'sim/flight_controls/door_toggle_{channel}',
                )

    def test_shipped_overhead_controls_have_native_animation_feedback(self):
        project = Path(__file__).resolve().parents[1]
        controls = load_controls(project / 'config' / 'cockpit_controls.json')
        mapping = json.loads(
            (project / 'config' / 'animation_datarefs.json').read_text(encoding='utf-8')
        )
        rules = [
            (re.compile(rule['pattern']), rule)
            for rule in mapping['mappings']
            if rule.get('dataref')
        ]
        overhead = [control for control in controls if '_OVHD_' in control.node]
        self.assertEqual(len(overhead), 41)
        for control in overhead:
            matches = [rule for pattern, rule in rules if pattern.fullmatch(control.node)]
            self.assertEqual(
                len(matches), 1,
                f'{control.node} must have one unambiguous visible-state animation mapping',
            )
            expected_prefix = ('a380x/xp12/adirs/' if '_ADIRS_' in control.node else
                               'a380x/xp12/elec/pb/' if '_OVHD_ELEC_' in control.node else 'sim/')
            self.assertTrue(matches[0]['dataref'].startswith(expected_prefix))
            self.assertLess(matches[0]['value_min'], matches[0]['value_max'])

    def test_shipped_efis_and_engine_controls_have_native_animation_feedback(self):
        project = Path(__file__).resolve().parents[1]
        controls = load_controls(project / 'config' / 'cockpit_controls.json')
        mapping = json.loads(
            (project / 'config' / 'animation_datarefs.json').read_text(encoding='utf-8')
        )
        rules = [
            (re.compile(rule['pattern']), rule)
            for rule in mapping['mappings']
            if rule.get('dataref')
        ]
        selected = [
            control for control in controls
            if ('_EFIS_' in control.node or control.node.startswith('SWITCH_ENGINES_ENG'))
            # DU brightness has its own mapping and command contract below.
            and not (control.command_up or '').startswith('a380x/xp12/du/')
        ]
        self.assertEqual(len(selected), 38)
        for control in selected:
            matches = [rule for pattern, rule in rules if pattern.fullmatch(control.node)]
            self.assertEqual(
                len(matches), 1,
                f'{control.node} must have one unambiguous visible-state animation mapping',
            )
            self.assertTrue(matches[0]['dataref'].startswith(
                'a380x/xp12/efis/' if '_EFIS_' in control.node else 'sim/'))
            self.assertLess(matches[0]['value_min'], matches[0]['value_max'])

    def test_du_knobs_animate_from_their_own_brightness_channels(self):
        project = Path(__file__).resolve().parents[1]
        controls = load_controls(project / 'config' / 'cockpit_controls.json')
        mapping = json.loads((project / 'config' / 'animation_datarefs.json').read_text())
        rules = [(re.compile(rule['pattern']), rule) for rule in mapping['mappings']]
        knobs = [control for control in controls
                 if (control.command_up or '').startswith('a380x/xp12/du/')]
        self.assertEqual(len(knobs), 8)
        for control in knobs:
            matches = [rule for pattern, rule in rules if pattern.fullmatch(control.node)]
            self.assertEqual(len(matches), 1, control.node)
            channel = control.command_up.removesuffix('_up')
            self.assertEqual(control.command_down, channel + '_down')
            self.assertEqual(matches[0]['dataref'], channel)
            self.assertEqual((matches[0]['value_min'], matches[0]['value_max']), (0, 1))


if __name__ == '__main__':
    unittest.main()
