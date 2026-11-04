"""ENG OBJ migration must preserve the authored mesh and ordinary click target."""
import copy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from ecam_button_geometry import (
    BODY_PATH, COMMAND, CONTROLS_PATH, ENG_OBJECT_PATHS, LEGEND_PATH,
    LAMP_MARKER, MARKER, PRESS_REF, SELECTION_PATH, SELECTED_REF,
    button_targets, parsed_geometry, patch_eng_geometry,
)
from patch_efis_animations import vertex_payload


def obj(quad=False, controls=False):
    vertices = [(0., 0., 0.), (1., 0., 0.), (0., 0., 1.)]
    indices = [0, 1, 2]
    if quad:
        vertices.append((1., 0., 1.))
        indices += [1, 3, 2]
    header = ['I', '800', 'OBJ',
              'TEXTURE\t../../../cockpit_3d/fcu_labels.png',
              'TEXTURE_LIT\t../../../cockpit_3d/fcu_labels_LIT.png',
              f'POINT_COUNTS\t{len(vertices)}\t0\t0\t{len(indices)}']
    rows = ['VT\t'+'\t'.join(map(str, (*p, 0., 1., 0., 0., 0.))) for p in vertices]
    draws = ['ATTR_draw_disable', 'ATTR_manip_command\thand\t'+COMMAND+'\tENG'] if controls else []
    return '\n'.join([*header, *rows, *('IDX\t'+str(i) for i in indices),
                      *draws, f'TRIS\t0\t{len(indices)}', 'ATTR_manip_none', 'ATTR_draw_enable', ''])


def targets():
    rows = []
    for name, material, count in (
            ('PUSH_ECAM_ENG', 'a380_cockpit_pedestal_knobs02_4k', 3),
            ('PUSH_ECAM_ENG_SEQ1', 'a380x_cockpit_decal_emis', 6),
            ('PUSH_ECAM_ENG_SEQ2', 'a380x_cockpit_decal_lovis', 6)):
        rows.append(dict(name=name, material=material, count=count, points=None,
                         low=(-.1, -.1, -.1), high=(1.1, .1, 1.1),
                         kind='translation', poses=[(0., 0., 0.), (0., -.0006, .00004)],
                         dataref=PRESS_REF, values=[0., 1.], pivot=(0., 0., 0.),
                         animation='PUSH_ECAM_ENG'))
    return rows


class EngButtonGeometry(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.source = Path(self.temporary.name)/'source.gltf'
        self.source.write_text('{}')
        self.objects = {BODY_PATH: obj(), LEGEND_PATH: obj(True),
                        SELECTION_PATH: obj(True), CONTROLS_PATH: obj(True, True)}
        self.loader = patch('ecam_button_geometry.source_targets', side_effect=lambda *a, **kw: copy.deepcopy(targets()))
        self.loader.start()
        self.addCleanup(self.loader.stop)

    def generate(self, objects=None, distance=None):
        return patch_eng_geometry(self.source, objects or self.objects, distance)

    def test_body_legends_uv_indices_and_texture_are_unchanged(self):
        changed, report = self.generate()
        for name in ENG_OBJECT_PATHS[:-1]:
            self.assertEqual(vertex_payload(self.objects[name]), vertex_payload(changed[name]))
            self.assertEqual(changed[name].count('TRIS\t0\t'), 1)
            self.assertIn(PRESS_REF, changed[name])
            self.assertIn('ANIM_trans_key\t0\t0\t0\t0', changed[name])
            self.assertIn('ANIM_trans_key\t1\t0\t-0.0006\t0.00004', changed[name])
        self.assertEqual(report['source_mesh_indices']['PUSH_ECAM_ENG_SEQ1'], 6)
        self.assertTrue(report['original_mesh_geometry_unchanged'])

    def test_lamp_is_independent_state_lit_on_real_face(self):
        changed, _ = self.generate()
        text = changed[CONTROLS_PATH]
        self.assertIn(f'ATTR_light_level\t0\t1\t{SELECTED_REF}', text)
        self.assertIn(f'ANIM_hide\t-0.5\t0.5\t{SELECTED_REF}', text)
        tail = text.split(LAMP_MARKER)[1]
        self.assertIn('ATTR_manip_none', tail)
        self.assertIn(PRESS_REF, tail)
        self.assertNotIn('ATTR_manip_command', tail)
        old_vertices, old_indices = parsed_geometry(self.objects[CONTROLS_PATH])
        new_vertices, new_indices = parsed_geometry(text)
        self.assertEqual(new_vertices[:len(old_vertices)], old_vertices)
        self.assertEqual(new_indices[:len(old_indices)], old_indices)
        self.assertEqual(len(new_vertices)-len(old_vertices), 4)
        self.assertEqual(len(new_indices)-len(old_indices), 6)
        for vertex in new_vertices[len(old_vertices):]:
            self.assertAlmostEqual(vertex[1], .00006)
            self.assertEqual(vertex[-2:], (.0625, .6875))

    def test_click_target_remains_exactly_one_ordinary_command(self):
        changed, _ = self.generate()
        self.assertEqual(changed[CONTROLS_PATH].count(COMMAND), 1)
        old_command = next(l for l in self.objects[CONTROLS_PATH].splitlines() if l.startswith('ATTR_manip_command'))
        self.assertIn(old_command, changed[CONTROLS_PATH].splitlines())
        self.assertNotIn('ATTR_manip_drag', changed[CONTROLS_PATH])

    def test_idempotent_byte_identical(self):
        first, _ = self.generate()
        second, _ = self.generate(first)
        self.assertEqual(first, second)
        self.assertEqual(second[CONTROLS_PATH].count(LAMP_MARKER), 1)

    def test_distance_config_changes_only_stroke_not_axis(self):
        short = button_targets(self.source, .0003)[0]['poses'][1]
        self.assertAlmostEqual(sum(v*v for v in short)**.5, .0003)
        self.assertAlmostEqual(short[2]/short[1], .00004/-.0006)
        changed, _ = self.generate(distance=.0003)
        with self.assertRaisesRegex(ValueError, 'configured stroke'):
            self.generate(changed, .0008)

    def test_invalid_distance_rejected(self):
        for distance in (0, -1, .01, float('nan'), float('inf')):
            with self.subTest(distance=distance), self.assertRaises(ValueError):
                button_targets(self.source, distance)

    def test_partial_and_ambiguous_geometry_refused(self):
        for mutation in ('missing', 'extra_tri', 'wrong_command', 'missing_atlas', 'duplicate_lamp'):
            objects = dict(self.objects)
            if mutation == 'missing':
                del objects[BODY_PATH]
            elif mutation == 'extra_tri':
                objects[BODY_PATH] += 'TRIS\t0\t3\n'
            elif mutation == 'wrong_command':
                objects[CONTROLS_PATH] = objects[CONTROLS_PATH].replace('ATTR_manip_command', 'ATTR_manip_drag_axis')
            elif mutation == 'missing_atlas':
                objects[CONTROLS_PATH] = objects[CONTROLS_PATH].replace('TEXTURE_LIT', 'NO_LIT')
            else:
                objects[CONTROLS_PATH] += LAMP_MARKER+'\n'+LAMP_MARKER+'\n'
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                self.generate(objects)

    def test_wrong_press_axis_refused_before_adding_lamp(self):
        wrong = targets()
        for target in wrong:
            target['poses'][1] = (0., .0006, -.00004)
        with patch('ecam_button_geometry.source_targets', return_value=wrong):
            with self.assertRaisesRegex(ValueError, 'out of the button'):
                self.generate()


if __name__ == '__main__':
    unittest.main()
