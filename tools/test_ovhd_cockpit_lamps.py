"""Overhead annunciator lamps: contract, native table, OBJ patch idempotency, decal motion, legend
cells (synthetic + installed aircraft)."""
import re
import tempfile
import unittest
from pathlib import Path

from PIL import Image

from avionics_provider import AVIONICS_COMMANDS, AVIONICS_DATAREFS
from build_cockpit_controls import ADIRS_ANNUNCIATOR_FIRST_CELL, ELEC_LEGEND_CELLS
from elec_panel_contract import DATAREFS as ELEC_DATAREFS, LEGEND_COLORS
from elec_system_contract import DATAREFS as ELEC_NETWORK_DATAREFS
from ovhd_cockpit_lamps import (ANN_CLICK, ANN_MARKER, CONTROLS_OBJ, DECAL_OBJ, KNOBS_OBJ, MARKER, DecalMotion,
                                _append, atlas_patch, atlas_usage, augment_controls, cell_box, legend_cell_image,
                                patch_knobs, ann_lt_targets)
from ovhd_lamps_contract import (BUSES, COMMANDS, DATAREFS, LEGENDS, NEW_LEGEND_CELLS, PREFIX, PUSHBUTTONS, SOURCES,
                                 emit_c_table, fault_ids, lamps, parse_source, parse_supply)

ROOT = Path(__file__).resolve().parents[1]
GLTF = Path('/Volumes/Untitled/A380X-XPlane12/upstream-aircraft-large-files/fbw-a380x/src/base/'
            'flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/attachments/flybywire/'
            'Part_Interior_Cockpit/model/A380_COCKPIT_LOD00.gltf')
AIRCRAFT = ROOT / 'output/A380X-XP12'
HEADER = ('I\n800\nOBJ\nTEXTURE t.png\nPOINT_COUNTS\t3\t0\t0\t3\n'   # samples cell 56 (u, v ~ 0.05)
          'VT\t0\t0\t0\t0\t1\t0\t0.05\t0.05\nVT\t1\t0\t0\t0\t1\t0\t0.09\t0.05\nVT\t0\t0\t1\t0\t1\t0\t0.05\t0.09\n'
          'IDX\t0\nIDX\t1\nIDX\t2\nTRIS\t0\t3\n')


class ContractTests(unittest.TestCase):
    def test_rows_are_unique_and_valid(self):
        ids = [row[0] for row in PUSHBUTTONS]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual(len(PUSHBUTTONS), 127)
        self.assertEqual(len(lamps()), 217)
        for pid, _caption, node, upper, lower, supply, src1, src2 in PUSHBUTTONS:
            self.assertTrue(node.startswith('PUSH_OVHD_'), pid)
            self.assertFalse(node.startswith(('PUSH_OVHD_ELEC_', 'PUSH_OVHD_ADIRS_')), pid)  # own modules
            self.assertTrue(upper or lower, pid)
            for legend, source in ((upper, src1), (lower, src2)):
                self.assertEqual(legend is None, source is None, pid)
                if legend:
                    self.assertIn(legend, LEGENDS)
                    self.assertIn(parse_source(source)[0], SOURCES)
            any_of, all_of = parse_supply(supply)
            self.assertTrue(set(any_of) | set(all_of) <= set(BUSES), pid)
        self.assertEqual(len(DATAREFS), len(set(DATAREFS)))
        self.assertFalse(set(DATAREFS) & (set(ELEC_DATAREFS) | set(ELEC_NETWORK_DATAREFS)))
        self.assertTrue(set(DATAREFS) <= set(AVIONICS_DATAREFS) and set(COMMANDS) <= set(AVIONICS_COMMANDS))
        self.assertIn(PREFIX + 'lamp/apu_master_lower', DATAREFS)
        self.assertIn(PREFIX + 'fail/apu_bleed', DATAREFS)
        self.assertIn('pax_oxy', fault_ids())

    def test_fbw_supply_and_conditions_of_the_simulated_pushbuttons(self):
        rows = {row[0]: row for row in PUSHBUTTONS}
        expected = {
            'apu_master': ('fault', 'on', 'DC_APU', 'apu_master_fault', 'apu_master_on'),
            'apu_start': ('avail', 'on', 'AC_ESS|DC_APU', 'apu_avail', 'apu_start_on'),
            'apu_bleed': ('fault', 'on', 'DC_ESS', 'apu_bleed_fault', 'apu_bleed_on'),
            'eng1_bleed': ('fault', 'off', 'DC2', 'eng_bleed_fault:0', 'eng_bleed_off:0'),
            'eng2_bleed': ('fault', 'off', 'DC_ESS', 'eng_bleed_fault:1', 'eng_bleed_off:1'),
            'pack1': ('fault', 'off', 'DC1&DC_ESS', 'pack_fault:0', 'pack_off:0'),
            'feed1_main': ('fault', 'off', 'AC_ESS', 'fault', 'feed_pump_off:0'),
            'feed4_main': ('fault', 'off', 'AC2', 'fault', 'feed_pump_off:3'),
            'wing_anti_ice': ('fault', 'on', 'AC_ESS', 'wing_ai_fault', 'wing_ai_on'),
            'probes_window_heat': (None, 'on', 'AC2', None, 'window_heat_on'),
            'crew_oxy': (None, 'off', 'AC_ESS', None, 'crew_oxy_off'),
        }
        for pid, values in expected.items():
            self.assertEqual(rows[pid][3:], values, pid)

    def test_native_table_is_generated_from_the_contract(self):
        self.assertEqual((ROOT / 'native/avionics/a380x_ovhd_lamps_table.h').read_text(), emit_c_table())

    def test_legend_cells_reuse_identical_elec_adirs_cells_and_fill_label_cells(self):
        self.assertEqual(ADIRS_ANNUNCIATOR_FIRST_CELL, 47)
        for key in ('altn', 'disc', 'avail', 'on'):
            self.assertEqual(LEGENDS[key][2], ELEC_LEGEND_CELLS[key])
        for key in ('fault', 'off', 'altn', 'disc', 'avail', 'on'):
            self.assertEqual(LEGENDS[key][1], LEGEND_COLORS[key])
        cells = list(NEW_LEGEND_CELLS.values())
        self.assertEqual(sorted(cells), list(range(14)))
        used = {LEGENDS[row[3]][2] for row in lamps()}
        self.assertTrue(set(cells) <= used)


class SyntheticPatchTests(unittest.TestCase):
    def test_append_shifts_draws_and_counts(self):
        text = _append(HEADER, [(0, 1, 0, 0, 1, 0, 0, 0)] * 3, [0, 1, 2], ['TRIS\t0\t3', 'ATTR_draw_enable'])
        self.assertIn('POINT_COUNTS\t6\t0\t0\t6', text)
        self.assertIn(MARKER + '\nTRIS\t3\t3\nATTR_draw_enable', text)
        self.assertEqual(text.count('VT\t'), 6)

    def test_decal_motion_replays_the_latched_travel(self):
        obj = ('I\n800\nOBJ\nPOINT_COUNTS\t4\t0\t0\t6\n'
               'VT\t0\t0\t0\t0\t1\t0\t0\t0\nVT\t0.01\t0\t0\t0\t1\t0\t0\t0\n'
               'VT\t0.01\t0\t0.005\t0\t1\t0\t0\t0\nVT\t0\t0\t0.005\t0\t1\t0\t0\t0\n'
               'IDX\t0\nIDX\t1\nIDX\t2\nIDX\t0\nIDX\t2\nIDX\t3\n'
               'ANIM_begin\nANIM_trans_begin\tsim/cockpit2/bleedair/actuators/apu_bleed\n'
               'ANIM_trans_key\t0\t0\t0\t0\nANIM_trans_key\t1\t0\t0.0015\t0\nANIM_trans_end\nTRIS\t0\t6\nANIM_end\n')
        corners = [(0, 0, 0.005), (0.01, 0, 0.005), (0.01, 0, 0), (0, 0, 0)]
        ref, keyed = DecalMotion(obj).offsets(corners, (0, 1, 0))
        self.assertEqual(ref, 'sim/cockpit2/bleedair/actuators/apu_bleed')
        self.assertEqual([value for value, _ in keyed], [0.0, 1.0])
        self.assertAlmostEqual(keyed[1][1][1], 0.0015)
        far = [(x + 1, y, z) for x, y, z in corners]
        self.assertEqual(DecalMotion(obj).offsets(far, (0, 1, 0))[0], None)          # no decal there
        bad = obj.replace('sim/cockpit2/bleedair/actuators/apu_bleed', 'sim/flightmodel2/misc/door_open_ratio[16]')
        with self.assertRaises(ValueError):
            DecalMotion(bad).offsets(corners, (0, 1, 0))

    def test_atlas_patch_writes_only_the_new_cells_and_reruns_clean(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'fcu_labels.png'
            atlas = Image.new('RGBA', (1024, 512), (0, 0, 0, 0))
            atlas.paste((10, 20, 30, 255), (0, 0, 1024, 512))
            atlas.save(path)
            image, cells = atlas_patch(path, HEADER)
            self.assertEqual(cells, list(range(14)))
            self.assertEqual(image.crop(cell_box(20, 1024, 512)).getpixel((5, 5)), (10, 20, 30, 255))
            box = cell_box(NEW_LEGEND_CELLS['open'], 1024, 512)
            self.assertEqual(image.crop(box).tobytes(), legend_cell_image('open', (128, 64)).tobytes())
            image.save(path)
            self.assertEqual(atlas_patch(path, HEADER)[1], [])
            # a cell sampled by drawn geometry is never overwritten
            path2 = Path(directory) / 'b.png'
            atlas.save(path2)
            sampling = HEADER.replace('0.05\t0.05\n', '0.05\t0.95\n').replace('0.09\t0.05', '0.09\t0.95') \
                             .replace('0.05\t0.09', '0.05\t0.99')
            self.assertIn(0, atlas_usage(sampling))
            with self.assertRaises(ValueError):
                atlas_patch(path2, sampling)


@unittest.skipUnless(GLTF.is_file() and (AIRCRAFT / 'objects').is_dir(), 'pinned FBW cockpit glTF / aircraft not present')
class InstalledAircraftTests(unittest.TestCase):
    def test_installed_objects_carry_every_lamp_once_and_rerun_is_a_no_op(self):
        controls = (AIRCRAFT / CONTROLS_OBJ).read_text()
        if MARKER not in controls:
            self.skipTest('overhead lamps not installed yet')
        self.assertEqual(controls.count(MARKER), 1)
        for dataref, *_rest in lamps():
            self.assertEqual(controls.count(f'ATTR_light_level\t0\t1\t{dataref}\n'), 1, dataref)
            self.assertEqual(controls.count(f'ANIM_hide\t-1\t0.001\t{dataref}\n'), 1, dataref)
        self.assertEqual(controls.count(ANN_CLICK), 1)
        self.assertEqual(controls.count('ATTR_manip_command\tbutton\tsim/oxy/crew_valve_toggle\t'), 1)
        decal = (AIRCRAFT / DECAL_OBJ).read_text()
        knobs = (AIRCRAFT / KNOBS_OBJ).read_text()
        self.assertEqual(augment_controls(controls, GLTF, decal, knobs)[0], controls)
        self.assertEqual(patch_knobs(knobs, ann_lt_targets(GLTF))[0], knobs)
        self.assertEqual(knobs.count(ANN_MARKER), 1)
        for relative in ('cockpit_3d/fcu_labels.png', 'cockpit_3d/fcu_labels_LIT.png'):
            self.assertEqual(atlas_patch(AIRCRAFT / relative, controls)[1], [], relative)
        # earlier in-place patches are preserved
        for marker in ('# A380X ELEC PANEL V1', '# A380X FCU SHARED DISPLAYS V1', '# A380X NATIVE ND DEVICE QUADS V1',
                       '# A380X NATIVE ECAM SD V2', '# A380X NATIVE PFD DEVICE QUADS V1', '# A380X NATIVE ISIS 1 V1',
                       '# A380X NATIVE ISIS 2 V1', '# A380X ECAM ENG SELECTION V1', '# ADIRS ANNUNCIATOR'):
            self.assertEqual(controls.count(marker) >= 1, True, marker)
        self.assertLess(controls.index('# A380X ELEC PANEL V1'), controls.index(MARKER))
        # every lamp quad is a 14 x 7 mm legend (FBW's ENG 2 BLEED SEQ1 covers the whole cap)
        from elec_cockpit_panel import Source
        from ovhd_cockpit_lamps import face
        import math
        source = Source(GLTF)
        for _dataref, _pid, _seq, _legend, node in lamps():
            corners, _normal = face(source, node)
            self.assertAlmostEqual(math.dist(corners[0], corners[1]), 0.01394, delta=0.0002, msg=node)
            self.assertAlmostEqual(math.dist(corners[0], corners[3]), 0.00697, delta=0.0002, msg=node)
        # every lamp dataref the objects use is published by the native contract
        used = set(re.findall(r'ATTR_light_level\t0\t1\t(a380x/xp12/ovhd/[^\s]+)', controls))
        self.assertEqual(used, {row[0] for row in lamps()})


if __name__ == '__main__':
    unittest.main()
