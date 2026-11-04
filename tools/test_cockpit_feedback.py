#!/usr/bin/env python3
"""tools/cockpit_feedback_contract.py and tools/cockpit_feedback.py."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import cockpit_feedback as cf  # noqa: E402
import cockpit_feedback_contract as contract  # noqa: E402
from obj8_numbers import has_exponent  # noqa: E402

AIRCRAFT = ROOT / 'output/A380X-XP12'


class Contract(unittest.TestCase):
    def test_ids_and_datarefs_unique(self):
        ids = [c.id for c in contract.CONTROLS]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual(len(contract.DATAREFS), len(set(contract.DATAREFS)))
        self.assertTrue(all(ref.startswith('a380x/xp12/') for ref in contract.DATAREFS))
        self.assertTrue(all(cmd.startswith('a380x/xp12/') for cmd in contract.COMMANDS))

    def test_generated_table_is_current(self):
        table = (ROOT / 'native/avionics/a380x_cockpit_feedback_table.h').read_text()
        self.assertEqual(table, contract.emit_c_table())

    def test_kinds(self):
        for c in contract.CONTROLS:
            self.assertIn(c.kind, ('press', 'selector', 'switch', 'follow', 'static', 'toggle', 'rotary'))
            if c.kind in ('toggle', 'rotary'):
                self.assertTrue(0.0 <= c.initial <= 1.0 and c.travel >= 0.0, c.id)
            if c.kind == 'rotary':
                self.assertTrue(0.0 < c.step <= 1.0 and len(c.commands) in (0, 2), c.id)
            if c.kind == 'press':
                self.assertTrue(c.commands, c.id)
            if c.kind in ('selector', 'switch'):
                self.assertTrue(0 <= c.default < c.positions, c.id)
            if c.click and c.kind != 'follow':
                self.assertIsNotNone(contract.click_manipulator(c))

    def test_hooked_commands_belong_to_other_modules(self):
        from avionics_provider import KCCU_COMMANDS, SD_COMMANDS
        from ovhd_lamps_contract import COMMANDS as OVHD_COMMANDS
        from avionics_provider import GLARESHIELD_COMMANDS
        others = (set(KCCU_COMMANDS) | set(SD_COMMANDS) | set(OVHD_COMMANDS) | set(contract.RMP_COMMANDS)
                  | set(GLARESHIELD_COMMANDS) | {'sim/oxy/crew_valve_toggle'})
        self.assertTrue(set(contract.hooked_commands()) <= others)
        self.assertNotIn('a380x/xp12/ecam/sd/engine', contract.hooked_commands())  # ENG keeps its own motion


@unittest.skipUnless(cf.DEFAULT_SOURCE.is_file(), 'FBW cockpit glTF not available')
class Motion(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = cf.Source(cf.DEFAULT_SOURCE)
        cls.by_id = {c.id: c for c in contract.CONTROLS}

    def test_press_runs_zero_to_one(self):
        m = cf.compile_motion(self.source, self.by_id['ecam_hyd'])
        stage = m['stages'][0]
        self.assertEqual(stage['kind'], 'translation')
        self.assertEqual((stage['values'][0], stage['values'][-1]), (0.0, 1.0))
        travel = sum(v * v for v in stage['poses'][-1]) ** .5
        self.assertTrue(0.0003 < travel < 0.002, travel)

    def test_selector_keys_are_detents(self):
        m = cf.compile_motion(self.source, self.by_id['switching_att_hdg'])
        stage = m['stages'][0]
        self.assertEqual(stage['kind'], 'rotation')
        self.assertEqual(stage['values'], [0.0, 1.0, 2.0])
        self.assertAlmostEqual(abs(stage['axis'][2]), 0.94, places=2)   # panel normal, not in-plane

    def test_switch_values_rise_with_the_lever(self):
        m = cf.compile_motion(self.source, self.by_id['captain_console_floor_lt'])
        self.assertEqual(sorted(m['stages'][0]['values']), [0.0, 1.0, 2.0])

    def test_emitted_numbers_have_no_exponent(self):
        for cid in ('ecam_hyd', 'switching_fms', 'eng1_master_decals', 'kccu_captain_a'):
            for line in cf.stage_lines(cf.compile_motion(self.source, self.by_id[cid])):
                self.assertFalse(has_exponent(line), line)


@unittest.skipUnless((AIRCRAFT / cf.CONTROLS_OBJ).is_file(), 'installable aircraft not present')
class InstalledAircraft(unittest.TestCase):
    def test_every_clickable_control_has_a_target(self):
        text = (AIRCRAFT / cf.CONTROLS_OBJ).read_text(encoding='latin-1')
        for c in contract.CONTROLS:
            if contract.click_manipulator(c):
                self.assertIn(f'{cf.MARKER} CLICK {c.id}\n', text, c.id)
            for lamp in c.lamps:
                if lamp.cell is not None:
                    self.assertIn(f'{cf.MARKER} LAMP {c.id} {lamp.face}\n', text)

    def test_every_control_moves_visible_geometry(self):
        files = {p.name: p.read_text(encoding='latin-1') for d in cf.PANEL_DIRS for p in (AIRCRAFT / d).glob('*.obj')
                 if not p.name.startswith('._')}
        source = cf.Source(cf.DEFAULT_SOURCE) if cf.DEFAULT_SOURCE.is_file() else None
        if source is not None:
            motions = cf.link_motions(source, [cf.compile_motion(source, c) for c in contract.CONTROLS])
            owned = {m['control'].id for m in motions if cf.mesh_targets(source, m)}
            owns_mesh = owned.__contains__
        for c in contract.CONTROLS:
            if c.kind == 'static':
                continue
            if source is not None:   # a material with no exported OBJ (e.g. the RMP glass guard) has nothing to move
                materials = {m for i in source.descendants(source.names[c.node]) for m, _t, _n in source.primitives(i)}
                if not any(name.endswith(m + '.obj') for name in files for m in materials):
                    continue
                if not owns_mesh(c.id):   # moves only nodes below it that another control owns
                    continue
            self.assertTrue(any(f'{cf.MARKER} {c.id} ' in t for t in files.values()), c.id)

    def test_range_knobs_spin_on_one_axis(self):
        from fix_euler_knobs import MARKER as EULER
        knob = (AIRCRAFT / 'objects/interior/cockpit-panels/panel_a380_cockpit_mip_knob01_4k.obj').read_text(encoding='latin-1')
        self.assertEqual(knob.count(EULER), 2)
        for side in ('captain', 'first_officer'):
            self.assertEqual(knob.count(f'ANIM_rotate_begin\t') and
                             sum(1 for line in knob.splitlines()
                                 if line.strip().startswith('ANIM_rotate_begin') and line.endswith(f'{side}/range_index')), 1)

    def test_rmp_screens(self):
        text = (AIRCRAFT / cf.SCREENS_OBJ).read_text(encoding='latin-1')
        self.assertEqual(text.count(cf.RMP_MARKER), 1)
        for device in contract.RMP_DEVICES:
            self.assertEqual(text.count(f'ATTR_cockpit_device\t{device}\t'), 1)

    def test_manifest_section(self):
        import json
        doc = json.loads((AIRCRAFT / cf.PROVIDER_MANIFEST).read_text())
        self.assertEqual(doc['cockpit_feedback']['commands'], list(contract.COMMANDS))
        self.assertEqual(doc['cockpit_feedback']['datarefs'], list(contract.DATAREFS))
        self.assertEqual(doc['cockpit_feedback']['rmp_commands'], list(contract.RMP_COMMANDS))
        self.assertEqual(doc['cockpit_feedback']['cockpit_devices'], list(contract.RMP_DEVICES))


if __name__ == '__main__':
    unittest.main()
