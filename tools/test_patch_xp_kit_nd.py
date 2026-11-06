"""ND page TERRAIN layer patch: anchors, idempotence, layer updates, rejection, reuse/install hashes.

The installed aircraft page is only ever read (A380X_ND_HTML overrides its path); without it the tests
that need the real page are skipped. Fixtures live in a temporary directory.
"""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest

import patch_xp_kit_nd as patcher
from package_xp_kit_cockpit import reuse_verified_assets

INSTALLED = Path(os.environ.get('A380X_ND_HTML', '/Users/sava/Library/Application Support/Steam/steamapps/common/'
                                'X-Plane 12/Aircraft/A380X-XP12 Latest/plugins/A380XAvionics/mac_x64/xp-kit/nd.html'))


def sha(data):
    return hashlib.sha256(data).hexdigest()


def original_page():
    if not INSTALLED.exists():
        raise unittest.SkipTest('installed ND page not present')
    data = patcher.original_of(INSTALLED.read_bytes())
    if sha(data) not in patcher.KNOWN_INPUT_SHA256:
        raise unittest.SkipTest('installed ND page is not the verified original')
    return data


class PatchNdPage(unittest.TestCase):
    def test_inserts_the_layer_after_the_native_bridge_once(self):
        original = original_page()
        patched = patcher.patch_page(original)
        text = patched.decode()
        layer = patcher.LAYER.read_text()
        self.assertEqual(text.count(patcher.MARKER), 1)
        self.assertEqual(text.count('<script>' + layer + '</script>'), 1)
        bridge_end = text.index('</script>', text.index(patcher.BRIDGE_HEAD)) + len('</script>')
        self.assertTrue(text.startswith('<script>' + layer, bridge_end))
        self.assertEqual(patcher.original_of(patched), original)
        self.assertEqual(len(patched), len(original) + len(layer) + len('<script></script>'))

    def test_idempotent_and_refreshes_an_older_layer(self):
        original = original_page()
        patched = patcher.patch_page(original)
        self.assertIs(patcher.patch_page(patched), patched)
        older = patcher.patch_page(original, layer_source=patcher.LAYER.read_text().replace('const CELL = 8;', 'const CELL = 8; // old'))
        self.assertNotEqual(older, patched)
        self.assertEqual(patcher.patch_page(older), patched)

    def test_rejects_unknown_pages_and_bad_layers(self):
        original = original_page()
        with self.assertRaisesRegex(ValueError, 'Unknown ND page'):
            patcher.patch_page(original.replace(b'ND loading', b'ND Loading'))
        with self.assertRaisesRegex(ValueError, 'Not the native ND page'):
            patcher.patch_page(b'<html>native view nd.html</html>')
        with self.assertRaisesRegex(ValueError, 'terminate'):
            patcher.layer_block('// x\n' + patcher.MARKER + '\n</script>')
        with self.assertRaisesRegex(ValueError, 'marker'):
            patcher.layer_block('// x\n// no marker\n')
        doubled = patcher.patch_page(original).replace(patcher.LAYER_HEAD.encode(), patcher.LAYER_HEAD.encode() * 2, 1)
        with self.assertRaises(ValueError):
            patcher.patch_page(doubled)

    def test_reuse_patches_the_page_and_records_its_hashes(self):
        original = original_page()
        with tempfile.TemporaryDirectory(prefix='a380x-nd-terrain-test-') as temporary:
            root = Path(temporary)
            source = root / 'installed' / 'xp-kit'
            source.mkdir(parents=True)
            files = {name: ('native view ' + name).encode() for name in ('pfd.html', 'mfd.html', 'sd.html')}
            files['nd.html'] = original
            files['sd-source-manifest.json'] = json.dumps({'native': True, 'scope': 'Native ECAM SD; no preview fixtures'}).encode()
            for name, data in files.items():
                (source / name).write_bytes(data)
            manifest = {'format': 1, 'browser_server_required': False,
                        'views': ['captain_pfd', 'first_officer_pfd', 'captain_nd', 'first_officer_nd',
                                  'captain_mfd', 'first_officer_mfd', 'ecam_sd'],
                        'asset_hashes': {name: sha(data) for name, data in files.items()}}
            for name, key in (('pfd.html', 'html_sha256'), ('nd.html', 'nd_html_sha256'),
                              ('mfd.html', 'mfd_html_sha256'), ('sd.html', 'sd_html_sha256')):
                manifest[key] = manifest['asset_hashes'][name]
            (source.parent / 'xp-kit-build-report.json').write_text(json.dumps({'xp_kit_native_pfd': manifest}))
            destination = root / 'staging' / 'xp-kit'
            result = reuse_verified_assets(source, destination)
            staged = (destination / 'nd.html').read_bytes()
            self.assertEqual(staged, patcher.patch_page(original))
            self.assertEqual(result['nd_html_sha256'], sha(staged))
            self.assertEqual(result['asset_hashes']['nd.html'], sha(staged))
            self.assertEqual(result['nd_terrain_patch']['original_sha256'], sha(original))
            self.assertEqual(result['nd_terrain_patch']['layer_sha256'], sha(patcher.LAYER.read_bytes()))
            self.assertEqual(json.loads((destination / 'manifest.json').read_text())['nd_html_sha256'], sha(staged))
            self.assertEqual((source / 'nd.html').read_bytes(), original)   # the source is never written

    def test_fixtures_are_not_patched(self):
        with tempfile.TemporaryDirectory(prefix='a380x-nd-terrain-test-') as temporary:
            page = Path(temporary) / 'nd.html'
            page.write_bytes(b'native view nd.html')
            self.assertIsNone(patcher.patch_file(page))
            self.assertEqual(page.read_bytes(), b'native view nd.html')


if __name__ == '__main__':
    unittest.main()
