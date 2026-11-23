"""Verify native display reuse on isolated fixtures, never the installed aircraft."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from package_xp_kit_cockpit import reuse_verified_assets


class ReuseNativeViews(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='a380x-reuse-assets-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / 'installed' / 'xp-kit'
        self.source.mkdir(parents=True)
        self.destination = self.root / 'staging' / 'xp-kit'
        self.files = {
            name: ('native view ' + name).encode()
            for name in ('pfd.html', 'nd.html', 'mfd.html', 'sd.html')
        }
        self.files.update({
            'sd-source-manifest.json': json.dumps({
                'native': True,
                'scope': 'Native ECAM SD; read-only live values; no preview fixtures',
            }).encode(),
            'Fonts/fbw-a380x/Ecam.ttf': b'font fixture',
            'Images/fbw-a380x/icon.svg': b'<svg/>',
            'LICENSE': b'GPL-3.0-or-later',
        })
        for name, data in self.files.items():
            target = self.source / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        self.manifest = {
            'format': 1,
            'browser_server_required': False,
            'views': ['captain_pfd', 'first_officer_pfd', 'captain_nd',
                      'first_officer_nd', 'captain_mfd', 'first_officer_mfd', 'ecam_sd'],
            'helper_sha256': 'old-native-helper-must-not-be-reused',
            'architectures': ['arm64', 'x86_64'],
            'asset_hashes': {name: self.sha(data) for name, data in self.files.items()},
        }
        for name, key in (('pfd.html', 'html_sha256'), ('nd.html', 'nd_html_sha256'),
                          ('mfd.html', 'mfd_html_sha256'), ('sd.html', 'sd_html_sha256')):
            self.manifest[key] = self.manifest['asset_hashes'][name]
        self.report_path = self.source.parent / 'xp-kit-build-report.json'
        self.write_report()

    @staticmethod
    def sha(data):
        return hashlib.sha256(data).hexdigest()

    def write_report(self):
        self.report_path.write_text(json.dumps({'xp_kit_native_pfd': self.manifest}))

    def rejected(self, message):
        with self.assertRaisesRegex(ValueError, message):
            reuse_verified_assets(self.source, self.destination)
        self.assertFalse(self.destination.exists(), 'Validation must precede destination creation')

    def test_complete_seven_view_package_is_copied_byte_identically(self):
        # An unrecorded file is never trusted or copied, and no old helper is copied.
        (self.source / 'unrecorded.html').write_text('untrusted')
        manifest = reuse_verified_assets(self.source, self.destination)
        for name, expected in self.files.items():
            self.assertEqual((self.destination / name).read_bytes(), expected)
            self.assertEqual(self.sha((self.destination / name).read_bytes()),
                             manifest['asset_hashes'][name])
        self.assertFalse((self.destination / 'unrecorded.html').exists())
        # The web EFB page (views 7/8) is never reused: it is rebuilt from the repository sources.
        self.assertEqual(manifest['views'], [*self.manifest['views'], 'captain_efb', 'first_officer_efb'])
        self.assertEqual(manifest['asset_hashes']['efb.html'], manifest['efb_html_sha256'])
        self.assertTrue((self.destination / 'Fonts/efb/OFL.txt').is_file())
        self.assertNotIn('helper_sha256', manifest)
        self.assertNotIn('architectures', manifest)
        self.assertEqual(manifest['reused_assets_from'], str(self.source.resolve()))
        self.assertEqual(manifest['reused_build_report_sha256'],
                         self.sha(self.report_path.read_bytes()))
        self.assertEqual(json.loads((self.destination / 'manifest.json').read_text()), manifest)

    def test_corrupt_asset_is_rejected_before_destination_creation(self):
        (self.source / 'sd.html').write_bytes(b'not the verified renderer')
        self.rejected('asset hash mismatch: sd.html')

    def test_parent_traversal_is_rejected(self):
        outside = self.source.parent / 'outside.txt'
        outside.write_bytes(b'not an asset')
        self.manifest['asset_hashes']['../outside.txt'] = self.sha(outside.read_bytes())
        self.write_report()
        self.rejected('Unsafe renderer asset path')

    def test_absolute_path_is_rejected(self):
        outside = self.root / 'outside.txt'
        outside.write_bytes(b'not an asset')
        self.manifest['asset_hashes'][str(outside)] = self.sha(outside.read_bytes())
        self.write_report()
        self.rejected('Unsafe renderer asset path')

    def test_symlink_escape_is_rejected(self):
        outside = self.root / 'outside.txt'
        outside.write_bytes(b'not an asset')
        (self.source / 'escape.txt').symlink_to(outside)
        self.manifest['asset_hashes']['escape.txt'] = self.sha(outside.read_bytes())
        self.write_report()
        self.rejected('Unsafe renderer asset path')

    def test_missing_native_sd_marker_is_rejected(self):
        for sd in ({'scope': 'Native ECAM SD; no preview fixtures'},
                   {'native': False, 'scope': 'Native ECAM SD; no preview fixtures'},
                   {'native': True, 'scope': 'Browser preview fixtures'}):
            with self.subTest(sd=sd):
                data = json.dumps(sd).encode()
                (self.source / 'sd-source-manifest.json').write_bytes(data)
                self.manifest['asset_hashes']['sd-source-manifest.json'] = self.sha(data)
                self.write_report()
                self.rejected('Only verified native ECAM views')

    def test_missing_asset_hash_manifest_is_rejected(self):
        self.manifest.pop('asset_hashes')
        self.write_report()
        self.rejected('no complete asset hash manifest')

    def test_missing_required_asset_hash_is_rejected(self):
        for missing in ('pfd.html', 'nd.html', 'mfd.html', 'sd.html', 'sd-source-manifest.json'):
            with self.subTest(missing=missing):
                expected = self.manifest['asset_hashes'].pop(missing)
                self.write_report()
                self.rejected('no complete asset hash manifest')
                self.manifest['asset_hashes'][missing] = expected

    def test_incomplete_view_contract_is_rejected(self):
        self.manifest['views'].remove('ecam_sd')
        self.write_report()
        self.rejected('complete native seven-view package')

    def test_browser_dependent_package_is_rejected(self):
        self.manifest['browser_server_required'] = True
        self.write_report()
        self.rejected('complete native seven-view package')

    def test_top_level_view_hash_mismatch_is_rejected(self):
        self.manifest['sd_html_sha256'] = 'not the declared asset hash'
        self.write_report()
        self.rejected('View hash/report mismatch: sd.html')

    def test_existing_destination_is_not_overwritten(self):
        self.destination.mkdir(parents=True)
        target = self.destination / 'sd.html'
        target.write_bytes(b'existing staging content')
        with self.assertRaises(FileExistsError):
            reuse_verified_assets(self.source, self.destination)
        self.assertEqual(target.read_bytes(), b'existing staging content')


if __name__ == '__main__':
    unittest.main()
