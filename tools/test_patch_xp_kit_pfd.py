"""PFD page animation clock patch: anchors, idempotence, clock updates, rejection, reuse hashes, and the
clock itself (JavaScriptCore, when present): suspended CSS animations advance on the native frame clock,
running ones are left alone.

The installed aircraft page is only ever read (A380X_PFD_HTML overrides its path); without it the tests
that need the real page are skipped. Fixtures live in a temporary directory.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import patch_xp_kit_pfd as patcher
from package_xp_kit_cockpit import reuse_verified_assets

INSTALLED = Path(os.environ.get('A380X_PFD_HTML', '/Users/sava/Library/Application Support/Steam/steamapps/common/'
                                'X-Plane 12/Aircraft/A380X-XP12 Latest/plugins/A380XAvionics/mac_x64/xp-kit/pfd.html'))
JSC = Path('/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def original_page():
    if not INSTALLED.exists():
        raise unittest.SkipTest('installed PFD page not present')
    data = patcher.original_of(INSTALLED.read_bytes())
    if sha(data) not in patcher.KNOWN_INPUT_SHA256:
        raise unittest.SkipTest('installed PFD page is not the verified original')
    return data


class PatchPfdPage(unittest.TestCase):
    def test_inserts_the_clock_after_the_bridge_once(self):
        original = original_page()
        patched = patcher.patch_page(original)
        text = patched.decode()
        clock = patcher.CLOCK.read_text()
        self.assertEqual(text.count(patcher.MARKER), 1)
        self.assertEqual(text.count('<script>' + clock + '</script>'), 1)
        bridge_end = text.index('</script>', text.index(patcher.BRIDGE_HEAD)) + len('</script>')
        self.assertTrue(text.startswith('<script>' + clock, bridge_end))
        self.assertLess(bridge_end, text.index('XPHFBW'))   # before the runtime that starts the page
        self.assertEqual(patcher.original_of(patched), original)
        self.assertEqual(len(patched), len(original) + len(clock.encode()) + len('<script></script>'))

    def test_idempotent_and_refreshes_an_older_clock(self):
        original = original_page()
        patched = patcher.patch_page(original)
        self.assertIs(patcher.patch_page(patched), patched)
        older = patcher.patch_page(original, clock_source=patcher.CLOCK.read_text().replace('250', '200'))
        self.assertNotEqual(older, patched)
        self.assertEqual(patcher.patch_page(older), patched)

    def test_rejects_unknown_pages_and_bad_clocks(self):
        original = original_page()
        with self.assertRaisesRegex(ValueError, 'Unknown PFD page'):
            patcher.patch_page(original.replace(b'PFD loading', b'PFD Loading'))
        with self.assertRaisesRegex(ValueError, 'Not the native PFD page'):
            patcher.patch_page(b'<html>native view pfd.html</html>')
        with self.assertRaisesRegex(ValueError, 'terminate'):
            patcher.clock_block('// x\n' + patcher.MARKER + '\n</script>')
        with self.assertRaisesRegex(ValueError, 'marker'):
            patcher.clock_block('// x\n// no marker\n')
        doubled = patcher.patch_page(original).replace(patcher.CLOCK_HEAD.encode(), patcher.CLOCK_HEAD.encode() * 2, 1)
        with self.assertRaises(ValueError):
            patcher.patch_page(doubled)

    def test_reuse_patches_the_page_and_records_its_hashes(self):
        original = original_page()
        with tempfile.TemporaryDirectory(prefix='a380x-pfd-clock-test-') as temporary:
            root = Path(temporary)
            source = root / 'installed' / 'xp-kit'
            source.mkdir(parents=True)
            files = {name: ('native view ' + name).encode() for name in ('nd.html', 'mfd.html', 'sd.html')}
            files['pfd.html'] = original
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
            staged = (destination / 'pfd.html').read_bytes()
            self.assertEqual(staged, patcher.patch_page(original))
            self.assertEqual(result['html_sha256'], sha(staged))
            self.assertEqual(result['asset_hashes']['pfd.html'], sha(staged))
            self.assertEqual(result['pfd_animation_clock_patch']['original_sha256'], sha(original))
            self.assertEqual(result['pfd_animation_clock_patch']['clock_sha256'], sha(patcher.CLOCK.read_bytes()))
            self.assertEqual(json.loads((destination / 'manifest.json').read_text())['html_sha256'], sha(staged))
            self.assertEqual((source / 'pfd.html').read_bytes(), original)   # the source is never written

    def test_fixtures_are_not_patched(self):
        with tempfile.TemporaryDirectory(prefix='a380x-pfd-clock-test-') as temporary:
            page = Path(temporary) / 'pfd.html'
            page.write_bytes(b'native view pfd.html')
            self.assertIsNone(patcher.patch_file(page))
            self.assertEqual(page.read_bytes(), b'native view pfd.html')


HARNESS = r'''
var now = 0, frames = 0;
var window = this; window.__A380_NATIVE = true; window.__a380NativeFrame = function () { ++frames; };
var performance = {now: function () { return now; }};
function Anim(running, selfTimed) { this.playState = running ? 'running' : 'finished'; this.t = 0; this.self = selfTimed; }
Object.defineProperty(Anim.prototype, 'currentTime', {get: function () { return this.t; }, set: function (v) { this.t = v; this.sets = (this.sets || 0) + 1; }});
var suspended = new Anim(true, false), live = new Anim(true, true), done = new Anim(false, false);
var document = {getAnimations: function () { return [suspended, live, done]; }};
load(arguments[0]);
for (var k = 0; k < 50; ++k) {
  now += 20; if (live.self) live.t += 20;   // an ordinary timeline moves by itself
  window.__a380NativeFrame();
}
if (frames !== 50) throw new Error('frame clock not chained: ' + frames);
if (Math.abs(suspended.t - 48 * 20) > 1e-9) throw new Error('suspended animation at ' + suspended.t);
if (live.sets) throw new Error('a running timeline was touched');
if (done.t !== 0) throw new Error('finished animation advanced');
now += 5000; window.__a380NativeFrame();
if (Math.abs(suspended.t - (48 * 20 + 250)) > 1e-9) throw new Error('long frame not capped: ' + suspended.t);
print('PFD ANIMATION CLOCK OK');
'''


class AnimationClock(unittest.TestCase):
    def test_advances_suspended_animations_only(self):
        if not JSC.exists():
            self.skipTest('JavaScriptCore not present')
        with tempfile.TemporaryDirectory(prefix='a380x-pfd-clock-js-') as temporary:
            harness = Path(temporary) / 'harness.js'
            harness.write_text(HARNESS)
            result = subprocess.run([str(JSC), str(harness), '--', str(patcher.CLOCK)], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('PFD ANIMATION CLOCK OK', result.stdout)


if __name__ == '__main__':
    unittest.main()
